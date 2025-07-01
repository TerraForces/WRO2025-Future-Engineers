/**
 * car.cpp - the (hidden) functional core of our autonomously driving car
 * by TerraForce
 */

// optimize program for higher execution speed
#pragma GCC optimize("-Ofast")

// include header file
#include "car.hpp"

// include library for the st7735 lcd controller
#include <Adafruit_ST7735.h>

// include library for the ds18b20 temperature sensor
#include <DallasTemperature.h>

// include ledc channel control driver
#include <driver/ledc.h>

// include uart bus driver
#include <driver/uart.h>

// include bluetooth and wifi drivers for deactivation functions
#include <esp_bt.h>
#include <esp_wifi.h>

// include c library for the mpu9250 gyro and acceleration sensor
extern "C" {
    #include <inv_mpu.h>
    #include <inv_mpu_dmp_motion_driver.h>
}

// include for vector support
#include <vector>

// undefine MPU9250 macro to use it as class name
#undef MPU9250

// macro for optional error logging
#define SNIPE(condition) if(condition) { log_e("sniped"); }

// pin configuration
constexpr uint8_t Pin_ST7735_LED             = 0;  // output - lcd background led control over pwm
constexpr uint8_t Pin_MPU9250_Data_Ready     = 2;  // input  - interrupt when mpu9250 fifo receives new data
constexpr uint8_t Pin_ST7735_DC              = 4;  // output - 
constexpr uint8_t Pin_ST7735_RST             = 5;
constexpr uint8_t Pin_ST7735_CS              = 12; // output - lcd spi chip select
constexpr uint8_t Pin_ST7735_MOSI            = 13; // output - lcd spi data out
constexpr uint8_t Pin_ST7735_SCK             = 14; // output - lcd spi serial clock
constexpr uint8_t Pin_Start_Button_LED       = 15; // output - start button led
constexpr uint8_t Pins_VL53L1X_Shutdown[4]   = { 16, 17, 18, 19 };
constexpr uint8_t Pin_I2C_0_SDA			     = 21; // in/out - i2c bus 0 data (bidirectional)
constexpr uint8_t Pin_I2C_0_SCL			     = 22; // output - i2c bus 0 serial clock
constexpr uint8_t Pin_Servo_PWM				 = 23; // output - servo control pwm signal
constexpr uint8_t Pin_Motor_Direction_Invert = 25; // output - motor direction invert relais control
constexpr uint8_t Pin_Motor_PWM				 = 26; // output - motor control pwm signal
constexpr uint8_t Pin_DS18B20_OneWire        = 27; // in/out - OneWire bus for communication with the ds18b20 temperature sensor
constexpr uint8_t Pin_I2C_1_SDA			     = 32; // in/out - i2c bus 1 data (bidirectional)
constexpr uint8_t Pin_I2C_1_SCL			     = 33; // output - i2c bus 1 serial clock
constexpr uint8_t Pin_Battery_Voltage		 = 34; // input  - battery voltage after voltage divider (10V battery voltage = 3.3V voltage at pin)
constexpr uint8_t Pin_Total_Current		     = 35; // input  - total current measured over 0.25 ohms resistor and multiplied by 20
constexpr uint8_t Pin_Motor_Current		     = 36; // input  - total current measured over 0.33 ohms resistor and multiplied by 20
constexpr uint8_t Pin_Start_Button           = 39; // input  - start button (externally pulled up)

// motor pwm configuration
constexpr uint16_t Motor_PWM_Min_Frequency   = 8000;  // minimal pwm frequency used for motor pwm
constexpr uint16_t Motor_PWM_Max_Frequency   = 12000; // maximum pwm frequency used for motor pwm
constexpr float Motor_PWM_Time_To_Max        = 2.0f;  // time in seconds for the motor pwm to reach 1.0f or -1.0f from 0.0f
constexpr float Motor_PWM_Time_To_Null       = 0.5f;  // time in seconds for the motor pwm to reach 0.0f from 1.0f or -1.0f
constexpr float Motor_PWM_Update_Interval    = 0.05f; // motor pwm update interval in seconds

// servo pwm configuration
constexpr uint16_t Servo_PWM_0_Duty          = 7940;  // servo pwm duty in 0° position (might be adjusted after hardware changes)
constexpr float Servo_PWM_Time_To_Max        = 0.5f;  /////////////////////////////////////////////////////////////////////////////////////////////////////////////// TODO
constexpr float Servo_PWM_Update_Interval	 = 0.05f; // servo pwm update interval in seconds

// lcd configuration
constexpr uint16_t LCD_LED_PWM_Frequency     = 1000;  // frequency of lcd backlight pwm
constexpr float Battery_Voltage_Warning      = 7.0f;  // battery voltage less than this value is shown yellow on lcd
constexpr float Battery_Voltage_Empty        = 6.6f;  // battery voltage less than this value is shown red on lcd

// handles of background tasks
TaskHandle_t motorAccelerationThread, servoAccelerationThread, i2cThread0, i2cThread1;

// data transport between background tasks and main
volatile float lastRotation[3] = {};
volatile double _rotationOffset[3] = {};
volatile int16_t rounds[3] = {};
volatile uint16_t distances[4] = {};
volatile uint8_t ledState = 0x00;
volatile int32_t motorTargetDuty = 0;
volatile int32_t servoTargetDuty = Servo_PWM_0_Duty;
volatile uint64_t taskFlowFailures[4] = {};

Adafruit_ST7735 st7735(Pin_ST7735_CS, Pin_ST7735_DC, Pin_ST7735_MOSI, Pin_ST7735_SCK, Pin_ST7735_RST);
OneWire oneWire = OneWire(Pin_DS18B20_OneWire);
DallasTemperature ds18b20 = DallasTemperature(&oneWire);
DeviceAddress ds18b20Address = {};
std::vector<CAR::OBJECT> detections = { CAR::OBJECT {0, 0, 0, 0, 0, 0 } }, currentDetections;
SemaphoreHandle_t uartMutexLock = 0;
uart_t* uart = 0;
constexpr uint16_t objectColors[4] = { 0x0000, 0xF800, 0x0600, 0xF81F };
constexpr uint8_t vl53l1xConfiguration[91] = {
	0x00, 0x01, 0x01, 0x01, 0x02, 0x00, 0x02, 0x08,
    0x00, 0x08, 0x10, 0x01, 0x01, 0x00, 0x00, 0x00,
    0x00, 0xff, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x20, 0x0b, 0x00, 0x00, 0x02, 0x0a, 0x21,
    0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xc8,
    0x00, 0x00, 0x38, 0xff, 0x01, 0x00, 0x08, 0x00,
    0x00, 0x01, 0xdb, 0x0f, 0x01, 0xf1, 0x0d, 0x01,
    0x68, 0x00, 0x80, 0x08, 0xb8, 0x00, 0x00, 0x00,
	0x00, 0x0f, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x01, 0x0f, 0x0d, 0x0e, 0x0e, 0x00,
    0x00, 0x02, 0xc7, 0xff, 0x9B, 0x00, 0x00, 0x00,
    0x01, 0x00, 0x00
};
volatile uint8_t vl53l1xViewCenters[4] = { 63, 62, 62, 60 };
volatile uint8_t lastVl53l1xViewCenters[4] = {};

uint8_t vl53l1xReadByte(uint8_t add, uint16_t reg) {
    for(uint8_t i = 0; i < 5; i++) {
        Wire1.beginTransmission(add);
        Wire1.write(reg >> 8);
        Wire1.write(reg & 0xFF);
        if(Wire1.endTransmission(false) == 0) break;
    }
    Wire1.requestFrom(add, (uint8_t)1);
    return Wire1.available() ? Wire1.read() : 0;
}

uint16_t vl53l1xReadWord(uint8_t add, uint16_t reg) {
    for(uint8_t i = 0; i < 5; i++) {
        Wire1.beginTransmission(add);
        Wire1.write(reg >> 8);
        Wire1.write(reg & 0xFF);
        if(Wire1.endTransmission(false) == 0) break;
    }
    Wire1.requestFrom(add, (uint8_t)2);
    uint8_t firstByte = Wire1.read();
    uint8_t secondByte = Wire1.read();
    return (firstByte << 8) + secondByte;
}

void vl53l1xWriteByte(uint8_t add, uint16_t reg, uint8_t data) {
    Wire1.beginTransmission(add);
    Wire1.write(reg >> 8);
    Wire1.write(reg & 0xFF);
    Wire1.write(data);
    Wire1.endTransmission(true);
}

void vl53l1xWriteWord(uint8_t add, uint16_t reg, uint16_t data) {
    Wire1.beginTransmission(add);
    Wire1.write(reg >> 8);
    Wire1.write(reg & 0xFF);
    Wire1.write(data >> 8);
    Wire1.write(data & 0xFF);
    Wire1.endTransmission(true);
}

void vl53l1xWriteDWord(uint8_t add, uint16_t reg, uint32_t data) {
    Wire1.beginTransmission(add);
    Wire1.write(reg >> 8);
    Wire1.write(reg & 0xFF);
    Wire1.write(data >> 24);
    Wire1.write((data >> 16) & 0xFF);
    Wire1.write((data >> 8) & 0xFF);
    Wire1.write(data & 0xFF);
    Wire1.endTransmission(true);
}

void mpu9250_interrupt() {
    BaseType_t higherPriorityTaskWoken = 0;
	xTaskNotifyFromISR(i2cThread0, 0, eNoAction, &higherPriorityTaskWoken);
}

inline float qToFloat(long value) {
    return (value >> 30) + ((value & 0x3FFFFFFF) / (float)0x40000000);
}

void i2c0_task(void* unused) {
    attachInterrupt(Pin_MPU9250_Data_Ready, mpu9250_interrupt, FALLING);
    while(true) {
		if(xTaskNotifyWait(0, 0, 0, 20) == pdPASS) {
            short gyro[3], accel[3], sensors;
            long quat[4], raw[4];
            unsigned long timestamp;
            unsigned char more;
            dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more);
            memcpy(raw, quat, 16);
            if (!dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more)) memcpy(raw, quat, 16);
            float a0 = qToFloat(quat[0]);
            float a1 = qToFloat(quat[1]);
            float a2 = qToFloat(quat[2]);
            float a3 = qToFloat(quat[3]);
            float b = -2.0f * ((a1 * a3) + (a0 * a2));
            float degrees[3] = {
                asin(constrain(b, -1.0f, 1.0f)) * 114.59155902616465f,
                atan2(2.0f * ((a2 * a3) - (a0 * a1)), (-2.0f * ((a1 * a1) + (a2 * a2))) + 1.0f) * 57.29577951308232f,
                atan2(2.0f * ((a1 * a2) - (a0 * a3)), (-2.0f * ((a2 * a2) + (a3 * a3))) + 1.0f) * 57.29577951308232f
            };
            for(uint8_t i = 0; i < 3; i++) {
                if(degrees[i] < 0.0f) degrees[i] += 360.0f;
                if((degrees[i] < 60.0f) && (lastRotation[i] > 300.0f)) rounds[i]++;
                if((degrees[i] > 300.0f) && (lastRotation[i] < 60.0f)) rounds[i]--;
                lastRotation[i] = degrees[i];
            }
        }
        else taskFlowFailures[0]++;
	}
}

void i2c1_task(void* unused) {
    Wire1.beginTransmission(0x20);
    Wire1.write(0x00);
    Wire1.write(0x00);
    Wire1.endTransmission(true);

    for(uint8_t i = 0; i < 4; i++) {
        vl53l1xWriteByte(0x30 + i, 0x0086, 0x01);
        vl53l1xWriteByte(0x30 + i, 0x0087, 0x40);
    }
    TickType_t lastWaitTime = xTaskGetTickCount();
    uint8_t lastLedState = 0;
    while(true) {
		if(!xTaskDelayUntil(&lastWaitTime, 6)) taskFlowFailures[1]++;

        // set new led state only if it has changed
        if(lastLedState != ledState) {
            lastLedState = ledState;

            // start I2C transmission to address 0x20 (MCP23008-E/P)
            Wire1.beginTransmission(0x20);

            // write new led state to register 0x09
            SNIPE(!Wire1.write(0x09));
            SNIPE(!Wire1.write(ledState));

            // end I2C transmission
            SNIPE(Wire1.endTransmission(true));
        }
        for(uint8_t i = 0; i < 4; i++) {
            if((vl53l1xReadByte(0x30 + i, 0x0031) & 1) == (!((vl53l1xReadByte(0x30 + i, 0x0030) >> 4) & 1))) {
                vl53l1xWriteByte(0x30 + i, 0x0086, 0x01);
                distances[i] = vl53l1xReadWord(0x30 + i, 0x0096);
            }
            if(lastVl53l1xViewCenters[i] != vl53l1xViewCenters[i]) {
                vl53l1xWriteByte(0x30 + i, 0x0087, 0x00);
                vl53l1xWriteByte(0x30 + i, 0x007F, vl53l1xViewCenters[i]);
                vl53l1xWriteByte(0x30 + i, 0x0086, 0x01);
                vl53l1xWriteByte(0x30 + i, 0x0087, 0x40);
                lastVl53l1xViewCenters[i] = vl53l1xViewCenters[i];
            }
        }
	}
}

void motor_control_task(void* unused) {
	TickType_t lastWakeTime = xTaskGetTickCount();
    constexpr TickType_t updateInterval = (1000 * Servo_PWM_Update_Interval) + 0.5;
	int32_t currentMotorDuty = 0;
	constexpr int32_t motorMaxDuty = 1 << LEDC_TIMER_12_BIT;
	constexpr int32_t motorAccelerationRate = (motorMaxDuty * Servo_PWM_Update_Interval / Motor_PWM_Time_To_Max) + 0.5;
	constexpr int32_t motorDeaccelerationRate = (motorMaxDuty * Servo_PWM_Update_Interval / Motor_PWM_Time_To_Null) + 0.5;
	while(true) {
		if(!xTaskDelayUntil(&lastWakeTime, updateInterval)) taskFlowFailures[2]++;
        if     ((currentMotorDuty >= 0) && (motorTargetDuty > currentMotorDuty)) currentMotorDuty = _min(motorTargetDuty, currentMotorDuty + motorAccelerationRate);
        else if((currentMotorDuty >= 0) && (motorTargetDuty < currentMotorDuty)) currentMotorDuty = _max(motorTargetDuty, currentMotorDuty - motorDeaccelerationRate);
        else if((currentMotorDuty <= 0) && (motorTargetDuty > currentMotorDuty)) currentMotorDuty = _min(motorTargetDuty, currentMotorDuty + motorDeaccelerationRate);
        else if((currentMotorDuty <= 0) && (motorTargetDuty < currentMotorDuty)) currentMotorDuty = _max(motorTargetDuty, currentMotorDuty - motorAccelerationRate);
		SNIPE(ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, (uint32_t)((Motor_PWM_Max_Frequency - Motor_PWM_Min_Frequency) * (abs(currentMotorDuty) / (float)motorMaxDuty)) + Motor_PWM_Min_Frequency));
		SNIPE(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, abs(currentMotorDuty)));
		SNIPE(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
		digitalWrite(Pin_Motor_Direction_Invert, currentMotorDuty < 0);
	}
}

void servo_control_task(void* unused) {
	TickType_t lastWakeTime = xTaskGetTickCount();
    constexpr TickType_t updateInterval = (1000 * Servo_PWM_Update_Interval) + 0.5;
	int32_t currentServoDuty = Servo_PWM_0_Duty;
    constexpr int32_t servoMaxDuty = 1 << LEDC_TIMER_14_BIT;
	constexpr int32_t servoChangeRate = (servoMaxDuty * Servo_PWM_Update_Interval / Servo_PWM_Time_To_Max) + 0.5;
	while(true) {
		if(!xTaskDelayUntil(&lastWakeTime, updateInterval)) taskFlowFailures[3]++;
		if(servoTargetDuty > currentServoDuty) currentServoDuty = _min(servoTargetDuty, currentServoDuty + servoChangeRate);
		if(servoTargetDuty < currentServoDuty) currentServoDuty = _max(servoTargetDuty, currentServoDuty - servoChangeRate);
		SNIPE(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, currentServoDuty));
		SNIPE(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1));
	}
}

void uart_task(void* unused) {
    uart_event_t event;
    QueueHandle_t uartEventQueue = 0;
    uartGetEventQueue(uart, &uartEventQueue);
    while(true) {
        if(xQueueReceive(uartEventQueue, &event, portMAX_DELAY)) {
            if(event.type == UART_DATA) {
                while(uartAvailable(uart) >= sizeof(CAR::OBJECT)) {
                    CAR::OBJECT object = {};
                    uartReadBytes(uart, (uint8_t*)&object, sizeof(CAR::OBJECT), 0);
                    if(object.c) currentDetections.push_back(object);
                    else {
                        detections = currentDetections;
                        currentDetections.clear();
                    }
                }
            }
        }
    }
}

CAR::BUTTON::BUTTON() {
    pinMode(Pin_Start_Button, INPUT);
    pinMode(Pin_Start_Button_LED, OUTPUT);
    digitalWrite(Pin_Start_Button_LED, LOW);
}

bool CAR::BUTTON::pressed() {
    return !digitalRead(Pin_Start_Button);
}

void CAR::BUTTON::setLED(bool state) {
    digitalWrite(Pin_Start_Button_LED, state);
}

void CAR::BUTTON::wait() {
    digitalWrite(Pin_Start_Button_LED, HIGH);
    while(digitalRead(Pin_Start_Button)) vTaskDelay(1);
    digitalWrite(Pin_Start_Button_LED, LOW);
}

CAR::BUTTON CAR::button;

CAR::CAMERA::CAMERA() {
    uartMutexLock = xSemaphoreCreateMutex();
    uartSetPins(0, SOC_RX0, SOC_TX0, -1, -1);
    xSemaphoreTake(uartMutexLock, portMAX_DELAY);
    int8_t rxPin = uart_get_RxPin(0);
    int8_t txPin = uart_get_TxPin(0);
    uart = uartBegin(0, 115200, SERIAL_8N1, rxPin < 0 ? (int8_t)SOC_RX0 : rxPin, txPin < 0 ? (int8_t)SOC_TX0 : txPin, 256, 0, false, 120);
    xTaskCreatePinnedToCore(uart_task, "UART Task", 3000, 0, 24, 0, 1 - xPortGetCoreID());
    uartSetRxTimeout(uart, 2);
    uartSetRxFIFOFull(uart, 120);
    xSemaphoreGive(uartMutexLock);
}

CAR::OBJECT CAR::CAMERA::getObject(uint8_t idx) {
    if(idx >= detections.size()) return {};
    return detections[idx];
}

bool CAR::CAMERA::ready() {
    if(detections.size() == 1) {
        if(detections[0].c == 0) return false;
    }
    return true;
}

CAR::CAMERA CAR::camera;

CAR::LCD::LCD() {
    pinMode(Pin_ST7735_LED, OUTPUT);
    digitalWrite(Pin_ST7735_LED, LOW);
    ledc_channel_config_t ledcChannelConfig = {
		.gpio_num       = Pin_ST7735_LED,
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_2,
		.intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER_2,
        .duty           = 0x3FFF,
        .hpoint         = 0
    };
    SNIPE(ledc_channel_config(&ledcChannelConfig));
	ledc_timer_config_t ledcTimerConfig = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
		.duty_resolution  = LEDC_TIMER_14_BIT,
        .timer_num        = LEDC_TIMER_2,
        .freq_hz          = LCD_LED_PWM_Frequency,
        .clk_cfg          = LEDC_AUTO_CLK
    };
	SNIPE(ledc_timer_config(&ledcTimerConfig));
    st7735.initR(INITR_BLACKTAB);
    st7735.fillScreen(0x0000);
    st7735.setTextWrap(false);
    st7735.setTextSize(2);
    st7735.fillRect(4, 4, 32, 14, 0xFFFF);
    st7735.fillRect(36, 7, 4, 8, 0xFFFF);
    showBatteryVoltage();
    st7735.setTextColor(0xFFFF);
    st7735.setCursor(4, 40);
    st7735.print("System");
    st7735.setCursor(4, 60);
    st7735.print("starting");
    esp_wifi_deinit();
    esp_bt_controller_deinit();
}

void CAR::LCD::setBrightness(float brightness) {
    SNIPE(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, constrain(brightness, 0.0f, 1.0f) * 0x3FFF));
	SNIPE(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2));
}

void CAR::LCD::showBatteryVoltage() {
    float voltage = supervision.getVoltage();
    st7735.setTextColor(voltage < Battery_Voltage_Empty ? 0xF800 : (voltage < Battery_Voltage_Warning ? 0xFFE0 : 0x07E0), 0x0000);
    st7735.setCursor(48, 4);
    st7735.print(voltage, 2);
    st7735.print(" V ");
}

void CAR::LCD::showDistanceCalibration(uint8_t i) {
    st7735.fillScreen(0x0000);
    showBatteryVoltage();
    st7735.setTextColor(0xFFFF);
    st7735.drawLine(i < 2 ? 4 : 11, (i < 2 ? 31 : 24), i < 2 ? 18 : 11, (i < 2 ? 31 : 38), 0xFFFF);
    st7735.drawLine(i % 2 ? 18 : 4, 31, 11, (i % 2 ? 38 : 24), 0xFFFF);
    st7735.drawLine((i < 1 || i > 2) ? 4 : 18, 31, 11, ((i < 1 || i > 2) ? 38 : 24), 0xFFFF);
    st7735.setCursor(26, 24);
    st7735.print("2m");
    st7735.setCursor(4, 50);
    st7735.print("Press");
    st7735.setCursor(4, 70);
    st7735.print("button");
    st7735.setCursor(4, 90);
    st7735.print("to start");
    button.wait();
    showBatteryVoltage();
    st7735.setTextColor(0xFFFF, 0x0000);
    st7735.setCursor(4, 50);
    st7735.print("Testing");
    st7735.setCursor(4, 70);
    st7735.print("Viewport");
    st7735.setCursor(4, 90);
    st7735.print("Centers ");
    uint8_t lastVl53l1xViewCenter = vl53l1xViewCenters[i];
    uint32_t testingResults[11] = {};
    TickType_t lastDelay = xTaskGetTickCount();
    for(uint16_t j = 0; j < 11; j++) {
        vl53l1xViewCenters[i] = (j < 5) ? (59 + j) : (204 - j);
        st7735.setCursor(4, 110);
        st7735.printf("%hus left ", 11 - j);
        xTaskDelayUntil(&lastDelay, 100);
        for(uint8_t k = 0; k < 90; k++) {
            testingResults[j] += distances[i];
            xTaskDelayUntil(&lastDelay, 10);
        }
    }
    vl53l1xViewCenters[i] = lastVl53l1xViewCenter;
    st7735.fillRect(3, 24, 110, 100, 0x0000);
    st7735.setTextColor(0xFFFF);
    st7735.setTextSize(1);
    for(uint8_t j = 0; j < 11; j++) {
        st7735.setCursor(4, 140 - (j * 10));
        st7735.print(((j < 5) ? (59 + j) : (204 - j)));
        st7735.fillRect(30, 140 - (j * 10), abs(testingResults[j] - 180000ll) / 450, 8, (abs(testingResults[j] - 180000ll) < 9000) ? 0x7E0 : ((abs(testingResults[j] - 180000ll) < 18000) ? 0xFFE0 : 0xF800));
    }
    for(uint16_t j = 0; j < 5; j++) {
        st7735.drawLine(30 + (j * 20), 36, 30 + (j * 20), 152, 0xFFFF);
        st7735.setCursor(30 + (j * 20), 26);
        st7735.printf("%hu%%", j * 5);
    }
    st7735.setTextSize(2);
}

void CAR::LCD::showDistances() {
    showBatteryVoltage();
    st7735.fillRect(3, 24, 110, 100, 0x0000);
    st7735.setTextColor(0xFFFF);
    for(uint8_t i = 0; i < 4; i++) {
        st7735.drawLine(i < 2 ? 4 : 11, (i < 2 ? 31 : 24) + (i * 20), i < 2 ? 18 : 11, (i < 2 ? 31 : 38) + (i * 20), 0xFFFF);
        st7735.drawLine(i % 2 ? 18 : 4, 31 + (i * 20), 11, (i % 2 ? 38 : 24) + (i * 20), 0xFFFF);
        st7735.drawLine((i < 1 || i > 2) ? 4 : 18, 31 + (i * 20), 11, ((i < 1 || i > 2) ? 38 : 24) + (i * 20), 0xFFFF);
        st7735.setCursor(26, (i * 20) + 24);
        st7735.print(vl53l1x.getDistance(i));
        st7735.print(" mm  ");
    }
}

void CAR::LCD::showFinish(uint32_t duration) {
    duration /= 1000u;
    showBatteryVoltage();
    st7735.setTextColor(0x07E0, 0x0000);
    st7735.setCursor(4, 40);
    st7735.print("Race finished");
    st7735.setTextColor(0xFFFF, 0x0000);
    st7735.fillRect(3, 24, 125, 120, 0x0000);
    st7735.setCursor(4, 70);
    st7735.print("Time:");
    st7735.setCursor(4, 90);
    st7735.printf("%02u:%02u:%03u", duration / 60000u, (duration / 1000u) % 60u, duration % 1000u);
    if(*std::min_element(taskFlowFailures, taskFlowFailures + 4) > 0) {
        st7735.setCursor(4, 110);
        st7735.printf("%lu %lu", taskFlowFailures[0], taskFlowFailures[1]);
        st7735.setCursor(4, 130);
        st7735.printf("%lu %lu", taskFlowFailures[2], taskFlowFailures[3]);
    }
}

void CAR::LCD::showObjects() {
    std::vector<CAR::OBJECT> objectBuf = detections;
    showBatteryVoltage();
    st7735.fillRect(0, 22, 128, 138, 0x0000);
    for(uint16_t i = 1; i <= objectBuf.size(); i++) {
        uint16_t j = objectBuf.size() - i;
        st7735.fillRect(10 + (objectBuf[j].l / 12u), 32 + (objectBuf[j].t / 12u), objectBuf[j].w / 12u, objectBuf[j].h / 12u, objectColors[objectBuf[j].c]);
    }
    st7735.drawRect(9, 40, 110, 110, 0xFFFF);
}

void CAR::LCD::showReady(uint32_t duration) {
    duration /= 1000u;
    showBatteryVoltage();
    st7735.setTextColor(0x07E0, 0x0000);
    st7735.setCursor(4, 40);
    st7735.print("System");
    st7735.setCursor(4, 60);
    st7735.print("ready       ");
    st7735.setTextColor(0xFFFF, 0x0000);
    st7735.setCursor(4, 90);
    st7735.print("Time:");
    st7735.setCursor(4, 110);
    st7735.printf("%02u:%02u:%03u", duration / 60000u, (duration / 1000u) % 60u, duration % 1000u);
    esp_reset_reason_t resetReason = esp_reset_reason();
    if((resetReason != ESP_RST_POWERON) && (resetReason != ESP_RST_SW)) {
        st7735.setCursor(4, 130);
        st7735.printf("%d %d %d %d", esp_reset_reason(), esp_rom_get_reset_reason(0), esp_rom_get_reset_reason(1), xPortGetCoreID());
    }
}

void CAR::LCD::showRotation() {
    showBatteryVoltage();
    st7735.fillRect(3, 24, 110, 100, 0x0000);
    st7735.setTextColor(0xFFFF, 0x0000);
    MPU9250::FLOAT3 rotation = mpu9250.getRotation();
    for(uint8_t i = 0; i < 3; i++) {
        st7735.setCursor(4, 40 + (i * 20));
        st7735.write('X' + i);
        st7735.print(": ");
        st7735.print(rotation.data[i]);
    }
}

void CAR::LCD::showString(String str) {
    showBatteryVoltage();
    st7735.fillRect(0, 22, 128, 138, 0x0000);
    st7735.setTextColor(0xFFFF);
    st7735.setCursor(4, 40);
    st7735.print(str);
}

void CAR::LCD::showSupervision() {
    showBatteryVoltage();
    st7735.fillRect(3, 24, 100, 100, 0x0000);
    for(uint8_t i = 0; i < 3; i++) {
        float data = 0.0f;
        if(i == 0) {
            data = supervision.getTotalCurrent();
            st7735.setTextColor(data > 4.0f ? 0xF800 : (data > 3.0f ? 0xFFE0 : 0x07E0));
            st7735.fillRect(4, 27, 12, 8, 0xFFFF);
            st7735.fillRect(16, 29, 2, 4, 0xFFFF);
        }
        if(i == 1) {
            data = supervision.getMotorCurrent();
            st7735.setTextColor(0xFFFF, 0x0000);
            st7735.setTextSize(1);
            st7735.drawCircle(11, 51, 7, 0xFFFF);
            st7735.setCursor(9, 48);
            st7735.write('M');
            st7735.setTextSize(2);
            st7735.setTextColor(data > 4.0f ? 0xF800 : (data > 3.0f ? 0xFFE0 : 0x07E0));
        }
        if(i == 2) {
            data = supervision.getMosfetTemp();
            st7735.setTextColor(data > 50.0f ? 0xF800 : (data > 40.0f ? 0xFFE0 : 0x07E0));
            st7735.drawLine(4, 71, 6, 71, 0xFFFF);
            st7735.drawLine(7, 67, 7, 75, 0xFFFF);
            st7735.drawLine(10, 67, 10, 75, 0xFFFF);
            st7735.drawLine(10, 67, 18, 67, 0xFFFF);
            st7735.drawLine(10, 75, 18, 75, 0xFFFF);
            st7735.drawLine(18, 64, 18, 67, 0xFFFF);
            st7735.drawLine(18, 75, 18, 78, 0xFFFF);
        }
        st7735.setCursor(26, (i * 20) + 24);
        st7735.print(data, 2);
        st7735.print(i < 2 ? " A" : " C");
    }
}

CAR::LCD CAR::lcd;

void CAR::LEDS::setBrightness(uint8_t left, uint8_t right) {
    ledState = (_min(left, 15) << 4) | _min(right, 15);
}

CAR::LEDS CAR::leds;

CAR::MOTOR::MOTOR() {
    pinMode(Pin_Motor_PWM, OUTPUT);
    pinMode(Pin_Motor_Direction_Invert, OUTPUT);
    digitalWrite(Pin_Motor_PWM, LOW);
    digitalWrite(Pin_Motor_Direction_Invert, LOW);
    ledc_channel_config_t ledcChannelConfig = {
		.gpio_num       = Pin_Motor_PWM,
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
		.intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER_0,
        .duty           = 0,
        .hpoint         = 0
    };
    SNIPE(ledc_channel_config(&ledcChannelConfig));
	ledc_timer_config_t ledcTimerConfig = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
		.duty_resolution  = LEDC_TIMER_12_BIT,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = Motor_PWM_Max_Frequency,
        .clk_cfg          = LEDC_AUTO_CLK
    };
	SNIPE(ledc_timer_config(&ledcTimerConfig));
    SNIPE(xTaskCreatePinnedToCore(motor_control_task, "Motor Acceleration", 5000, 0, 2, &motorAccelerationThread, 1 - xPortGetCoreID()));
}

void CAR::MOTOR::setSpeed(float speed) {
    motorTargetDuty = constrain(speed, -1.0, 1.0) * (1 << LEDC_TIMER_12_BIT);
}

CAR::MOTOR CAR::motor;

void CAR::MPU9250::init(bool rotX, bool rotY, bool rotZ) {
    pinMode(Pin_MPU9250_Data_Ready, INPUT);
    Wire.begin(Pin_I2C_0_SDA, Pin_I2C_0_SCL, 400000);
    int_param_s int_param = {};
    mpu_init(&int_param);
    mpu_set_bypass(1);
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
	mpu_set_int_level(0);
	mpu_set_int_latched(0);
    set_int_enable(1);
    mpu_set_lpf(5);
    mpu_set_gyro_fsr(2000);
    mpu_set_sample_rate(1000);
	mpu_reset_fifo();
    dmp_load_motion_driver_firmware();
    dmp_enable_6x_lp_quat(1);
    dmp_enable_feature(DMP_FEATURE_TAP | DMP_FEATURE_6X_LP_QUAT);
    dmp_set_fifo_rate(60);
    mpu_set_dmp_state(1);
    SNIPE(xTaskCreatePinnedToCore(i2c0_task, "I2C0 Task", 10000, 0, 2, &i2cThread0, 1 - xPortGetCoreID()));
    vTaskDelay(1000);
    FLOAT3 startRotation = getRotation();
    vTaskDelay(4000);
    FLOAT3 endRotation = getRotation();
    for(uint8_t i = 0; i < 3; i++) _rotationOffset[i] = (startRotation.data[i] - endRotation.data[i]) / 4000000;
}

CAR::MPU9250::FLOAT3 CAR::MPU9250::getAcceleration() {
    int16_t raw[3] = {};
    mpu_get_accel_reg(raw, 0);
    uint16_t accelDivisor = 0;
    mpu_get_accel_sens(&accelDivisor);
    FLOAT3 result = {};
    for(uint8_t i = 0; i < 3; i++) result.data[i] = raw[0] / (float)accelDivisor;
    return result;
}

CAR::MPU9250::FLOAT3 CAR::MPU9250::getRotation() {
    int64_t time = esp_timer_get_time();
    return {
        (rounds[0] * 360.0f) + lastRotation[0] + (float)(_rotationOffset[0] * time),
        (rounds[1] * 360.0f) + lastRotation[1] + (float)(_rotationOffset[1] * time),
        (rounds[2] * 360.0f) + lastRotation[2] + (float)(_rotationOffset[2] * time)
    };
}

CAR::MPU9250::FLOAT3 CAR::MPU9250::getRotationChange() {
    int16_t raw[3] = {};
    mpu_get_gyro_reg(raw, 0);
    float gyroDivisor = 0;
    mpu_get_gyro_sens(&gyroDivisor);
    FLOAT3 result = {};
    for(uint8_t i = 0; i < 3; i++) result.data[i] = raw[0] / gyroDivisor;
    return result;
}

CAR::MPU9250 CAR::mpu9250;

CAR::SERVO::SERVO() {
    pinMode(Pin_Servo_PWM, OUTPUT);
    digitalWrite(Pin_Servo_PWM, LOW);
    ledc_channel_config_t ledcChannelConfig = {
		.gpio_num       = Pin_Servo_PWM,
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_1,
		.intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER_1,
        .duty           = Servo_PWM_0_Duty,
        .hpoint         = 0
    };
    SNIPE(ledc_channel_config(&ledcChannelConfig));
	ledc_timer_config_t ledcTimerConfig = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
		.duty_resolution  = LEDC_TIMER_14_BIT,
        .timer_num        = LEDC_TIMER_1,
        .freq_hz          = 333,
        .clk_cfg          = LEDC_AUTO_CLK
    };
	SNIPE(ledc_timer_config(&ledcTimerConfig));
    SNIPE(xTaskCreatePinnedToCore(servo_control_task, "Servo Acceleration", 5000, 0, 2, &servoAccelerationThread, 1 - xPortGetCoreID()));
}

void CAR::SERVO::setAngle(float angle) {
    servoTargetDuty = Servo_PWM_0_Duty + (constrain(angle, -80.0, 80.0) * 70);
}

CAR::SERVO CAR::servo;

CAR::VCT_SUPERVISION::VCT_SUPERVISION() {
    pinMode(Pin_Battery_Voltage, INPUT);
    pinMode(Pin_Total_Current, INPUT);
    pinMode(Pin_Motor_Current, INPUT);
    ds18b20.begin();
    ds18b20.getAddress(ds18b20Address, 0);
}

float CAR::VCT_SUPERVISION::getCoreTemp() {
    return temperatureRead();
}

float CAR::VCT_SUPERVISION::getMosfetTemp() {
    ds18b20.requestTemperaturesByAddress(ds18b20Address);
    return ds18b20.getTempC(ds18b20Address);
}

float CAR::VCT_SUPERVISION::getMotorCurrent() {
    float result = analogReadMilliVolts(Pin_Motor_Current) * 0.001f;
    return result <= 0.15f ? 0.0f : result;
}

float CAR::VCT_SUPERVISION::getTotalCurrent() {
    float result = analogReadMilliVolts(Pin_Total_Current) * 0.002f;
    return result <= 0.3f ? 0.0f : result;
}

float CAR::VCT_SUPERVISION::getVoltage() {
    return analogReadMilliVolts(Pin_Battery_Voltage) * 0.0031273f;
}

CAR::VCT_SUPERVISION CAR::supervision;

void CAR::VL53L1X::init() {
    Wire1.begin(Pin_I2C_1_SDA, Pin_I2C_1_SCL, 400000);
    for(uint8_t i = 0; i < 4; i++) {
        pinMode(Pins_VL53L1X_Shutdown[i], OUTPUT);
        digitalWrite(Pins_VL53L1X_Shutdown[i], LOW);
    }
    delay(15);
    for(uint8_t i = 0; i < 4; i++) {
        digitalWrite(Pins_VL53L1X_Shutdown[i], HIGH);
        delay(15);
        vl53l1xWriteByte(0x29, 0x0001, 0x30 + i);
        for(uint8_t j = 0; j < 91; j++) vl53l1xWriteByte(0x30 + i, 0x002D + j, vl53l1xConfiguration[j]);
        vl53l1xWriteByte(0x30 + i, 0x0086, 0x01);
        vl53l1xWriteByte(0x30 + i, 0x0087, 0x40);
        do vTaskDelay(1);
        while((vl53l1xReadByte(0x30 + i, 0x0031) & 1) == ((vl53l1xReadByte(0x30 + i, 0x0030) >> 4) & 1));
        vl53l1xWriteByte(0x30 + i, 0x0086, 0x01);
        vl53l1xWriteByte(0x30 + i, 0x0087, 0x00);
        vl53l1xWriteByte(0x30 + i, 0x0008, 0x09);
        vl53l1xWriteByte(0x30 + i, 0x000B, 0x00);
        vl53l1xWriteByte(0x30 + i, 0x004B, 0x0A);
		vl53l1xWriteByte(0x30 + i, 0x0060, 0x0F);
		vl53l1xWriteByte(0x30 + i, 0x0063, 0x0D);
		vl53l1xWriteByte(0x30 + i, 0x0069, 0xB8);
		vl53l1xWriteWord(0x30 + i, 0x0078, 0x0F0D);
		vl53l1xWriteWord(0x30 + i, 0x007A, 0x0E0E);
        vl53l1xWriteWord(0x30 + i, 0x005E, 0x01AE);
		vl53l1xWriteWord(0x30 + i, 0x0061, 0x01E8);
        vl53l1xWriteDWord(0x30 + i, 0x006C, (uint32_t)((vl53l1xReadWord(0x30 + i, 0x00DE) & 0x03FF) * 64.5f));
        vl53l1xWriteByte(0x30 + i, 0x007F, vl53l1xViewCenters[i]);
	    vl53l1xWriteByte(0x30 + i, 0x0080, 0x55); // ((Y - 1) << 4) | (X - 1)
        lastVl53l1xViewCenters[i] = vl53l1xViewCenters[i];
        delay(15);
    }
    SNIPE(xTaskCreatePinnedToCore(i2c1_task, "I2C1 Task", 10000, 0, 2, &i2cThread1, 1 - xPortGetCoreID()));
}

uint16_t CAR::VL53L1X::getDistance(uint8_t pos) {
    return distances[pos];
}

CAR::VL53L1X CAR::vl53l1x;