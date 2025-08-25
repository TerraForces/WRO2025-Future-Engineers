/**
 * car.cpp - the (hidden) functional core of our autonomously driving car
 * by TerraForce
 */

// optimize program for higher execution speed
#pragma GCC optimize("-Ofast")

// include header file
#include "car.hpp"

// include library for the st7735 lcd controller with modified SPI
#include <Adafruit_ST7735.h>

// include ledc channel control driver
#include <driver/ledc.h>

// include uart bus driver
#include <driver/uart.h>

// include bluetooth and wifi drivers for deactivation functions
#include <esp_bt.h>
#include <esp_wifi.h>

// include c library for the MPU9250 gyro and acceleration sensor
extern "C" {
    #include <inv_mpu.h>
    #include <inv_mpu_dmp_motion_driver.h>
}

// include modified OneWire library for communication with the ds18b20 temperature sensor
#include <OneWire.hpp>

// include for vector support
#include <vector>

// undefine MPU9250 macro to use it as class name
#undef MPU9250

// macro for error handling (currently only optional error logging)
#define SNIPE(condition) if(condition) { log_e("snipe"); }

// pin configuration
constexpr uint8_t Pin_ST7735_LED             = 0;  // output - lcd background led (PWM or digital signal)
constexpr uint8_t Pin_MPU9250_Data_Ready     = 2;  // input  - interrupt when mpu9250 fifo receives new data
constexpr uint8_t Pin_ST7735_DC              = 4;  // output - lcd data/command
constexpr uint8_t Pin_ST7735_RST             = 5;  // output - lcd reset
constexpr uint8_t Pin_ST7735_CS              = 12; // output - lcd SPI chip select
constexpr uint8_t Pin_ST7735_MOSI            = 13; // output - lcd SPI data out
constexpr uint8_t Pin_ST7735_SCK             = 14; // output - lcd SPI serial clock
constexpr uint8_t Pin_Start_Button_LED       = 15; // output - start button led
constexpr uint8_t Pins_VL53L1X_Shutdown[4]   = { 16, 17, 18, 19 }; // output - vl53l1x shutdown pins (shutdown = low, active = high)
constexpr uint8_t Pin_I2C_0_SDA			     = 21; // in/out - i2c bus 0 data (bidirectional)
constexpr uint8_t Pin_I2C_0_SCL			     = 22; // output - i2c bus 0 serial clock
constexpr uint8_t Pin_Servo_PWM				 = 23; // output - servo pwm signal
constexpr uint8_t Pin_Motor_Direction_Invert = 25; // output - motor direction invert (reverse when high)
constexpr uint8_t Pin_Motor_PWM				 = 26; // output - motor pwm signal
constexpr uint8_t Pin_DS18B20_OneWire        = 27; // in/out - OneWire bus for communication with the ds18b20 temperature sensor
constexpr uint8_t Pin_I2C_1_SDA			     = 32; // in/out - i2c bus 1 data (bidirectional)
constexpr uint8_t Pin_I2C_1_SCL			     = 33; // output - i2c bus 1 serial clock
constexpr uint8_t Pin_Battery_Voltage		 = 34; // input  - battery voltage after voltage divider (10V battery voltage = 3.3V voltage at pin)
constexpr uint8_t Pin_Total_Current		     = 35; // input  - total current measured as voltage over 0.25 ohms resistor and multiplied by 20
constexpr uint8_t Pin_Motor_Current		     = 36; // input  - total current measured as voltage over 0.33 ohms resistor and multiplied by 20
constexpr uint8_t Pin_Start_Button           = 39; // input  - start button (externally pulled up)

// motor pwm configuration
constexpr uint16_t Motor_PWM_Min_Frequency   = 8000;  // minimal pwm frequency used for motor pwm (hz)
constexpr uint16_t Motor_PWM_Max_Frequency   = 12000; // maximum pwm frequency used for motor pwm (hz)
constexpr float Motor_PWM_Time_To_Max        = 1.60f; // time in seconds for the motor pwm to reach maximum speed from idle
constexpr float Motor_PWM_Time_To_Null       = 0.50f; // time in seconds for the motor pwm to reach idle from maximum speed
constexpr float Motor_PWM_Update_Interval    = 0.04f; // motor pwm update interval in seconds

// servo pwm configuration
constexpr uint16_t Servo_PWM_0_Duty          = 7940;  // servo pwm duty in 0° position (might be adjusted after hardware changes)
constexpr float Servo_PWM_Max_Angle          = 80.0f; // max angle in both directions

// lcd configuration
constexpr uint16_t LCD_LED_PWM_Frequency     = 1000;  // frequency of lcd backlight pwm (hz)
constexpr float Battery_Voltage_Warning      = 7.0f;  // battery voltage (V) less than this value is shown yellow on lcd
constexpr float Battery_Voltage_Empty        = 6.6f;  // battery voltage (V) less than this value is shown red on lcd

// handles of background tasks
TaskHandle_t motorAccelerationThread, i2cThread0, i2cThread1;

// data transport between background tasks and main
volatile float lastRotation[3] = {};        // last rotation calculated from sensor data (always between 0° and 360°)
volatile double _rotationOffset[3] = {};    // values used to correct slow rotation drift
volatile int16_t rounds[3] = {};            // full rotation rounds performed (rounds + lastRotation = rotation relative to starting position)
volatile uint16_t distances[4] = {};        // last distance data received from vl53l1x sensors (in mm)
volatile uint8_t ledState = 0x00;           // led brightness to be sent to IO extender
volatile int32_t motorTargetDuty = 0;       // motor target duty for acceleration ramp
volatile uint64_t taskFlowFailures[3] = {}; // number of skipped task delays (indicator for critical core usage)

// global working variables
SPIClass hspi(HSPI);                                                         // hardware SPI bus object modified to work without MISO
Adafruit_ST7735 st7735(&hspi, Pin_ST7735_CS, Pin_ST7735_DC, Pin_ST7735_RST); // lcd display object with communication using hardware SPI
OneWire oneWire = OneWire(Pin_DS18B20_OneWire);                              // OneWire bus object for communication with the DS18B20 temperature sensor
uint8_t ds18b20Address[8] = {};                                              // 64-bit DS18B20 temperature sensor address
std::vector<CAR::OBJECT> currentDetections,                                  // UART receive data buffer (list of detected objects)
    detections = { CAR::OBJECT {0, 0, 0, 0, 0, 0 } };                        // last complete object data transmission (empty object until first data received from RPi 5)
uart_t* uart = 0;                                                            // UART bus handle (used to control UART bus)
constexpr uint16_t objectColors[4] = { 0x0000, 0xF800, 0x0600, 0xF81F };     // object colors on display (purely cosmetical)
volatile uint8_t vl53l1xViewCenters[4] = { 198, 199, 63, 61 };               // index of center sensor of the active sensoring area of the VL53L1X distance sensors
constexpr uint8_t vl53l1xConfiguration[91] = {                               // VL53L1X distance sensor default register configuration
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

// read 8-bit register from VL53L1X ToF sensor
uint8_t vl53l1xReadByte(uint8_t add, uint16_t reg) {
    for(uint8_t i = 0; i < 5; i++) {                 // try sending register address up to 5 times
        Wire1.beginTransmission(add);                // start I2C transmission to sensor address
        Wire1.write(reg >> 8);                       // write first byte of register address
        Wire1.write(reg & 0xFF);                     // write second byte of register address
        if(Wire1.endTransmission(false) == 0) break; // end I2C transmission without sending stop command
    }
    Wire1.requestFrom(add, (uint8_t)1);              // request one byte from VL53L1X sensor
    return Wire1.read();                             // read data byte from receive buffer
}

// read 16-bit register from VL53L1X ToF sensor
uint16_t vl53l1xReadWord(uint8_t add, uint16_t reg) {
    for(uint8_t i = 0; i < 5; i++) {                 // try sending register address up to 5 times
        Wire1.beginTransmission(add);                // start I2C transmission to sensor address
        Wire1.write(reg >> 8);                       // write first byte of register address
        Wire1.write(reg & 0xFF);                     // write second byte of register address
        if(Wire1.endTransmission(false) == 0) break; // end I2C transmission without sending stop command
    }
    Wire1.requestFrom(add, (uint8_t)2);              // request two bytes from VL53L1X sensor
    uint16_t firstByte = Wire1.read();               // read first byte from receive buffer
    return (firstByte << 8) | (uint8_t)Wire1.read(); // read second byte from receive buffer and combine to 16-bit value
}

// write to 8-bit register of VL53L1X sensor
void vl53l1xWriteByte(uint8_t add, uint16_t reg, uint8_t data) {
    Wire1.beginTransmission(add); // start I2C transmission to sensor address
    Wire1.write(reg >> 8);        // write first byte of register address
    Wire1.write(reg & 0xFF);      // write second byte of register address
    Wire1.write(data);            // write data to the register
    Wire1.endTransmission(true);  // end transmission and send stop bit
}

// write to 16-bit register of VL53L1X sensor
void vl53l1xWriteWord(uint8_t add, uint16_t reg, uint16_t data) {
    Wire1.beginTransmission(add); // start I2C transmission to sensor address
    Wire1.write(reg >> 8);        // write first byte of register address
    Wire1.write(reg & 0xFF);      // write second byte of register address
    Wire1.write(data >> 8);       // write first byte of data to the register
    Wire1.write(data & 0xFF);     // write second byte of data to the register
    Wire1.endTransmission(true);  // end transmission and send stop bit
}

// write to 32-bit register of VL53L1X sensor
void vl53l1xWriteDWord(uint8_t add, uint16_t reg, uint32_t data) {
    Wire1.beginTransmission(add);     // start I2C transmission to sensor address
    Wire1.write(reg >> 8);            // write first byte of register address
    Wire1.write(reg & 0xFF);          // write second byte of register address
    Wire1.write(data >> 24);          // write first byte of data to the register
    Wire1.write((data >> 16) & 0xFF); // write second byte of data to the register
    Wire1.write((data >> 8) & 0xFF);  // write third byte of data to the register
    Wire1.write(data & 0xFF);         // write fourth byte of data to the register
    Wire1.endTransmission(true);      // end transmission and send stop bit
}

// interrupt function activated when state of Pin_MPU9250_Data_Ready changes from HIGH to LOW
void mpu9250_interrupt() {
    BaseType_t higherPriorityTaskWoken = 0;
    // send notification to i2cThread0 running i2c0_task
	xTaskNotifyFromISR(i2cThread0, 0, eNoAction, &higherPriorityTaskWoken);
}

// convert quaternion value to float for further calculations
inline float qToFloat(long value) {
    return (value >> 30) + ((value & 0x3FFFFFFF) / (float)0x40000000);
}

// task which reads MPU-9250 sensor data when receiving a data ready interrupt
void i2c0_task(void* unused) {

    // attach interrupt function which sends notifications to this task when pin state changes from HIGH to LOW
    attachInterrupt(Pin_MPU9250_Data_Ready, mpu9250_interrupt, FALLING);
    while(true) {

        // wait for notification from interrupt function
		if(xTaskNotifyWait(0, 0, 0, 20) == pdPASS) {

            // read data from MPU-9250 digital motion processor FiFo buffer
            short gyro[3], accel[3], sensors;
            long quat[4], raw[4];
            unsigned long timestamp;
            unsigned char more;
            dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more);
            memcpy(raw, quat, 16);

            // try to read newer data if available (required to always receive latest data)
            if (!dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more)) memcpy(raw, quat, 16);

            // convert quaternion rotation data to euler angles (0° - 360°)
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

            // count rounds for each rotation dimension by simply in-/decreasing the rounds when it jumps between 0°/360°
            for(uint8_t i = 0; i < 3; i++) {
                if(degrees[i] < 0.0f) degrees[i] += 360.0f;
                if((degrees[i] < 60.0f) && (lastRotation[i] > 300.0f)) rounds[i]++;
                if((degrees[i] > 300.0f) && (lastRotation[i] < 60.0f)) rounds[i]--;
                lastRotation[i] = degrees[i];
            }
        }

        // count when no notifications arrive during 20 ms (should not happen normally)
        else taskFlowFailures[0]++;
	}
}

// task which periodically reads distance data and changes LED state
void i2c1_task(void* unused) {

    // start I2C transmission to address 0x20 (MCP23008-E/P)
    Wire1.beginTransmission(0x20);

    // write 0x00 to register 0x00 (configures all pins to be outputs)
    Wire1.write(0x00);
    Wire1.write(0x00);

    // end I2C transmission
    Wire1.endTransmission(true);

    // start periodic measuring on all of the 4 sensors
    uint8_t lastVl53l1xViewCenters[4] = {};
    for(uint8_t i = 0; i < 4; i++) {
        lastVl53l1xViewCenters[i] = vl53l1xViewCenters[i];
        vl53l1xWriteByte(0x30 + i, 0x0086, 0x01);
        vl53l1xWriteByte(0x30 + i, 0x0087, 0x40);
    }

    TickType_t lastWaitTime = xTaskGetTickCount();
    uint8_t lastLedState = 0;
    while(true) {

        // try to execute every 6 ms and increase fail counter when loop takes more than 6 ms or other tasks steal computing time (should not happen, only for debugging)
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

        // read distance data and set new view centers for all 4 sensors
        for(uint8_t i = 0; i < 4; i++) {

            // read distance data if new value is available
            if((vl53l1xReadByte(0x30 + i, 0x0031) & 1) == (!((vl53l1xReadByte(0x30 + i, 0x0030) >> 4) & 1))) {
                vl53l1xWriteByte(0x30 + i, 0x0086, 0x01);
                distances[i] = vl53l1xReadWord(0x30 + i, 0x0096);
            }

            // set new viewport centers if asked for
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

// acceleration control task (motor PWM regulation)
void motor_control_task(void* unused) {
	TickType_t lastWakeTime = xTaskGetTickCount();
    constexpr TickType_t updateInterval = (1000 * Motor_PWM_Update_Interval) + 0.5;
	int32_t currentMotorDuty = 0;
	constexpr int32_t motorMaxDuty = 1 << LEDC_TIMER_12_BIT;
	constexpr int32_t motorAccelerationRate = (motorMaxDuty * Motor_PWM_Update_Interval / Motor_PWM_Time_To_Max) + 0.5;
	constexpr int32_t motorDeaccelerationRate = (motorMaxDuty * Motor_PWM_Update_Interval / Motor_PWM_Time_To_Null) + 0.5;
	while(true) {
		
        // wait for next execution or increase failure count if execution takes too long (indicates overloaded CPU core)
        if(!xTaskDelayUntil(&lastWakeTime, updateInterval)) taskFlowFailures[2]++; 
        
        // copy motor target speed value
        int32_t motorTargetDuty = ::motorTargetDuty;

        // add next (de-)acceleration step
        if     ((currentMotorDuty >= 0) && (motorTargetDuty > currentMotorDuty)) currentMotorDuty = _min(motorTargetDuty, currentMotorDuty + motorAccelerationRate);
        else if((currentMotorDuty > 0) && (motorTargetDuty < currentMotorDuty)) currentMotorDuty = _max(_max(motorTargetDuty, 0), currentMotorDuty - motorDeaccelerationRate);
        else if((currentMotorDuty < 0) && (motorTargetDuty > currentMotorDuty)) currentMotorDuty = _min(_min(motorTargetDuty, 0), currentMotorDuty + motorDeaccelerationRate);
        else if((currentMotorDuty <= 0) && (motorTargetDuty < currentMotorDuty)) currentMotorDuty = _max(motorTargetDuty, currentMotorDuty - motorAccelerationRate);
		
        // update PWM frequency and duty cycle and set relais state
        SNIPE(ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, (uint32_t)((Motor_PWM_Max_Frequency - Motor_PWM_Min_Frequency) * (abs(currentMotorDuty) / (float)motorMaxDuty)) + Motor_PWM_Min_Frequency));
		SNIPE(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, abs(currentMotorDuty)));
		SNIPE(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
		digitalWrite(Pin_Motor_Direction_Invert, currentMotorDuty < 0);
	}
}

// task with seconds highest priority possible; receives data from the Raspberry Pi 5 over UART
void uart_task(void* unused) {
    uart_event_t event;
    QueueHandle_t uartEventQueue = 0;
    uartGetEventQueue(uart, &uartEventQueue);
    while(true) {
        
        // wait for new UART bus event
        if(xQueueReceive(uartEventQueue, &event, portMAX_DELAY)) {

            // only do something when full object structure available
            if(event.type == UART_DATA) {
                while(uartAvailable(uart) >= sizeof(CAR::OBJECT)) {
                    CAR::OBJECT object = {};

                    // read new object data from UART
                    uartReadBytes(uart, (uint8_t*)&object, sizeof(CAR::OBJECT), 0);

                    // add object to receive buffer
                    if(object.c) currentDetections.push_back(object);

                    // or copy received objects and clear receive buffer when receiving an empty object
                    else {
                        detections = currentDetections;
                        currentDetections.clear();
                    }
                }
            }
        }
    }
}

// sets button pin modes and deactivate LED
CAR::BUTTON::BUTTON() {
    pinMode(Pin_Start_Button, INPUT);
    pinMode(Pin_Start_Button_LED, OUTPUT);
    digitalWrite(Pin_Start_Button_LED, LOW);
}

// button is pressed when pin reads LOW (external pullup)
bool CAR::BUTTON::pressed() {
    return !digitalRead(Pin_Start_Button);
}

// set LED state by writing to output pin
void CAR::BUTTON::setLED(bool state) {
    digitalWrite(Pin_Start_Button_LED, state);
}

// activate LED, wait for button press and deactivate LED again
void CAR::BUTTON::wait() {
    digitalWrite(Pin_Start_Button_LED, HIGH);
    while(digitalRead(Pin_Start_Button)) vTaskDelay(1);
    digitalWrite(Pin_Start_Button_LED, LOW);
}

CAR::BUTTON CAR::button;

// initialize UART bus at 115200 bit per second and create UART task
CAR::CAMERA::CAMERA() {
    SemaphoreHandle_t uartMutexLock = xSemaphoreCreateMutex();
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

// returns object at the specified index from the list or an empty object if none exists
CAR::OBJECT CAR::CAMERA::getObject(uint16_t idx) {
    if(idx >= detections.size()) return {};
    return detections[idx];
}

// returns true when the Raspberry Pi 5 is ready (has deleted the empty "dummy" object from the beginning)
bool CAR::CAMERA::ready() {
    if(detections.size() == 1) {
        if(detections[0].c == 0) return false;
    }
    return true;
}

CAR::CAMERA CAR::camera;

CAR::LCD::LCD() {

    // set LCD background LED pin mode
    pinMode(Pin_ST7735_LED, OUTPUT);
    digitalWrite(Pin_ST7735_LED, LOW);

    // enable brightness control using the builtin LEDC controller (usually at maximum brightness = constant HIGH without PWM signal)
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

    // init LCD display and print starting message
    st7735.initR(INITR_BLACKTAB);
    st7735.fillScreen(0x0000);
    st7735.setTextWrap(false);
    st7735.setTextSize(2);
    showBatteryVoltage();
    st7735.setTextColor(0xFFFF);
    st7735.setCursor(4, 40);
    st7735.print("System");
    st7735.setCursor(4, 60);
    st7735.print("starting");

    // deactivate any WiFi or Bluetooth components (shouldn't be enabled anyways)
    esp_wifi_deinit();
    esp_bt_controller_deinit();
}

// set LCD brightness by adjusting LEDC duty cycle
void CAR::LCD::setBrightness(float brightness) {
    SNIPE(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, constrain(brightness, 0.0f, 1.0f) * 0x3FFF));
	SNIPE(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2));
}

// clear screen, draw battery icon and show colored voltage
void CAR::LCD::showBatteryVoltage() {
    st7735.fillScreen(0x0000);
    float voltage = supervision.getVoltage();
    st7735.fillRect(4, 4, 32, 14, 0xFFFF);
    st7735.fillRect(36, 7, 4, 8, 0xFFFF);
    st7735.setTextColor(voltage < Battery_Voltage_Empty ? 0xF800 : (voltage < Battery_Voltage_Warning ? 0xFFE0 : 0x07E0), 0x0000);
    st7735.setCursor(50, 4);
    st7735.print(voltage, 2);
    st7735.print(" V ");
}

void CAR::LCD::showDistanceCalibration(uint8_t i) {
    showBatteryVoltage();
    st7735.setTextColor(0xFFFF);

    // show which distance sensor needs to be placed 2m from the border
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

    // wait for button press
    vTaskDelay(300);
    button.wait();

    // show waiting screen
    showBatteryVoltage();
    st7735.setTextColor(0xFFFF, 0x0000);
    st7735.setCursor(4, 50);
    st7735.print("Testing");
    st7735.setCursor(4, 70);
    st7735.print("Viewport");
    st7735.setCursor(4, 90);
    st7735.print("Centers ");

    // store viewport center to be restored later
    uint8_t lastVl53l1xViewCenter = vl53l1xViewCenters[i];

    // measure average distances for all possible viewport centers
    int64_t testingResults[11] = {};
    TickType_t lastDelay = xTaskGetTickCount();
    for(uint16_t j = 0; j < 11; j++) {
        vl53l1xViewCenters[i] = (j < 5) ? (59 + j) : (204 - j);
        st7735.setCursor(4, 110);
        st7735.printf("%hus left ", (11 - j) * 2);
        xTaskDelayUntil(&lastDelay, 200);
        for(uint16_t k = 0; k < 360; k++) {
            testingResults[j] += distances[i];
            xTaskDelayUntil(&lastDelay, 5);
        }
        testingResults[j] /= 4;
    }

    // restore viewport center
    vl53l1xViewCenters[i] = lastVl53l1xViewCenter;

    // draw results chart
    showBatteryVoltage();
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

// show latest distance measurements of every sensor
void CAR::LCD::showDistances() {
    showBatteryVoltage();
    st7735.setTextColor(0xFFFF);
    for(uint8_t i = 0; i < 4; i++) {

        // draw arrow to indicate sensor position
        st7735.drawLine(i < 2 ? 4 : 11, (i < 2 ? 31 : 24) + (i * 20), i < 2 ? 18 : 11, (i < 2 ? 31 : 38) + (i * 20), 0xFFFF);
        st7735.drawLine(i % 2 ? 18 : 4, 31 + (i * 20), 11, (i % 2 ? 38 : 24) + (i * 20), 0xFFFF);
        st7735.drawLine((i < 1 || i > 2) ? 4 : 18, 31 + (i * 20), 11, ((i < 1 || i > 2) ? 38 : 24) + (i * 20), 0xFFFF);

        // print measurement
        st7735.setCursor(26, (i * 20) + 24);
        st7735.print(vl53l1x.getDistance(i));
        st7735.print(" mm  ");
    }
}

// show finish screen including run time and task flow problems if existing
void CAR::LCD::showFinish(uint32_t duration) {
    duration /= 1000u;
    showBatteryVoltage();
    st7735.setTextColor(0x07E0, 0x0000);
    st7735.setCursor(4, 40);
    st7735.print("Completed");
    st7735.setTextColor(0xFFFF, 0x0000);
    st7735.setCursor(4, 70);
    st7735.print("Time:");
    st7735.setCursor(4, 90);
    st7735.printf("%02u:%02u:%03u", duration / 60000u, (duration / 1000u) % 60u, duration % 1000u);
    if(*std::min_element(taskFlowFailures, taskFlowFailures + 2) > 0) {
        st7735.setTextColor(0xF800, 0x0000);
        st7735.setCursor(4, 110);
        st7735.printf("%lu %lu", taskFlowFailures[0], taskFlowFailures[1]);
        st7735.setCursor(4, 130);
        st7735.printf("%lu", taskFlowFailures[2]);
    }
}

// show all detected objects on the screen (like the camera would see them)
void CAR::LCD::showObjects() {
    std::vector<CAR::OBJECT> objectBuf = detections;
    showBatteryVoltage();
    for(uint16_t i = 1; i <= objectBuf.size(); i++) {
        uint16_t j = objectBuf.size() - i;
        st7735.fillRect(10 + (objectBuf[j].l / 12u), 32 + (objectBuf[j].t / 12u), objectBuf[j].w / 12u, objectBuf[j].h / 12u, objectColors[objectBuf[j].c]);
    }
    st7735.drawRect(9, 40, 110, 110, 0xFFFF);
}

// show ready screen including required start time and reset reasons if unusual (should not happen, only for debug)
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

// shows latest rotation data
void CAR::LCD::showRotation() {
    showBatteryVoltage();
    st7735.setTextColor(0xFFFF, 0x0000);
    MPU9250::FLOAT3 rotation = mpu9250.getRotation();
    for(uint8_t i = 0; i < 3; i++) {
        st7735.setCursor(4, 40 + (i * 20));
        st7735.write('X' + i);
        st7735.print(": ");
        st7735.print(rotation.data[i]);
    }
}

// shows string str (only for debug)
void CAR::LCD::showString(String str) {
    showBatteryVoltage();
    st7735.setTextColor(0xFFFF);
    st7735.setCursor(4, 40);
    st7735.print(str);
}

// shows total current, motor current and MOSFET temperature
void CAR::LCD::showSupervision() {
    showBatteryVoltage();
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

// set brightness for left and right LED groups
void CAR::LEDS::setBrightness(uint8_t left, uint8_t right) {
    ledState = (_min(left, 15) << 4) | _min(right, 15);
}

CAR::LEDS CAR::leds;

CAR::MOTOR::MOTOR() {

    // set motor pin states
    pinMode(Pin_Motor_PWM, OUTPUT);
    pinMode(Pin_Motor_Direction_Invert, OUTPUT);
    digitalWrite(Pin_Motor_PWM, LOW);
    digitalWrite(Pin_Motor_Direction_Invert, LOW);

    // init LEDC controller for motor PWM control
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

    // create background task for acceleration control
    SNIPE(xTaskCreatePinnedToCore(motor_control_task, "acceleration control", 5000, 0, 2, &motorAccelerationThread, 1 - xPortGetCoreID()));
}

// sets new motor target speed
void CAR::MOTOR::setSpeed(float speed) {
    motorTargetDuty = constrain(speed, -1.0, 1.0) * (1 << LEDC_TIMER_12_BIT);
}

CAR::MOTOR CAR::motor;

// starts MPU-9250 gyro sensor and performs fast calibration
void CAR::MPU9250::init() {

    // set pin mode for interrupt pin
    pinMode(Pin_MPU9250_Data_Ready, INPUT);

    // init I2C bus 0 @ 400 kHz
    Wire.begin(Pin_I2C_0_SDA, Pin_I2C_0_SCL, 400000);

    // init MPU-9250 sensor
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

    // create task to receive rotation data
    SNIPE(xTaskCreatePinnedToCore(i2c0_task, "I2C0 Task", 10000, 0, 2, &i2cThread0, 1 - xPortGetCoreID()));
    vTaskDelay(1000);

    // perform simple calibration to avoid some accumulating errors
    FLOAT3 startRotation = getRotation();
    vTaskDelay(4000);
    FLOAT3 endRotation = getRotation();
    for(uint8_t i = 0; i < 3; i++) _rotationOffset[i] = (startRotation.data[i] - endRotation.data[i]) / 4000000;
}

// requests acceleration measurements
CAR::MPU9250::FLOAT3 CAR::MPU9250::getAcceleration() {
    int16_t raw[3] = {};
    mpu_get_accel_reg(raw, 0);
    uint16_t accelDivisor = 0;
    mpu_get_accel_sens(&accelDivisor);
    FLOAT3 result = {};
    for(uint8_t i = 0; i < 3; i++) result.data[i] = raw[0] / (float)accelDivisor;
    return result;
}

// calculates rotation relative to starting position including rounds
CAR::MPU9250::FLOAT3 CAR::MPU9250::getRotation() {
    int64_t time = esp_timer_get_time();
    return {
        (rounds[0] * 360.0f) + lastRotation[0] + (float)(_rotationOffset[0] * time),
        (rounds[1] * 360.0f) + lastRotation[1] + (float)(_rotationOffset[1] * time),
        (rounds[2] * 360.0f) + lastRotation[2] + (float)(_rotationOffset[2] * time)
    };
}

// request rotation change measurement
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

    // set servo pin mode
    pinMode(Pin_Servo_PWM, OUTPUT);
    digitalWrite(Pin_Servo_PWM, LOW);

    // init LEDC channel for servo control
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
}

// set new servo angle by adjusting PWM duty
void CAR::SERVO::setAngle(float angle) {
    SNIPE(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, Servo_PWM_0_Duty + (constrain(angle, -Servo_PWM_Max_Angle, Servo_PWM_Max_Angle) * 70)));
	SNIPE(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1));
}

CAR::SERVO CAR::servo;

CAR::VCT_SUPERVISION::VCT_SUPERVISION() {

    // set pin modes for sensor input pins
    pinMode(Pin_Battery_Voltage, INPUT);
    pinMode(Pin_Total_Current, INPUT);
    pinMode(Pin_Motor_Current, INPUT);

    // seach temperature sensor address
	oneWire.search(ds18b20Address);
}

// request MOSFET temperature from DS18B20 sensor
float CAR::VCT_SUPERVISION::getMosfetTemp() {
	oneWire.reset();
	oneWire.select(ds18b20Address);
	oneWire.write_byte(0x44);
    uint64_t waitStart = esp_timer_get_time();
    while ((oneWire.read_bit() != 1) && (esp_timer_get_time() < (waitStart + 750000)));
    oneWire.reset();
    oneWire.select(ds18b20Address);
	oneWire.write_byte(0xBE);
    uint8_t buf[9];
    for (uint8_t i = 0; i < 9; i++) buf[i] = oneWire.read_byte();
    oneWire.reset();
    return ((((int16_t)buf[1]) << 11) | (((int16_t)buf[0]) << 3) | (0xFFF80000 * (buf[1] & 0x80))) * 0.0078125f;
}

// calculate motor current
float CAR::VCT_SUPERVISION::getMotorCurrent() {
    float result = analogReadMilliVolts(Pin_Motor_Current) * 0.001f;
    return result <= 0.15f ? 0.0f : result;
}

// calculate total current
float CAR::VCT_SUPERVISION::getTotalCurrent() {
    float result = analogReadMilliVolts(Pin_Total_Current) * 0.002f;
    return result <= 0.3f ? 0.0f : result;
}

// calculate battery voltage
float CAR::VCT_SUPERVISION::getVoltage() {
    return analogReadMilliVolts(Pin_Battery_Voltage) * 0.0031273f;
}

CAR::VCT_SUPERVISION CAR::supervision;

void CAR::VL53L1X::init() {

    // start I2C bus 1
    Wire1.begin(Pin_I2C_1_SDA, Pin_I2C_1_SCL, 400000);

    // deactivate all distance sensors
    for(uint8_t i = 0; i < 4; i++) {
        pinMode(Pins_VL53L1X_Shutdown[i], OUTPUT);
        digitalWrite(Pins_VL53L1X_Shutdown[i], LOW);
    }
    delay(15);

    // wake up one sensor, assign a new address and configure it, then wake up the next one
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
	    vl53l1xWriteByte(0x30 + i, 0x0080, 0x55);
        delay(15);
    }

    // create background task to read sensor measurements
    SNIPE(xTaskCreatePinnedToCore(i2c1_task, "I2C1 Task", 10000, 0, 2, &i2cThread1, 1 - xPortGetCoreID()));
}

// returns latest distance measurement
uint16_t CAR::VL53L1X::getDistance(uint8_t pos) {
    return distances[pos];
}

CAR::VL53L1X CAR::vl53l1x;