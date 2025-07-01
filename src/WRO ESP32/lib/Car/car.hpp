/**
 * car.hpp - API containing many useful functions for controlling our autonomously driving car
 * by TerraForce
 */

// optimize program for higher execution speed
#pragma GCC optimize("-Ofast")

// prevent including multiple times
#ifndef CAR_H
#define CAR_H

// include basic types and arduino framework functions
#include <Arduino.h>

namespace CAR {

    // start button with LED
    struct BUTTON {

        // constructs button object, initializes pin modes and states
        BUTTON();

        // reads start button state
        // @returns whether the start button is pressed
        bool pressed();

        // sets the state of the button LED
        // @returns nothing
        void setLED(bool state);

        // activates LED while waiting, waits until the start button is pressed
        // @returns nothing
        void wait();

    } extern button;

    // possible object colors
    enum COLORS : uint8_t {
        CLEAR,
        RED,
        GREEN,
        PINK
    };

    // description of detected objects
    struct OBJECT {
        uint16_t l; // left x position of the bounding rect in image pixels (0 - 1295)
        uint16_t r; // right x position of the bounding rect in image pixels (0 - 1295)
        uint16_t t; // top y position of the bounding rect in image pixels (0 - 1295)
        uint16_t b; // bottom y position of the bounding rect in image pixels (0 - 1295)
        uint16_t w; // bounding rect width in image pixels (0 - 1295)
        uint16_t h; // bounding rect height in image pixels (0 - 1295)
        uint16_t c; // object color
        uint16_t p; // detection probability in 0.1% (0 - 1000)
    };

    // optical object detection
    struct CAMERA {

        // constructs camera object, initializes UART communication
        CAMERA();

        // returns object at a specified index
        // @param idx index of object to fetch
        // @returns object at the specified index or empty object if not existing
        OBJECT getObject(uint8_t idx);

        // returns whether the Raspberry Pi 5 is ready and has started image detection
        // @returns true if ready, false if not
        bool ready();

    } extern camera;

    // ST7735 controlled 128x160 pixels RGB display
    struct LCD {

        // constructs lcd object; initializes spi, display and ledc channel 2 for brightness control; deactivates WiFi and BT
        LCD();

        // updates battery voltage
        // @returns nothing
        void showBatteryVoltage();

        // sets lcd brightness (0.0 - 1.0)
        // @returns nothing
        void setBrightness(float brightness);

        // displays vl53l1x view center calibration interface
        // @param direction direction of the distance sensor to calibrate
        void showDistanceCalibration(uint8_t direction);

        // updates main screen content with distance data
        // @returns nothing
        void showDistances();

        // updates main screen content with finish message
        // @param duration total run duration in microseconds
        // @returns nothing 
        void showFinish(uint32_t duration);

        // updates main screen content showing detected objects
        // @returns nothing
        void showObjects();

        // updates main screen content with ready message
        // @param duration total start duration in microseconds
        // @returns nothing 
        void showReady(uint32_t duration);

        // updates main screen content with rotation data
        // @returns nothing
        void showRotation();

        // updates main screen content with custom string (for debugging only)
        // @param str string to show
        // @returns nothing 
        void showString(String str);

        // updates main screen content with supervision data (total current, motor current, mosfet temperature)
        // @returns nothing
        void showSupervision();

    } extern lcd;

    // front LEDs (brightness control without PWM)
    struct LEDS {

        // sets LED brightness
        // @param left brightness of LEDs on the left side (0 - 15)
        // @param right brightness of LEDs on the right side (0 - 15)
        // @returns nothing
        void setBrightness(uint8_t left, uint8_t right);

    } extern leds;

    // Motor PWM signal generator + reverse relais
    struct MOTOR {

        // constructs motor object, sets pin modes + states, initializes ledc channel 0
        MOTOR();

        // changes motor speed and direction
        // @param speed new speed (-1.0 - 1.0)
        // @returns nothing
        void setSpeed(float speed);

    } extern motor;

    // MPU9250 9-axis motion tracking device
    struct MPU9250 {

        // 3-dimensional component consisting of float values with two possible notations
        union FLOAT3 {
            struct {
                float x, y, z;
            };
            float data[3];
        };

        // initializes i2c and sensor; starts update thread
        // @param rotX 
        // @returns nothing
        void init(bool rotX = false, bool rotY = false, bool rotZ = true);

        // requests current acceleration
        // @returns 3-dimensional acceleration in g
        FLOAT3 getAcceleration();

        // calculates current rotation relative to starting position
        // @returns rotation in ° of all components specified in init() or 0.0 if set to false
        FLOAT3 getRotation();

        // requests current rotation change
        // @returns 3-dimensional rotation change in °/s
        FLOAT3 getRotationChange();

    } extern mpu9250;

    // servo PWM signal generator
    struct SERVO {

        // constructs servo object; sets pin modes + states; initializes ledc channel 1
        SERVO();

        // sets servo angle
        // @param angle target angle in ° (driving left = negative values, driving right = positive values)
        // @returns nothing
        void setAngle(float angle);

    } extern servo;

    // voltage, current and temperature supervision
    struct VCT_SUPERVISION {

        // constructs supervision object; sets pin modes; initializes DS18B20
        VCT_SUPERVISION();

        // reads core temperature
        // @returns core temperature in °C
        float getCoreTemp();

        // requests mosfet temperature
        // @returns mosfet temperature in °C
        float getMosfetTemp();

        // reads motor current
        // @returns motor current in A
        float getMotorCurrent();

        // reads total current
        // @returns total current in A
        float getTotalCurrent();

        // reads battery voltage
        // @returns battery voltage in V
        float getVoltage();

    } extern supervision;

    // ToF sensor positions
    enum POSITION : uint8_t {
        LEFT,
        RIGHT,
        FRONT,
        BACK
    };

    // 4x VL53L1X ToF sensors
    struct VL53L1X {

        // sets pin modes and states; initializes i2c bus and sensors
        // @returns nothing
        void init();

        // returns last distance measurement from the specified sensor
        // @param pos VL53L1X sensor index (see POSITION)
        // @returns distance in mm
        uint16_t getDistance(uint8_t pos);
        
    } extern vl53l1x;
}

#endif