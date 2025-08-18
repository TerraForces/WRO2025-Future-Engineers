/**
 * car.hpp - API containing many useful functions for controlling our autonomously driving car
 * by TerraForce
 */

// optimize program for higher execution speed
#pragma GCC optimize("-Ofast")

// prevent including multiple times
#ifndef CAR_HPP
#define CAR_HPP

// include basic types and arduino framework functions
#include <Arduino.h>

namespace CAR {

    // start button with builtin LED
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
        CLEAR, // sent as first object in each transmission, clears object buffer
        RED,   // red traffic sign
        GREEN, // green traffic sign
        PINK   // pink parking space borders
    };

    // description of detected objects
    struct OBJECT {
        uint16_t l; // left x position of the bounding rect in image pixels (0 - 1295)
        uint16_t r; // right x position of the bounding rect in image pixels (0 - 1295)
        uint16_t t; // top y position of the bounding rect in image pixels (0 - 1295)
        uint16_t b; // bottom y position of the bounding rect in image pixels (0 - 1295)
        uint16_t w; // bounding rect width in image pixels (0 - 1295)
        uint16_t h; // bounding rect height in image pixels (0 - 1295)
        uint16_t c; // object color (see possible object colors above)
        uint16_t p; // detection probability in 0.1% (0 - 1000)
    };

    // object detection by Raspberry Pi 5
    struct CAMERA {

        // constructs camera object, initializes UART communication
        CAMERA();

        // returns object at a specified index (presorted by relevance)
        // @param idx index of object to fetch
        // @returns object at the specified index or empty object if not existing
        OBJECT getObject(uint8_t idx);

        // returns whether the Raspberry Pi 5 is ready and has started inference
        // @returns true if ready, false if not
        bool ready();

    } extern camera;

    // ST7735 controlled 128x160 pixels RGB display
    struct LCD {

        // constructs lcd object; initializes spi bus, display and ledc channel 2 for brightness control
        // deactivates all kinds of wireless communication
        LCD();

        // clears screen and shows battery voltage
        void showBatteryVoltage();

        // sets lcd brightness (0.0 - 1.0)
        void setBrightness(float brightness);

        // displays vl53l1x view center calibration interface
        // @param direction direction of the distance sensor to calibrate
        void showDistanceCalibration(uint8_t direction);

        // updates main screen content with distance data
        void showDistances();

        // updates main screen content with finish message
        // @param duration total run duration in microseconds
        void showFinish(uint32_t duration);

        // updates main screen content with detected objects
        void showObjects();

        // updates main screen content with ready message
        // @param duration total start duration in microseconds
        void showReady(uint32_t duration);

        // updates main screen content with rotation data
        void showRotation();

        // updates main screen content with custom string (recommended for debugging only)
        // @param str string to show
        void showString(String str);

        // updates main screen content with supervision data
        // (total current, motor current, MOSFET temperature)
        void showSupervision();

    } extern lcd;

    // front LEDs (brightness control without PWM)
    struct LEDS {

        // sets LED brightness
        // @param left brightness of LEDs on the left side (0 - 15)
        // @param right brightness of LEDs on the right side (0 - 15)
        void setBrightness(uint8_t left, uint8_t right);

    } extern leds;

    // motor PWM signal control + relais for reverse drive
    struct MOTOR {

        // constructs motor object; sets pin modes + states; initializes ledc channel 0 for PWM
        MOTOR();

        // changes motor speed and direction over acceleration ramp
        // @param speed new speed (-1.0 - 1.0)
        void setSpeed(float speed);

    } extern motor;

    // MPU9250 9-axis motion tracking device
    struct MPU9250 {

        // 3-dimensional component consisting of 3 floating point values with two possible notations
        union FLOAT3 {
            struct {
                float x, y, z;
            };
            float data[3];
        };

        // initializes i2c and sensor; starts update thread
        void init();

        // requests current acceleration
        // @returns 3-dimensional acceleration in g
        FLOAT3 getAcceleration();

        // calculates current rotation relative to starting position
        // @returns 3-dimensional rotation in °
        FLOAT3 getRotation();

        // requests current rotation change
        // @returns 3-dimensional rotation change in °/s
        FLOAT3 getRotationChange();

    } extern mpu9250;

    // servo PWM signal generator
    struct SERVO {

        // constructs servo object; sets pin modes + states; initializes ledc channel 1 for PWM
        SERVO();

        // sets servo angle
        // @param angle target angle in ° (driving left = negative values, driving right = positive values)
        void setAngle(float angle);

    } extern servo;

    // voltage, current and temperature supervision
    struct VCT_SUPERVISION {

        // constructs supervision object; sets pin modes; initializes DS18B20
        VCT_SUPERVISION();

        // requests MOSFET temperature
        // @returns MOSFET temperature in °C
        float getMosfetTemp();

        // reads motor current
        // @returns motor current in A
        float getMotorCurrent();

        // reads total current
        // @returns current out of the battery in A
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
        void init();

        // returns last distance measurement from the specified sensor
        // @param pos VL53L1X sensor index (see POSITION)
        // @returns distance in front of the sensor in mm
        uint16_t getDistance(uint8_t pos);
        
    } extern vl53l1x;
}

#endif