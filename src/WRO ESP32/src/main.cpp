/**
 * main.cpp - pure driving code containing both challenge types and test modes
 * by TerraForce
 */

// optimize program for higher execution speed
#pragma GCC optimize("-Ofast")

// program selection definations
#define OBJECT_DETECTION
//#define TEST

// include car.hpp for sensor and control functions
#include "car.hpp"

// automatically use namespace CAR in this document
using namespace CAR;

#ifdef TEST
// test mode for distance sensor calibration or object detection test
#pragma region test mode

void setup() {

	// start MPU9250
	mpu9250.init();
	
	// start ToF sensors
	vl53l1x.init();

#ifdef OBJECT_DETECTION

	// wait for Raspberry Pi 5 to get ready
	while(!camera.ready()) vTaskDelay(1);

#endif

	// show ready screen with time since boot
	lcd.showReady(esp_timer_get_time());

	// wait for button press
	button.wait();
}

void loop() {

#ifdef OBJECT_DETECTION

	// update screen periodically with objects (every 500 ms)
	static TickType_t lastTicks = xTaskGetTickCount();
	lcd.showObjects();
	xTaskDelayUntil(&lastTicks, 500);

#else

	// show uncalibrated distances
	lcd.showDistances();
	vTaskDelay(200);
	button.wait();

	// show calibration interface for each of the 4 ToF sensors
	for(uint8_t i = 0; i < 4; i++) {
		lcd.showDistanceCalibration(i);
		vTaskDelay(200);
		button.wait();
	}
	
	// show other sensor data
	lcd.showRotation();
	vTaskDelay(200);
	button.wait();
	lcd.showSupervision();
	vTaskDelay(200);
	button.wait();

#endif

}

#else

#ifndef OBJECT_DETECTION
// open challenge without objects but with variable width of the inner border
#pragma region open challenge

// driving speed selection (speed as a fraction of maximum speed; recommended 0.2f - 1.0f)
constexpr float SPEED_START  = 1.0f; // speed while looking for the first corner
constexpr float SPEED_NORMAL = 1.0f; // speed while close to next turn or border
constexpr float SPEED_HIGH   = 1.0f; // speed while driving on optimal track

// global working variables
bool outsideBorder = 0;      // outside border relative to vehicle driving direction (0 = left; 1 = right)
float targetRotation = 0.0f; // target driving rotation for direction correction relative to starting position (in °)
float rotationOffset = 0.0f; // rotation offset at starting position
int64_t startTime = 0;       // driving start time in microseconds from boot
int64_t lastCurve = 0;       // last curve end time in microseconds from boot

void setup() {

	// start MPU9250
	mpu9250.init();
	
	// start ToF sensors
	vl53l1x.init();

	// show ready screen with time since boot
	lcd.showReady(esp_timer_get_time());

	// wait for button press
	button.wait();

	// capture start time
	startTime = esp_timer_get_time();

	// set rotation offset to current rotation
	rotationOffset = mpu9250.getRotation().z;

	// start driving forward
	motor.setSpeed(SPEED_START);

	// wait until open space on one side is detected
	bool borderLeft = true, borderRight = true;
	while(borderLeft && borderRight) {
		borderLeft = vl53l1x.getDistance(LEFT) < 1000;
		borderRight = vl53l1x.getDistance(RIGHT) < 1000;
		vTaskDelay(1);
	}

	// set outside border to the side where no open space or close border was detected
	outsideBorder = borderRight;

	// increase driving speed
	motor.setSpeed(SPEED_NORMAL);
}

void loop() {

	// get current rotation and subtract offset
	float rotation = mpu9250.getRotation().z - rotationOffset;

	// get current distance measurements
	// (excluding back/rear distance - needed only once - no storage required)
	uint16_t distances[3] = {};
	for(uint8_t i = 0; i < 3; i++) distances[i] = vl53l1x.getDistance(i);

	// end condition: stop after 3 rounds = 1080°
	// and sufficient distance on the rear sensor to stop inside of the section
	// (cooldown after last curve needed because of sensor latency)
	if ((abs(targetRotation) >= 1080.0f) && (vl53l1x.getDistance(BACK) > 1000) && (esp_timer_get_time() > (lastCurve + 500000))) {

		// stop driving
		motor.setSpeed(0.0f);

		// repeat until really stopped
		while(supervision.getMotorCurrent() > 0) {

			// get current rotation and subtract offset
			rotation = mpu9250.getRotation().z - rotationOffset;

			// drive away from borders when getting too close
			if (vl53l1x.getDistance(LEFT) < 150) {
				servo.setAngle(6.0f);
				vTaskDelay(200);
			}
			else if (vl53l1x.getDistance(RIGHT) < 150) {
				servo.setAngle(-6.0f);
				vTaskDelay(200);
			}

			// rotation correction (rotate towards target rotation when not driving away from borders)
			else if ((targetRotation - rotation) < -2.0f) servo.setAngle(max(targetRotation - rotation + 2.0f, -7.0f));
			else if ((targetRotation - rotation) >  2.0f) servo.setAngle(min(targetRotation - rotation - 2.0f,  7.0f));
		}

		// show finish screen with driving time
		lcd.showFinish(esp_timer_get_time() - startTime);

		// wait indefinitely until shut down or restarted (by power or enable button)
		while(true) vTaskDelay(10000000);
	}

	// only perform calculations if driving speed increase possible
	if(SPEED_NORMAL < SPEED_HIGH) {

		// increase driving speed when driving on optimal track, far away from curves or borders
		if ((distances[FRONT] > 1000) && (distances[LEFT] > 160) && (distances[RIGHT] > 160) && (abs(targetRotation - rotation) < 4.0f)) motor.setSpeed(SPEED_HIGH);

		// drive slower when not on optimal track or close to border or next curve
		else motor.setSpeed(SPEED_NORMAL);
	}

	// curve mode - activated min. 750 ms after last curve (because of sensor latency)
	// starting when wide space at one side detected and front distance < 1.2 m
	// or when front distance < 650 mm
	if (((((distances[LEFT] > 1000) || (distances[RIGHT] > 1000)) && (distances[FRONT] < 1200)) || (distances[FRONT] < 650)) && (esp_timer_get_time() > (lastCurve + 750000))) {
		
		// activate LEDs on the side where the vehicle is turning to (purely decorative, not necessary)
		leds.setBrightness(outsideBorder ? 15 : 0, outsideBorder ? 0 : 15);

		// set servo to 20° angle
		servo.setAngle(outsideBorder ? -20.0f : 20.0f);

		// drive until rotation difference > 55° (not 90° because of sensor and servo latency)
		targetRotation += outsideBorder ? -55.0f : 55.0f;
		do {
			rotation = mpu9250.getRotation().z - rotationOffset;
			vTaskDelay(5);
		} while (outsideBorder ? (targetRotation < rotation) : (targetRotation > rotation));

		// set "real" target rotation (before the curve + 90°)
		targetRotation += outsideBorder ? -35.0f : 35.0f;

		// turn of cosmetic LEDs
		leds.setBrightness(0, 0);

		// set servo to 0° again
		servo.setAngle(0.0f);

		// save time since boot for curve cooldown
		lastCurve = esp_timer_get_time();

		// small delay when driving on the final section - to be tested
		if(abs(targetRotation) >= 1080.0f) vTaskDelay(500);
	}

	// border correction (drive away from borders when getting too close)
	// inactive when next curve is expected (front distance <= 900 mm)
	else if ((distances[LEFT] < 150) && (distances[FRONT] > 900)) {

		// limit angle while driving away from border to 20°
		servo.setAngle(((targetRotation - rotation) > -20.0f) ? 6.0f : 0.0f);
		vTaskDelay(50);
	}
	else if ((distances[RIGHT] < 150) && (distances[FRONT] > 900)) {

		// limit angle while driving away from border to 20°
		servo.setAngle(((targetRotation - rotation) < 20.0f) ? -6.0f : 0.0f);
		vTaskDelay(50);
	}

	// rotation correction (rotate towards target rotation when not driving away from borders)
	else if ((targetRotation - rotation) < -2.0f) servo.setAngle(max(targetRotation - rotation + 2.0f, -7.0f));
	else if ((targetRotation - rotation) >  2.0f) servo.setAngle(min(targetRotation - rotation - 2.0f,  7.0f));

	// drive straight when no correction is required
	else servo.setAngle(0.0f);
}

#else
// obstacle challenge with traffic signs but fixed inner border
#pragma region obstacle challenge

// activate to enter parking space at the end (always starts from parking space)
#define PARK_IN

// speed and angle configuration (speed as a fraction of the maximum speed; angle in degrees)
constexpr float PARK_ANGLE    = 45.0f; // servo angle used for leaving the parking space
constexpr float SPEED_PARK    = 0.15f; // motor speed while leaving the parking space
constexpr float SPEED_DRIVE   = 0.40f; // regular driving speed
constexpr float MIN_RCR_ANGLE = 0.20f; // minimum rotation difference to be corrected

// global working variables
bool outsideBorder = 0;          // position of the outside border relative to the vehicle (left = 0; right = 1)
bool drivingSide = 0;            // current driving side (side of closest border; left = 0; right = 1)
int32_t curveTargetRotation = 0; // driving target rotation (always multiples of 90°)
int64_t startTime = 0;           // driving start time in microseconds since boot
float rotationOffset = 0.0f;     // offset from sensor rotation value (currently only used to compensate long waiting times before start)
uint8_t section[4][3] = {};      // array containing all necessary information for object avoidance (4 straigtforward section with 3 positions in driving order; values represent traffic sign colors: 0 = unknown, 1 = red, 2 = green, 3 = no traffic sign)

// try to correct rotation measurements by calculating the true rotation using two measurements of distance to the side and the front distance
void correctRotationOffset(bool side, uint64_t waitTicks = 500) {
	uint16_t frontDistance = vl53l1x.getDistance(FRONT);
	uint16_t sideDistance = vl53l1x.getDistance(side);
	vTaskDelay(waitTicks); // longer wait time can improve results
	rotationOffset = (atan((frontDistance - vl53l1x.getDistance(FRONT)) / ((float)(sideDistance - vl53l1x.getDistance(side)))) * 57.295779513f) - curveTargetRotation + mpu9250.getRotation().z;
	if(side) rotationOffset = -rotationOffset;
}

// curve mode - drive a 90° curve in the specified direction
void curve(bool direction, float sharpness = 0.0f) {
	motor.setSpeed(0.3f);                                       // slow down before entering the curve
	leds.setBrightness(direction ? 0 : 15, direction ? 15 : 0); // activate LED on the side the vehicle is driving to (purely cosmetical)
	servo.setAngle(direction ? 42.0f : -42.0f);                 // set servo angle to start curve
	curveTargetRotation += direction ? 90 : -90;                // set new target rotation

	// wait until rotation says that curve is completed
	while (direction ? ((mpu9250.getRotation().z + rotationOffset) < (curveTargetRotation - 24.0f + sharpness)) : ((mpu9250.getRotation().z + rotationOffset) > (curveTargetRotation + 24.0f - sharpness))) vTaskDelay(1);
	
	leds.setBrightness(0, 0);    // deactivate LEDs
	servo.setAngle(0.0f);        // drive forward
	motor.setSpeed(SPEED_DRIVE); // accelerate again
}

// correct orientation using the rotation data provided by the gyro sensor
void rotationCorrection(bool invert = false) {
	float rotation = mpu9250.getRotation().z + rotationOffset;
	if ((curveTargetRotation - rotation) < MIN_RCR_ANGLE) servo.setAngle(invert ? -max(curveTargetRotation - rotation + MIN_RCR_ANGLE, -10.0f) : max(curveTargetRotation - rotation + MIN_RCR_ANGLE, -10.0f));
	else if ((curveTargetRotation - rotation) > MIN_RCR_ANGLE) servo.setAngle(invert ? -min(curveTargetRotation - rotation - MIN_RCR_ANGLE,  10.0f) : min(curveTargetRotation - rotation - MIN_RCR_ANGLE,  10.0f));
	
	// when driving perfectly straight, a rotation offset calculation can be performed to correct errors in the gyro sensor that might accumulate over time
	else if (vl53l1x.getDistance(FRONT) > 1500) {
		servo.setAngle(0.0f);
		vTaskDelay(10);
		correctRotationOffset(outsideBorder, 400);
	}
	vTaskDelay(1);
}

void setup() {

	// wait for other systems to get ready
	vTaskDelay(500);

	// start MPU9250
	mpu9250.init();
	
	// start ToF sensors
	vl53l1x.init();

	// wait for Raspberry Pi 5 to get ready
	while(!camera.ready()) vTaskDelay(1);

	// show ready screen
	lcd.showReady(esp_timer_get_time());

	// wait for button press
	button.wait();

	// capture start time
	startTime = esp_timer_get_time();

	// adjust rotation offset
	rotationOffset = -mpu9250.getRotation().z;

	// detect outside border
	outsideBorder = vl53l1x.getDistance(LEFT) > 300;

	// leave parking space using 4 move consisting of inverting the servo and motor direction and driving until reaching specified rotations (determined experimentally)
	servo.setAngle(outsideBorder ? PARK_ANGLE : -PARK_ANGLE);
	motor.setSpeed(-SPEED_PARK);
	while (outsideBorder ? ((mpu9250.getRotation().z + rotationOffset) > -20.0f) : ((mpu9250.getRotation().z + rotationOffset) < 20.0f)) vTaskDelay(1);
	servo.setAngle(outsideBorder ? -PARK_ANGLE : PARK_ANGLE);
	motor.setSpeed(SPEED_PARK);
	while (outsideBorder ? ((mpu9250.getRotation().z + rotationOffset) > -40.0f) : ((mpu9250.getRotation().z + rotationOffset) < 40.0f)) vTaskDelay(1);
	servo.setAngle(outsideBorder ? PARK_ANGLE : -PARK_ANGLE);
	motor.setSpeed(-SPEED_PARK);
	while (outsideBorder ? ((mpu9250.getRotation().z + rotationOffset) > -60.0f) : ((mpu9250.getRotation().z + rotationOffset) < 60.0f)) vTaskDelay(1);
	servo.setAngle(outsideBorder ? -PARK_ANGLE : PARK_ANGLE);
	motor.setSpeed(SPEED_PARK);
	while (outsideBorder ? ((mpu9250.getRotation().z + rotationOffset) > -92.0f) : ((mpu9250.getRotation().z + rotationOffset) < 92.0f)) vTaskDelay(1);
	servo.setAngle(0.0f);

	// set new target rotation
	curveTargetRotation = outsideBorder ? -90 : 90;

	// set driving side to the inner border
	drivingSide = 1 - outsideBorder;

	// object avoidance code for the starting section
	#pragma region section 0 // region defined for easier code navigation

	// different approaches for the two possible driving directions necessary
	// approach for the possibility of the car driving counterclockwise
	if(outsideBorder) {
		motor.setSpeed(SPEED_DRIVE);

		// check whether there is an object to avoid while leaving the parking space
		for (uint16_t i = 0; (camera.getObject(i).c != 0) && (section[0][2] == 0); i++) {	// loop that checks every object detected
			if(camera.getObject(i).c != PINK) {												// condition that checks whether the current object is not pink
				if(camera.getObject(i).l > 650) {											// condition that checks whether the current object is on the left side of the picture
					if(camera.getObject(i).b > 480) section[0][2] = camera.getObject(i).c;	// condition that checks whether the current object starts near the bottom of the picture, and saves the object colour and position in the array, called section
				}
			}
		}
		if(section[0][2] == 0) section[0][2] = 3;								// save in the section array whether no object was detected
		if(section[0][2] == RED) {												// condition that checks whether the car has to pass an object at the side of the outside border

			// let the car drive until it is time to turn to pass the object at the outer side
			drivingSide = outsideBorder;
			while (vl53l1x.getDistance(FRONT) > 600) rotationCorrection();
		}
		else {

			// let the car drive until it is time to turn to pass the object at the inner side
			drivingSide = !outsideBorder;
			while (vl53l1x.getDistance(FRONT) > 350) rotationCorrection();
		}
		curve(outsideBorder); 	// turn into the direction of the outside border to align to drive into the next section
	}

	else {
		motor.setSpeed(0.0f);
		vTaskDelay(50);

		// checks whether there is an object to avoid at the start of the second round
		for (uint16_t i = 0; (camera.getObject(i).c != 0) && (section[0][0] == 0); i++) {	// loop that checks every object detected
			if(camera.getObject(i).c != PINK) {												// condition that checks whether the current object is not pink
				if(camera.getObject(i).l > 650) {											// condition that checks whether the current object is on the left side of the picture
					if(camera.getObject(i).b > 480) section[0][0] = camera.getObject(i).c;	// condition that checks whether the current object starts near the bottom of the picture, and saves the object colour and position in the array, called section
				}
			}
		}

		// reposition in a 60° angle to check whether the car has to avoid a obstacle while leaving the first section
		motor.setSpeed(0.45f);
		leds.setBrightness(15, 0);
		servo.setAngle(-40.0f);
		curveTargetRotation -= 60;
		while (((mpu9250.getRotation().z + rotationOffset) > (curveTargetRotation + 25.0f))) vTaskDelay(1);
		leds.setBrightness(0, 0);
		servo.setAngle(0.0f);
		motor.setSpeed(0.0f);
		vTaskDelay(100);
		motor.setSpeed(SPEED_DRIVE);

		// checks whether there is an object to avoid while leaving the first section
		for (uint16_t i = 0; (camera.getObject(i).c != 0) && (section[0][1] == 0); i++) {			// loop that checks every object detected
			if(camera.getObject(i).c != PINK) {														// condition that checks whether the current object is not pink
				if(camera.getObject(i).l < 650) {													// condition that checks whether the current object is on the right side of the picture
					if(camera.getObject(i).b > 500) section[0][1] = camera.getObject(i).c;			// condition that checks whether the current object is in the middle of the current section, and saves the object colour and position in the array, called section
					else if (camera.getObject(i).b > 440) section[0][2] = camera.getObject(i).c;	// condition that checks whether the current object is at the end of the current section of the section, and saves the object colour and position in the array, called section
				}
			}
		}
		if(section[0][0] == 0) section[0][0] = 3;		// save in the section array whether no object was detected at the start of the section
		if(section[0][2] == 0) section[0][2] = 3;		// save in the section array whether no object was detected in the middle of the section
		if(section[0][1] == 0) section[0][1] = 3;		// save in the section array whether no object was detected at the end of the section

		// condition that checks whether the car has to pass an object at the side of the outside border
		if((section[0][1] == GREEN) || ((section[0][2] == GREEN) && (section[0][1] == 3))) {

			// align parallel to the outside border to pass the object at the side of the outside border
			drivingSide = outsideBorder;
			motor.setSpeed(0.45f);
			leds.setBrightness(15, 0);
			servo.setAngle(-40.0f);
			curveTargetRotation -= 30;
			while (((mpu9250.getRotation().z + rotationOffset) > (curveTargetRotation + 5.0f))) vTaskDelay(1);
			leds.setBrightness(0, 0);
			servo.setAngle(0.0f);
		}
		else {

			// drive to the inside border
			drivingSide = !outsideBorder;
			motor.setSpeed(0.45f);
			leds.setBrightness(0, 15);
			servo.setAngle(20.0f);
			curveTargetRotation += 60;
			while ((mpu9250.getRotation().z + rotationOffset) < (curveTargetRotation - 5.0f)) vTaskDelay(1);
			leds.setBrightness(0, 0);
			servo.setAngle(0.0f);
			while (vl53l1x.getDistance(FRONT) > 340) rotationCorrection();

			// align parallel to the inside border 
			curve(outsideBorder);
		}
	}
	while (vl53l1x.getDistance(!outsideBorder) < 1500) rotationCorrection(); 		// pass the object and drive into the next section

	lcd.showString(String(section[0][0]) + '\n' + String(section[0][1]) + '\n' + String(section[0][2]));	// show the objects that where detected in the first section (for debug only)

	// object avoidance code for the sections 1 to 3
	#pragma region sections 1 - 3

	// loop to repeat the code for the sections 1 to 3
	for(uint8_t j = 1; j <= 3; j++) {

		// align at the outside border to check the current section for obstacles
		if(drivingSide == outsideBorder) {										// check the current position of the car to decide how to align at the outside border
			while(vl53l1x.getDistance(FRONT) > 975) rotationCorrection();		// drive into the corner section
			curve(!outsideBorder);												// turn in the direction of the new section
			motor.setSpeed(-0.35f);	
			while(vl53l1x.getDistance(BACK) > 110) rotationCorrection(true);	// drive backwards to the outside border to align at it
			motor.setSpeed(0.0f);
		}
		else {
			while(vl53l1x.getDistance(FRONT) > 500) rotationCorrection();		// drive into the corner section
			motor.setSpeed(-SPEED_DRIVE);
			while(vl53l1x.getDistance(FRONT) < 580) rotationCorrection();	
			
			// turn while driving backwards to face into the direction of the new section
			if(outsideBorder) {
				motor.setSpeed(-0.3f);
				leds.setBrightness(0, 15);
				servo.setAngle(40.0f);
				curveTargetRotation += -90;
				while ((mpu9250.getRotation().z + rotationOffset) > (curveTargetRotation + 25.0f)) vTaskDelay(1);
			}
			else {
				motor.setSpeed(-0.3f);
				leds.setBrightness(0, 15);
				servo.setAngle(-40.0f);
				curveTargetRotation += 90;
				while ((mpu9250.getRotation().z + rotationOffset) < (curveTargetRotation - 25.0f)) vTaskDelay(1);
			}
			
			motor.setSpeed(-0.5f);
			while(vl53l1x.getDistance(BACK) > 110) rotationCorrection(true); 	// drive backwards to the outside border to align at it
			motor.setSpeed(0.0f);
		}

		drivingSide = !outsideBorder;		// reset the position variable

		// loop to detect whether there is an object at the nearest 2 positions in the new section
		for (uint16_t i = 0; (camera.getObject(i).c != 0) && (section[j][0] == 0); i++) {		// loop that checks every object detected
			if(camera.getObject(i).c != PINK) {													// condition that checks whether the current object is not pink
				if(outsideBorder? camera.getObject(i).l > 650 : camera.getObject(i).l < 650) { 	// condition that checks whether the current object is on the side of the outsideborder
					if(camera.getObject(i).b > 440) section[j][0] = camera.getObject(i).c;		// condition that checks whether the current object is at the start of the new section of the section, and saves the object colour and position in the array, called section
				}
			}
		}

		// detect the position of the object, if there was no object in the first position
		if(section[j][0] == 0) {
			section[j][0] = 3;

			// check whether there is an object in the middle of the section
			for (uint16_t i = 0; (camera.getObject(i).c != 0) && (section[j][1] == 0); i++) {
				if(camera.getObject(i).c != PINK) {
					if(outsideBorder? camera.getObject(i).l > 650 : camera.getObject(i).l < 650) {
						if(camera.getObject(i).b > 380) section[j][1] = camera.getObject(i).c;
					}
				}
			}

			// check whether there is an object at the end of the section
			if(section[j][1] == 0) {
				section[j][1] = 3;
				for (uint16_t i = 0; (camera.getObject(i).c != 0) && (section[j][2] == 0); i++) {
					if(camera.getObject(i).c != PINK) {
						if(outsideBorder? camera.getObject(i).l > 650 : camera.getObject(i).l < 650) {
							if(camera.getObject(i).b > 350) section[j][2] = camera.getObject(i).c;
						}
					}
				}	
				if(section[j][2] == 0) section[j][2] = 3;
			}
			else section[j][2] = 3;
		}


		if(section[j][0] != 3) {
			if(section[j][0] == (outsideBorder ? RED : GREEN)) {				// checks whether the car has to pass the object at the side of the outside border
				curve(outsideBorder);											// turn to the outside border
				while(vl53l1x.getDistance(FRONT) > 340) rotationCorrection();	// drive to the outside border
				curve(!outsideBorder);											// realign parallel to the outside border
				drivingSide = outsideBorder;
			}
			motor.setSpeed(0.5f);
			uint64_t wait_time = esp_timer_get_time();
			while(wait_time > (esp_timer_get_time() - 1000000)) rotationCorrection();
			while((vl53l1x.getDistance(FRONT) > 1900) || (vl53l1x.getDistance(BACK) < 1100)) rotationCorrection();		// drive completely into the straight section
			vTaskDelay(100);

			// check whether there is a second object in the current section
			for (uint16_t i = 0; (camera.getObject(i).c != 0) && (section[j][2] == 0); i++) {
				if(camera.getObject(i).c != PINK) {
					if(camera.getObject(i).b > 410) section[j][2] = camera.getObject(i).c;
				}
			}
			vTaskDelay(100); // wait for a new image

			// ckeck again if there is a second object, in case the car does not recognise it at the first image
			for (uint16_t i = 0; (camera.getObject(i).c != 0) && (section[j][2] == 0); i++) {
				if(camera.getObject(i).c != PINK) {
					if(camera.getObject(i).b > 410) section[j][2] = camera.getObject(i).c;
				}
			}
			if(section[j][1] == 0) section[j][1] = 3;
			if(section[j][2] == 0) section[j][2] = 3;

			// if required drive back to the inside border
			if((section[j][2] == (outsideBorder ? GREEN : RED)) && (drivingSide == outsideBorder)) {
				curve(!outsideBorder);
				while(vl53l1x.getDistance(FRONT) > 350) rotationCorrection();
				curve(outsideBorder);
				drivingSide = !outsideBorder;
				vTaskDelay(50);
			}

			// if required drive to the outside border
			if((section[j][2] == (outsideBorder ? RED : GREEN)) && drivingSide != outsideBorder) {
				curve(outsideBorder);
				while(vl53l1x.getDistance(FRONT) > 350) rotationCorrection();
				curve(!outsideBorder);
				drivingSide = outsideBorder;
				vTaskDelay(50);
			}
			motor.setSpeed(0.5f);
			while((vl53l1x.getDistance(!outsideBorder) < 1100) || (vl53l1x.getDistance(FRONT) > 1500)) rotationCorrection(); // pass the object and drive to the next corner section
		}

		// drive code for the current section if there only can be one object, because there was no object in the first position
		else {
			if((section[j][1] == (outsideBorder ? RED : GREEN)) || ((section[j][1] == 3) && (section[j][2] == (outsideBorder ? RED : GREEN)))) {	// checks whether the car has to pass the object at the side of the outside border
				curve(outsideBorder);												// turn to the outside border
				while(vl53l1x.getDistance(FRONT) > 350) rotationCorrection();		// drive to the outsideborder
				curve(!outsideBorder);												// realign parallel to the outsideborder
				drivingSide = outsideBorder;
				vTaskDelay(50);
			}
			motor.setSpeed(0.5f);
			while((vl53l1x.getDistance(!outsideBorder) < 1100) || (vl53l1x.getDistance(FRONT) > 1500)) rotationCorrection(); // pass the object and drive to the next corner section
		}
	}

	#pragma region section 4

	// align at the outside border and check the position of the first object, the same way as in the last 3 sections
	if(drivingSide == outsideBorder) {
		while(vl53l1x.getDistance(FRONT) > 975) rotationCorrection();
		curve(!outsideBorder);
		motor.setSpeed(-0.35f);
		while(vl53l1x.getDistance(BACK) > 110) rotationCorrection(true);
		motor.setSpeed(0.0f);
	}
	else {
		while(vl53l1x.getDistance(FRONT) > 500) rotationCorrection();
		motor.setSpeed(-SPEED_DRIVE);
		while(vl53l1x.getDistance(FRONT) < 620) rotationCorrection();
		if(outsideBorder) {
			motor.setSpeed(-0.3f);
			leds.setBrightness(0, 15);
			servo.setAngle(40.0f);
			curveTargetRotation += -90;
			while ((mpu9250.getRotation().z + rotationOffset) > (curveTargetRotation + 25.0f)) vTaskDelay(1);
		}
		else {
			motor.setSpeed(-0.3f);
			leds.setBrightness(0, 15);
			servo.setAngle(-40.0f);
			curveTargetRotation += 90;
			while ((mpu9250.getRotation().z + rotationOffset) < (curveTargetRotation - 25.0f)) vTaskDelay(1);
		}
		motor.setSpeed(-0.4f);
		while(vl53l1x.getDistance(BACK) > 110) rotationCorrection(true);
		motor.setSpeed(0.0f);
	}

	drivingSide = !outsideBorder;

	if(outsideBorder) {
		for (uint16_t i = 0; (camera.getObject(i).c != 0) && (section[0][0] == 0); i++) {
			if(camera.getObject(i).c != PINK) {
				if(outsideBorder ? (camera.getObject(i).l > 650) : (camera.getObject(i).l < 650)) {
					if(camera.getObject(i).b > 440) section[0][0] = camera.getObject(i).c;
				}
			}
		}
		if(section[0][0] == 0) {
			section[0][0] = 3;
			for (int i = 0; (camera.getObject(i).c != 0) && (section[0][1] == 0); i++) {
				if(camera.getObject(i).c != PINK) {
					if(outsideBorder ? (camera.getObject(i).l > 650) : (camera.getObject(i).l < 650)) {
						if(camera.getObject(i).b > 380) section[0][1] = camera.getObject(i).c;
					}
				}
			}
			if(section[0][1] == 0) {
				section[0][1] = 3;
				for (uint16_t i = 0; (camera.getObject(i).c != 0) && (section[0][2] == 0); i++) {
					if(camera.getObject(i).c != PINK) {
						if(outsideBorder ? (camera.getObject(i).l > 650) : (camera.getObject(i).l < 650)) {
							if(camera.getObject(i).b > 350) section[0][2] = camera.getObject(i).c;
						}
					}
				}
				if(section[0][2] == 0) section[0][2] = 3;
			}
			else section[0][2] = 3;
		}
	}
	// drive arround the objects, the same way as in the last three sections, but this time without looking for a second object, because if there is a second object it would already be known
	if(section[0][0] != 3) {
		if((section[0][0] == (outsideBorder ? RED : GREEN))) {
			curve(outsideBorder);
			while(vl53l1x.getDistance(FRONT) > 520) rotationCorrection();
			curve(!outsideBorder);
			drivingSide = outsideBorder;
		}
		motor.setSpeed(0.5f);
		uint64_t wait_time = esp_timer_get_time();
		while(wait_time > (esp_timer_get_time() - 1000000)) rotationCorrection();
		while((vl53l1x.getDistance(FRONT) > 1900) || (vl53l1x.getDistance(BACK) < 1100)) rotationCorrection();
		vTaskDelay(50);
		
		if((section[0][2] == (outsideBorder ? GREEN : RED)) && (drivingSide == outsideBorder)) {
			curve(!outsideBorder);
			while(vl53l1x.getDistance(FRONT) > 370) rotationCorrection();
			curve(outsideBorder);
			drivingSide = !outsideBorder;
			vTaskDelay(50);
		}
		if((section[0][2] == (outsideBorder ? RED : GREEN)) && (drivingSide != outsideBorder)) {
			curve(outsideBorder);
			while(vl53l1x.getDistance(FRONT) > 520) rotationCorrection();
			curve(!outsideBorder);
			drivingSide = outsideBorder;
			vTaskDelay(50);
		}
		motor.setSpeed(0.5f);
		while((vl53l1x.getDistance(!outsideBorder) < 1100) || (vl53l1x.getDistance(FRONT) > 1500)) rotationCorrection();
	}
	else {
		if((section[0][1] == (outsideBorder ? RED : GREEN)) || ((section[0][1] == 3) && (section[0][2] == (outsideBorder ? RED : GREEN)))) {
			curve(outsideBorder);
			while(vl53l1x.getDistance(FRONT) > 520) rotationCorrection();
			curve(!outsideBorder);
			drivingSide = outsideBorder;
			vTaskDelay(50);
		}
		motor.setSpeed(0.5f);
		while((vl53l1x.getDistance(!outsideBorder) < 1100) || (vl53l1x.getDistance(FRONT) > 1500)) rotationCorrection();
	}

	// object avoidance code for the sections 5 to 11
	#pragma region sections 5 - 11

	for(uint8_t k = 1; k <= 7; k++) {		//loop to repeat the code for the sections 5 to 11
		if(((section[k % 4][0] == (outsideBorder ? RED : GREEN)) || ((section[k % 4][0] == 3) && (section[k % 4][1] == (outsideBorder ? RED : GREEN)))) || ((section[k % 4][0] == 3) && (section[k % 4][1] == 3) && (section[k % 4][2] == (outsideBorder ? RED : GREEN)))) { // check if the car has to pass the first object of the next straight section at the side of the outside border
			drivingSide = outsideBorder;
			while(vl53l1x.getDistance(FRONT) > (((k % 4) == 0) ? 520 : 300)) rotationCorrection(); 	// drive until nearly reaching the outsideborder to pass the first object after the turn at the side of the outside border
			curve(!outsideBorder, 10.0f);															// turn into the direction of the new section
		}
		else {
			drivingSide = !outsideBorder;
			curve(!outsideBorder);			// turn into the direction of the new section without waiting to pass the first object at the side of the inner border
		}
		int64_t t = esp_timer_get_time() + 1000000;
		while(esp_timer_get_time() < t) rotationCorrection();														// wait a second while correcting the alignment
		while((vl53l1x.getDistance(FRONT) > 1800) || (vl53l1x.getDistance(BACK) < 1200)) rotationCorrection(); 		// drive completely into the straight section
		if((section[k % 4][0] != 3) && (section[k % 4][0] != section[k % 4][2])) {									// check whether there is a second object the car has to pass on the other side
			
			// drive to the outside border and realign if the car has to pass the second object at the side of the outside border
			if((section[k % 4][2] == (outsideBorder ? RED : GREEN)) && (drivingSide != outsideBorder)) {
				curve(outsideBorder);
				while(vl53l1x.getDistance(FRONT) > (((k % 4) == 0) ? 520 : 350)) rotationCorrection();
				curve(!outsideBorder);
				drivingSide = outsideBorder;
			}

			// drive to the inner border and realign if the car has to pass the second object at the side of the inner border
			else if((section[k % 4][2] == (outsideBorder ? GREEN : RED)) && (drivingSide == outsideBorder)) {
				curve(!outsideBorder);
				while(vl53l1x.getDistance(FRONT) > 350) rotationCorrection();
				curve(outsideBorder);
				drivingSide = !outsideBorder;
			}
		}

		// drive into the next curve section
		vTaskDelay(100);
		while((vl53l1x.getDistance(!outsideBorder) < 1000) || (vl53l1x.getDistance(FRONT) > 1500) || (vl53l1x.getDistance(BACK) < 800)) rotationCorrection();
		vTaskDelay(100);																																		// wait for new distance data
		while((vl53l1x.getDistance(!outsideBorder) < 1000) || (vl53l1x.getDistance(FRONT) > 1500) || (vl53l1x.getDistance(BACK) < 800)) rotationCorrection(); 	// check again if the car reached the next curve section
	}

	#pragma region section 12

	if(((section[0][0] == (outsideBorder ? RED : GREEN)) || ((section[0][0] == 3) && (section[0][1] == (outsideBorder ? RED : GREEN)))) || ((section[0][0] == 3) && (section[0][1] == 3) && (section[0][2] == (outsideBorder ? RED : GREEN)))) { // ckeck whether the car has to pass the first object of the section at the side of the outside border
		drivingSide = outsideBorder;
		while(vl53l1x.getDistance(FRONT) > 520) rotationCorrection(); 	// keep driving until the car passed the object
	}
	else drivingSide = !outsideBorder;
	curve(!outsideBorder);				// turn into the last section
	vTaskDelay(150);


#ifndef PARK_IN
// stop in the middle of the starting section

	// correct driving rotation and keep driving until reaching the middle of the starting section
	while((vl53l1x.getDistance(FRONT) > 1800) || (vl53l1x.getDistance(BACK) < 1200)) rotationCorrection();

#else
// perform non-parallel parking

	// drive slower while approaching parking space for non-parallel parking
	motor.setSpeed(0.3f);

	// different approach for the two different driving directions
	if(outsideBorder) {

		
		while((vl53l1x.getDistance(FRONT) > 1500) || (vl53l1x.getDistance(BACK) < 1500)) rotationCorrection();		// drive to the middle of the last section
		if(outsideBorder != drivingSide) {
			while((vl53l1x.getDistance(FRONT) > 1800) || (vl53l1x.getDistance(BACK) < 1200)) rotationCorrection();	// if the car is at the side of the inner border the car keeps driving until it passed the position in the middle of the section where a obstacle can be placed
		}
		uint16_t lastDistance = vl53l1x.getDistance(outsideBorder);
		while((lastDistance - vl53l1x.getDistance(outsideBorder)) < 100) rotationCorrection(); // the car drives until the measured distance to the outside border distinguishes the previous measurement by more than 100mm
		drivingSide = 2;
		motor.setSpeed(-0.3f);
		vTaskDelay(950);			// driving 950ms backward
		curve(outsideBorder);		// turn towards the parking space
		while(vl53l1x.getDistance(FRONT) > 15) rotationCorrection();	// the car drives into the parking space until it reached the outside border
		motor.setSpeed(0.0f);
	}
	else {

		// wait at least 0.5s before entering the parking space because of possible sensor latencies
		uint64_t wait_time = esp_timer_get_time();
		while(wait_time > (esp_timer_get_time() - 500000)) rotationCorrection();

		// keep correcting the rotation until the rear distance sensor reads more than 0.9m (determined experimentally)
		while(vl53l1x.getDistance(BACK) < 900) rotationCorrection();

		// drive the curve into the parking space
		curve(outsideBorder);

		// wait until positioned completely in the parking space
		while(vl53l1x.getDistance(FRONT) > 15) rotationCorrection();
	}

#endif

	// stop driving and wait until standstill
	motor.setSpeed(0.0f);
	while(supervision.getMotorCurrent() > 0) vTaskDelay(1);
	
	// show finish with total drive time
	lcd.showFinish(esp_timer_get_time() - startTime);
}

// wait until switched off or restarted
void loop() { vTaskDelay(1000000); }

#endif
#endif