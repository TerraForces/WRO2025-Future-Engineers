/**
 * main.cpp - pure driving code containing both challenge types and test modes
 * by TerraForce
 */

// optimize program for higher execution speed
#pragma GCC optimize("-Ofast")

//#define OBJECT_DETECTION
//#define TEST

#include "car.hpp"
using namespace CAR;

#ifdef TEST
#pragma region speed test

void setup() {

	// start MPU9250
	mpu9250.init();
	
	// start ToF sensors
	vl53l1x.init(true);

#ifdef OBJECT_DETECTION

	// wait for Raspberry Pi 5 to get ready
	while(!camera.ready()) vTaskDelay(1);

#endif

	// show ready screen
	lcd.showReady(esp_timer_get_time());

	// wait for button press
	button.wait();
}

void loop() {

#ifdef OBJECT_DETECTION

	// show objects periodically
	static TickType_t lastTicks = xTaskGetTickCount();
	lcd.showObjects();
	xTaskDelayUntil(&lastTicks, 2000);

#else

	// show calibration interface for each ToF sensor
	for(uint8_t i = 0; i < 4; i++) {
		lcd.showDistanceCalibration(i);
		button.wait();
	}

#endif

}

#else

#ifndef OBJECT_DETECTION
#pragma region without objects

constexpr float SPEED_START  = 1.0f;
constexpr float SPEED_NORMAL = 1.0f;
constexpr float SPEED_HIGH   = 1.0f;

bool outsideBorder = 0;
float targetRotation = 0.0f, rotationOffset = 0.0f;
int64_t startTime = 0, lastCurve = 0;
uint32_t intervalTimer = 0;

void setup() {

	// start MPU9250
	mpu9250.init();
	
	// start ToF sensors
	vl53l1x.init();

	// show ready screen
	lcd.showReady(esp_timer_get_time());

	// wait for button press
	button.wait();

	// capture start time
	startTime = esp_timer_get_time();

	// start driving forward
	motor.setSpeed(SPEED_START);

	// wait until open space is detected
	bool borderLeft = 1, borderRight = 1;
	while(borderLeft && borderRight) {
		borderLeft = vl53l1x.getDistance(LEFT) < 1500;
		borderRight = vl53l1x.getDistance(RIGHT) < 1500;
		vTaskDelay(1);
	}
	outsideBorder = borderRight;

	// increase driving speed
	motor.setSpeed(SPEED_NORMAL);

	// set current time for loop interval control
	intervalTimer = xTaskGetTickCount();
}

void loop() {

	// get current rotation
	MPU9250::FLOAT3 rotation = mpu9250.getRotation();
	rotation.z += rotationOffset;

	// get current distance measurements
	uint16_t distances[3] = {};
	for(uint8_t i = 0; i < 3; i++) distances[i] = vl53l1x.getDistance(i);

	// end condition
	if ((abs(targetRotation) >= 1080.0f) && (vl53l1x.getDistance(BACK) > 1000) && (esp_timer_get_time() > (lastCurve + 500000))) {
		motor.setSpeed(0.0f);

		// repeat until v=0
		while(supervision.getMotorCurrent() > 0) {

			// get current rotation
			MPU9250::FLOAT3 rotation = mpu9250.getRotation();
			rotation.z += rotationOffset;

			// get current distance measurements
			uint16_t distances[3] = {};
			for(uint8_t i = 0; i < 3; i++) distances[i] = vl53l1x.getDistance(i);

			// border correction
			if ((distances[LEFT] < 150) && (distances[FRONT] > 900)) {
				servo.setAngle(6.0f);
				vTaskDelay(200);
			}
			else if ((distances[RIGHT] < 150) && (distances[FRONT] > 900)) {
				servo.setAngle(-6.0f);
				vTaskDelay(200);
			}

			// gyro correction
			else if ((targetRotation - rotation.z) < -2.0f) servo.setAngle(max(targetRotation - rotation.z + 2.0f, -7.0f));
			else if ((targetRotation - rotation.z) >  2.0f) servo.setAngle(min(targetRotation - rotation.z - 2.0f,  7.0f));
			vTaskDelay(10);
		}

		// show finish screen and wait for shutdown
		lcd.showFinish(esp_timer_get_time() - startTime);
		while(true) vTaskDelay(10000000);
	}

	// increase driving speed when possible
	if(SPEED_NORMAL != SPEED_HIGH) {
		if ((distances[FRONT] > 1000) && (distances[LEFT] > 160) && (distances[RIGHT] > 160) && (abs(targetRotation - rotation.z) < 4.0f)) motor.setSpeed(SPEED_HIGH);
		else motor.setSpeed(SPEED_NORMAL);
	}

	// curve mode
	if (((((distances[LEFT] > 1100) || (distances[RIGHT] > 1100)) && (distances[FRONT] < 1100)) || (distances[FRONT] < 550)) && (esp_timer_get_time() > (lastCurve + 750000))) {
		leds.setBrightness(outsideBorder ? 15 : 0, outsideBorder ? 0 : 15);
		servo.setAngle(outsideBorder ? -20.0f : 20.0f);
		targetRotation += outsideBorder ? -55.0f : 55.0f;
		do {
			rotation = mpu9250.getRotation();
			rotation.z += rotationOffset;
			vTaskDelay(5);
		} while (outsideBorder ? (targetRotation < rotation.z) : (targetRotation > rotation.z));
		targetRotation += outsideBorder ? -35.0f : 35.0f;
		leds.setBrightness(0, 0);
		servo.setAngle(0.0f);
		motor.setSpeed(SPEED_NORMAL);
		lastCurve = esp_timer_get_time();
		if(abs(targetRotation) >= 1080.0f) vTaskDelay(500);
		if(!outsideBorder) rotationOffset += 1.5f;
	}

	// border correction
	else if ((distances[LEFT] < 150) && (distances[FRONT] > 900)) {
		servo.setAngle(((targetRotation - rotation.z) > -20.0f) ? 6.0f : 0.0f);
		vTaskDelay(50);
	}
	else if ((distances[RIGHT] < 150) && (distances[FRONT] > 900)) {
		servo.setAngle(((targetRotation - rotation.z) < 20.0f) ? -6.0f : 0.0f);
		vTaskDelay(50);
	}

	// gyro correction
	else if ((targetRotation - rotation.z) < -2.0f) servo.setAngle(max(targetRotation - rotation.z + 2.0f, -7.0f));
	else if ((targetRotation - rotation.z) >  2.0f) servo.setAngle(min(targetRotation - rotation.z - 2.0f,  7.0f));

	// straight driving
	else servo.setAngle(0.0f);
}

#else
#pragma region with objects

#define PARK_OUT
#define PARK_IN

constexpr float PARK_ANGLE    	 	= 45.0f;
constexpr float SPEED_PARK    	 	= 0.15f;
constexpr float SPEED_MIN1  	 	= 0.40f;
constexpr float SPEED_MAX1		 	= 0.50f;
constexpr float SPEED_MIN23 	 	= 0.50f;
constexpr float SPEED_MAX23		 	= 0.60f;
constexpr float SPEED_END		 	= 0.50f;
constexpr float MIN_RCR_ANGLE	 	= 1.00f;
constexpr float TRACK_CR_ANGLE	 	= 10.0f;
constexpr float TRACK_CR_STRENGTH 	= 1.00f;

bool outsideBorder = 0;
float targetRotation = 0.0f;
int32_t curveTargetRotation = 0;
int64_t startTime = 0;
float rotationOffset = 0.0f;
int64_t lastCurve = 0;
bool section[4] = {};
bool drivingSide = 0;
bool parkingSpace = 0;

void correctRotationOffset(bool side, uint64_t waitTicks = 500) {
	uint16_t frontDistance = vl53l1x.getDistance(FRONT);
	uint16_t sideDistance = vl53l1x.getDistance(side);
	vTaskDelay(waitTicks);
	rotationOffset = (atan((frontDistance - vl53l1x.getDistance(FRONT)) / ((float)(sideDistance - vl53l1x.getDistance(side)))) * 57.295779513f) - curveTargetRotation + mpu9250.getRotation().z;
	if(side) rotationOffset = -rotationOffset;
}

void curve(bool direction) {
	motor.setSpeed(0.45f);
	leds.setBrightness(direction ? 0 : 15, direction ? 15 : 0);
	servo.setAngle(direction ? 40.0f : -40.0f);
	curveTargetRotation += direction ? 90 : -90;
	targetRotation = curveTargetRotation;
	while (direction ? ((mpu9250.getRotation().z + rotationOffset) < (curveTargetRotation - 25.0f)) : ((mpu9250.getRotation().z + rotationOffset) > (curveTargetRotation + 25.0f))) vTaskDelay(1);
	leds.setBrightness(0, 0);
	servo.setAngle(0.0f);
}

void rotationCorrection(bool invert = false, bool offsetCorrection = false, bool endCorrection = false) {
	float rotation = mpu9250.getRotation().z + rotationOffset;
	if 		(targetRotation - rotation < MIN_RCR_ANGLE) servo.setAngle(invert ? -max(targetRotation - rotation + MIN_RCR_ANGLE, -10.0f) : max(targetRotation - rotation + MIN_RCR_ANGLE, -10.0f));
	else if (targetRotation - rotation > MIN_RCR_ANGLE) servo.setAngle(invert ? -min(targetRotation - rotation - MIN_RCR_ANGLE,  10.0f) : min(targetRotation - rotation - MIN_RCR_ANGLE,  10.0f));
	else if (vl53l1x.getDistance(FRONT) > 1500) {
		servo.setAngle(0.0f);
		vTaskDelay(10);
		correctRotationOffset(endCorrection ? drivingSide : outsideBorder, 400);
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

	// adjust offset
	rotationOffset = -mpu9250.getRotation().z;

#ifdef PARK_OUT

	// detect outside border
	outsideBorder = vl53l1x.getDistance(LEFT) > 300;

	// leave parking space
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
	while (outsideBorder ? ((mpu9250.getRotation().z + rotationOffset) > -80.0f) : ((mpu9250.getRotation().z + rotationOffset) < 80.0f)) vTaskDelay(1);
	curveTargetRotation = outsideBorder ? -90 : 90;
	targetRotation = curveTargetRotation;
	drivingSide = 1 - outsideBorder;
	servo.setAngle(0.0f);
	motor.setSpeed(SPEED_MIN1);
	while (vl53l1x.getDistance(FRONT) > 370) rotationCorrection();
	parkingSpace = vl53l1x.getDistance(outsideBorder) < 1500;
	curve(outsideBorder);

#else

	// start driving forward
	motor.setSpeed(SPEED_MIN1);

	// wait until open space is detected
	while((vl53l1x.getDistance(LEFT) < 1500) && (vl53l1x.getDistance(RIGHT) < 1500)) vTaskDelay(1);
	outsideBorder = vl53l1x.getDistance(RIGHT) < 1500;

#endif

}

void loop() {

	// 3 rounds completed
	if(abs(targetRotation) >= 1080) {
		motor.setSpeed(SPEED_MIN1);
		if(((vl53l1x.getDistance(FRONT) < 1800) || (vl53l1x.getDistance(BACK) > 1100)) && (esp_timer_get_time() > (lastCurve + 500000))) {
			leds.setBrightness(1, 1);

			// correct if wrong
			if(vl53l1x.getDistance(FRONT) < 1100) {
				motor.setSpeed(-SPEED_MIN1);
				while(vl53l1x.getDistance(FRONT) < 1100) vTaskDelay(1);
			}

			// stop car
			motor.setSpeed(0.0f);
			servo.setAngle(0.0f);
			vTaskDelay(600);

			// show finish screen
			lcd.showFinish(esp_timer_get_time() - startTime);
			
#ifdef PARK_IN

			// wait 3 seconds
			vTaskDelay(3000);

			// search parking space
			if((vl53l1x.getDistance(FRONT) < 1350) && parkingSpace) {
				motor.setSpeed(-0.2f);
				while (vl53l1x.getDistance(FRONT) < 1350) rotationCorrection(1);
			}
			else if((vl53l1x.getDistance(FRONT) > 1650) && (!parkingSpace)) {
				motor.setSpeed(0.2f);
				while (vl53l1x.getDistance(FRONT) > 1650) rotationCorrection();
			}

			// rotate for entering parking space
			rotationOffset = curveTargetRotation - mpu9250.getRotation().z;
			motor.setSpeed(parkingSpace ? SPEED_PARK : -SPEED_PARK);
			while(((vl53l1x.getDistance(LEFT) + vl53l1x.getDistance(RIGHT)) < 550) || ((vl53l1x.getDistance(LEFT) + vl53l1x.getDistance(RIGHT)) > 800) || (parkingSpace ? (vl53l1x.getDistance(FRONT) > 1400) : (vl53l1x.getDistance(FRONT) < 1500))) rotationCorrection(!parkingSpace);
			leds.setBrightness(15, 15);
			if (!parkingSpace) vTaskDelay(1000);
			
			// enter parking space
			leds.setBrightness(outsideBorder ? 0 : 15, outsideBorder ? 15 : 0);
			servo.setAngle(outsideBorder ? PARK_ANGLE : -PARK_ANGLE);
			curveTargetRotation += (parkingSpace == outsideBorder) ? 90 : -90;
			targetRotation = curveTargetRotation;
			while ((parkingSpace != outsideBorder) ? ((mpu9250.getRotation().z + rotationOffset) > (curveTargetRotation + 6.0f)) : ((mpu9250.getRotation().z + rotationOffset) < (curveTargetRotation - 6.0f))) vTaskDelay(1);
			leds.setBrightness(0, 0);
			servo.setAngle(0.0f);
			vTaskDelay(10000);
			//while ((vl53l1x.getDistance(FRONT + parkingSpace) < 790) && (vl53l1x.getDistance(BACK - parkingSpace) > 20)) rotationCorrection(!parkingSpace);
			//motor.setSpeed(0.0f);

#endif

			// end program
			while(true) vTaskDelay(1000);
		}
	}

	// speed control
	else if(abs(targetRotation) > 360.0f) motor.setSpeed((vl53l1x.getDistance(FRONT) > 1450) ? SPEED_MAX23 : SPEED_MIN23);
	else motor.setSpeed((vl53l1x.getDistance(FRONT) > 1400) ? SPEED_MAX1 : SPEED_MIN1);

	// get current rotation
	MPU9250::FLOAT3 rotation = mpu9250.getRotation();
	rotation.z += rotationOffset;

	// curve mode
	if (((vl53l1x.getDistance(FRONT) < 1100) && (vl53l1x.getDistance(1 - outsideBorder) > 1300)) && (esp_timer_get_time() > (lastCurve + 500000))) {
		if(abs(curveTargetRotation) < 360) {
			if((drivingSide != outsideBorder) && (((camera.getObject(0).c == (GREEN - outsideBorder)) /*&& ((!outsideBorder) ? (camera.getObject(0).l > 600) : (camera.getObject(0).r < 700))*/) || ((camera.getObject(1).c == (GREEN - outsideBorder))/* && ((!outsideBorder) ? (camera.getObject(1).l > 600) : (camera.getObject(1).r < 700))*/))) {
				section[(abs(curveTargetRotation) / 90) % 4] = 1;
				servo.setAngle(0.0f);
				vTaskDelay(100);
#if defined(PARK_IN) || defined(PARK_OUT)
				while(vl53l1x.getDistance(FRONT) > ((((abs(curveTargetRotation) / 90) % 4) == 3) ? 570 : 350)) vTaskDelay(1);
#else
				while(vl53l1x.getDistance(FRONT) > 350) vTaskDelay(1);
#endif
				drivingSide = outsideBorder;
			}
			else if(camera.getObject(0).c == outsideBorder + 1) drivingSide = 1 - outsideBorder;
			else {
				vTaskDelay(100);
				if((drivingSide != outsideBorder) && (((camera.getObject(0).c == (GREEN - outsideBorder)) && ((!outsideBorder) ? (camera.getObject(0).l > 700) : (camera.getObject(0).r < 600))) || ((camera.getObject(1).c == (GREEN - outsideBorder)) && ((!outsideBorder) ? (camera.getObject(1).l > 700) : (camera.getObject(1).r < 600))))) {
					section[(abs(curveTargetRotation) / 90) % 4] = 1;
					servo.setAngle(0.0f);
					vTaskDelay(100);
#if defined(PARK_IN) || defined(PARK_OUT)
					while(vl53l1x.getDistance(FRONT) > ((((abs(curveTargetRotation) / 90) % 4) == 3) ? 570 : 350)) vTaskDelay(1);
#else
					while(vl53l1x.getDistance(FRONT) > 350) vTaskDelay(1);
#endif
					drivingSide = outsideBorder;
				}

				else drivingSide = 1 - outsideBorder;
			}
			curve(!outsideBorder);
			if(drivingSide != outsideBorder) {
				for(uint8_t i = 0; i < 8; i++) {
					if((camera.getObject(0).c == (GREEN - outsideBorder)) && (outsideBorder ? (camera.getObject(0).l > 575) : (camera.getObject(0).r < 725))) {
						if(((abs(curveTargetRotation) / 90) % 4) == 0) {
							curve(outsideBorder);
							motor.setSpeed(-SPEED_MIN1);
							while(vl53l1x.getDistance(FRONT) < 550) vTaskDelay(1);
							motor.setSpeed(SPEED_MIN1);
							while(vl53l1x.getDistance(FRONT) > 550) vTaskDelay(1);
							curve(!outsideBorder);
							drivingSide = outsideBorder;
							section[((abs(curveTargetRotation) / 90) + 3) % 4] = 1;
							vTaskDelay(50);
						}
						else {
							curve(outsideBorder);
							while(vl53l1x.getDistance(FRONT) > 350) vTaskDelay(1);
							curve(!outsideBorder);
							drivingSide = outsideBorder;
							section[((abs(curveTargetRotation) / 90) + 3) % 4] = 1;
							vTaskDelay(50);
						}
						break;
					}
					vTaskDelay(50);
				}
			}
			lastCurve = esp_timer_get_time() + (section[((abs(curveTargetRotation) / 90) + 2) % 4] * 2000000);
		}
		else {
			motor.setSpeed(SPEED_MIN23);
			if(section[(abs(curveTargetRotation) / 90) % 4]) {
				servo.setAngle(0.0f);
#if defined(PARK_IN) || defined(PARK_OUT)
				while(vl53l1x.getDistance(FRONT) > ((((abs(curveTargetRotation) / 90) % 4) == 3) ? 610 : 410)) vTaskDelay(1);
#else
				while(vl53l1x.getDistance(FRONT) > 410) vTaskDelay(1);
#endif
				drivingSide = outsideBorder;
			}
			else drivingSide = 1 - outsideBorder;
			curve(!outsideBorder);
			lastCurve = esp_timer_get_time() + (section[((abs(curveTargetRotation) / 90) - 2) % 4] * 2000000);
		}
	}

	// rotation correction
	else rotationCorrection(false, true);

	// track correction
#if defined(PARK_IN) || defined(PARK_OUT)
	/*uint16_t trackDifference = vl53l1x.getDistance(drivingSide) - (((((abs(curveTargetRotation) / 90) % 4) == 0) && section[0]) ? 350 : 150);
	float correctionAngle = (drivingSide ? trackDifference : -trackDifference) * TRACK_CR_STRENGTH;
	targetRotation = constrain(correctionAngle, -TRACK_CR_ANGLE, TRACK_CR_ANGLE) + curveTargetRotation;*/
	if ((vl53l1x.getDistance(drivingSide) > (((((abs(curveTargetRotation) / 90) % 4) == 0) && section[0]) ? 370 : 170)) && (vl53l1x.getDistance(drivingSide) < 1200)) targetRotation = curveTargetRotation + (drivingSide ? TRACK_CR_ANGLE : -TRACK_CR_ANGLE);
	else if (vl53l1x.getDistance(drivingSide) < (((((abs(curveTargetRotation) / 90) % 4) == 0) && section[0]) ? 330 : 130)) targetRotation = curveTargetRotation + (drivingSide ? -TRACK_CR_ANGLE : TRACK_CR_ANGLE);
	else targetRotation = curveTargetRotation;
#else
	if ((vl53l1x.getDistance(drivingSide) > 170) && (vl53l1x.getDistance(drivingSide) < 1200)) targetRotation = curveTargetRotation + (drivingSide ? TRACK_CR_ANGLE : -TRACK_CR_ANGLE);
	else if (vl53l1x.getDistance(drivingSide) < 130) targetRotation = curveTargetRotation + (drivingSide ? -TRACK_CR_ANGLE : TRACK_CR_ANGLE);
	else targetRotation = curveTargetRotation;
#endif

}

#endif
#endif