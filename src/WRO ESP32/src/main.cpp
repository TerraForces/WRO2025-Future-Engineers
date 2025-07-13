/**
 * main.cpp - pure driving code containing both challenge types and test modes
 * by TerraForce
 */

// optimize program for higher execution speed
#pragma GCC optimize("-Ofast")

#define OBJECT_DETECTION
//#define TEST

#include "car.hpp"
using namespace CAR;

#ifdef TEST
#pragma region speed test

void setup() {

	// start MPU9250
	mpu9250.init();
	
	// start ToF sensors
	vl53l1x.init();

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
		vTaskDelay(200);
		button.wait();
	}

	// show calibrated distances
	lcd.showDistances();
	vTaskDelay(200);
	button.wait();
	
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
uint8_t section[4][3] = {};
bool drivingSide = 0;
bool parkingSpace = 0;

int distance;

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
	while (outsideBorder ? ((mpu9250.getRotation().z + rotationOffset) > -92.0f) : ((mpu9250.getRotation().z + rotationOffset) < 92.0f)) vTaskDelay(1);
	curveTargetRotation = outsideBorder ? -90 : 90;
	targetRotation = curveTargetRotation;
	drivingSide = 1 - outsideBorder;
	servo.setAngle(0.0f);

	if(outsideBorder) {
		motor.setSpeed(SPEED_MIN1);
		for (int i = 0; (camera.getObject(i).c != 0 ) && (section[0][2] == 0); i++) {
			if(camera.getObject(i).c != PINK) {
				if(camera.getObject(i).l > 650) {
					if(camera.getObject(i).b > 480) section[0][2] = camera.getObject(i).c;
				}
			}
		}
		if(!section[0][2]) section[0][2] = 3;
		if(section[0][2] == RED) {
			drivingSide = outsideBorder;
			while (vl53l1x.getDistance(FRONT) > 600) rotationCorrection();
		}
		else {
			drivingSide = !outsideBorder;
			while (vl53l1x.getDistance(FRONT) > 350) rotationCorrection();
		}
		curve(outsideBorder);
	}
	else {
		motor.setSpeed(0.0f);
		vTaskDelay(50);
		for (uint16_t i = 0; (camera.getObject(i).c != 0) && (section[0][0] == 0); i++) {
			if(camera.getObject(i).c != PINK) {
				if(camera.getObject(i).l > 650) {
					if(camera.getObject(i).b > 480) section[0][0] = camera.getObject(i).c;
				}
			}
		}
		motor.setSpeed(0.45f);
		leds.setBrightness(15, 0);
		servo.setAngle(-40.0f);
		curveTargetRotation -= 60;
		targetRotation = curveTargetRotation;
		while (((mpu9250.getRotation().z + rotationOffset) > (curveTargetRotation + 25.0f))) vTaskDelay(1);
		leds.setBrightness(0, 0);
		servo.setAngle(0.0f);
		motor.setSpeed(0.0f);
		vTaskDelay(100);
		motor.setSpeed(SPEED_MIN1);
		for (int i = 0; (camera.getObject(i).c != 0) && (section[0][1] == 0); i++) {
			if(camera.getObject(i).c != PINK) {
				if(camera.getObject(i).l < 650) {
					if(camera.getObject(i).b > 500) section[0][1] = camera.getObject(i).c;
					else if (camera.getObject(i).b > 420) section[0][2] = camera.getObject(i).c;
				}
			}
		}
		if(section[0][0] == 0) section[0][0] = 3;
		if(section[0][2] == 0) section[0][2] = 3;
		if(section[0][1] == 0) section[0][1] = 3;
		if((section[0][1] == GREEN) || ((section[0][2] == GREEN) && (section[0][1] == 3))) {
			drivingSide = outsideBorder;
			motor.setSpeed(0.45f);
			leds.setBrightness(15, 0);
			servo.setAngle(-40.0f);
			curveTargetRotation -= 30;
			targetRotation = curveTargetRotation;
			while (((mpu9250.getRotation().z + rotationOffset) > (curveTargetRotation + 5.0f))) vTaskDelay(1);
			leds.setBrightness(0, 0);
			servo.setAngle(0.0f);
		}
		else {
			drivingSide = !outsideBorder;
			motor.setSpeed(0.45f);
			leds.setBrightness(0, 15);
			servo.setAngle(20.0f);
			curveTargetRotation += 60;
			targetRotation = curveTargetRotation;
			while ((mpu9250.getRotation().z + rotationOffset) < (curveTargetRotation - 5.0f)) vTaskDelay(1);
			leds.setBrightness(0, 0);
			servo.setAngle(0.0f);
			while (vl53l1x.getDistance(FRONT) > 340) rotationCorrection();
			curve(outsideBorder);
		}
	}
	while (vl53l1x.getDistance(!outsideBorder) < 1500) rotationCorrection();

	motor.setSpeed(0.0f);
	lcd.showString(String(section[0][0]) + '\n' + String(section[0][1]) + '\n' + String(section[0][2]));












	for(int j = 1; j <= 3; j++) {
		if(drivingSide == outsideBorder) {
			while(vl53l1x.getDistance(FRONT) > 950) rotationCorrection();
			curve(!outsideBorder);
			motor.setSpeed(-0.35f);
			while(vl53l1x.getDistance(BACK) > 110) rotationCorrection(true);
			motor.setSpeed(0);
			
		}
		else {
			while(vl53l1x.getDistance(FRONT) > 500) rotationCorrection();
			motor.setSpeed(-SPEED_MIN1);
			while(vl53l1x.getDistance(FRONT) < 620) rotationCorrection();
			if(outsideBorder == 0){
				motor.setSpeed(-0.45f);
				leds.setBrightness(0,15);
				servo.setAngle(-40.0f);
				curveTargetRotation += 90;
				targetRotation = curveTargetRotation;
				while ((mpu9250.getRotation().z + rotationOffset) < (curveTargetRotation - 25.0f)) vTaskDelay(1);
			}
			else if(outsideBorder == 1){
				motor.setSpeed(-0.45f);
				leds.setBrightness(0,15);
				servo.setAngle(40.0f);
				curveTargetRotation += -90;
				targetRotation = curveTargetRotation;
				while ((mpu9250.getRotation().z + rotationOffset) > (curveTargetRotation + 25.0f)) vTaskDelay(1);
			}
			
			motor.setSpeed(-0.5f);
			while(vl53l1x.getDistance(BACK) > 110) rotationCorrection(true);
			motor.setSpeed(0);

		}

		drivingSide = !outsideBorder;


		for (int i = 0; (camera.getObject(i).c != 0 )&&(section[j][0] == 0); i++){
			if(camera.getObject(i).c != PINK){
				if(outsideBorder? camera.getObject(i).l > 650 : camera.getObject(i).l < 650){
					if(camera.getObject(i).b > 420) section[j][0] = camera.getObject(i).c;
				}
			}
		}
		if(section[j][0] == 0){
			section[j][0] = 3;
			for (int i = 0; (camera.getObject(i).c != 0 )&&(section[j][1] == 0); i++){
				if(camera.getObject(i).c != PINK){
					if(outsideBorder? camera.getObject(i).l > 650 : camera.getObject(i).l < 650){
						if(camera.getObject(i).b > 380) section[j][1] = camera.getObject(i).c;
					}
				}
			}
			if(section[j][1] == 0){
				section[j][1] = 3;
				for (int i = 0; (camera.getObject(i).c != 0 )&&(section[j][2] == 0); i++){
					if(camera.getObject(i).c != PINK){
						if(outsideBorder? camera.getObject(i).l > 650 : camera.getObject(i).l < 650){
							if(camera.getObject(i).b > 350) section[j][2] = camera.getObject(i).c;
						}
					}
				}	
				if(section[j][2]==0) section[j][2] = 3;
			}
			else {
				section[j][2] = 3;
				
			}
		}
		if(section[j][0] != 3){
			if((section[j][0] == (outsideBorder? RED : GREEN))){
				curve(outsideBorder);
				while(vl53l1x.getDistance(FRONT) > 370) rotationCorrection();
				curve(!outsideBorder);
				drivingSide = outsideBorder;
			}
			motor.setSpeed(0.5);
			uint64_t wait_time = esp_timer_get_time();
			while(wait_time > (esp_timer_get_time() - 1000000)) rotationCorrection;
			while(vl53l1x.getDistance(FRONT) > 1900|| vl53l1x.getDistance(BACK) < 1100) rotationCorrection();
			motor.setSpeed(0);
			vTaskDelay(100);
			for (int i = 0; (camera.getObject(i).c != 0 )&&(section[j][2] == 0); i++){
				if(camera.getObject(i).c != PINK){
					
					if(camera.getObject(i).b > 410) section[j][2] = camera.getObject(i).c;
					
				}
				
			}
			vTaskDelay(100);
			for (int i = 0; (camera.getObject(i).c != 0 )&&(section[j][2] == 0); i++){
				if(camera.getObject(i).c != PINK){
					
					if(camera.getObject(i).b > 410) section[j][2] = camera.getObject(i).c;
					
				}
				
			}
			if(section[j][1] == 0) section[j][1] = 3;
			if(section[j][2] == 0) section[j][2] = 3;
			if((section[j][2] == (!outsideBorder? RED : GREEN))&& drivingSide == outsideBorder){
				curve(!outsideBorder);
				while(vl53l1x.getDistance(FRONT) > 350) rotationCorrection();
				curve(outsideBorder);
				drivingSide = !outsideBorder;
				vTaskDelay(50);
			}
			if((section[j][2] == (outsideBorder? RED : GREEN))&& drivingSide != outsideBorder){
				curve(outsideBorder);
				while(vl53l1x.getDistance(FRONT) > 350) rotationCorrection();
				curve(!outsideBorder);
				drivingSide = outsideBorder;
				vTaskDelay(50);
				motor.setSpeed(0);
				vTaskDelay(200);
			}
			motor.setSpeed(0.5f);
			while(vl53l1x.getDistance(!outsideBorder) < 1100 || vl53l1x.getDistance(FRONT) > 1500) rotationCorrection();
		}
		else{
			if((section[j][1] == (outsideBorder? RED : GREEN))||((section[j][1]==3)&&(section[j][2] == (outsideBorder? RED : GREEN)))){
				curve(outsideBorder);
				while(vl53l1x.getDistance(FRONT) > 350) rotationCorrection();
				curve(!outsideBorder);
				drivingSide = outsideBorder;
				vTaskDelay(50);
			}
			motor.setSpeed(0.5);
			while(vl53l1x.getDistance(!outsideBorder) < 1100 || vl53l1x.getDistance(FRONT) > 1500) rotationCorrection();
			
		}
	}













	if(drivingSide == outsideBorder){
		while(vl53l1x.getDistance(FRONT) > 950) rotationCorrection();
		curve(!outsideBorder);
		motor.setSpeed(-0.35f);
		while(vl53l1x.getDistance(BACK) > 110) rotationCorrection(true);
		motor.setSpeed(0);
		
	}
	else{
		while(vl53l1x.getDistance(FRONT) > 500) rotationCorrection();
		motor.setSpeed(-SPEED_MIN1);
		while(vl53l1x.getDistance(FRONT) < 620) rotationCorrection();
		if(outsideBorder == 0){
			motor.setSpeed(-0.45f);
			leds.setBrightness(0,15);
			servo.setAngle(-40.0f);
			curveTargetRotation += 90;
			targetRotation = curveTargetRotation;
			while ((mpu9250.getRotation().z + rotationOffset) < (curveTargetRotation - 25.0f)) vTaskDelay(1);
		}
		else if(outsideBorder == 1){
			motor.setSpeed(-0.45f);
			leds.setBrightness(0,15);
			servo.setAngle(40.0f);
			curveTargetRotation += -90;
			targetRotation = curveTargetRotation;
			while ((mpu9250.getRotation().z + rotationOffset) > (curveTargetRotation + 25.0f)) vTaskDelay(1);
		}
		
		motor.setSpeed(-0.4f);
		while(vl53l1x.getDistance(BACK) > 110) rotationCorrection(true);
		motor.setSpeed(0);

	}

	drivingSide = !outsideBorder;

	int j = 1;
	if(outsideBorder == 1){
		

		

		for (int i = 0; (camera.getObject(i).c != 0 )&&(section[j][0] == 0); i++){
			if(camera.getObject(i).c != PINK){
				if(outsideBorder? camera.getObject(i).l > 650 : camera.getObject(i).l < 650){
					if(camera.getObject(i).b > 420) section[j][0] = camera.getObject(i).c;
				}
			}
		}
		if(section[j][0] == 0){
			section[j][0] = 3;
			for (int i = 0; (camera.getObject(i).c != 0 )&&(section[j][1] == 0); i++){
				if(camera.getObject(i).c != PINK){
					if(outsideBorder? camera.getObject(i).l > 650 : camera.getObject(i).l < 650){
						if(camera.getObject(i).b > 380) section[j][1] = camera.getObject(i).c;
					}
				}
			}
			if(section[j][1] == 0){
				section[j][1] = 3;
				for (int i = 0; (camera.getObject(i).c != 0 )&&(section[j][2] == 0); i++){
					if(camera.getObject(i).c != PINK){
						if(outsideBorder? camera.getObject(i).l > 650 : camera.getObject(i).l < 650){
							if(camera.getObject(i).b > 350) section[j][2] = camera.getObject(i).c;
						}
					}
				}	
				if(section[j][2]==0) section[j][2] = 3;
			}
			else {
				section[j][2] = 3;
				
			}
		}

	}

	if(section[j][0] != 3){
		if((section[j][0] == (outsideBorder? RED : GREEN))){
			curve(outsideBorder);
			while(vl53l1x.getDistance(FRONT) > 570) rotationCorrection();
			curve(!outsideBorder);
			drivingSide = outsideBorder;
		}
		motor.setSpeed(0.5);
		uint64_t wait_time = esp_timer_get_time();
		while(wait_time > (esp_timer_get_time() - 1000000)) rotationCorrection;
		while(vl53l1x.getDistance(FRONT) > 1900|| vl53l1x.getDistance(BACK) < 1100) rotationCorrection();
		motor.setSpeed(0);
		vTaskDelay(100);
		
		if((section[j][2] == (!outsideBorder? RED : GREEN))&& drivingSide == outsideBorder){
			curve(!outsideBorder);
			while(vl53l1x.getDistance(FRONT) > 370) rotationCorrection();
			curve(outsideBorder);
			drivingSide = !outsideBorder;
			vTaskDelay(50);
		}
		if((section[j][2] == (outsideBorder? RED : GREEN))&& drivingSide != outsideBorder){
			curve(outsideBorder);
			while(vl53l1x.getDistance(FRONT) > 570) rotationCorrection();
			curve(!outsideBorder);
			drivingSide = outsideBorder;
			vTaskDelay(50);
			motor.setSpeed(0);
			vTaskDelay(200);
		}
		motor.setSpeed(0.5f);
		while(vl53l1x.getDistance(!outsideBorder) < 1100 || vl53l1x.getDistance(FRONT) > 1500) rotationCorrection();
	}
	else{
		if((section[j][1] == (outsideBorder? RED : GREEN))||((section[j][1]==3)&&(section[j][2] == (outsideBorder? RED : GREEN)))){
			curve(outsideBorder);
			while(vl53l1x.getDistance(FRONT) > 570) rotationCorrection();
			curve(!outsideBorder);
			drivingSide = outsideBorder;
			vTaskDelay(50);
		}
		motor.setSpeed(0.5f);
		while(vl53l1x.getDistance(!outsideBorder) < 1100 || vl53l1x.getDistance(FRONT) > 1500) rotationCorrection();
		
	}
	//motor.setSpeed(0);










	

	

	for(int k = 1; k <= 7; k++ ){
		if(((section[k % 4][0] == (outsideBorder? RED : GREEN)) || ((section[k % 4][0] == 3)&&(section[k % 4][1] == (outsideBorder? RED : GREEN)))) || ((section[k % 4][0] == 3)&&(section[k % 4][1] == 3 )&&(section[k % 4][2] == (outsideBorder? RED : GREEN)))) {
			drivingSide = outsideBorder;
			//motor.setSpeed(0);
			while(vl53l1x.getDistance(FRONT) > (((k % 4) == 0)? 570 : 360)) rotationCorrection();
		}
		else drivingSide = !outsideBorder;
		//motor.setSpeed(0);
		//vTaskDelay(1000);
		curve(!outsideBorder);
		vTaskDelay(150);
		while((vl53l1x.getDistance(FRONT) > 1800) || (vl53l1x.getDistance(BACK) < 1200)) rotationCorrection(); ////////////////////////////////////////////////////////////
		//motor.setSpeed(0);
		//vTaskDelay(1000);
		if(section[k % 4][0] != 3 && section[k % 4][0] != section[k % 4][2]){
			if((section[k % 4][2] == (outsideBorder? RED : GREEN))&&(drivingSide!= outsideBorder)){
				curve(outsideBorder);
				while(vl53l1x.getDistance(FRONT) > (((k % 4) == 0)? 570 : 360)) rotationCorrection();
				curve(!outsideBorder);
				drivingSide = outsideBorder;
			}
			else if((section[k % 4][2] == (!outsideBorder? RED : GREEN))&&(drivingSide == outsideBorder)){
				curve(!outsideBorder);
				while(vl53l1x.getDistance(FRONT) > 370) rotationCorrection();
				curve(outsideBorder);
				drivingSide = !outsideBorder;
			}
		}
		/*else{
			distance = vl53l1x.getDistance(drivingSide);
			servo.setAngle(0);
			while (vl53l1x.getDistance(FRONT) > 1100) vTaskDelay(1);
			rotationOffset += (((distance- vl53l1x.getDistance(drivingSide))> 10 )? -10 : (((distance- vl53l1x.getDistance(drivingSide))> 10 )? 10 : 0));
			
		}*/
		//motor.setSpeed(0);
		vTaskDelay(200);
		while((vl53l1x.getDistance(!outsideBorder) < 1100)||(vl53l1x.getDistance(FRONT) > 1500)) rotationCorrection();
		
	
	}

	//motor.setSpeed(0);






	if(((section[0][0] == (outsideBorder? RED : GREEN)) || ((section[0][0] == 3)&&(section[0][1] == (outsideBorder? RED : GREEN)))) || ((section[0][0] == 3)&&(section[0][1] == 3 )&&(section[0][2] == (outsideBorder? RED : GREEN)))) {
		drivingSide = outsideBorder;
		//motor.setSpeed(0);
		while(vl53l1x.getDistance(FRONT) > (570)) rotationCorrection();
	}
	else drivingSide = !outsideBorder;
	//motor.setSpeed(0);
	//vTaskDelay(1000);
	curve(!outsideBorder);
	vTaskDelay(150);
	while((vl53l1x.getDistance(FRONT) > 1800) || (vl53l1x.getDistance(BACK) < 1200)) rotationCorrection();
	motor.setSpeed(0);

	
	

#endif

}

void loop() {


}

#endif
#endif