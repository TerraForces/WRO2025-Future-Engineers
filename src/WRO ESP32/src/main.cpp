/**
 * main.cpp - pure driving code containing both challenge types and test modes
 * by TerraForce
 */

// optimize program for higher execution speed
#pragma GCC optimize("-Ofast")

// program selection definations
//#define OBJECT_DETECTION
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
// first challenge without objects but with variable width of the inner border
#pragma region without objects

// driving speed selection (recommended 0.2f - 1.0f)
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
	// or skip this step if one border is really close (indicating a start inside of a narrowed section)
	bool borderLeft = vl53l1x.getDistance(LEFT) > 100;
	bool borderRight = vl53l1x.getDistance(RIGHT) > 100;
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
constexpr float MIN_RCR_ANGLE	 	= 0.20f;
constexpr float TRACK_CR_ANGLE	 	= 10.0f;
constexpr float TRACK_CR_STRENGTH 	= 1.00f;

bool outsideBorder = 0;
float targetRotation = 0.0f;
int32_t curveTargetRotation = 0;
int64_t startTime = 0;
float rotationOffset = 0.0f;
int64_t lastCurve = 0;
uint8_t section[4][3] = {};
int8_t drivingSide = 2;
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
	motor.setSpeed(0.3f);
	leds.setBrightness(direction ? 0 : 15, direction ? 15 : 0);
	servo.setAngle(direction ? 40.0f : -40.0f);
	curveTargetRotation += direction ? 90 : -90;
	targetRotation = curveTargetRotation;
	while (direction ? ((mpu9250.getRotation().z + rotationOffset) < (curveTargetRotation - 25.0f)) : ((mpu9250.getRotation().z + rotationOffset) > (curveTargetRotation + 25.0f))) vTaskDelay(1);
	leds.setBrightness(0, 0);
	servo.setAngle(0.0f);
	motor.setSpeed(0.5f);
}

void rotationCorrection(bool invert = false, bool offsetCorrection = false, bool endCorrection = false) {
	float rotation = mpu9250.getRotation().z + rotationOffset;
	if (targetRotation - rotation < MIN_RCR_ANGLE) servo.setAngle(invert ? -max(targetRotation - rotation + MIN_RCR_ANGLE, -10.0f) : max(targetRotation - rotation + MIN_RCR_ANGLE, -10.0f));
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
		if(section[0][2] == 0) section[0][2] = 3;
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
					else if (camera.getObject(i).b > 440) section[0][2] = camera.getObject(i).c;
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

	/*motor.setSpeed(0.0f);
	*/lcd.showString(String(section[0][0]) + '\n' + String(section[0][1]) + '\n' + String(section[0][2]));

	/*while(true);*/












	for(int j = 1; j <= 3; j++) {
		if(drivingSide == outsideBorder) {
			while(vl53l1x.getDistance(FRONT) > 975) rotationCorrection();
			curve(!outsideBorder);
			motor.setSpeed(-0.35f);
			while(vl53l1x.getDistance(BACK) > 110) rotationCorrection(true);
			motor.setSpeed(0);
			
		}
		else {
			while(vl53l1x.getDistance(FRONT) > 500) rotationCorrection();
			motor.setSpeed(-SPEED_MIN1);
			while(vl53l1x.getDistance(FRONT) < 650) rotationCorrection();
			if(outsideBorder == 0){
				motor.setSpeed(-0.3f);
				leds.setBrightness(0,15);
				servo.setAngle(-40.0f);
				curveTargetRotation += 90;
				targetRotation = curveTargetRotation;
				while ((mpu9250.getRotation().z + rotationOffset) < (curveTargetRotation - 25.0f)) vTaskDelay(1);
			}
			else if(outsideBorder == 1){
				motor.setSpeed(-0.3f);
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
				while(vl53l1x.getDistance(FRONT) > 340) rotationCorrection();
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
		while(vl53l1x.getDistance(FRONT) > 975) rotationCorrection();
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
			motor.setSpeed(-0.3f);
			leds.setBrightness(0,15);
			servo.setAngle(-40.0f);
			curveTargetRotation += 90;
			targetRotation = curveTargetRotation;
			while ((mpu9250.getRotation().z + rotationOffset) < (curveTargetRotation - 25.0f)) vTaskDelay(1);
		}
		else if(outsideBorder == 1){
			motor.setSpeed(-0.3f);
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

	int j = 0;
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
			while(vl53l1x.getDistance(FRONT) > 520) rotationCorrection();
			curve(!outsideBorder);
			drivingSide = outsideBorder;
		}
		motor.setSpeed(0.5);
		uint64_t wait_time = esp_timer_get_time();
		while(wait_time > (esp_timer_get_time() - 1000000)) rotationCorrection;
		while(vl53l1x.getDistance(FRONT) > 1900|| vl53l1x.getDistance(BACK) < 1100) rotationCorrection();
		motor.setSpeed(0);
		vTaskDelay(1000);
		//lcd.showString(String(section[0][0]) + '\n' + String(section[0][1]) + '\n' + String(section[0][2]));
		
		if((section[j][2] == (!outsideBorder? RED : GREEN))&& drivingSide == outsideBorder){
			curve(!outsideBorder);
			while(vl53l1x.getDistance(FRONT) > 370) rotationCorrection();
			curve(outsideBorder);
			drivingSide = !outsideBorder;
			vTaskDelay(50);
		}
		if((section[j][2] == (outsideBorder? RED : GREEN))&& drivingSide != outsideBorder){
			curve(outsideBorder);
			while(vl53l1x.getDistance(FRONT) > 520) rotationCorrection();
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
			while(vl53l1x.getDistance(FRONT) > 520) rotationCorrection();
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
			while(vl53l1x.getDistance(FRONT) > (((k % 4) == 0)? 520 : 300)) rotationCorrection();
		}
		else drivingSide = !outsideBorder;
		//motor.setSpeed(0);
		//vTaskDelay(1000);
		curve(!outsideBorder);
		int64_t t = esp_timer_get_time() + 1000000;
		while(esp_timer_get_time() < t) rotationCorrection();
		while((vl53l1x.getDistance(FRONT) > 1800) || (vl53l1x.getDistance(BACK) < 1200)) rotationCorrection(); ////////////////////////////////////////////////////////////
		//motor.setSpeed(0);
		//vTaskDelay(1000);
		if(section[k % 4][0] != 3 && section[k % 4][0] != section[k % 4][2]){
			if((section[k % 4][2] == (outsideBorder? RED : GREEN))&&(drivingSide!= outsideBorder)){
				curve(outsideBorder);
				while(vl53l1x.getDistance(FRONT) > (((k % 4) == 0)? 520 : 360)) rotationCorrection();
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
		vTaskDelay(100);
		while((vl53l1x.getDistance(!outsideBorder) < 1000)||(vl53l1x.getDistance(FRONT) > 1500)||(vl53l1x.getDistance(BACK) < 800)) rotationCorrection();
		vTaskDelay(100);
		while((vl53l1x.getDistance(!outsideBorder) < 1000)||(vl53l1x.getDistance(FRONT) > 1500)||(vl53l1x.getDistance(BACK) < 800)) rotationCorrection();
		
	
	}

	//motor.setSpeed(0);






	if(((section[0][0] == (outsideBorder? RED : GREEN)) || ((section[0][0] == 3)&&(section[0][1] == (outsideBorder? RED : GREEN)))) || ((section[0][0] == 3)&&(section[0][1] == 3 )&&(section[0][2] == (outsideBorder? RED : GREEN)))) {
		drivingSide = outsideBorder;
		//motor.setSpeed(0);
		while(vl53l1x.getDistance(FRONT) > (520)) rotationCorrection();
	}
	else drivingSide = !outsideBorder;
	//motor.setSpeed(0);
	//vTaskDelay(1000);
	curve(!outsideBorder);
	vTaskDelay(150);




	/*while((vl53l1x.getDistance(FRONT) > 1800) || (vl53l1x.getDistance(BACK) < 1200)) rotationCorrection();
	motor.setSpeed(0);*/


	/*if(!outsideBorder){
		while(vl53l1x.getDistance(outsideBorder) > 900 || vl53l1x.getDistance(BACK) <1000)rotationCorrection();
		curve(outsideBorder);
		while(vl53l1x.getDistance(FRONT) > 15) rotationCorrection();
		motor.setSpeed(0);
	}
	else{
		motor.setSpeed(0.35f);
		while(vl53l1x.getDistance(FRONT) > 1200) rotationCorrection();

		curve(outsideBorder);
		while(vl53l1x.getDistance(FRONT) > 15) rotationCorrection();
		motor.setSpeed(0);
	}*/

	motor.setSpeed(0.3);
	if(outsideBorder){
		while(vl53l1x.getDistance(FRONT) > 1500 || vl53l1x.getDistance(BACK) < 1500) rotationCorrection();
		if(outsideBorder == drivingSide){
			int16_t lastdistance = vl53l1x.getDistance(outsideBorder);
			while(lastdistance - vl53l1x.getDistance(outsideBorder) < 100) rotationCorrection();
			drivingSide = 2;
			motor.setSpeed(-0.3);
			vTaskDelay(950);
			curve(outsideBorder);
			while(vl53l1x.getDistance(FRONT) > 15) rotationCorrection();
			motor.setSpeed(0);
		}
		else{
			while((vl53l1x.getDistance(FRONT) > 1800) || (vl53l1x.getDistance(BACK) < 1200)) rotationCorrection();
			uint16_t lastdistance = vl53l1x.getDistance(outsideBorder);
			while(lastdistance - vl53l1x.getDistance(outsideBorder)  <  100) rotationCorrection();
			drivingSide = 2;
			motor.setSpeed(-0.3);
			vTaskDelay(950);
			curve(outsideBorder);
			while(vl53l1x.getDistance(FRONT) > 15) rotationCorrection();
			motor.setSpeed(0);
		}
	}
	else {
		while(vl53l1x.getDistance(BACK) < 850) rotationCorrection();
		curve(outsideBorder);
		while(vl53l1x.getDistance(FRONT) > 15) rotationCorrection();
			motor.setSpeed(0);
	}
	
	lcd.showFinish(esp_timer_get_time() - startTime);

#endif

}

void loop() {}

#endif
#endif