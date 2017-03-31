/*
 * driving.c
 *
 *  Created on: Feb 23, 2017
 *      Author: pirzi
 */

#include "serial_communication.h"
#include "motor.h"
#include "FRTOS1.h"
#include "VL6180X.h"
#include "LED_GREEN.h"
#include "RED.h"
#include "L3G.h"
#include <math.h>
#include <stdlib.h>
#include "pid.h"
#include "driving.h"
#include "PWM_Gyro.h"
#include "PWM_Servo.h"

#define DELAY 20	// in ms

static uint8_t kpToF, kiToF, kdToF;			// PID values ToF
static uint8_t kpGyro, kiGyro, kdGyro;		//PID values Gyro
static int8_t factor;						// inverses the correction direction
static uint8_t device;						// left or right tof
static uint8_t distanceSide = 140;			// distance of the tofs to the side wall
static uint8_t distanceFront = 200;
static int8_t speed = 40;
static int16_t angel = 0;
static uint8_t letter;
static uint8_t stop;						// if set to 1 the motors will stop
static int8_t parcour;						// -1 = LeftParcour, 1 = RightParcour

/*
 * Drive to the stair until the front ToF detects the stair
 */
void driveToStair(void) {
	uint8_t err = ERR_OK;
	uint8_t done = 0;
	int16_t range = 0;

	motorsStartup(speed, speed, 50);
	LED_GREEN_Put(1);
	
	while (!done) {
		// toggel device to change mesurement side
		if(device == TOFRIGHT){
			device = TOFLEFT;
		}else if ( device == TOFLEFT){
			device = TOFRIGHT;
		}
		
		err = regulateMotor(speed, distanceSide, angel);
		if(err != ERR_OK){
			setErrorState(err, "DrivingTask, State1");
		}
		
		vTaskDelay(pdMS_TO_TICKS(DELAY) );
		
		// check if stair is detected
		VL_GetDistance(TOFFRONT, &range);
		if(err != ERR_OK){
			setErrorState(err, "DrivingTask, State1");
		}
		if (range <= distanceFront && range > 0) {
			LED_GREEN_Put(0);
			RED_Put(1);
			done = 1;
		}
	}
	RED_Put(0);
	setState(2);
}

/*
 * drive over the stair until the gyro/accel detects, that the car is on the flat ground after the stair
 */
void driveOverStair(void){
	uint8_t err = ERR_OK;
	uint8_t stairState = 0;
	int16_t angelNick = 0;
	uint8_t done = 0;
	uint8_t counter = 0;

	while(!done){
		// toggel device to change mesurement side
		if(device == TOFRIGHT){
			device = TOFLEFT;
		}else if ( device == TOFLEFT){
			device = TOFRIGHT;
		}
		
		err = regulateMotor(speed, distanceSide, angel);
		if(err != ERR_OK){
			setErrorState(err, "DrivingTask, State2");
		}
		
		vTaskDelay(pdMS_TO_TICKS(DELAY));
		
		err = L3GgetDegree(NICK, &angelNick);
		if(err != ERR_OK){
			setErrorState(err, "DrivingTask, State2");
		}
		// stair upwards
		if(stairState == 0 && angelNick <= -20){
			counter++;
			if(counter >= 2){
				counter = 0;
				stairState = 1;	
				LED_GREEN_Put(1);
			}
		}
		// on top of the stair
		else if(stairState == 1 && angelNick < 5 && angelNick > -5){
			counter++;
			if(counter >= 2){
				counter = 0;
				stairState = 2;	
				RED_Put(1);
				speed -= 20;
			}
		}
		// stair downwards
		else if(stairState == 2 && angelNick >= 20){
			counter++;
			if(counter >= 2){
				counter = 0;
				stairState = 3; 
				LED_GREEN_Put(0);
			}
		}
		// stair done
		else if(stairState == 3 && angelNick < 5 && angelNick > -5){
			counter++;
			if(counter >= 2){
				counter = 0;
				RED_Put(0);
				speed += 20;
				done = 1;
			}
		}
	}
	setState(3);
}

/*
 * drive to the turningpoint, if the left and right ToF don't detect anything, the turningpoint is reached
 */
void driveToTurningPlace(void){
	uint8_t err = ERR_OK;
	uint8_t done = 0;
	int16_t ToFLeft;
	int16_t ToFRight;
	
	while(!done){
		// toggel device to change mesurement side
		if(device == TOFRIGHT){
			device = TOFLEFT;
		}else if ( device == TOFLEFT){
			device = TOFRIGHT;
		}
		
		err = regulateMotor(speed, distanceSide, angel);
		if (err != ERR_OK) {
			setErrorState(err, "DrivingTask, State3");
		}
		
		vTaskDelay(pdMS_TO_TICKS(DELAY));
		
		// stop condition
		err = VL_GetDistance(TOFRIGHT, &ToFRight);
		if (err != ERR_OK) {
			setErrorState(err, "DrivingTask, State3");
		}
		err = VL_GetDistance(TOFLEFT, &ToFLeft);
		if (err != ERR_OK) {
			setErrorState(err, "DrivingTask, State3");
		}
		
		if((ToFLeft == 255 || ToFLeft <= 0) && (ToFRight == 255 || ToFRight <= 0)){
			done = 1;
		}
	}
	setState(4);
}

/*
 * drive through the turningpoint with two 90 degree turns
 */
void driveThroughTurningPlace(void){
	int16_t range = 0, rangeOld = 0;
	uint8_t err = ERR_OK;
	uint8_t partState = 0;

	
	
	if(parcour == -1){			// left parcour
		device = TOFLEFT;
	}else if (parcour == 1){	// right parcour
		device = TOFRIGHT;	
	}
	
	// turn 90 degree
	while(partState == 0){
		angel -= 2*parcour;
		err = regulateMotor(speed-20, 190, angel);
		if (err != ERR_OK) {
			setErrorState(err, "DrivingTask, State4");
		}
		
		 vTaskDelay(pdMS_TO_TICKS(DELAY));
		 
		 // check end condition
		if(angel == -90*parcour){
			partState = 2;
			angel = -90*parcour;
		}
	}
	
	// drive to the wall
	while(partState == 1){
		LED_GREEN_Put(1);
		err = regulateMotor(speed-20, 190, angel);
		if (err != ERR_OK) {
			setErrorState(err, "DrivingTask, State4");
		}

		vTaskDelay(pdMS_TO_TICKS(DELAY));
		
		// check end condition
		err = VL_GetDistance(TOFFRONT, &range);
		if (err != ERR_OK) {
			setErrorState(err, "DrivingTask, State4");
		}
		if(range <= distanceFront && range > 0){
			LED_GREEN_Put(0);
			partState = 3;
		}
	}
	if(parcour == -1){	// left parcour
		device = TOFRIGHT;
	}else if(device == 1){ // right parcour
		device = TOFLEFT;
	}
	// turn 90 degree
	while(partState == 2){
		RED_Put(1);
		angel -= 2*parcour;
		err = regulateMotor(0, distanceSide, angel);
		if (err != ERR_OK) {
			setErrorState(err, "DrivingTask, State4");
		}
		
		vTaskDelay(pdMS_TO_TICKS(DELAY));	 
			 
		// check end condition
		if(angel == -180*parcour){
			angel = -180*parcour;
			RED_Put(0);
			partState = 4;
		}
	}
	
	//drive to the door
	range = 0;
	while(partState == 3){
		LED_GREEN_Put(1);
		err = regulateMotor(speed-20, distanceSide, angel);
		if (err != ERR_OK) {
			setErrorState(err, "DrivingTask, State4");
		}
		
		vTaskDelay(pdMS_TO_TICKS(DELAY));
		
		//end condition
		rangeOld = range;
		err = VL_GetDistance(device, &range);
		if (err != ERR_OK) {
			setErrorState(err, "DrivingTask, State4");
		}
		if((rangeOld-range) >= 40 && (rangeOld-range)<=60){
			partState = 5;
		}
	}
	// drive through the door
	range = 0;
	while(partState == 4){
		RED_Put(1);
		err = regulateMotor(speed, distanceSide-50, angel);
		if (err != ERR_OK) {
			setErrorState(err, "DrivingTask, State4");
		}
		
		vTaskDelay(pdMS_TO_TICKS(DELAY));
		
		//end condition
		rangeOld = range;
		err = VL_GetDistance(device, &range);
		if (err != ERR_OK) {
			setErrorState(err, "DrivingTask, State4");
		}
		if((range-rangeOld) >= 40 && (rangeOld-range)<=60){
			RED_Put(0);
			LED_GREEN_Put(0);
			partState = 6;
		}		
	}
		setState(5);
}

/*
 * drive to the endzone, if the wall of the buttons is detected with the front ToF, the frontzone is reached
 */
void driveToEndZone(void){
	uint8_t err = ERR_OK;
	uint8_t done = 0;
	int16_t rangeFront, rangeSide;
	int8_t deviceIn;
	
	while(!done){
		// toggel device to change measurement side
		if(device == TOFRIGHT){
			device = TOFLEFT;
		}else if ( device == TOFLEFT){
			device = TOFRIGHT;
		}
		
		err = regulateMotor(speed, distanceSide, angel);
		if (err != ERR_OK) {
			setErrorState(err, "DrivingTask, State4");
		}
		
		vTaskDelay(pdMS_TO_TICKS(DELAY));
		
		 // stop condition
		 err = VL_GetDistance(TOFFRONT, &rangeFront);
		if (err != ERR_OK) {
			setErrorState(err, "DrivingTask, State4");
		}
		if(parcour == LEFT){
			deviceIn = TOFLEFT;
		}else if (parcour == RIGHT){
			deviceIn = TOFRIGHT;
		}
		err = VL_GetDistance(deviceIn, &rangeSide);
		if ( err != ERR_OK){
			setErrorState(err, "DrivingTask, State4");
		}
		 if(rangeFront <= distanceFront && rangeFront > 0 && (rangeSide<=0 || rangeSide == 255)){
			 done = 1;
		 }
		
	}
	
	motorsStartup(0, 0, 0);
	setState(6);
}

/*
 * push the right button in the endzone
 */
void pushTheButton(void){
	//\todo
}

/*
 * regulate the motors 
 * @param speed the speed with whitch to drive (PWM)
 * @param optRange optimal Range of the side ToF
 * @param optAngel optimal Angel of the gyro
 */
uint8_t regulateMotor(int8_t speed, uint8_t optRange,int16_t optAngel){
	uint8_t err = ERR_OK;
	int16_t corrGyro = 0;
	int16_t corrToF = 0;
	if (stop) {
		motorsStartup(0, 0, 0);
		return ERR_OK;
	} else {
		if(device == TOFRIGHT){
			factor = -1;
		}else if(device == TOFLEFT){
			factor = 1;
		}
		// Motor-Regulation
		err = calcPID(GYRO, kpGyro, kiGyro, kdGyro, optAngel, &corrGyro);
		if (err != ERR_OK) {
			corrGyro = 0;
			// error 
		}
		if (optRange != 0) {
			err = calcPID(device, kpToF, kiToF, kdToF, optRange, &corrToF);
			if (err != ERR_OK) {
				corrToF = 0;
				// error
			}
		}
		motorsStartup(speed - corrGyro + factor * corrToF, speed + corrGyro - factor * corrToF, 0);
		return err;
	}
}


/*
 * initialice the Driving Source with the PID values of the ToFs and the Gyro
 * @param kpT proportional factor of the ToF
 * @param kiT integral factor of the ToF
 * @param kdT differential factor of the ToF
 * @param kpG proportional factor of the Gyro
 * @param kiG integral factor of the Gyro
 * @param kdG differential factor of the Gyro
 * @param leftParcour sets the parcours deriction: true-> left parcours, false-> right parcours
 */
void initDriving(uint8_t kpT, uint8_t kiT, uint8_t kdT, uint8_t kpG, uint8_t kiG, uint8_t kdG, bool leftParcour){
	if(leftParcour){
		device = TOFRIGHT;
		parcour = LEFT;
	}
	else{
		device = TOFLEFT;
		parcour = RIGHT;
	}
	setPID(kpT, kiT, kdT, kpG, kiG, kdG);
}

/*
 * sets new PID factors for the ToFs and Gyro sensor
 * @param kpT proportional factor of the ToF
 * @param kiT integral factor of the ToF
 * @param kdT differential factor of the ToF
 * @param kpG proportional factor of the Gyro
 * @param kiG integral factor of the Gyro
 * @param kdG differential factor of the Gyro
 * 
 */
void setPID(uint8_t kpT, uint8_t kiT, uint8_t kdT, uint8_t kpG, uint8_t kiG, uint8_t kdG){
	kpToF = kpT;
	kiToF = kiT;
	kdToF = kdT;
	kpGyro = kpG;
	kiGyro = kiG;
	kdGyro = kdG;
}

/*
 * set the opimal side Distance of the parcours
 * @param value Distance is set as optimal side Distance
 */
void setDistanceSide(int16_t value){
	distanceSide = value;
}

/*
 * set the optimal front Distance to stop in front of a stair or a wall
 * @param value Distance is set as front Distance
 */
void setDistanceFront(int16_t value){
	distanceFront = value;
}

/*
 * set the optimal speed of the Car 
 * @ value speed of the Motors from -127...0...127
 */
void setSpeed(int8_t value){
	speed = value;
}

/*
 * set the optimal speed of the Gyroskop 
 * @ value speed of the Motor from 0...127
 */
void setGyroskopPWM(uint8_t value){
	PWM_Gyro_SetRatio16((0xFFFF-1)/127*value);
}

/*
 * Set the right number to push in the end-Zone
 * @param value letter from 1-5
 */
void setLetter(uint8_t value){
	letter = value;
}

/*
 * set the optimal angel of the Servo 
 * @ value angel of the Servo from 0...127
 */
void setServoPWM(uint8_t value){
	PWM_Servo_SetRatio16((0xFFFF-1)/127*value);
}

/*
 * run this method to stop the motors of the car
 */
void stopDriving(void){
	stop = 1;
}
