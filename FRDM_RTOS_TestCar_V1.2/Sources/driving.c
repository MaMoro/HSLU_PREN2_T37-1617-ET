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

#define PI 3.14159265
#define DELAY 15	// in ms

static uint8_t kpToF, kiToF, kdToF;			// PID values ToF
static uint8_t kpGyro, kiGyro, kdGyro;		//PID values Gyro
static int8_t factor;						// inverses the correction direction
static uint8_t device;						// left or right tof
static uint8_t distanceSide = 150;
static uint8_t distanceFront = 200;
static int8_t speed = 50;
static int16_t angel = 0;
static uint8_t letter;



void driveToStair(void) {
	uint8_t err = ERR_OK;
	uint8_t done = 0;
	int16_t range = 0;

	motorsStartup(speed, speed, 50);
	LED_GREEN_Put(1);
	while (!done) {
		err = regulateMotor(speed, distanceSide, angel);
		if(err != ERR_OK){
			setErrorState(err, "DrivingTask, State1");
		}
		//refreshMovingOffset('x');

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
		vTaskDelay(pdMS_TO_TICKS(DELAY) );
	}
	RED_Put(0);
	setState(2);
}

void driveOverStair(void){
	uint8_t err = ERR_OK;
	uint8_t stairState = 0;
	int16_t angelNick = 0;
	uint8_t done = 0;

	while(!done){
		err = regulateMotor(speed, distanceSide, angel);
		if(err != ERR_OK){
			setErrorState(err, "DrivingTask, State2");
		}
		
		/* \todo test if it helps to avoid drift */
		//refreshMovingOffset('x'); // Test if good or not
		
		err = L3GgetDegree('Z', &angelNick);
		if(err != ERR_OK){
			setErrorState(err, "DrivingTask, State2");
		}
		// stair upwards
		if(stairState == 0 && angelNick <= -25){
			stairState = 1;	
			LED_GREEN_Put(1);
		}
		// on top of the stair
		else if(stairState == 1 && angelNick < 5 && angelNick > -5){
			stairState = 2;	
			RED_Put(1);
			speed -= 20;
		}
		// stair downwards
		else if(stairState == 2 && angelNick >= 20){
			stairState = 3; 
			LED_GREEN_Put(0);
		}
		// stair done
		else if(stairState == 3 && angelNick < 10 && angelNick > -10){
			RED_Put(0);
			vTaskDelay(pdMS_TO_TICKS(20));
			done = 1;
		}
		vTaskDelay(pdMS_TO_TICKS(DELAY));
	}
	setState(3);
}

void driveToTurningPlace(void){
	uint8_t err = ERR_OK;
	uint8_t done = 0;
	int16_t ToFLeft;
	int16_t ToFRight;
	
	while(!done){
		err = regulateMotor(speed, distanceSide, angel);
		if (err != ERR_OK) {
			setErrorState(err, "DrivingTask, State3");
		}
		/* \todo test if it helps to avoid drift */
		//refreshMovingOffset('x'); // Test if good or not
		
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
		vTaskDelay(pdMS_TO_TICKS(DELAY));
	}
	setState(4);
}

void driveThroughtTurningPlace(void){
	int16_t range = 0, rangeOld = 0;
	uint8_t err = ERR_OK;
	uint8_t partState = 0;
	uint16_t count = 0;
	int16_t currentAngel;
	uint16_t time = 800; // drive for 1 seconds over the end
	int16_t ToFRight, ToFLeft;

	
	// drive straight for defined time
	while(partState == 0){
		LED_GREEN_Put(1);
		err = regulateMotor(speed, distanceSide, angel);
		if (err != ERR_OK) {
			setErrorState(err, "DrivingTask, State4");
		}
		
		err = VL_GetDistance(TOFRIGHT, &ToFRight);
		if (err != ERR_OK) {
			setErrorState(err, "DrivingTask, State4");
		}
		err = VL_GetDistance(TOFLEFT, &ToFLeft);
		if (err != ERR_OK) {
			setErrorState(err, "DrivingTask, State4");
		}
		if(ToFRight!=0 || ToFRight !=255 || ToFLeft!=0 || ToFLeft!=255){
			motorsStartup(0,0,0);
		}
		
		 // check end condition
		 count++;
		 if(count >= (time/DELAY)){
			 LED_GREEN_Put(0);
			 partState = 1;
			 count = 0;
		 }
		 vTaskDelay(pdMS_TO_TICKS(DELAY));
	}
	
	// turn 90 degree
	while(partState == 1){
		angel = -1*factor;
		err = regulateMotor(0, 0, angel);
		if (err != ERR_OK) {
			setErrorState(err, "DrivingTask, State4");
		}
		 
		 // check end condition
		err = L3GgetDegree('x', &currentAngel);
		if (err != ERR_OK) {
			setErrorState(err, "DrivingTask, State4");
		}
		if(currentAngel == -90*factor){
			angel = -90*factor;
			partState = 2;
		}
		 vTaskDelay(pdMS_TO_TICKS(DELAY));
	}
	
	// drive to the wall
	while(partState == 2){
		LED_GREEN_Put(1);
		err = regulateMotor(speed, 0, angel);
		if (err != ERR_OK) {
			setErrorState(err, "DrivingTask, State4");
		}

		
		// check end condition
		err = VL_GetDistance(TOFFRONT, &range);
		if (err != ERR_OK) {
			setErrorState(err, "DrivingTask, State4");
		}
		if(range <= distanceFront && range > 0){
			LED_GREEN_Put(0);
			partState = 3;
		}
		vTaskDelay(pdMS_TO_TICKS(DELAY));
	}
	
	// turn 90 degree
	while(partState == 3){
		RED_Put(1);
		angel = -1*factor;
		err = regulateMotor(0, 0, angel);
		if (err != ERR_OK) {
			setErrorState(err, "DrivingTask, State4");
		}
			 
			 
		// check end condition
		L3GgetDegree('x', &currentAngel);
		if(currentAngel == -180*factor){
			RED_Put(0);
			angel = -180*factor;
			partState = 4;
		}
		vTaskDelay(pdMS_TO_TICKS(DELAY));
	}
	
	//drive to the door
	range = 0;
	while(partState == 4){
		LED_GREEN_Put(1);
		err = regulateMotor(speed, distanceSide, angel);
		if (err != ERR_OK) {
			setErrorState(err, "DrivingTask, State4");
		}
		
		//end condition
		rangeOld = range;
		err = VL_GetDistance(device, &range);
		if (err != ERR_OK) {
			setErrorState(err, "DrivingTask, State4");
		}
		if((rangeOld-range) >= 40 && (rangeOld-range)<=60){
			partState = 5;
		}
		vTaskDelay(pdMS_TO_TICKS(DELAY));
	}
	// drive through the door
	range = 0;
	while(partState == 5){
		RED_Put(1);
		err = regulateMotor(speed, distanceSide-50, angel);
		if (err != ERR_OK) {
			setErrorState(err, "DrivingTask, State4");
		}
		
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
		vTaskDelay(pdMS_TO_TICKS(DELAY));		
	}
		setState(5);
}

//Task6
void driveToEndZone(void){
	uint8_t err = ERR_OK;
	uint8_t done = 0;
	int16_t range;
	while(!done){
		err = regulateMotor(speed, distanceSide, angel);
		if (err != ERR_OK) {
			setErrorState(err, "DrivingTask, State4");
		}
		
		/* \todo test if it helps to avoid drift */
		//refreshMovingOffset('x'); // Test if good or not
		
		 // stop condition
		 err = VL_GetDistance(TOFFRONT, &range);
		if (err != ERR_OK) {
			setErrorState(err, "DrivingTask, State4");
		}
		 if(range <= distanceFront && range > 0){
			 done = 1;
		 }
		vTaskDelay(pdMS_TO_TICKS(DELAY));
	}
	
	motorsStartup(0, 0, 50);
	setState(6);
}

void pushTheButton(void){
	
}

uint8_t regulateMotor(int8_t speed, uint8_t optRange,int16_t optAngel){
	uint8_t err = ERR_OK;
	int16_t corrGyro = 0;
	int16_t corrToF = 0;
	
	// Motor-Regulation
	 err = calcPID(GYRO, kpGyro, kiGyro, kdGyro, optAngel, &corrGyro);
	 if(err != ERR_OK){
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
	 motorsStartup(speed-corrGyro+factor*corrToF, speed+corrGyro-factor*corrToF, 0);
	 return err;
}



void initDriving(uint8_t kpT, uint8_t kiT, uint8_t kdT, uint8_t kpG, uint8_t kiG, uint8_t kdG, bool leftParcour){
	if(leftParcour){
		factor = -1;
		device = TOFRIGHT;
	}
	else{
		factor = 1;
		device = TOFLEFT;
	}
	kpToF = kpT;
	kiToF = kiT;
	kdToF = kdT;
	kpGyro = kpG;
	kiGyro = kiG;
	kdGyro = kdG;
	
}
void setDistanceSide(int16_t value){
	distanceSide = value;
}

void setDistanceFront(int16_t value){
	distanceFront = value;
}

void setSpeed(int8_t value){
	speed = value;
}

void setGyroskopPWM(uint8_t value){
	PWM_Gyro_SetRatio16((0xFFFF-1)/127*value);
}

void setLetter(uint8_t value){
	letter = value;
}

void setServoPWM(uint8_t value){
	PWM_Servo_SetRatio16((0xFFFF-1)/127*value);
}
