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
#include "CLS1.h"
#include "pid.h"

#define TOFFRONT 0
#define TOFLEFT 1
#define TOFRIGHT 2
#define GYRO 3
#define PI 3.14159265
#define DELAY 15	// in ms

uint8_t kpToF, kiToF, kdToF;
uint8_t kpGyro, kiGyro, kdGyro;
int8_t factor;
uint8_t device;

//Task2
void driveToStair(int8_t speed, uint8_t optRange, uint8_t startupTime, uint16_t frontdistance){
	uint8_t err;
	uint8_t done = 0;
	int16_t range = 0;
	int16_t corrGyro = 0;
	int16_t corrToF = 0;
	
	RED_Put(0);
	LED_GREEN_Put(1);
	motorsStartup(speed, speed, startupTime);
	while(!done){
		// Motor-Regulation
		 err = calcPID(GYRO, kpGyro, kiGyro, kdGyro, 0, &corrGyro);
		 if(err == ERR_OK){
			 err = calcPID(device,kpToF, kiToF, kdToF, optRange, &corrToF);
			 	 if(err != ERR_OK){
			 		 corrToF = 0;
			 	 }
			 motorsStartup(speed-corrGyro+factor*corrToF, speed+corrGyro-factor*corrToF, 2);
		 }
		 else{
			 // error
		 }
		 
		 // check if stair is detected
		 VL_GetDistance(TOFFRONT, &range);
		 if(range <= frontdistance && range > 0){
			 LED_GREEN_Put(0);
			 RED_Put(1);
			 done = 1;
		 }
		 
		 refreshMovingOffset();
		 vTaskDelay(pdMS_TO_TICKS(DELAY));
	}
	motorsStartup(0, 0, 0);
	RED_Put(0);
	taskDone(2);
}

//Task3
void driveOverStair(int8_t speed, uint8_t optRange){
	uint8_t err = ERR_OK;
	int16_t corrToF = 0;
	int16_t corrGyro = 0;
	uint8_t stairState = 0;
	int16_t angel;
	uint8_t done = 0;
	L3GSetAngel('z', 0);
	motorsStartup(speed,speed,0);
	do{
		// Regulate Motors
		err = calcPID(GYRO, kpGyro, kiGyro, kdGyro, 0, &corrGyro);
		if(err == ERR_OK){
			err = calcPID(device, kpToF, kiToF, kdToF, optRange, &corrToF);
		}
		motorsStartup(speed-corrGyro+factor*corrToF, speed+corrGyro-factor*corrToF, 2);
		if(err != ERR_OK){
			motorsStartup(0, 0, 0);		// error
		}
		
		//
		
		L3GgetDegree('Z', &angel);
		if(stairState == 0 && angel <= -25){
			stairState = 1;	// stair upwards
			LED_GREEN_Put(1);
		}
		else if(stairState == 1 && angel < 5 && angel > -5){
			stairState = 2;	// on top of the stair
			L3GSetAngel('z', 0);
			RED_Put(1);
			speed -= 20;
		}
		else if(stairState == 2 && angel >= 25){
			stairState = 3; // stair downwards
			LED_GREEN_Put(0);
		}
		else if(stairState == 3 && angel < 10 && angel > -10){
			stairState = 4; // stair done
			L3GSetAngel('z', 0);
			RED_Put(0);
			vTaskDelay(pdMS_TO_TICKS(20));
			done = 1;
		}
		vTaskDelay(pdMS_TO_TICKS(DELAY));
	}
	while(!done);
	/* \todo stop doesnt work with 10ms*/
	motorsStartup(0, 0, 0);
	
	taskDone(3);
}

//Task4
void driveToTurningPlace(int8_t speed, uint8_t optRange){
	uint8_t err = ERR_OK;
	int16_t corrToF = 0;
	int16_t corrGyro = 0;
	uint8_t done = 0;
	int16_t ToFLeft;
	int16_t ToFRight;
	motorsStartup(speed,speed,0);
	do{
		// Regulate Motors
		err = calcPID(GYRO, kpGyro, kiGyro, kdGyro, 0, &corrGyro);
		if(err == ERR_OK){
			err = calcPID(device, kpToF, kiToF, kdToF, optRange, &corrToF);
		}
		motorsStartup(speed-corrGyro+factor*corrToF, speed+corrGyro-factor*corrToF, 2);
		if(err != ERR_OK){
			//motorsStartup(0, 0, 0);		// error
		}
		
		// stop condition
		VL_GetDistance(TOFRIGHT, &ToFRight);
		VL_GetDistance(TOFLEFT, &ToFLeft);
		if((ToFLeft == 255 || ToFLeft == 0) && (ToFRight == 255 || ToFRight == 0)){
			done = 1;
		}
		
		vTaskDelay(pdMS_TO_TICKS(DELAY));
	}
	while(!done);
	
	motorsStartup(0, 0, 0);
	
	taskDone(4);
}

//Task5
void driveThroughtTurningPlace(uint8_t speed, uint8_t optRange, uint8_t frontdistance){
	int16_t range = 0;
	uint8_t err = ERR_OK;
	int16_t corrGyro = 0;
	int16_t optAngel = 0;
	uint8_t partState = 0;
	int16_t corrToF = 0;
	uint16_t count = 0;
	uint16_t time = 1000; // drive for 1 seconds over the end

	motorsStartup(speed, speed, 2);
	
	
	// drive straight for defined time
	while(partState == 0){
		// Motor-Regulation
		 err = calcPID(GYRO, kpGyro, kiGyro, kdGyro, optAngel, &corrGyro);
		 if(err == ERR_OK){
			 motorsStartup(speed-corrGyro, speed+corrGyro, 2);
		 }
		 else{
			 // error
		 }
		 
		 // check end condition
		 count++;
		 if(count >= (time/DELAY)){
			 partState = 1;
			 motorsStartup(0, 0, 0);
			 count = 0;
		 }
		 
		 vTaskDelay(pdMS_TO_TICKS(DELAY));
	}
	
	// turn 90 degree
	optAngel = -90*factor;
	while(partState == 1){
		// Motor-Regulation
			 err = calcPID(GYRO, kpGyro, kiGyro, kdGyro, optAngel, &corrGyro);
			 if(err == ERR_OK){
				 motorsStartup(speed-corrGyro/4, speed+corrGyro/4, 10);
			 }
			 else{
				 // error
			 }
			 
			 // check end condition
			 if(corrGyro <= 5){
				 motorsStartup(0, 0, 0);
				 partState = 2;
			 }
			 
			 vTaskDelay(pdMS_TO_TICKS(DELAY));
	}
	
	// drive to the wall
	while(partState ==2){
		 err = calcPID(GYRO, kpGyro, kiGyro, kdGyro, optAngel, &corrGyro);
		 if(err == ERR_OK){
			 motorsStartup(speed-corrGyro, speed+corrGyro, 2);
		 }
		 else{
			 // error
		 }
		
		
		// check end condition
		VL_GetDistance(TOFFRONT, &range);
		if(range <= frontdistance && range > 0){
			motorsStartup(0, 0, 0);
			partState = 3;
		}
	}
	
	// turn 90 degree
	optAngel = -180*factor;
	while(partState == 3){
		// Motor-Regulation
			 err = calcPID(GYRO, kpGyro, kiGyro, kdGyro, optAngel, &corrGyro);
			 if(err == ERR_OK){
				 motorsStartup(speed-corrGyro/4, speed+corrGyro/4, 10);
			 }
			 else{
				 // error
			 }
			 
			 // check end condition
			 if(corrGyro <= 5){
				 motorsStartup(0, 0, 0);
				 partState = 4;
			 }
			 
			 vTaskDelay(pdMS_TO_TICKS(DELAY));
	}
	
	//drive further
	while(partState == 4){
		
		// Regulate Motors
		err = calcPID(GYRO, kpGyro, kiGyro, kdGyro, optAngel, &corrGyro);
		if(err == ERR_OK){
			err = calcPID(device, kpToF, kiToF, kdToF, optRange, &corrToF);
		}
		motorsStartup(speed-corrGyro+factor*corrToF, speed+corrGyro-factor*corrToF, 2);
		if(err != ERR_OK){
			//motorsStartup(0, 0, 0);		// error
		}	
			
	}
		motorsStartup(0, 0, 0);
		taskDone(5);
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

