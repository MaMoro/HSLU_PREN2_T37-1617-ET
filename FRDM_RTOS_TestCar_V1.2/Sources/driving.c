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
#include "Toggle.h"

#define DELAY 20	// in ms
#define ULTRASLOW 8	// default 8
#define SLOW 20		// default 20
#define MEDIUM 80	// default 40
#define FAST 60		// default 60

static uint8_t kpToF, kiToF, kdToF;			// PID values ToF
static uint8_t kpGyro, kiGyro, kdGyro;		//PID values Gyro
static uint8_t device;						// left or right tof
static uint8_t distanceSide = 145;			// distance of the tofs to the side wall
static uint8_t distanceFront = 200;
static int8_t speed = 0;
static uint8_t letter = 5;
static uint8_t stop;						// if set to 1 the motors will stop
static int8_t parcour;						// -1 = LeftParcour, 1 = RightParcour
static uint16_t timeout;
static uint8_t optRangeLeft, optRangeRight;
static int16_t optAngel;

/*
 * Drive to the stair until the front ToF detects the stair
 */
void driveToStair(void) {
	uint8_t err = ERR_OK;
	uint8_t done = 0;
	int16_t range = 0;
	optRangeLeft = distanceSide;
	optRangeRight = distanceSide;
	optAngel = 0;
	
	setGyroskopPWM(13);
	
	LED_GREEN_Put(1);
	while (!done) {
		if(speed < MEDIUM){
			speed += 2;
		}else{
			speed = MEDIUM;
		}
		err = regulateMotor();
		if(err != ERR_OK){
			setErrorState(err,"Driving Task");
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
			speed = MEDIUM;
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
	uint8_t brake;
	uint8_t counter = 0;
	optRangeLeft = distanceSide;
	optRangeRight = distanceSide;
	optAngel = 0;

	while(!done){
		err = regulateMotor();
		if(err != ERR_OK){
			setErrorState(err,"Driving Task");
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
			}
		}
		else if(stairState == 2){
			if(speed > ULTRASLOW){
				speed = speed*0.7;
			}else{
				speed = ULTRASLOW;
			}
			if(angelNick >= 2){	// stair downwards
				counter++;
				if(counter>3){
					speed = 0;
					counter = 0;
					stairState = 3; 
					LED_GREEN_Put(0);
					brake = 60;
				}
			}
		}
		
		else if(stairState == 3){
			motorsbrake(brake);
			if(brake > 0){
				brake -= 2;
			}
			if(angelNick < 28){
				motorsbrake(0);
				speed++;
			}
			if(angelNick < 5 && angelNick > -5){	// stair done
			counter++;
			if(counter >= 2){
				counter = 0;
				RED_Put(0);
				speed = MEDIUM;
				done = 1;
			}
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
	optRangeLeft = distanceSide;
	optRangeRight = distanceSide;
	optAngel = 0;
	
	while(!done){	
		//angel correction
		//angelCorrection(optAngel);
		
		err = regulateMotor();
		if(err != ERR_OK){
			setErrorState(err,"Driving Task");
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
	uint16_t time;
	int16_t angel;
	uint8_t counter = 0;
	optRangeLeft = distanceSide;
	optRangeRight = distanceSide;
	optAngel = 0;
	distanceSide = 190;
	
	// turn 90 degree and drive to the wall
	counter = 0;
	speed = SLOW;
	while(partState == 0){
		if(abs(optAngel) < 90){
			optAngel -= 5*parcour;
		}else{
			optAngel = -90*parcour;
		}
		err = regulateMotor();
		if(err != ERR_OK){
			setErrorState(err,"Driving Task");
		}
		 vTaskDelay(pdMS_TO_TICKS(DELAY));
		 
		 // check end condition
		L3GgetDegree(GEAR,&angel);
		if(abs(angel) >= 85 && abs(angel) < 95){
			speed = MEDIUM;
			LED_GREEN_Put(1);
			optAngel = -90*parcour;
		}
		// end condition for the front wall
		err = VL_GetDistance(TOFFRONT, &range);
		if (err != ERR_OK) {
			setErrorState(err, "DrivingTask, State4");
		}
		if(range <= distanceFront && range > 0){
			LED_GREEN_Put(0);
			partState = 1;
		}
	}
	
	speed = 0;
	RED_Put(1);
	optRangeLeft = 0;
	optRangeRight = 0;
	counter = 0;
	// turn 90 degree
	while(partState == 1){
		if(abs(optAngel)<177){
		optAngel -= 4*parcour;
		}else{
			optAngel = -180*parcour;
		}
		err = regulateMotor();
		if(err != ERR_OK){
			setErrorState(err,"Driving Task");
		}
		vTaskDelay(pdMS_TO_TICKS(DELAY));	 
			 
		L3GgetDegree(GEAR, &angel);
		// check end condition
		if(angel*(-parcour) < -175 || angel*(-parcour) > 175){	// so angel has gone over 180 degrees to -180 or less degrees
			counter++;
			if(counter > 5){
				optAngel = -180*parcour;
				RED_Put(0);
				partState = 2;
				optRangeLeft = distanceSide-10;
				optRangeRight = distanceSide-10;
			}
		}
	}
	
	//drive to the door
	range = 0;
	timeout = 0;
	speed = MEDIUM;
	LED_GREEN_Put(1);
	while(partState == 2){
		err = regulateMotor();
		if(err != ERR_OK){
			setErrorState(err,"Driving Task");
		}
		vTaskDelay(pdMS_TO_TICKS(DELAY));
		
		//end condition
		rangeOld = range;
		getTime(&time);
		timeout += time;
		err = VL_GetDistance(device, &range);
		if (err != ERR_OK) {
			setErrorState(err, "DrivingTask, State4");
		}
		if((rangeOld-range) >= 40 && (rangeOld-range)<=60){
			partState = 3;
		}
		if(timeout >= MAXTIMEOUT){
			partState = 4;				// already drove through door
		}
	}
	
	// drive through the door
	range = 0;
	timeout = 0;
	optRangeLeft = distanceSide-50;
	optRangeRight = distanceSide-50;
	RED_Put(1);
	while(partState == 3){
		err = regulateMotor();
		if(err != ERR_OK){
			setErrorState(err,"Driving Task");
		}
		vTaskDelay(pdMS_TO_TICKS(DELAY));
		
		//end condition
		getTime(&time);
		timeout += time;
		rangeOld = range;
		err = VL_GetDistance(device, &range);
		if (err != ERR_OK) {
			setErrorState(err, "DrivingTask, State4");
		}
		if(((range-rangeOld) >= 40 && (rangeOld-range)<=60) || timeout >= MAXTIMEOUT){
			RED_Put(0);
			LED_GREEN_Put(0);
			partState = 4;
		}		
	}
		setState(5);
}


void driveOverChannel(void){
	uint8_t err = ERR_OK;
	uint8_t done = 0;
	int16_t angelNick = 0;
	uint8_t partState = 0;
	uint16_t time;
	
	timeout = 0;
	optRangeLeft = distanceSide;
	optRangeRight = distanceSide;
	speed = FAST;
	while(!done){
		//angel correction
		//angelCorrection(optAngel);
		
		err = regulateMotor();
		if(err != ERR_OK){
			setErrorState(err,"Driving Task");
		}
		getTime(&time);
		timeout += time;
		vTaskDelay(pdMS_TO_TICKS(DELAY));
		
		//end Condition
		L3GgetDegree(NICK, &angelNick);
		if(partState == 0 && angelNick <= -3){		// drive on channel
			partState = 1;
			speed = MEDIUM;
		}
		else if(partState == 1 && angelNick >= 3){		// drive down from channel
			partState = 2;
		}
		else if(partState == 2 && angelNick < 3 && angelNick > -3){		// channel done
			done = 1;
			speed = FAST;
		}
		if(timeout >= MAXTIMEOUT){
			done = 1;
			speed = FAST;
		}
	}
	setState(6);
}

/*
 * drive to the endzone, if the wall of the buttons is detected with the front ToF, the frontzone is reached
 */
void driveToEndZone(void){
	uint8_t err = ERR_OK;
	uint8_t done = 0;
	int16_t rangeSide;
	int8_t deviceIn;
	int8_t difference = 30;
	optRangeLeft = distanceSide-difference;
	optRangeRight = distanceSide+difference;

	if(parcour == LEFT){
		deviceIn = TOFLEFT;
	}else if (parcour == RIGHT){
		deviceIn = TOFRIGHT;
	}
	
	speed = FAST;
	while(!done){
		//angel correction
		angelCorrection(optAngel);
		
		err = regulateMotor();
		if(err != ERR_OK){
			setErrorState(err,"Driving Task");
		}
		vTaskDelay(pdMS_TO_TICKS(DELAY));
		
		 // stop condition
		err = VL_GetDistance(deviceIn, &rangeSide);
		if ( err != ERR_OK){
			setErrorState(err, "DrivingTask, State4");
		}
		 if(rangeSide<=0 || rangeSide == 255){
			 if(speed > SLOW){
				 speed -=5;
			 }
			 else{
				 speed = SLOW;
			 }
			 if(speed <= SLOW){
				 done = 1;
			 }
		 }
		
	}
	motorsStartup(0,0);
	//motorsbrake(50);
	setState(7);
}

/*
 * push the right button in the endzone
 */
void pushTheButton(void){
	uint8_t err;
	int16_t rangeFront, rangeSide;		
	uint8_t deviceOutside = 0;
	uint16_t rangeWallButton = 0;
	uint8_t rangeTofServoCalc = 0;
	double function;
	bool done = 0;
	const uint8_t rangeTofServo = 106;		// distance from ToF right to Servo axis
	const uint8_t lengthStick = 170;		// noch zu bestimmen
	const uint8_t widthChassis = 120;
	int16_t angle = 0;
	int8_t difference = 30;
	
	speed = ULTRASLOW;
	optRangeLeft = distanceSide-difference;
	optRangeRight = distanceSide+difference;
		
	// chose Tof-Sensor on the Outside
	if(parcour == LEFT){
		deviceOutside = TOFRIGHT;
		rangeWallButton = 90 + (240 - (letter-1) * 60);	 	// calculate distance from the outside Wall to the Button
		rangeTofServoCalc = rangeTofServo;
	}else if (parcour == RIGHT){
		deviceOutside = TOFLEFT;
		rangeWallButton = 90 + ((letter-1)* 60);			// calculate distance from the outside Wall to the Button
		rangeTofServoCalc = widthChassis - rangeTofServo;
	}
	
		while(!done){

			err = VL_GetDistance(deviceOutside, &rangeSide);
			if (err != ERR_OK){
				setErrorState(err, "DrivingTask, State6");
			}
			if(rangeSide >= 0 && rangeSide <= 200){
				function = ((double)rangeTofServoCalc + (double)rangeSide - (double)rangeWallButton)/(double)lengthStick;
				if(parcour == LEFT){
					angle = (900 - (int16_t)((asin(function))*(1800.0f/PI)));
				}else{
					angle = (900 + (int16_t)((asin(function))*(1800.0f/PI)));
				}
			}
			
			setServoPWM(angle);
			
			err = regulateMotor();
			if(err != ERR_OK){
				setErrorState(err,"Driving Task");
			}
			vTaskDelay(pdMS_TO_TICKS(DELAY));
		}
}

/*
 * regulate the motors 
 * @param speed the speed with whitch to drive (PWM)
 * @param optRange optimal Range of the side ToF
 * @param optAngel optimal Angel of the gyro
 */
uint8_t regulateMotor(void){
	uint8_t err = ERR_OK;
	int16_t corrGyro = 0;
	int16_t corrToFleft = 0, corrToFright = 0, corrToF = 0;
	Toggle_PutVal(Toggle_DeviceData, !Toggle_GetVal((Toggle_DeviceData)));
	if (stop) {
		motorsStartup(0, 0);
		return ERR_OK;
	} else {
		// Motor-Regulation
		err = calcPID(GYRO, kpGyro, kiGyro, kdGyro, optAngel, &corrGyro);
		if (err != ERR_OK) {
			corrGyro = 0;
			// error \todo
		}
		if (optRangeLeft != 0 || optRangeRight != 0) {
			err = calcPID(TOFLEFT, kpToF, kiToF, kdToF, optRangeLeft, &corrToFleft);
			if (err != ERR_OK) {
				corrToFleft = 0;
				// error \todo
			}
			err = calcPID(TOFRIGHT, kpToF, kiToF, kdToF, optRangeRight, &corrToFright);
			if (err != ERR_OK) {
				corrToFright = 0;
				// error \todo
			}
			if(corrToFleft != 0 && corrToFright != 0){			// both sides detected
				corrToF = (corrToFleft-corrToFright)/2;
			}
			else{										// one or no side detected
				corrToF = corrToFleft-corrToFright;
			}
		}
		motorsStartup(speed - corrGyro + corrToF, speed + corrGyro - corrToF);
		return ERR_OK;
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
	PWM_Gyro_SetRatio8(0xFF-value);
}

/*
 * Set the right number to push in the end-Zone
 * @param value letter from 1-5
 */
void setLetter(uint8_t value){
	letter = value;
}

/*0x0FFFF-0x1FFFF
 * set the optimal angel of the Servo 
 * @ value angel of the Servo from 0...1200 for 0°-120°
 */
void setServoPWM(uint16_t value){
	if(value > 1200){
		value = 1200;
	}
	//PWM_Servo_SetDutyUS(20000-(2500-value*20/18));		// dutysicle is between 17.5ms to 19.5ms
	PWM_Servo_SetDutyUS(17500 + value*0.988);
}

/*
 * run this method to stop the motors of the car
 */
void stopDriving(bool stp){
	stop = stp;
}

uint8_t getTime(uint16_t* time){
	static uint8_t timems;
	static portTickType timecount;
	
	if(timems == 0){
	timems = (uint8_t)(FRTOS1_xTaskGetTickCount() - timecount)*10;	// timecount in ms	deltaTicks /100Hz*1000ms/s
	timecount = FRTOS1_xTaskGetTickCount();
	*time = 0;
	return ERR_OK;
	}
	timems = (uint8_t)(FRTOS1_xTaskGetTickCount() - timecount)*10;	// timecount in ms	deltaTicks /100Hz*1000ms/s
	timecount = FRTOS1_xTaskGetTickCount();
	*time = timems;
	return ERR_OK;
}

