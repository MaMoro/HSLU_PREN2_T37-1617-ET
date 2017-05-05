/*
 * motor.c
 *
 *  Created on: Dec 7, 2016
 *      Author: pirzi
 */
#include "motor.h"
#include <stdlib.h>
#include "FRTOS1.h"

#define MOTOR_OFFSET	0
#define MODULO          (0xFFFF-1)



static int16_t pwmLeft;
static int16_t pwmRight;
static int16_t safetyVal;
static uint8_t brake;

/**
 * returns the pwm value of the left motor
 * 
 * @returns
 *    the value 0..+/-127
 */
int8_t motorGetPWMLeft(void)
{
  return pwmLeft;
}

/**
 * returns the pwm value of the right motor
 * 
 * @returns
 *    the value 0..+/-127
 */
int8_t motorGetPWMRight(void)
{
  return pwmRight;
}

/**
 * increments the speed of the left motor
 * 
 * @param [in] value
 *    the desired offset
 */
void motorIncrementPWMLeft(int8_t value)
{
  int16_t v = pwmLeft + value;
  if (v > 127) v = 127;
  if (v < -127) v = -127;
  motorSetPWMLeft((int8_t)v);
}


/**
 * increments the speed of the right motor
 * 
 * @param [in] value
 *    the desired offset
 */
void motorIncrementPWMRight(int8_t value)
{
  int8_t v = pwmRight + value;
  if (v > 127) v = 127;
  if (v < -127) v = -127;
  motorSetPWMRight((int8_t)v);
}


/**
 * sets the speed of the left motor
 * 
 * @param [in] value
 *  +1..+127 => speed in forward direction
 *  -1..-127 => speed in backward direction
 *  0 => stop
 *  
 *  Ratio is set in Events.c
 */
void motorSetPWMLeft(int8_t value)
{
	if(brake > 0){				// brake
		  DIR_LEFT_PutVal(DIR_LEFT_DeviceData, 0);
		  DIR_LEFT1_PutVal(DIR_LEFT1_DeviceData, 0);
		  pwmLeft = brake;
	}else{
  if (value < 0)              // backward
  {
	  if (value < -127) value = -127;
    DIR_LEFT_PutVal(DIR_LEFT_DeviceData, 1);
#if DUALMOTORDRIVER
    DIR_LEFT1_PutVal(DIR_LEFT1_DeviceData, 0);
#endif
  }
  else if (value > 0)         // forward
  {
	  if (value > 127) value = 127;
	DIR_LEFT_PutVal(DIR_LEFT_DeviceData,0);
#if DUALMOTORDRIVER
	DIR_LEFT1_PutVal(DIR_LEFT1_DeviceData, 1);
#endif
  } 
  else                        // stop
  {
	  DIR_LEFT_PutVal(DIR_LEFT_DeviceData, 0);
#if DUALMOTORDRIVER
	  DIR_LEFT1_PutVal(DIR_LEFT1_DeviceData, 0);
#endif
	  }
  safetyVal = abs(pwmLeft-value)/MAXDIFERENCE+1;	// Motordriver safety if difference is too high then shorten the incrementation
  pwmLeft = value / safetyVal;
	}
}

/**
 * sets the speed of the right motor
 * 
 * @param [in] value
 *  +1..+127 => speed in forward direction
 *  -1..-127 => speed in backward direction
 *  0 => stop
 */
void motorSetPWMRight(int8_t value)
{
	if(brake > 0){				// brake
		  DIR_RIGHT_PutVal(DIR_RIGHT_DeviceData, 0);
		  DIR_RIGHT1_PutVal(DIR_RIGHT1_DeviceData, 0);
		  pwmRight = brake;
	}
	else{
	  if (value < 0)              // backward
	  {
		  if (value < -127) value = -127;
	    DIR_RIGHT_PutVal(DIR_RIGHT_DeviceData,0);
#if DUALMOTORDRIVER
	  DIR_RIGHT1_PutVal(DIR_RIGHT1_DeviceData, 1);  
#endif
	    //PWM_RIGHT_SetRatio16(MODULO/127*(-value));
	  //PWM_RIGHT_SetRatio8(-value);
	  }
	  else if (value > 0)         // forward
	  {
		  if (value > 127) value = 127;
		DIR_RIGHT_PutVal(DIR_RIGHT_DeviceData,1);
#if DUALMOTORDRIVER
		DIR_RIGHT1_PutVal(DIR_RIGHT1_DeviceData, 0);
#endif
		//PWM_RIGHT_SetRatio16(MODULO/127*value);
		//PWM_RIGHT_SetRatio8(value);
	  } 
	  else                        // stop
	  {
		  //PWM_RIGHT_SetRatio16(0);
		  //PWM_RIGHT_SetRatio8(0);
		  DIR_RIGHT_PutVal(DIR_RIGHT_DeviceData, 0);
#if DUALMOTORDRIVER
		DIR_RIGHT1_PutVal(DIR_RIGHT1_DeviceData, 0);  
#endif
	  }  
  safetyVal = abs(pwmRight-value)/MAXDIFERENCE+1;		// Motordriver safety if difference is too high then shorten the incrementation
  pwmRight = value/safetyVal;
	}
}

/**
 * Increment or decrement the speed of the motor in a specified time
 * 
 * @param [in] value
 *  +1..+127 => speed in forward direction
 *  -1..-127 => speed in backward direction
 *  0 => stop
 *  @param [in] time
 *  time in witch the motor has to reach the speed of value (in ms)
 */
void motorStartupRight(int8_t value, uint8_t time){
	int8_t dif = value - pwmRight; 
	uint8_t i;
	int8_t increment = dif/abs(dif);
	for(i = 0; i <= abs(dif); i++){
		motorIncrementPWMRight(increment);
		FRTOS1_vTaskDelay(pdMS_TO_TICKS(abs(dif)/time));
	}
}

/**
 * Increment or decrement the speed of the motor in a specified time
 * 
 * @param [in] value
 *  +1..+127 => speed in forward direction
 *  -1..-127 => speed in backward direction
 *  0 => stop
 *  @param [in] time
 *  time in witch the motor has to reach the speed of value (in ms)
 */
void motorStartupLeft(int8_t value, uint8_t time){
	int8_t dif = value - pwmLeft; 
	uint8_t i;
	int8_t increment = dif/abs(dif);
	for(i = 0; i <= abs(dif); i++){
		motorIncrementPWMLeft(increment);
		vTaskDelay(pdMS_TO_TICKS(abs(dif)/time));
	}
}

/**
 * Increment or decrement the speed of the motors in a specified time
 * 
 * @param [in] valueLeft
 *  +1..+127 => speed in forward direction
 *  -1..-127 => speed in backward direction
 *  0 => stop
 *  @param [in] valueRight
 *  +1..+127 => speed in forward direction
 *  -1..-127 => speed in backward direction
 *  0 => stop
 *  @param [in] time
 *  time in witch the motor has to reach the speed of value (in ms)
 */
void motorsStartup(int16_t valueLeft, int16_t valueRight){
	if(valueLeft>127){
		valueLeft = 127;
	}else if(valueLeft<-127){
		valueLeft = -127;
	}
	if(valueRight>127){
		valueRight = 127;
	}else if(valueRight<-127){
		valueRight = -127;
	}

	motorSetPWMRight(valueRight);
	motorSetPWMLeft(valueLeft);
}

void motorsbrake(uint8_t doBrake){
	if(doBrake > 127){
		doBrake = 127;
	}
	else if(doBrake < 0){
		doBrake = 0;
	}
	brake = doBrake;
}
