/*
 * motor.h
 *
 *  Created on: Dec 7, 2016
 *      Author: pirzi
 */

#ifndef MOTOR_H_
#define MOTOR_H_
#include "stdint.h"
#include "DIR_LEFT.h"
#include "DIR_RIGHT.h"
#include "DIR_LEFT1.h"
#include "DIR_RIGHT1.h"
#include "PWM_LEFT.h"
#include "PWM_RIGHT.h"
#include "PwmLdd1.h"
#include "PwmLdd2.h"
#include "TU1.h"

#define DUALMOTORDRIVER 1			// put 1 if dualmotordriver with 2 IO Pins for direction is used
#define MAXDIFERENCE 42				// max difference of actual pwm to next pwm if div%MAXDIFERENCE+1 -> pwm = nextpwm/(div%MAXDIFERENCE+1)
#define PERIODEMS 20

int8_t motorGetPWMLeft(void);
void motorSetPWMLeft(int8_t value);
void motorIncrementPWMLeft(int8_t value);

int8_t motorGetPWMRight(void);
void motorSetPWMRight(int8_t value);
void motorIncrementPWMRight(int8_t value);
void motorStartupLeft(int8_t value, uint8_t time);
void motorStartupRight(int8_t value, uint8_t time);
void motorsStartup(int16_t valueLeft, int16_t valueRight, uint16_t time);



#endif /* MOTOR_H_ */
