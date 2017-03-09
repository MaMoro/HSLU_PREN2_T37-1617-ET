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
#include "PWM_LEFT.h"
#include "PWM_RIGHT.h"
#include "PwmLdd1.h"
#include "PwmLdd2.h"
#include "TU1.h"

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
