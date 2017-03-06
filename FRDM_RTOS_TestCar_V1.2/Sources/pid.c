/**
 *--------------------------------------------------------------------\n
 *          HSLU T&A Hochschule Luzern Technik+Architektur            \n
 *--------------------------------------------------------------------\n
 *
 * \brief         PID control
 * \file
 * \author        Pirmin Zimmermann
 * \date          12.12.2016
 *
 * \b Language:   C \n\n
 * \b Target:     PREN \n
 *
 *--------------------------------------------------------------------
 */
#include "pid.h"
#include "L3G.h"
#include "FRTOS1.h"
#include "CLS1.h"
#include <math.h>
#include "VL6180X.h"

#define PI 3.14159265

pid_t pid[4];


/*
 * Device: 	0 = ToF Front
 * 			1 = ToF Left
 * 			2 = ToF Right
 * 			3 = Gyro
 */
uint8_t calcPID(uint8_t device, uint8_t kP, uint8_t kI, uint8_t kD, int16_t optValue, int16_t* corr){
	uint8_t err = ERR_OK;
	int16_t value;
	int16_t angel;
	int16_t correction;
	
	switch(device){
	case 0: err = VL_GetDistance(device, &value);
	break;
	case 1: err = VL_GetDistance(device, &value);
	break;
	case 2: err = VL_GetDistance(device, &value);
	break;
	case 3: ;
	break;
	default: value = 0;
			err = ERR_VALUE;
	}
	
	L3GgetDegree('x', &angel);
	
	if(err != ERR_OK){
		return err;
	}
	
	if(device==0 || device==1 || device==2){
		if(value==255 || value<0){
			*corr = 0;
			return ERR_RANGE;
		}
		if(device!=0){
			value = value/cos(angel*PI/180);
		}
	}
	else{
		value = angel;
	}
	
	pid[device].devOld = pid[device].dev;
	pid[device].dev = optValue - value;
	if(kI != 0){
		pid[device].integ += value;
	}
	correction = kP*pid[device].dev/16 + kD*(pid[device].dev-pid[device].devOld)/16 + kI*pid[device].integ/16;
	
	if(correction>127){
		correction = 127;
	}else if(correction<-127){
		correction = -127;
	}
	*corr = correction;
	
	
	return ERR_OK;
}


void setDeviceToZero(uint8_t device){
	pid[device].dev = pid[device].devOld = pid[device].integ = 0;
}
