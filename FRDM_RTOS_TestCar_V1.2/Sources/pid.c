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

#define PI 3.14159265

pid_t pid[4];


/*

 * 
 * Calculate the PID correction of a device
 *  Device: 0 = ToF Front
 * 			1 = ToF Left
 * 			2 = ToF Right
 * 			3 = Gyro
 * @param device device to calculate from
 * @param kP proportional factor
 * @param kI integral factor
 * @param kD differential factor
 * @param optValue the optimal value to reach
 * @param corr pointer to the correction value
 */
uint8_t calcPID(uint8_t device, uint8_t kP, uint8_t kI, uint8_t kD, int16_t optValue, int16_t* corr){
	uint8_t err = ERR_OK;
	int16_t value;
	int16_t angel;
	int16_t correction;
	static uint8_t timems;
	
	if(timems == 0){
	timems = (uint8_t)(FRTOS1_xTaskGetTickCount() - pid[device].timecount)*10;	// timecount in ms	deltaTicks /100Hz*1000ms/s
	pid[device].timecount = FRTOS1_xTaskGetTickCount();
	*corr = 0;
	return ERR_OK;
	}
	timems = (uint8_t)(FRTOS1_xTaskGetTickCount() - pid[device].timecount)*10;	// timecount in ms	deltaTicks /100Hz*1000ms/s
	pid[device].timecount = FRTOS1_xTaskGetTickCount();
	
	
	switch (device) {
	case TOFFRONT:;
	case TOFLEFT:;
	case TOFRIGHT:;
		err = VL_GetDistance(device, &value);
		if(value <= 30 || value >= 255){
			*corr = 0;
			return ERR_OK;
		}
		break;
	case GYRO:
		err = L3GgetDegree(GEAR, &angel);
		if (optValue > 120 && angel < -120) {
			value = angel + 360;
		} else if (optValue < -120 && angel > 120) {
			value = angel - 360;
		} else {
			value = angel;
		}
		break;
	default:
		err = ERR_VALUE;
	}
	if(err != ERR_OK){
		*corr = 0;
		return err;
	}
	
	pid[device].devOld = pid[device].dev;
	pid[device].dev = optValue - value;
	if(kI != 0){
		if(pid[device].dev == 0){
			pid[device].integ = 0;
		}else{
			pid[device].integ += pid[device].dev;
		}
		if(pid[device].integ > 10000){					//\ todo antiwindup testen
			pid[device].integ = 10000;
		}else if(pid[device].integ < -10000){
			pid[device].integ = -10000;
		}
	}else{
		pid[device].integ = 0;
	}
	
	correction = kP*pid[device].dev/16 + kD*(pid[device].dev-pid[device].devOld)/16*100/timems + kI*pid[device].integ*timems/100/16;
	
	
	if(correction>255){
		correction = 255;
	}else if(correction<-255){
		correction = -255;
	}
	
	*corr = correction;
	
	
	return ERR_OK;
}


/*
 * Set dev, devOld and integ in the pid structure to zero for a defined device
 * @param device 
 * Device:	0 = ToF Front
 * 			1 = ToF Left
 * 			2 = ToF Right
 * 			3 = Gyro
 */
void setDeviceToZero(uint8_t device){
	pid[device].dev = pid[device].devOld = pid[device].integ = 0;
}

void setIntegToZero(uint8_t device){
	pid[device].integ = 0;
}
