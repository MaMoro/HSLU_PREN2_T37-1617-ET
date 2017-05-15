/**
 *--------------------------------------------------------------------\n
 *          HSLU T&A Hochschule Luzern Technik+Architektur            \n
 *--------------------------------------------------------------------\n
 *
 * \brief         PID control
 * \file
 * \author        Christian Jost
 * \date          21.11.2012
 *
 * \b Language:   Ansi-C \n\n
 * \b Target:     MC Car \n
 *
 * $Id: pid.h 533 2013-11-24 19:07:45Z zajost $
 *--------------------------------------------------------------------
 */
#ifndef PID_H
#define PID_H

#include "stdlib.h"
#include "L3G.h"
#include "FRTOS1.h"
#include <math.h>
#include "VL6180X.h"

typedef struct {
	int16_t dev, devOld, integ;
	portTickType timecount;
}pid_t;

uint8_t calcPID(uint8_t device, uint8_t kP, uint8_t kI, uint8_t kD, int16_t optValue, int16_t* corr);
void setToZero(void);
void setIntegToZero(uint8_t device);

#endif /* PID_H */
