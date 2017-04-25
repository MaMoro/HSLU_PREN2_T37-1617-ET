/*
 * serial_communication.h
 *
 *  Created on: Feb 23, 2017
 *      Author: pirzi
 */

#ifndef SERIAL_COMMUNICATION_H_
#define SERIAL_COMMUNICATION_H_

#include <stdlib.h>
#include "CLS1.h"

#define RXBUFSIZE 48


static const CLS1_ParseCommandCallback CmdParserTable[];


void startCommunication(void);

void readGyro(void);
uint8_t initAllSensors(void);
void sendStatus(void);
void readValues(void);
uint8_t getState(void);
void setState(uint8_t newState);
void sendStatusBT(void);
void setErrorState(uint8_t err, char* description);
void sendTestStatus(void);
void gyroReady(void);




#endif /* SERIAL_COMMUNICATION_H_ */
