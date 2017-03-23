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


static const CLS1_ParseCommandCallback CmdParserTable[];


void startCommunication(void);

void readGyro(void);
uint8_t initAllDevices(void);
void sendStatus(void);
void readValues(void);
uint8_t getState();
void setState(uint8_t newState);
void sendStatusBT(void);



#endif /* SERIAL_COMMUNICATION_H_ */
