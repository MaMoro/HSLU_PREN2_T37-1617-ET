/*
 * serial_communication.c
 *
 *  Created on: Feb 23, 2017
 *      Author: pirzi
 */

#include "serial_communication.h"
#include "frtos_tasks.h"
#include "L3G.h"
#include "RED.h"
#include "VL6180X.h"
#include "CLS1.h"
#include "FRTOS1.h"
#include "driving.h"

uint8_t state;
bool leftParcour;

void startCommunication(void){
	uint8_t i;
	int16_t angelX;
	int16_t angelZ;
	while(VL_Init()!=ERR_OK){
		VL_Init();
	}
	L3Ginit();
	RED_Put(1);
	calculateOffset();
	RED_Put(0);
	
	leftParcour = 1;		// temporär
	state = 1;				// temporär
	
	//Main part
	for(;;){
		for(i=0; i<32; i++){
			L3Gread('x');
			L3Gread('z');
		}
		L3GgetDegree('x', &angelX);
		L3GgetDegree('z', &angelZ);
		

		
		//read();
		//write();
		
		FRTOS1_taskENTER_CRITICAL();
		CLS1_SendNum16s(angelX, CLS1_GetStdio()->stdOut);
		CLS1_SendStr(", ", CLS1_GetStdio()->stdOut);
		CLS1_SendNum16s(angelZ, CLS1_GetStdio()->stdOut);
		CLS1_SendStr("\n\r", CLS1_GetStdio()->stdOut);
		FRTOS1_taskEXIT_CRITICAL();
		
		refreshTasks();
		vTaskDelay(pdMS_TO_TICKS(120));
	}
}

void taskDone(uint8_t taskNbr){
	state = taskNbr;
}

void refreshTasks(void){
	  switch(state){
	  case 1: initDriving(4, 0, 1, 40, 0, 10, leftParcour);
		  	  CreateTask2();	// drive to stair
	  break;
	  case 2: CreateTask3();	// drive over stair to entanglement (verschränkung)
	  break;
	  case 3: CreateTask4();	// drive throught turningpoint to gully
	  break;
	  case 4: CreateTask5();	// drive over gully to seesaw
	  break;
	  case 5: CreateTask6();	// drive over seesaw to finishing room
	  break;
	  case 6: CreateTask7();	// push the right button
	  break;
	  default: ; //error
	  }
}
