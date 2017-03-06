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

uint8_t state;
char leftParcour;

void startCommunication(void){
	while(VL_Init()!=ERR_OK){
		VL_Init();
	}
	L3Ginit();
	RED_Put(1);
	calculateOffset();
	RED_Put(0);
	
	leftParcour = 1;
	
	
	state = 1;		//default = 0
	//Main part
	for(;;){
		uint8_t i;
		for(i=0; i<32; i++){
			L3Gread('x');
		}
		refreshTasks();
		vTaskDelay(pdMS_TO_TICKS(120));
		
		//read();
		//write();
		
		
	}
}

void taskDone(uint8_t taskNbr){
	state = taskNbr;
}

void refreshTasks(void){
	  switch(state){
	  case 1: CreateTask2();	// drive to stair
	  break;
	  case 2: CreateTask3(leftParcour);	// drive over stair to entanglement (verschränkung)
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
