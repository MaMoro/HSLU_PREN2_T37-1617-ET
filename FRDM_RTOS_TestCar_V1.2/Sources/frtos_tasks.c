
/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.3 [05.09]
**     for the Freescale Kinetis series of microcontrollers.
**
** ###################################################################
*/

/* Begin of <includes> initialization, DO NOT MODIFY LINES BELOW */

#include "TSK1.h"
#include "FRTOS1.h"
#include "frtos_tasks.h"

/* End <includes> initialization, DO NOT MODIFY LINES ABOVE */

#include "RED.h"
#include "LED_GREEN.h"
#include <stdlib.h>
#include "serial_communication.h"
#include "driving.h"
#include "L3G.h"

uint8_t Task2Started;
uint8_t Task3Started;
uint8_t Task4Started;
uint8_t	Task5Started;
uint8_t Task6Started;
uint8_t Task7Started;
uint8_t GyroTaskStarted;

static portTASK_FUNCTION(ComunicationTask, pvParameters) {
  /* Write your task initialization code here ... */
(void) pvParameters;

  for(;;) {
	  
	  startCommunication();
	  
  }
  /* Destroy the task */
  vTaskDelete(ComunicationTask);
}

static portTASK_FUNCTION(DrivingTask, pvParameters) {

  /* Write your task initialization code here ... */
(void) pvParameters;

	driveToStair(50, 150, 200);

	driveOverStair(50, 150);

	driveToTurningPlace(50, 150);

	driveThroughtTurningPlace(50, 190, 200);

	driveToEndZone(50, 150, 200);

	pushTheButton(3, 190);
	
	vTaskDelay(pdMS_TO_TICKS(1) );
	FRTOS1_vTaskSuspend(NULL);

  /* Destroy the task */
  FRTOS1_vTaskDelete(DrivingTask);
}

static portTASK_FUNCTION(GyroTask, pvParameters) {

  /* Write your task initialization code here ... */
(void) pvParameters;
  	for(;;) {
    /* Write your task code here ... */
	
  		uint8_t i;
  		int8_t res;
  		//read the gyro
  		for(i=0; i<32; i++){
  			res = L3Gread('x');
  			if(res!=ERR_OK){
  				setErrorState(res, "GyroTask");
  			}
  			if(getState() == 2){
  				res = L3Gread('z');
  				if(res!=ERR_OK){
  					setErrorState(res, "GyroTask");
  				}
  			}
  		}
  		vTaskDelay(pdMS_TO_TICKS(125));
  }
  /* Destroy the task */
  vTaskDelete(GyroTask);
}

void CreateTasks(void) {
  if (FRTOS1_xTaskCreate(
	 ComunicationTask,  /* pointer to the task */
	  "Task1", /* task name for kernel awareness debugging */
	  configMINIMAL_STACK_SIZE + 400, /* task stack size */
	  (void*)NULL, /* optional task startup argument */
	  tskIDLE_PRIORITY + 4,  /* initial priority */
	  (xTaskHandle*)NULL /* optional task handle to create */
	) != pdPASS) {
	  /*lint -e527 */
	  for(;;){}; /* error! probably out of memory */
	  /*lint +e527 */
  }
}

void CreateDrivingTask(void){
	if(!Task2Started){
	  if (FRTOS1_xTaskCreate(
		 DrivingTask,  /* pointer to the task */
		  "Task2", /* task name for kernel awareness debugging */
		  configMINIMAL_STACK_SIZE + 800, /* task stack size */
		  (void*)NULL, /* optional task startup argument */
		  tskIDLE_PRIORITY + 5,  /* initial priority */
		  (xTaskHandle*)NULL /* optional task handle to create */
		) != pdPASS) {
		  /*lint -e527 */
		  for(;;){}; /* error! probably out of memory */
		  /*lint +e527 */
	  }
	}
	Task2Started = 1;
}


void CreateGyroTask(void){
	if(!GyroTaskStarted){
	  if (FRTOS1_xTaskCreate(
		 GyroTask,  /* pointer to the task */
		  "Gyro", /* task name for kernel awareness debugging */
		  configMINIMAL_STACK_SIZE + 200, /* task stack size */
		  (void*)NULL, /* optional task startup argument */
		  tskIDLE_PRIORITY + 6,  /* initial priority */
		  (xTaskHandle*)NULL /* optional task handle to create */
		) != pdPASS) {
		  /*lint -e527 */
		  for(;;){}; /* error! probably out of memory */
		  /*lint +e527 */
	  }
	}
	GyroTaskStarted = 1;
}

