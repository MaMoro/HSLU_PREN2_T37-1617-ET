/* ###################################################################
**     Filename    : main.c
**     Project     : FRDM_RTOS_TestCar_V1.2
**     Processor   : MKL25Z128VLK4
**     Version     : Driver 01.01
**     Compiler    : GNU C Compiler
**     Date/Time   : 2017-02-24, 17:59, # CodeGen: 0
**     Abstract    :
**         Main module.
**         This module contains user's application code.
**     Settings    :
**     Contents    :
**         No public methods
**
** ###################################################################*/
/*!
** @file main.c
** @version 01.01
** @brief
**         Main module.
**         This module contains user's application code.
*/         
/*!
**  @addtogroup main_module main module documentation
**  @{
*/         
/* MODULE main */


/* Including needed modules to compile this module/procedure */
#include "Cpu.h"
#include "Events.h"
#include "FRTOS1.h"
#include "KSDK1.h"
#include "WAIT1.h"
#include "CI2C1.h"
#include "CS1.h"
#include "TU1.h"
#include "TofCE3.h"
#include "PWM_Gyro.h"
#include "PwmLdd3.h"
#include "TU2.h"
#include "PWM_Servo.h"
#include "PwmLdd4.h"
#include "MMA1.h"
#include "GI2C2.h"
#include "CI2C2.h"
#include "DIR_LEFT1.h"
#include "DIR_RIGHT1.h"
#include "Watermark.h"
#include "UTIL1.h"
#include "GI2C1.h"
#include "AS1.h"
#include "ASerialLdd1.h"
#include "TofCE1.h"
#include "TofCE2.h"
#include "PWM_RIGHT.h"
#include "PwmLdd1.h"
#include "PWM_LEFT.h"
#include "PwmLdd2.h"
#include "DIR_LEFT.h"
#include "DIR_RIGHT.h"
#include "LED_GREEN.h"
#include "LEDpin3.h"
#include "BitIoLdd3.h"
#include "AD1.h"
#include "AdcLdd1.h"
#include "RED.h"
#include "LEDpin4.h"
#include "BitIoLdd4.h"
#include "TSK1.h"
/* Including shared modules, which are used for whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"

/* User includes (#include below this line is not maintained by Processor Expert) */

/*lint -save  -e970 Disable MISRA rule (6.3) checking. */
int main(void)
/*lint -restore Enable MISRA rule (6.3) checking. */
{
  /* Write your local variable definition here */

  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
  PE_low_level_init();
  /*** End of Processor Expert internal initialization.                    ***/

  /* Write your code here */
  /* For example: for(;;) { } */

  /*** Don't write any code pass this line, or it will be deleted during code generation. ***/
  /*** RTOS startup code. Macro PEX_RTOS_START is defined by the RTOS component. DON'T MODIFY THIS CODE!!! ***/
  #ifdef PEX_RTOS_START
    PEX_RTOS_START();                  /* Startup of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of RTOS startup code.  ***/
  /*** Processor Expert end of main routine. DON'T MODIFY THIS CODE!!! ***/
  for(;;){}
  /*** Processor Expert end of main routine. DON'T WRITE CODE BELOW!!! ***/
} /*** End of main routine. DO NOT MODIFY THIS TEXT!!! ***/

/* END main */
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
