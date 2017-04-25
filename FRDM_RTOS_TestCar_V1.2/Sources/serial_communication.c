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
#include "LED_GREEN.h"
#include "VL6180X.h"
#include "FRTOS1.h"
#include "driving.h"
#include "MMA1.h"
#include <math.h>
#include "motor.h"
#include "BT1.h"


#define RASPISET 0



static bool courseSet = FALSE;
static bool ready;


// Comunication values reads
static bool hello = FALSE;		// kommunikation starten mit RPI3
static bool start = FALSE;		// startbefehl
static bool course = 0;			// fahrbanwahl	links->true rechts->false
static int16_t tof_l_s = 0;		// tof_links_sollwert
static int16_t tof_r_s = 0;		// tof_rechts_sollwert
static int16_t tof_f_s = 0;		// tof_front_sollwert
static int8_t 	raupe_i_l = 0;	// raupe ist links
static int8_t	raupe_i_r = 0;	// raupe ist rechts
static uint8_t	gyroskop_s = 0; // gyroskop soll
static uint8_t	servo_s = 0;	// servo soll
static uint8_t	letter = 0;		// buchstabe

// Comunication values writes
static int16_t tof_l_i = 0;		// tof links ist
static int16_t tof_r_i = 0;		// tof rechts ist
static int16_t tof_f_i = 0;		// tof front ist
static int8_t	raupe_l_i = 0;	// raupe links ist
static int8_t	raupe_r_i = 0;	// raupe rechts ist
static int16_t gyro_n = 0;		// gyro nick 
static int16_t gyro_g = 0;		// gyro gear
static uint8_t 	gyroskop_i = 0;	// gyroskop ist
static uint8_t 	servo_i = 0;	// servo ist
static uint8_t 	state = 1;		// status auf parcour
static uint8_t 	errState = ERR_OK;	// errorStatus

static uint8_t kpT = 20, kiT = 0, kdT = 0;		// 20, 0, 0
static uint8_t kpG = 20, kiG = 1, kdG = 8;		// 15, 1, 5

static uint8_t ParseCommand(const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io);

static const CLS1_ParseCommandCallback CmdParserTable[] =
{
  CLS1_ParseCommand, /* default shell parser */
  ParseCommand, /* my own shell parser */
  NULL /* Sentinel, must be last */
};

/* Bluetooth stdio */
static CLS1_ConstStdIOType BT_stdio = {
  (CLS1_StdIO_In_FctType)BT1_StdIOReadChar, /* stdin */
  (CLS1_StdIO_OutErr_FctType)BT1_StdIOSendChar, /* stdout */
  (CLS1_StdIO_OutErr_FctType)BT1_StdIOSendChar, /* stderr */
  BT1_StdIOKeyPressed /* if input is not empty */
};


// Communication Task
void startCommunication(void){
	unsigned char RXbuffer[RXBUFSIZE];
	unsigned char RXbufferBT[RXBUFSIZE];
	RXbufferBT[0] = '\0';
	RXbuffer[0] = '\0';

	int16_t i;
	setServoPWM(0);
	setGyroskopPWM(0);
	//vTaskDelay(pdMS_TO_TICKS(8000));
	
	//Init sensors
	errState = initAllSensors();
	
	// Start GyroTask
	CreateGyroTask();
	
#if RASPISET
	//Say hello to Raspberry Pi
	while(!hello){
		(void)CLS1_ReadAndParseWithCommandTable(RXbuffer, sizeof(RXbuffer), CLS1_GetStdio(), CmdParserTable);
		(void)CLS1_ReadAndParseWithCommandTable(RXbufferBT, sizeof(RXbufferBT), &BT_stdio, CmdParserTable);
		refreshMovingOffset('x');
		vTaskDelay(pdMS_TO_TICKS(10));
	}

	  
	// wait until course is set
	while(!courseSet){
		(void)CLS1_ReadAndParseWithCommandTable(RXbuffer, sizeof(RXbuffer), CLS1_GetStdio(), CmdParserTable);
		(void)CLS1_ReadAndParseWithCommandTable(RXbufferBT, sizeof(RXbufferBT), &BT_stdio, CmdParserTable);
		refreshMovingOffset('x');
		vTaskDelay(pdMS_TO_TICKS(10));
	}
#endif

	initDriving(kpT, kiT, kdT, kpG, kiG, kdG, course);
	
#if RASPISET
	// Wait until start
	LED_GREEN_Put(1);
	while(!start){
		(void)CLS1_ReadAndParseWithCommandTable(RXbuffer, sizeof(RXbuffer), CLS1_GetStdio(), CmdParserTable);
		(void)CLS1_ReadAndParseWithCommandTable(RXbufferBT, sizeof(RXbufferBT), &BT_stdio, CmdParserTable);
		refreshMovingOffset('x');
		vTaskDelay(pdMS_TO_TICKS(10));
	}
	LED_GREEN_Put(0);
#endif
	int16_t rangeL;
	int16_t rangeR;
	
/*	for(;;){
		VL_GetDistance(TOFLEFT, &rangeL);
		VL_GetDistance(TOFRIGHT, &rangeR);
		CLS1_SendNum16s(rangeL, CLS1_GetStdio()->stdOut);
		CLS1_SendStr((uint8_t*)",\t", CLS1_GetStdio()->stdOut);
		//pwm right
		CLS1_SendNum16s(rangeR, CLS1_GetStdio()->stdOut);
		CLS1_SendStr((uint8_t*)"\n", CLS1_GetStdio()->stdOut);
		vTaskDelay(pdMS_TO_TICKS(30));
	}*/
	
	L3GSetAngel(GEAR, 0);
	L3GSetAngel(NICK, 0);
	while(!ready){
		vTaskDelay(pdMS_TO_TICKS(30));
	}
	CreateDrivingTask();
	
	//Loop
	for(;;){
		// Parse the commands
		(void)CLS1_ReadAndParseWithCommandTable(RXbuffer, sizeof(RXbuffer), CLS1_GetStdio(), CmdParserTable);
		(void)CLS1_ReadAndParseWithCommandTable(RXbufferBT, sizeof(RXbufferBT), &BT_stdio, CmdParserTable);
		readValues();
		//sendStatus();
		sendStatusBT();
		sendTestStatus();
		
		vTaskDelay(pdMS_TO_TICKS(300));
	}
}
/*
 * Set the status of the Parcours
 * @param newState new state whitch is set;
 */
void setState(uint8_t newState){
	state = newState;
}

/*
 * Get the state of the Parcours
 * @return ParcourState
 */
uint8_t getState(void){
	return state;
}

/*
 * ParseTable for Comunication
 * @param cmd pointer to the Comand
 * @param handled pointer to the boolen if the comand is handled or not
 * @param io pointer to the stdioType
 * @return error State
 */
static uint8_t ParseCommand(const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io) {
  uint8_t res = ERR_OK;
  int32_t tmp;
  const uint8_t *p;
  uint8_t buf[16];

  
  //Help
  if (UTIL1_strcmp((char*)cmd, CLS1_CMD_HELP)==0 || UTIL1_strcmp((char*)cmd, "help")==0) {
	//FRTOS1_taskENTER_CRITICAL(); ////////CRITICAL///////////
		CLS1_SendHelpStr((unsigned char*)"app", (const unsigned char*)"Group of app commands\n", io->stdOut);
		CLS1_SendHelpStr((unsigned char*)"  hello,<bool>", (const unsigned char*)"True->FRDM, False->RBI\n", io->stdOut);
		CLS1_SendHelpStr((unsigned char*)"  start,<bool>", (const unsigned char*)"Set 1 to start\n", io->stdOut);
		CLS1_SendHelpStr((unsigned char*)"  course,<bool>", (const unsigned char*)"true->links, false->rechts\n", io->stdOut);
		CLS1_SendHelpStr((unsigned char*)"  tof_l_s,<uint16>", (const unsigned char*)"tof_links_sollwert\n", io->stdOut);
		CLS1_SendHelpStr((unsigned char*)"  tof_r_s,<uint16>", (const unsigned char*)"tof_rechts_sollwert\n", io->stdOut);
		CLS1_SendHelpStr((unsigned char*)"  tof_f_s,<uint16>", (const unsigned char*)"tof_front_sollwert\n", io->stdOut);
		CLS1_SendHelpStr((unsigned char*)"  raupe_i_l,<int8>", (const unsigned char*)"raupe_links_ist\n", io->stdOut);
		CLS1_SendHelpStr((unsigned char*)"  raupe_i_r,<int8>", (const unsigned char*)"raupe_rechts_ist\n", io->stdOut);
		CLS1_SendHelpStr((unsigned char*)"  gyroskop_s,<uint8_t>", (const unsigned char*)"gyroskop_soll\n", io->stdOut);
		CLS1_SendHelpStr((unsigned char*)"  servo_s,<uint8_t>", (const unsigned char*)"servo_soll\n", io->stdOut);
		CLS1_SendHelpStr((unsigned char*)"  letter,<uint8_t>", (const unsigned char*)"letter_to_push\n", io->stdOut);
   // FRTOS1_taskEXIT_CRITICAL();	///////////Critical End//////////////
    *handled = TRUE;
    //Status
  } else if ((UTIL1_strcmp((char*)cmd, CLS1_CMD_STATUS)==0) || (UTIL1_strcmp((char*)cmd, "status")==0)) {
	CLS1_SendStatusStr((unsigned char*)"app", (const unsigned char*)"\n", io->stdOut);
	// start
	UTIL1_Num32sToStr(buf, sizeof(buf), start);						
	UTIL1_strcat(buf, sizeof(buf), (const unsigned char*)"\n");
	CLS1_SendStatusStr((const uint8_t*)"  start,", buf, io->stdOut);
    *handled = TRUE;
    // command handling
  } else if (UTIL1_strncmp((char*)cmd, "hello,", sizeof("hello,")-1)==0){		//Start
	  p = cmd+sizeof("hello,")-1;
	  res = UTIL1_xatoi(&p, &tmp);
	  if(res==ERR_OK){
		  hello = tmp;
		  CLS1_SendStr((uint8_t*)"hello,", CLS1_GetStdio()->stdOut);
		  CLS1_SendNum8u(hello, CLS1_GetStdio()->stdOut);
		  CLS1_SendStr((uint8_t*)"\n", CLS1_GetStdio()->stdOut);
		  *handled = TRUE;
	  }
  }else if (UTIL1_strncmp((char*)cmd, "start,", sizeof("start,")-1)==0){		//Start
	  p = cmd+sizeof("start,")-1;
	  res = UTIL1_xatoi(&p, &tmp);
	  if(res==ERR_OK){
		  start = tmp;
		  //bestätigen
		  CLS1_SendStr((uint8_t*)"start,", CLS1_GetStdio()->stdOut);
		  CLS1_SendNum8u(start, CLS1_GetStdio()->stdOut);
		  CLS1_SendStr((uint8_t*)"\n", CLS1_GetStdio()->stdOut);
		  *handled = TRUE;
	  }
  }else if (UTIL1_strncmp((char*)cmd, "course,", sizeof("course,")-1)==0){		// Course
	  p = cmd+sizeof("course,")-1;
	  res = UTIL1_xatoi(&p, &tmp);
	  if(res==ERR_OK){
		  course = tmp;
		  courseSet = 1;
		  //bestätigen
		  CLS1_SendStr((uint8_t*)"course,", CLS1_GetStdio()->stdOut);
		  CLS1_SendNum8u(course, CLS1_GetStdio()->stdOut);
		  CLS1_SendStr((uint8_t*)"\n", CLS1_GetStdio()->stdOut);
		  *handled = TRUE;
	  }
}else if (UTIL1_strncmp((char*)cmd, "tof_l_s,", sizeof("tof_l_s,")-1)==0){		// tof_l_s
	  p = cmd+sizeof("tof_l_s,")-1;
	  res = UTIL1_xatoi(&p, &tmp);
	  if(res==ERR_OK){
		  tof_l_s = tmp;
		  setDistanceSide(tof_l_s);
		  *handled = TRUE;
	  }
}else if (UTIL1_strncmp((char*)cmd, "tof_r_s,", sizeof("tof_r_s,")-1)==0){		// tof_r_s
	  p = cmd+sizeof("tof_r_s,")-1;
	  res = UTIL1_xatoi(&p, &tmp);
	  if(res==ERR_OK){
		  tof_r_s = tmp;
		  setDistanceSide(tof_r_s);
		  *handled = TRUE;
	  }
}else if (UTIL1_strncmp((char*)cmd, "tof_f_s,", sizeof("tof_f_s,")-1)==0){		// tof_f_s
	  p = cmd+sizeof("tof_f_s,")-1;
	  res = UTIL1_xatoi(&p, &tmp);
	  if(res==ERR_OK){
		  tof_f_s = tmp;
		  setDistanceFront(tof_f_s);
		  *handled = TRUE;
	  }
}else if (UTIL1_strncmp((char*)cmd, "raupe_i_l,", sizeof("raupe_i_l,")-1)==0){		// raupe_i_l
	  p = cmd+sizeof("raupe_i_l,")-1;
	  res = UTIL1_xatoi(&p, &tmp);
	  if(res==ERR_OK){
		  raupe_i_l = tmp;
		  setSpeed(raupe_i_l);
		  *handled = TRUE;
	  }
}else if (UTIL1_strncmp((char*)cmd, "raupe_i_r,", sizeof("raupe_i_r,")-1)==0){		// raupe_i_r
	  p = cmd+sizeof("raupe_i_r,")-1;
	  res = UTIL1_xatoi(&p, &tmp);
	  if(res==ERR_OK){
		  raupe_i_r = tmp;
		  setSpeed(raupe_i_r);
		  *handled = TRUE;
	  }
}else if (UTIL1_strncmp((char*)cmd, "gyroskop_s,", sizeof("gyroskop_s,")-1)==0){		// gyroskop_s
	  p = cmd+sizeof("gyroskop_s,")-1;
	  res = UTIL1_xatoi(&p, &tmp);
	  if(res==ERR_OK){
		  gyroskop_s = tmp;
		  setGyroskopPWM(gyroskop_s);
		  gyroskop_i = gyroskop_s;
		  *handled = TRUE;
	  }
}else if (UTIL1_strncmp((char*)cmd, "servo_s,", sizeof("servo_s,")-1)==0){		// servo_s
	  p = cmd+sizeof("servo_s,")-1;
	  res = UTIL1_xatoi(&p, &tmp);
	  if(res==ERR_OK){
		  servo_s = tmp;
		  setServoPWM(servo_s);
		  servo_i = servo_s;
		  *handled = TRUE;
	  }
}else if (UTIL1_strncmp((char*)cmd, "letter,", sizeof("letter,")-1)==0){		// letter
	  p = cmd+sizeof("letter,")-1;
	  res = UTIL1_xatoi(&p, &tmp);
	  if(res==ERR_OK){
		  letter = tmp;
		  setLetter(letter);
		  *handled = TRUE;
	  }
	  // bestätigen
		  CLS1_SendStr((uint8_t*)"letter,", CLS1_GetStdio()->stdOut);
		  CLS1_SendNum8u(letter, CLS1_GetStdio()->stdOut);
		  CLS1_SendStr((uint8_t*)"\n", CLS1_GetStdio()->stdOut);
}else if (UTIL1_strncmp((char*)cmd, "kpG,", sizeof("kpG,")-1)==0){		// kpG
	  p = cmd+sizeof("kpG,")-1;
	  res = UTIL1_xatoi(&p, &tmp);
	  if(res==ERR_OK){
		  kpG = tmp;
		  setPID(kpT, kiT, kdT, kpG, kiG, kdG);
		  *handled = TRUE;
	  }
}
else if (UTIL1_strncmp((char*)cmd, "kiG,", sizeof("kiG,")-1)==0){		// kiG
	  p = cmd+sizeof("kiG,")-1;
	  res = UTIL1_xatoi(&p, &tmp);
	  if(res==ERR_OK){
		  kiG = tmp;
		  setPID(kpT, kiT, kdT, kpG, kiG, kdG);
		  *handled = TRUE;
	  }
}else if (UTIL1_strncmp((char*)cmd, "kdG,", sizeof("kdG,")-1)==0){		// kdG
	  p = cmd+sizeof("kdG,")-1;
	  res = UTIL1_xatoi(&p, &tmp);
	  if(res==ERR_OK){
		  kdG = tmp;
		  setPID(kpT, kiT, kdT, kpG, kiG, kdG);
		  *handled = TRUE;
	  }
}else if (UTIL1_strncmp((char*)cmd, "kpT,", sizeof("kpT,")-1)==0){		// kpT
	  p = cmd+sizeof("kpT,")-1;
	  res = UTIL1_xatoi(&p, &tmp);
	  if(res==ERR_OK){
		  kpT = tmp;
		  setPID(kpT, kiT, kdT, kpG, kiG, kdG);
		  *handled = TRUE;
	  }
}else if (UTIL1_strncmp((char*)cmd, "kiT,", sizeof("kiT,")-1)==0){		// kiT
	  p = cmd+sizeof("kiT,")-1;
	  res = UTIL1_xatoi(&p, &tmp);
	  if(res==ERR_OK){
		  kiT = tmp;
		  setPID(kpT, kiT, kdT, kpG, kiG, kdG);
		  *handled = TRUE;
	  }
}else if (UTIL1_strncmp((char*)cmd, "kdT,", sizeof("kdT,")-1)==0){		// kdT
	  p = cmd+sizeof("kdT,")-1;
	  res = UTIL1_xatoi(&p, &tmp);
	  if(res==ERR_OK){
		  kdT = tmp;
		  setPID(kpT, kiT, kdT, kpG, kiG, kdG);
		  *handled = TRUE;
	  }
}else if (UTIL1_strncmp((char*)cmd, "stop,", sizeof("stop,")-1)==0){		// stop
	p = cmd+sizeof("kdT,")-1;
	res = UTIL1_xatoi(&p, &tmp);
	if(res==ERR_OK){
		stopDriving(tmp);
		*handled = TRUE;
	}
}
  return res;
}

/*
 * Init the devices for the DrivingTask
 * @return errorState
 */
uint8_t initAllSensors(void){
	uint8_t err = ERR_OK;
	RED_Put(1);
	TMOUT1_Init();
	//ToF Init
	err = VL_Init();
	while (err != ERR_OK) {
		err = VL_Init();
	setErrorState(err, "VL_init in comunication");
	}
	
	//BT Init
	BT1_Init();
	err = BT1_btSetDeviceName((byte*)"T37");
	err = BT1_btSetBaud(9600);
	err = BT1_StdOKCmd((byte*)"AT+NAME T37\r\n");
	err = BT1_StdOKCmd((byte*)"ENTER");
	//\todo BT init doesn't work

	//Accel init
	err = MMA1_Enable();
	while (err != ERR_OK) {
		err = MMA1_Enable();
		setErrorState(err, "MMA1_Enable in comunication");
	}
	err = MMA1_Init();
	while (err != ERR_OK) {
		err = MMA1_Init();
		setErrorState(err, "MMA1_Init in comunication");
	}
	MMA1_CalibrateX1g();
	MMA1_CalibrateY1g();
	MMA1_CalibrateZ1g();
	
	RED_Put(0);
	return err;
}

/*
 * Send the Informations about the Driving values to the Raspberry PI
 */
void sendStatus(void){
	CLS1_SendStr((uint8_t*)"tof_l_i,", CLS1_GetStdio()->stdOut);		// tof_l_i
	CLS1_SendNum16s(tof_l_i, CLS1_GetStdio()->stdOut);
	CLS1_SendStr((uint8_t*)"\n", CLS1_GetStdio()->stdOut);
	CLS1_SendStr((uint8_t*)"tof_r_i,", CLS1_GetStdio()->stdOut);		// tof_r_i
	CLS1_SendNum16s(tof_r_i, CLS1_GetStdio()->stdOut);
	CLS1_SendStr((uint8_t*)"\n", CLS1_GetStdio()->stdOut);
	CLS1_SendStr((uint8_t*)"tof_f_i,", CLS1_GetStdio()->stdOut);		// tof_f_i
	CLS1_SendNum16s(tof_f_i, CLS1_GetStdio()->stdOut);
	CLS1_SendStr((uint8_t*)"\n", CLS1_GetStdio()->stdOut);
	CLS1_SendStr((uint8_t*)"raupe_l_i,", CLS1_GetStdio()->stdOut);		// raupe_l_i
	CLS1_SendNum8s(raupe_l_i, CLS1_GetStdio()->stdOut);
	CLS1_SendStr((uint8_t*)"\n", CLS1_GetStdio()->stdOut);
	CLS1_SendStr((uint8_t*)"raupe_r_i,", CLS1_GetStdio()->stdOut);		// raupe_r_i
	CLS1_SendNum8s(raupe_r_i, CLS1_GetStdio()->stdOut);
	CLS1_SendStr((uint8_t*)"\n", CLS1_GetStdio()->stdOut);
	CLS1_SendStr((uint8_t*)"gyro_n,", CLS1_GetStdio()->stdOut);			// gyro_n
	CLS1_SendNum16s(gyro_n, CLS1_GetStdio()->stdOut);
	CLS1_SendStr((uint8_t*)"\n", CLS1_GetStdio()->stdOut);
	CLS1_SendStr((uint8_t*)"gyro_g,", CLS1_GetStdio()->stdOut);			// gyro_g
	CLS1_SendNum16s(gyro_g, CLS1_GetStdio()->stdOut);
	CLS1_SendStr((uint8_t*)"\n", CLS1_GetStdio()->stdOut);
	CLS1_SendStr((uint8_t*)"gyroskop_i,", CLS1_GetStdio()->stdOut);		// gyroskop_i
	CLS1_SendNum16u(gyroskop_i, CLS1_GetStdio()->stdOut);
	CLS1_SendStr((uint8_t*)"\n", CLS1_GetStdio()->stdOut);
	CLS1_SendStr((uint8_t*)"servo_i,", CLS1_GetStdio()->stdOut);		// servo_i
	CLS1_SendNum16u(servo_i, CLS1_GetStdio()->stdOut);
	CLS1_SendStr((uint8_t*)"\n", CLS1_GetStdio()->stdOut);
	CLS1_SendStr((uint8_t*)"parcstate,", CLS1_GetStdio()->stdOut);			// state
	CLS1_SendNum16u(state, CLS1_GetStdio()->stdOut);
	CLS1_SendStr((uint8_t*)"\n", CLS1_GetStdio()->stdOut);
	CLS1_SendStr((uint8_t*)"errstate,", CLS1_GetStdio()->stdOut);		// errState
	CLS1_SendNum16u(errState, CLS1_GetStdio()->stdOut);
	CLS1_SendStr((uint8_t*)"\n", CLS1_GetStdio()->stdOut);
	if(errState!=ERR_OK){
		errState = ERR_OK;	//reset errState after sending
	}
}

/*
 * Read all values for Comunication
 */
void readValues(void){
	tof_l_i = VL_GetLastDistance(TOFLEFT);
	tof_r_i = VL_GetLastDistance(TOFRIGHT);
	tof_f_i = VL_GetLastDistance(TOFFRONT);
	raupe_l_i = (int8_t)motorGetPWMLeft();
	raupe_r_i = (int8_t)motorGetPWMRight();
	L3GgetDegree(GEAR, &gyro_g);
	L3GgetDegree(NICK, &gyro_n);	
}

/*
 * Send status to the Bluetooth for the SerialChart (just for debugging purposes)
 */
void sendStatusBT(void){
	// gyro_g
	CLS1_SendNum16s(gyro_g, BT_stdio.stdOut);
	CLS1_SendStr((uint8_t*)",", BT_stdio.stdOut);
	// gyro_n
	CLS1_SendNum16s(gyro_n, BT_stdio.stdOut);
	CLS1_SendStr((uint8_t*)"\n", BT_stdio.stdOut);
}

/*
 * Send Test Status insteed of the Bluetooth (can be removed if Bluetooth is working)
 */
void sendTestStatus(void){
	// gyro_g
	CLS1_SendNum16s(gyro_g, CLS1_GetStdio()->stdOut);
	CLS1_SendStr((uint8_t*)",\t", CLS1_GetStdio()->stdOut);
	// gyro_n
	CLS1_SendNum16s(gyro_n, CLS1_GetStdio()->stdOut);
	CLS1_SendStr((uint8_t*)",\t", CLS1_GetStdio()->stdOut);
	// tof left
	CLS1_SendNum16s(tof_l_i, CLS1_GetStdio()->stdOut);
	CLS1_SendStr((uint8_t*)",\t", CLS1_GetStdio()->stdOut);
	//tof right
	CLS1_SendNum16s(tof_r_i, CLS1_GetStdio()->stdOut);
	CLS1_SendStr((uint8_t*)"\n", CLS1_GetStdio()->stdOut);
}

/*
 * Set the error state if an error occured somewhere
 * @param err errorState
 * @param description of the error and where it occured
 */
void setErrorState(uint8_t err, char* description){
	errState = err;
}

void gyroReady(void){
	ready = 1;
}
