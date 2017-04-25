/*
 * driving.h
 *
 *  Created on: Feb 23, 2017
 *      Author: pirzi
 */

#ifndef DRIVING_H_
#define DRIVING_H_


#define PI 3.14159265
#define LEFT -1
#define RIGHT 1
#define MAXTIMEOUT 5000		// timeout = 2s


/*
 * Drive stright to the stair
 * @param [in] speed, speed of the motors
 * @param [in] startupTime time from 0 to speed
 * @param [in] frontdistance drive until this range is detected
 */
void driveToStair(void);
void driveOverStair(void);
void initDriving(uint8_t kpT, uint8_t kiT, uint8_t kdT, uint8_t kpG, uint8_t kiG, uint8_t kdG, bool isLeftParcour);
void driveToTurningPlace(void);
void driveThroughTurningPlace(void);
void driveToEndZone(void);
void pushTheButton(void);
uint8_t regulateMotor(void);
void setDistanceSide(int16_t value);
void setDistanceFront(int16_t value);
void setSpeed(int8_t value);
void setGyroskopPWM(uint8_t value);
void setLetter(uint8_t value);
void setServoPWM(uint16_t value);
void setPID(uint8_t kpT, uint8_t kiT, uint8_t kdT, uint8_t kpG, uint8_t kiG, uint8_t kdG);
void stopDriving(bool stp);
uint8_t getTime(uint16_t* time);

#endif /* DRIVING_H_ */
