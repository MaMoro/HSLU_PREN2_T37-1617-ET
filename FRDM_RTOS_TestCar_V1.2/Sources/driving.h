/*
 * driving.h
 *
 *  Created on: Feb 23, 2017
 *      Author: pirzi
 */

#ifndef DRIVING_H_
#define DRIVING_H_

/*
 * Drive stright to the stair
 * @param [in] speed, speed of the motors
 * @param [in] startupTime time from 0 to speed
 * @param [in] frontdistance drive until this range is detected
 */
void driveToStair(int8_t speed, uint8_t optRange, uint8_t startupTime, uint16_t frontdistance);
void driveOverStair(int8_t speed, uint8_t optRange);
void initDriving(uint8_t kpT, uint8_t kiT, uint8_t kdT, uint8_t kpG, uint8_t kiG, uint8_t kdG, bool isLeftParcour);
void driveToTurningPlace(int8_t speed, uint8_t optRange);
void driveThroughtTurningPlace(uint8_t speed, uint8_t frontdistance);


#endif /* DRIVING_H_ */
