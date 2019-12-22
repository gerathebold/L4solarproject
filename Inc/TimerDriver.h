/*
 * TimerDriver.h
 *
 *  Created on: 13 dic. 2019
 *      Author: Gerardo
 */

#ifndef TIMERDRIVER_H_
#define TIMERDRIVER_H_
#include "stm32l4xx_hal.h"

void DriverConfigureTimer();
void DriverTimSetPWMDutyCycle(uint8_t value);
void DriverStartPWM();

#endif /* TIMERDRIVER_H_ */
