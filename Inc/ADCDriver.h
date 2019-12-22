/*
 * ADCDriver.h
 *
 *  Created on: 13 dic. 2019
 *      Author: Gerardo
 */

#ifndef ADCDRIVER_H_
#define ADCDRIVER_H_
#include "stm32l4xx_hal.h"

void DriverConfigureADC();
uint32_t DriverStartADC1ConverionChannel1();
uint32_t DriverStartADC1ConverionChannel4();
uint32_t DriverStartADC1ConverionChannel3();

#endif /* ADCDRIVER_H_ */
