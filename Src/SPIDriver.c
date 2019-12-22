/*
 * SPIDriver.c
 *
 *  Created on: 13 dic. 2019
 *      Author: Gerardo
 */
#include <SPIDriver.h>
#include "stm32l4xx_hal.h"

SPI_HandleTypeDef hspi3;

void DriverConfigureSPI(){
	/* SPI3 parameter configuration*/
	  hspi3.Instance = SPI3;
	  hspi3.Init.Mode = SPI_MODE_MASTER;
	  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
	  hspi3.Init.DataSize = SPI_DATASIZE_9BIT;
	  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
	  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
	  hspi3.Init.NSS = SPI_NSS_SOFT;
	  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
	  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
	  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	  hspi3.Init.CRCPolynomial = 7;
	  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	  if (HAL_SPI_Init(&hspi3) != HAL_OK)
	  {
	    _Error_Handler(__FILE__, __LINE__);
	  }
}

void DriverWriteSPIcommand(unsigned int command){
	uint16_t transmitBuffer[1];
	transmitBuffer[0] = command & ~0x100;
	HAL_SPI_Transmit(&hspi3, (uint8_t*)transmitBuffer, 1, 50);
}

void DriverWriteSPIdata(unsigned int data){
	uint16_t transmitBuffer[1];
	transmitBuffer[0] = data | 0x100;
	HAL_SPI_Transmit(&hspi3, (uint8_t*)transmitBuffer, 1, 50);
}
