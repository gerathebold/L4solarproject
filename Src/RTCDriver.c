/*
 * RTCDriver.c
 *
 *  Created on: 13 dic. 2019
 *      Author: Gerardo
 */
#include <RTCDriver.h>
#include "stm32l4xx_hal.h"

void DriverConfigureRTC(){

	LL_RTC_InitTypeDef RTC_InitStruct;
	LL_RTC_TimeTypeDef RTC_TimeStruct;
	LL_RTC_DateTypeDef RTC_DateStruct;

	/* Peripheral clock enable */
	LL_RCC_EnableRTC();

	/**Initialize RTC and set the Time and Date
	 */
	RTC_InitStruct.HourFormat = LL_RTC_HOURFORMAT_24HOUR;
	RTC_InitStruct.AsynchPrescaler = 127;
	RTC_InitStruct.SynchPrescaler = 255;
	LL_RTC_Init(RTC, &RTC_InitStruct);

	/**Initialize RTC and set the Time and Date
	 */

	if(LL_RTC_BAK_GetRegister(RTC, LL_RTC_BKP_DR0) != 0x32F2){

		RTC_TimeStruct.Hours = 0;
		RTC_TimeStruct.Minutes = 0;
		RTC_TimeStruct.Seconds = 0;
		LL_RTC_TIME_Init(RTC, LL_RTC_FORMAT_BCD, &RTC_TimeStruct);

		RTC_DateStruct.WeekDay = LL_RTC_WEEKDAY_MONDAY;
		RTC_DateStruct.Month = LL_RTC_MONTH_JANUARY;
		RTC_DateStruct.Year = 0;
		LL_RTC_DATE_Init(RTC, LL_RTC_FORMAT_BCD, &RTC_DateStruct);

		LL_RTC_BAK_SetRegister(RTC,LL_RTC_BKP_DR0,0x32F2);
	}

}
