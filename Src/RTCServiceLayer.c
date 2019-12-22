/*
 * RTCServiceLayer.c
 *
 *  Created on: 13 dic. 2019
 *      Author: Gerardo
 */


#include "stdbool.h"
#include "stm32l4xx_ll_rtc.h"
#include "RTCServiceLayer.h"
#include <time.h>
#include <stdio.h>


/*private defines*/

#define RTC_BKP_DATE_TIME_UPDTATED ((uint32_t)0x32F2)
#define RTC_ASYNCH_PREDIV          ((uint32_t)0x7F)
#define RTC_SYNCH_PREDIV           ((uint32_t)0x00F9)
#define RTC_WUT_TIME               ((uint32_t)5)     /* 5 s */

/*private prototypes*/


void StandByMode(uint32_t SleepTime){
	/*##-1- Disable RTC registers write protection ############################*/
	LL_RTC_DisableWriteProtection(RTC);

	/*##-2- Disable wake up timer to modify it ############################*/
	LL_RTC_WAKEUP_Disable(RTC);

	/*##-3- Wait for confirmation ################################ */
	while (LL_RTC_IsActiveFlag_WUTW(RTC) != 1)
	{

	}

	/*##-4- Alarm configuration #############################*/
	/* Setting the Wakeup time to RTC_WUT_TIME s
		       If LL_RTC_WAKEUPCLOCK_CKSPRE is selected, the frequency is 1Hz,
		       this allows to get a wakeup time equal to RTC_WUT_TIME s
		       if the counter is RTC_WUT_TIME */
	LL_RTC_WAKEUP_SetAutoReload(RTC, SleepTime);
	LL_RTC_WAKEUP_SetClock(RTC, LL_RTC_WAKEUPCLOCK_CKSPRE);

	/* Enable wake up counter and wake up interrupt */
	/* Note: Periodic wakeup interrupt should be enabled to exit the device
		     from low-power modes.*/
	LL_RTC_EnableIT_WUT(RTC);
	LL_RTC_WAKEUP_Enable(RTC);

	/* Enable RTC registers write protection */
	LL_RTC_EnableWriteProtection(RTC);

	/* ######## ENTER IN STANDBY MODE ######################################*/
	/** Request to enter STANDBY mode
	 * Following procedure describe in STM32L4xx Reference Manual
	 * See PWR part, section Low-power modes, Standby mode
	 */
	/* Reset Internal Wake up flag */
	LL_RTC_ClearFlag_WUT(RTC);

	/* Check that PWR Internal Wake-up is enabled */
	if (LL_PWR_IsEnabledInternWU() == 0)
	{
		/* Need to enable the Internal Wake-up line */
		LL_PWR_EnableInternWU();
	}

	/* Set Stand-by mode */
	LL_PWR_SetPowerMode(LL_PWR_MODE_STANDBY);

	/* Set SLEEPDEEP bit of Cortex System Control Register */
	LL_LPM_EnableDeepSleep();

	/* Request Wait For Interrupt */
	__WFI();

}

void changeRTCdate(uint8_t day, uint8_t month, uint8_t weekday, uint8_t year){

	LL_RTC_DateTypeDef RTC_DateStruct;

	RTC_DateStruct.Day = day;
	RTC_DateStruct.Month = month;
	RTC_DateStruct.WeekDay = weekday;
	RTC_DateStruct.Year = year;

	LL_RTC_DATE_Init(RTC, LL_RTC_FORMAT_BCD, &RTC_DateStruct);
}

void changeRTCTime(uint8_t seconds, uint8_t minutes, uint8_t hours){

	LL_RTC_TimeTypeDef RTC_TimeStruct;

	RTC_TimeStruct.Seconds = seconds;
	RTC_TimeStruct.Minutes = minutes;
	RTC_TimeStruct.Hours = hours;

	LL_RTC_TIME_Init(RTC, LL_RTC_FORMAT_BCD, &RTC_TimeStruct);
}

void getDate(LL_RTC_DateTypeDef * RTC_DateStruct){

	RTC_DateStruct->Day = __LL_RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetDay(RTC));
	RTC_DateStruct->Month = __LL_RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetMonth(RTC));
	RTC_DateStruct->WeekDay = __LL_RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetWeekDay(RTC));
	RTC_DateStruct->Year = __LL_RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetYear(RTC));
}

void getTime(LL_RTC_TimeTypeDef * RTC_TimeStruct){

	RTC_TimeStruct->Seconds = __LL_RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetSecond(RTC));
	RTC_TimeStruct->Minutes = __LL_RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetMinute(RTC));
	RTC_TimeStruct->Hours = __LL_RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetHour(RTC));
}
