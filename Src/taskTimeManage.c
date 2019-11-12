/*
 * taskTimeManage.c
 *
 *  Created on: 7 nov. 2019
 *      Author: Gerardo
 */
#include "FreeRTOS.h"
#include "task.h"
#include "taskTimeManage.h"
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_rcc.h"
#include "stm32l4xx_ll_system.h"
#include "stm32l4xx_ll_utils.h"
#include "stm32l4xx_ll_gpio.h"
#include "stdbool.h"
#include "stm32l4xx_ll_cortex.h"
#include "stm32l4xx_ll_pwr.h"
#include "stm32l4xx_ll_rtc.h"
#include "stm32l4xx_ll_exti.h"
#include "rtc.h"
#include <time.h>

/*private defines*/
typedef enum{
	JANUARY,
	FEBRUARY,
	MARCH,
	APRIL,
	MAY,
	JUNE,
	JULY,
	AUGUST,
	SEPTEMBER,
	OCTOBER,
	NOVEMBER,
	DECEMBER
}months;

uint32_t SleepTimeHour[12] =    {17, 18, 18, 20, 21, 21, 21, 20, 20, 19, 17, 16};
uint32_t SleepTimeMinutes[12] = {20, 15, 56, 30, 15, 30, 30, 40, 00, 05, 25, 50};

uint32_t WakeUpTimeHour[12] = {8,  7,  7,  7,  6,  6,  6,  7,  7,  8,  7,  8};
uint32_t WakeUpMinutes[12] = {35, 50, 00, 15, 30, 20, 30, 05, 30, 15, 50, 40};

/*private prototypes*/
void taskTimeManage_task(void * pvParametersid);

void taskTimeManage_init(void){

	if(LL_PWR_IsActiveFlag_SB() == 1){
		/* ##### Run after standby mode ##### */
		/* Clear Standby flag*/
		LL_PWR_ClearFlag_SB();

		/* Reset RTC Internal Wake up flag */
		LL_RTC_ClearFlag_WUT(RTC);
	}

	xTaskCreate(
			taskTimeManage_task,       /* Function that implements the task. */
			"Time management task",          /* Text name for the task. */
			configMINIMAL_STACK_SIZE,      /* Stack size in words, not bytes. */
			( void * ) 1,    /* Parameter passed into the task. */
			tskIDLE_PRIORITY,/* Priority at which the task is created. */
			NULL);      /* Used to pass out the created task's handle. */
}

uint32_t CalculateSeconds(months Month){
	return (((24 - SleepTimeHour[Month] + WakeUpTimeHour[Month])*60 - SleepTimeMinutes[Month] + WakeUpMinutes[Month])*60);
}

void CheckCurrentTime(int CurrentTimeHour, int CurrentTimeMinute, months Month){
	uint32_t SleepTime = 0;

	if((CurrentTimeHour == SleepTimeHour[Month]) && (CurrentTimeMinute == SleepTimeMinutes[Month])){
		SleepTime = CalculateSeconds(Month);
		StandByMode(SleepTime);
	}
}
void taskTimeManage_task(void * pvParameters ){

	int CurrentTimeMonth = 0;
	int CurrentTimeHour = 0;
	int CurrentTimeMinute = 0;

	while(1){
		CurrentTimeMonth = __LL_RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetMonth(RTC));
		CurrentTimeHour =  __LL_RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetHour(RTC));
		CurrentTimeMinute = __LL_RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetMinute(RTC));

		switch(CurrentTimeMonth){
		case JANUARY + 1:
		CheckCurrentTime(CurrentTimeHour, CurrentTimeMinute, JANUARY);
		break;
		case FEBRUARY + 1:
		CheckCurrentTime(CurrentTimeHour, CurrentTimeMinute, FEBRUARY);
		break;
		case MARCH + 1:
		CheckCurrentTime(CurrentTimeHour, CurrentTimeMinute, MARCH);
		break;
		case APRIL + 1:
		CheckCurrentTime(CurrentTimeHour, CurrentTimeMinute, APRIL);
		break;
		case MAY + 1:
		CheckCurrentTime(CurrentTimeHour, CurrentTimeMinute, MAY);
		break;
		case JUNE + 1:
		CheckCurrentTime(CurrentTimeHour, CurrentTimeMinute, JUNE);
		break;
		case JULY + 1:
		CheckCurrentTime(CurrentTimeHour, CurrentTimeMinute, JULY);
		break;
		case AUGUST + 1:
		CheckCurrentTime(CurrentTimeHour, CurrentTimeMinute, AUGUST);
		break;
		case SEPTEMBER + 1:
		CheckCurrentTime(CurrentTimeHour, CurrentTimeMinute, SEPTEMBER);
		break;
		case OCTOBER + 1:
		CheckCurrentTime(CurrentTimeHour, CurrentTimeMinute, OCTOBER);
		break;
		case NOVEMBER + 1:
		CheckCurrentTime(CurrentTimeHour, CurrentTimeMinute, NOVEMBER);
		break;
		case DECEMBER + 1:
		CheckCurrentTime(CurrentTimeHour, CurrentTimeMinute, DECEMBER);
		break;
		default:
			break;
		}

		vTaskDelay(100);
	}
}
