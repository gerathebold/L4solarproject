/*
 * globalInfo.h
 *
 *  Created on: 4 dic. 2019
 *      Author: Gerardo
 */

#ifndef GLOBALINFO_H_
#define GLOBALINFO_H_

#include "FreeRTOS.h"
#include "semphr.h"

typedef enum{
	WD_MONITORING_STATE,
	NORMAL_WORKING_MODE,
	EMERGENCY_MODE_RECHARGE_BAT1,
	EMERGENCY_MODE_RECHARGE_BAT2,
	EMERGENCY_MODE_OUTPUT_POWER,


	WE_MONITORING_STATE,
	CTE_CURRENT_BAT1,
	CTE_VOLTAGE_BAT1,
	CTE_CURRENT_BAT2,
	CTE_VOLTAGE_BAT2,
	FLOATING,
}states;

typedef enum{
	MONDAY = 1,
	TUESDAY,
	WEDNESDAY,
	THURSDAY,
	FRIDAY,
	SATURDAY,
	SUNDAY,
}day_of_the_week;


/* flag equals to 0 if weekday, else 1*/
_Bool DayofTheWeekflag;

states currentState;
states currentMode;

int batterywithHigherPrio;

/*Main state machine transition flags*/
_Bool HPV;
_Bool LPV;


_Bool LSoCbat1;
_Bool ISoCbat1;
_Bool NSoCbat1;
_Bool FSoCbat1;
_Bool LtoNSoCbat1;

_Bool LSoCbat2;
_Bool ISoCbat2;
_Bool NSoCbat2;
_Bool FSoCbat2;
_Bool LtoNSoCbat2;

_Bool Bat1, Bat2;
_Bool Parallel, Series;
_Bool XPow;

_Bool modeHasChangedBattConf;

SemaphoreHandle_t xBMSSemaphore;
SemaphoreHandle_t xEEPROMSemaphore;
SemaphoreHandle_t xSleepSemaphore;

uint32_t ADC1buffer[2];
uint32_t ADC2buffer[1];

#endif /* GLOBALINFO_H_ */
