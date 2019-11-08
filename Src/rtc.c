/*
 * rtc.c
 *
 *  Created on: 7 nov. 2019
 *      Author: Gerardo
 */


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
#include <stdio.h>

/*private defines*/

#define RTC_BKP_DATE_TIME_UPDTATED ((uint32_t)0x32F2)
#define RTC_ASYNCH_PREDIV          ((uint32_t)0x7F)
#define RTC_SYNCH_PREDIV           ((uint32_t)0x00F9)
#define RTC_WUT_TIME               ((uint32_t)5)     /* 5 s */

/*private prototypes*/
void Configure_RTC_Clock(void);
void Configure_RTC_Calendar(void);
void Configure_RTC_WakeUpAlarm(void);


/**
  * @brief  RTC setup
  * @param  None
  * @retval None
  */
void RTC_Full_Setup(void){
	//##-1- We configure the RTC to work the LSE clock
	Configure_RTC_Clock();

	//##-2- We configure the RTC calendar
	Configure_RTC_Calendar();

	//##-2- Configure RTC WakeUp alarm
	Configure_RTC_WakeUpAlarm();
}


/**
  * @brief  Configure RTC clock.
  * @param  None
  * @retval None
  */
void Configure_RTC_Clock(void){

	/*##-1- Enables the PWR Clock and Enables access to the backup domain #######*/
	/* To change the source clock of the RTC feature (LSE, LSI), you have to:
     - Enable the power clock
     - Enable write access to configure the RTC clock source (to be done once after reset).
     - Reset the Back up Domain
     - Configure the needed RTC clock source */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
	LL_PWR_EnableBkUpAccess();

	/*##-2- Configure LSE/LSI as RTC clock source ###############################*/
	/* Enable LSE only if disabled.*/

	if (LL_RCC_LSE_IsReady() == 0)
	{
		LL_RCC_ForceBackupDomainReset();
		LL_RCC_ReleaseBackupDomainReset();
		LL_RCC_LSE_Enable();

		while (LL_RCC_LSE_IsReady() != 1)
		{

		}
		LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSE);
	}
}


void Configure_RTC_Calendar(void){

	time_t t = time(NULL);
	struct tm tm = *localtime(&t);

	/*##-1- Disable RTC registers write protection ############################*/
	LL_RTC_DisableWriteProtection(RTC);

	/*##-2- Enter in initialization mode ######################################*/
	LL_RTC_EnableInitMode(RTC);

	/*##-3- Wait for confirmation ########################################*/
	while (LL_RTC_IsActiveFlag_INIT(RTC) != 1)
	{

	}

	/*##-4- Configure RTC prescaler and RTC data registers #########################*/
	/* Set Synch Prediv (value according to source clock) */
	LL_RTC_SetSynchPrescaler(RTC, RTC_SYNCH_PREDIV);

	/* Set Asynch Prediv (value according to source clock) */
	LL_RTC_SetAsynchPrescaler(RTC, RTC_ASYNCH_PREDIV);

	/*##-5- Configure the Date and Time ################################################*/
	/* Set Date: Monday March 31th 2015 */
	//LL_RTC_DATE_Config(RTC, LL_RTC_WEEKDAY_MONDAY, 0x31, LL_RTC_MONTH_MARCH, 0x15);
	LL_RTC_DATE_Config(RTC, __LL_RTC_CONVERT_BIN2BCD(tm.tm_wday), __LL_RTC_CONVERT_BIN2BCD(tm.tm_mday), __LL_RTC_CONVERT_BIN2BCD(tm.tm_mon), __LL_RTC_CONVERT_BIN2BCD(tm.tm_year));

	/* Set Time: 11:59:55 PM*/
	//LL_RTC_TIME_Config(RTC, LL_RTC_TIME_FORMAT_PM, 0x11, 0x59, 0x55);
	LL_RTC_TIME_Config(RTC, LL_RTC_TIME_FORMAT_PM, __LL_RTC_CONVERT_BIN2BCD(tm.tm_hour),__LL_RTC_CONVERT_BIN2BCD(tm.tm_min) , __LL_RTC_CONVERT_BIN2BCD(tm.tm_sec));

	/*##-6- Set hour format ################################################*/
	LL_RTC_SetHourFormat(RTC, LL_RTC_HOURFORMAT_AMPM);

	/*##-7- Exit of initialization mode #######################################*/
	LL_RTC_DisableInitMode(RTC);

	/*##-8- Enable RTC registers write protection #############################*/
	LL_RTC_EnableWriteProtection(RTC);

	/*##-9- Writes a data in a RTC Backup data Register1 #######################*/
	LL_RTC_BAK_SetRegister(RTC, LL_RTC_BKP_DR1, RTC_BKP_DATE_TIME_UPDTATED);
}

void Configure_RTC_WakeUpAlarm(void){

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
	LL_RTC_WAKEUP_SetAutoReload(RTC, RTC_WUT_TIME);
	LL_RTC_WAKEUP_SetClock(RTC, LL_RTC_WAKEUPCLOCK_CKSPRE);


	/*##-5- Enable RTC registers write protection #############################*/
	LL_RTC_EnableWriteProtection(RTC);
}

void Enter_Standby_mode(void){

	/* ######## ENABLE WUT #################################################*/
	/* Disable RTC registers write protection */
	LL_RTC_DisableWriteProtection(RTC);

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
