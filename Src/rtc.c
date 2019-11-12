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
