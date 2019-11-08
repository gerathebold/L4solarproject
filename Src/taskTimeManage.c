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
/*private defines*/
#define SLEEP_HOUR 8

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


void taskTimeManage_task(void * pvParameters ){

	while(1){
		/*if(__LL_RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetHour(RTC)) == SLEEP_HOUR){
			Enter_Standby_mode();
		}*/
		if(LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_13)){
			Enter_Standby_mode();
		}
		vTaskDelay(100);
	}
}
