/*
 * taskI2CManage.c
 *
 *  Created on: 12 nov. 2019
 *      Author: Gerardo
 */
#include "FreeRTOS.h"
#include "task.h"
#include "stm32l4xx_ll_i2c.h"
#include "semphr.h"
#include "globalInfo.h"

int precisionError = 5;

/*private prototypes*/
void taskBMScommuncation_task(void * pvParameters);

void taskBMScommunication_init(void){

	xTaskCreate(
			taskBMScommuncation_task,       /* Function that implements the task. */
			"BMS communication task",          /* Text name for the task. */
			configMINIMAL_STACK_SIZE,      /* Stack size in words, not bytes. */
			( void * ) 1,    /* Parameter passed into the task. */
			tskIDLE_PRIORITY,/* Priority at which the task is created. */
			NULL);      /* Used to pass out the created task's handle. */

}

void taskBMScommuncation_task(void * pvParameters){

	int SoCbatt1;
	int SoCbatt2;
	xBMSSemaphore = xSemaphoreCreateBinary();

	while(1){

		//Here we read SoC for battery 1 and 2 and turn it into percentage TODO

		if(SoCbatt1 < 50){
			LSoCbat1 = 1;
			LtoNSoCbat1 = 0;
		} else {
			LSoCbat1 = 0;
		}

		if((SoCbatt1 >= 80) && (SoCbatt1 < (100 - precisionError))){
			LtoNSoCbat1 = 1;
			NSoCbat1 = 1;
		} else {
			NSoCbat1 = 0;
		}

		if(SoCbatt1 == (100 - precisionError)){
			FSoCbat1 = 1;
		} else {
			FSoCbat1 = 0;
		}

		if(SoCbatt2 < 50){
			LSoCbat2 = 1;
			LtoNSoCbat2 = 0;
		} else {
			LSoCbat2 = 0;
		}

		if((SoCbatt2 >= 80) && (SoCbatt2 < (100 - precisionError))){
			LtoNSoCbat2 = 1;
			NSoCbat2 = 1;
		} else {
			NSoCbat2 = 0;
		}

		if(SoCbatt2 == (100 - precisionError)){
			FSoCbat2 = 1;
		} else {
			FSoCbat2 = 0;
		}

		xSemaphoreGive(xBMSSemaphore);

		vTaskDelay(1000);
	}
}
