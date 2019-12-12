/*
 * taskEEPROMManage.c
 *
 *  Created on: 12 dic. 2019
 *      Author: Gerardo
 */
#include "FreeRTOS.h"
#include "task.h"
#include "taskEEPROMManage.h"
#include "semphr.h"
#include "globalInfo.h"

/*Private prototype*/
void taskEEPROMManage_task(void * pvParameters);

void taskEEPROMManagel_init(void){

	xTaskCreate(
			taskEEPROMManage_task,       /* Function that implements the task. */
			"EEPROM management task",          /* Text name for the task. */
			configMINIMAL_STACK_SIZE,      /* Stack size in words, not bytes. */
			( void * ) 1,    /* Parameter passed into the task. */
			tskIDLE_PRIORITY,/* Priority at which the task is created. */
			NULL);      /* Used to pass out the created task's handle. */
}

void taskEEPROMManage_task(void * pvParameters){

	xEEPROMSemaphore = xSemaphoreCreateBinary();

	while(1){


		xSemaphoreTake(xSleepSemaphore, portMAX_DELAY);

		xSemaphoreGive(xEEPROMSemaphore);
		vTaskDelay(1000);
	}
}

