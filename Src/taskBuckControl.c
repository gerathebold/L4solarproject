/*
 * taskBuckManage.c
 *
 *  Created on: 27 nov. 2019
 *      Author: Gerardo
 */
#include "FreeRTOS.h"
#include "task.h"

/*Private prototype*/
void taskBuckControl_task(void * pvParameters);

void taskBuckControl_init(void){

	xTaskCreate(
			taskBuckControl_task,       /* Function that implements the task. */
			"Buck management task",          /* Text name for the task. */
			configMINIMAL_STACK_SIZE,      /* Stack size in words, not bytes. */
			( void * ) 1,    /* Parameter passed into the task. */
			tskIDLE_PRIORITY,/* Priority at which the task is created. */
			NULL);      /* Used to pass out the created task's handle. */
}

void taskBuckControl_task(void * pvParameters){


	while(1){

		vTaskDelay(1000);
	}
}
