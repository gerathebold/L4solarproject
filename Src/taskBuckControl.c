/*
 * taskBuckManage.c
 *
 *  Created on: 27 nov. 2019
 *      Author: Gerardo
 */
#include "FreeRTOS.h"
#include "task.h"
#include "globalInfo.h"
#include "ADCDriver.h"

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

	volatile uint32_t voltage1 = 0;
	volatile uint32_t voltage2 = 0;
	volatile uint32_t current = 0;

	while(1){
		voltage1 = DriverStartADC1ConverionChannel1();
		voltage2 = DriverStartADC1ConverionChannel4();
		current = DriverStartADC1ConverionChannel3();
		vTaskDelay(1000);
	}
}
