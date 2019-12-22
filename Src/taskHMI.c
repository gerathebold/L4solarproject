/*
 * taskHMI.c
 *
 *  Created on: 13 dic. 2019
 *      Author: Gerardo
 */
#include "FreeRTOS.h"
#include "task.h"
#include "HMIServiceLayer.h"

HMImenus currentMenu;
HMImenus cursorPosition;
_Bool MenuHasBeenSelected;
_Bool MenuReturn;

/*Private prototype*/
void taskHMI_task(void * pvParameters);

void taskHMI_init(void){

	xTaskCreate(
			taskHMI_task,       /* Function that implements the task. */
			"HMI task",          /* Text name for the task. */
			configMINIMAL_STACK_SIZE,      /* Stack size in words, not bytes. */
			( void * ) 1,    /* Parameter passed into the task. */
			tskIDLE_PRIORITY,/* Priority at which the task is created. */
			NULL);      /* Used to pass out the created task's handle. */
}

void taskHMI_task(void * pvParameters){


	while(1){
		switch(currentMenu){
		case INFORMATIVE_MENU:
			//here we print informative menu screen TODO

			if(ServiceLayerButtonHasBeenPressed()){
				currentMenu = INTERACTIVE_MENU_SELECTOR;
			}

			break;


		case INTERACTIVE_MENU_SELECTOR:
			ServiceLayerPrintInteractiveMenuSelector();
			cursorPosition = INTERACTIVE_MENU_BATTERY;
			ServiceLayerUpdateCursorPosition(cursorPosition);

			while(!MenuHasBeenSelected && !ServiceLayerAlertHasOccurred()){
				if(ServiceLayerUpPressed()){
					if(cursorPosition == INTERACTIVE_MENU_BATTERY){
						cursorPosition = INTERACTIVE_MENU_BATTERY;
					} else {
						cursorPosition++;
					}
				} else {
					if(ServiceLayerDownPressed()){
						if(cursorPosition == INTERACTIVE_MENU_ALERT){
							cursorPosition = INTERACTIVE_MENU_ALERT;
						} else {
							cursorPosition--;
						}
					}
				}

				if((cursorPosition == INTERACTIVE_MENU_BATTERY)  &&  ServiceLayerValidatePressed()){
					currentMenu = INTERACTIVE_MENU_BATTERY;
					MenuHasBeenSelected = 1;
				}

				if((cursorPosition == INTERACTIVE_MENU_DATE)  &&  ServiceLayerValidatePressed()){
					currentMenu = INTERACTIVE_MENU_DATE;
					MenuHasBeenSelected = 1;
				}

				if((cursorPosition == INTERACTIVE_MENU_DATE)  &&  ServiceLayerValidatePressed()){
					currentMenu = INTERACTIVE_MENU_DATE;
					MenuHasBeenSelected = 1;
				}

				if((cursorPosition == INTERACTIVE_MENU_BUCK)  &&  ServiceLayerValidatePressed()){
					currentMenu = INTERACTIVE_MENU_BUCK;
					MenuHasBeenSelected = 1;
				}

				if((cursorPosition == INTERACTIVE_MENU_ALERT)  &&  ServiceLayerValidatePressed()){
					currentMenu = INTERACTIVE_MENU_ALERT;
					MenuHasBeenSelected = 1;
				}
			}
			MenuHasBeenSelected = 0;
			break;

		case INTERACTIVE_MENU_BATTERY:

			while(!MenuReturn && !ServiceLayerAlertHasOccurred()){
				ServiceLayerPrintBatteryMenu();

				if(ServiceLayerBackPressed()){
					currentMenu = INTERACTIVE_MENU_SELECTOR;
					MenuReturn = 1;
				}
			}
			MenuReturn = 0;
			break;

		case INTERACTIVE_MENU_DATE:

			break;

		case INTERACTIVE_MENU_BUCK:

			while(!MenuReturn && !ServiceLayerAlertHasOccurred()){
				ServiceLayerPrintBuckMenu();

				if(ServiceLayerBackPressed()){
					currentMenu = INTERACTIVE_MENU_SELECTOR;
					MenuReturn = 1;
				}
			}
			MenuReturn = 0;
			break;

		case INTERACTIVE_MENU_ALERT:
			while(!MenuReturn && !ServiceLayerAlertHasOccurred()){
				ServiceLayerPrintAlertMenu();

				if(ServiceLayerBackPressed()){
					currentMenu = INTERACTIVE_MENU_SELECTOR;
					MenuReturn = 1;
				}
			}
			MenuReturn = 0;
			break;

		default:
			break;


			vTaskDelay(1000);
		}
	}
}
