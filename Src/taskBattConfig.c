#include "FreeRTOS.h"
#include "task.h"
#include "globalInfo.h"


/*Private prototype*/
void taskBattConfig_task(void * pvParameters);

void taskBattConfig_init(void){

	xTaskCreate(
			taskBattConfig_task,       /* Function that implements the task. */
			"Batt config management task",          /* Text name for the task. */
			configMINIMAL_STACK_SIZE,      /* Stack size in words, not bytes. */
			( void * ) 1,    /* Parameter passed into the task. */
			tskIDLE_PRIORITY,/* Priority at which the task is created. */
			NULL);      /* Used to pass out the created task's handle. */
}

void taskBattConfig_task(void * pvParameters){


	while(1){
		if(modeHasChangedBattConf){
			modeHasChangedBattConf = 0;
			//first step: All switches are turned off
			//TODO
			//Second step: we choose battery config
			switch(currentMode){

			case NORMAL_WORKING_MODE:
				if(HPV){
					//TODO : all opened
				}

				if(LPV){
					//TODO : C/D closed, A/B opened
				}
				break;

			case EMERGENCY_MODE_RECHARGE_BAT1:
				//TODO : A closed, C/B/D opened
				break;

			case EMERGENCY_MODE_RECHARGE_BAT2:
				//TODO : B closed, A/C/D opened
				break;

			case EMERGENCY_MODE_OUTPUT_POWER:
				//TODO : all opened
				break;

			case CTE_CURRENT_BAT1:
				//TODO : A closed, C/B/D opened
				break;

			case CTE_VOLTAGE_BAT1:
				//TODO : A closed, C/B/D opened
				break;

			case CTE_CURRENT_BAT2:
				//TODO : B closed, A/C/D opened
				break;

			case CTE_VOLTAGE_BAT2:
				//TODO : B closed, A/C/D opened
				break;

			case FLOATING:
				//TODO : A/B closed, C/D opened
				break;

			default:
				break;

			}

		}


		vTaskDelay(1000);
	}
}
