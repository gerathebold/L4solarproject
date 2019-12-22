#include "FreeRTOS.h"
#include "task.h"
#include "globalInfo.h"
#include "semphr.h"

states interMode = 0;
_Bool interBat1, interBat2, interParallel, interSeries;

/*private prototypes*/
void taskMainStateMachine_task(void * pvParameters);

void taskMainStateMachine_init(void){


	xTaskCreate(
			taskMainStateMachine_task,       /* Function that implements the task. */
			"Main state machine task",          /* Text name for the task. */
			configMINIMAL_STACK_SIZE,      /* Stack size in words, not bytes. */
			( void * ) 1,    /* Parameter passed into the task. */
			tskIDLE_PRIORITY,/* Priority at which the task is created. */
			NULL);      /* Used to pass out the created task's handle. */

}

void taskMainStateMachine_task(void * pvParameters){
	day_of_the_week currentDay;

	while(1){

		xSemaphoreTake(xBMSSemaphore, portMAX_DELAY);

		currentDay = __LL_RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetDay(RTC));
		if((currentDay == SATURDAY) ||(currentDay == SUNDAY)){
			currentState = WE_MONITORING_STATE;
		} else {
			currentState = WD_MONITORING_STATE;
		}

		switch(currentState){

		case WD_MONITORING_STATE:
			if(!LSoCbat1 && !LSoCbat2 && LtoNSoCbat2 && LtoNSoCbat1){
				currentState = NORMAL_WORKING_MODE;
			}

			if((LSoCbat1 || !LtoNSoCbat1)){
				currentState = EMERGENCY_MODE_RECHARGE_BAT1;
			}

			if((LSoCbat2 || !LtoNSoCbat2) && LtoNSoCbat1){
				currentState = EMERGENCY_MODE_RECHARGE_BAT2;
			}

			if(XPow){
				currentState = EMERGENCY_MODE_OUTPUT_POWER;
			}
			break;

		case NORMAL_WORKING_MODE:
			if(interMode != NORMAL_WORKING_MODE){
				modeHasChangedBattConf = 1;
			}
			interMode = NORMAL_WORKING_MODE;
			currentMode = NORMAL_WORKING_MODE;
			break;

		case EMERGENCY_MODE_RECHARGE_BAT1:
			if(interMode != EMERGENCY_MODE_RECHARGE_BAT1){
				modeHasChangedBattConf = 1;
			}
			interMode = EMERGENCY_MODE_RECHARGE_BAT1;
			currentMode = EMERGENCY_MODE_RECHARGE_BAT1;
			break;

		case EMERGENCY_MODE_RECHARGE_BAT2:
			if(interMode != EMERGENCY_MODE_RECHARGE_BAT2){
				modeHasChangedBattConf = 1;
			}
			interMode = EMERGENCY_MODE_RECHARGE_BAT2;
			currentMode = EMERGENCY_MODE_RECHARGE_BAT2;
			break;

		case EMERGENCY_MODE_OUTPUT_POWER:
			if(interMode != EMERGENCY_MODE_OUTPUT_POWER){
				modeHasChangedBattConf = 1;
			}
			interMode = EMERGENCY_MODE_OUTPUT_POWER;
			currentMode = EMERGENCY_MODE_OUTPUT_POWER;
			break;


		case WE_MONITORING_STATE:
			if(!NSoCbat1 && !FSoCbat1 ){
				currentState = CTE_CURRENT_BAT1;
			}

			if(NSoCbat1){
				currentState = CTE_VOLTAGE_BAT1;
			}

			if(!NSoCbat2 && !FSoCbat2 ){
				currentState = CTE_CURRENT_BAT2;
			}

			if(NSoCbat2){
				currentState = CTE_VOLTAGE_BAT2;
			}

			break;

		case CTE_CURRENT_BAT1:
			if(interMode != CTE_CURRENT_BAT1){
				modeHasChangedBattConf = 1;
			}
			interMode = CTE_CURRENT_BAT1;
			currentMode = CTE_CURRENT_BAT1;
			break;

		case CTE_VOLTAGE_BAT1:
			if(interMode != CTE_VOLTAGE_BAT1){
				modeHasChangedBattConf = 1;
			}
			interMode = CTE_VOLTAGE_BAT1;
			currentMode = CTE_VOLTAGE_BAT1;
			break;

		case CTE_CURRENT_BAT2:
			if(interMode != CTE_CURRENT_BAT2){
				modeHasChangedBattConf = 1;
			}
			interMode = CTE_CURRENT_BAT2;
			currentMode = CTE_CURRENT_BAT2;
			break;


		case CTE_VOLTAGE_BAT2:
			if(interMode != CTE_VOLTAGE_BAT2){
				modeHasChangedBattConf = 1;
			}
			interMode = CTE_VOLTAGE_BAT2;
			currentMode = CTE_VOLTAGE_BAT2;
			break;

		case FLOATING:
			if(interMode != FLOATING){
				modeHasChangedBattConf = 1;
			}
			interMode = FLOATING;
			currentMode = FLOATING;
			break;

		default:
			break;
		}
	}


	vTaskDelay(1000);
}
