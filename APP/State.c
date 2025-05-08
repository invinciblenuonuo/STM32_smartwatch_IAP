#include "State.h"


void State_task(void *pvParameters){
	const TickType_t xTicksToWait = pdMS_TO_TICKS( 1000UL );
	while(1){
		vTaskDelay(xTicksToWait/20);
		led1=!led1;
		vTaskDelay(xTicksToWait);
		led1=!led1;
		
	}
}
