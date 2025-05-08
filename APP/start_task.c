#include "start_task.h"

TaskHandle_t TaskHandle_1;
TaskHandle_t TaskHandle_2;
TaskHandle_t StartTask_Handler;
BaseType_t result;


void start_task(void *pvParameters){
	taskENTER_CRITICAL();
	
	result = xTaskCreate(LCD_task,
					"LCD_task",
					128,
					NULL,
					3,
					&TaskHandle_1);
	
	result = xTaskCreate(State_task,
					"task_2",
					128,
					NULL,
					3,
					&TaskHandle_2);
	
    taskEXIT_CRITICAL();            //退出临界区
	vTaskDelete(StartTask_Handler); //删除开始任务

}

void startTask(void){
	xTaskCreate(start_task,"start_task",64,NULL,2,(TaskHandle_t *)&StartTask_Handler);
}



