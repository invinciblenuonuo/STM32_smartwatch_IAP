#include "system.h"
#include "led.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "lcd_init.h"
#include "start_task.h"
#include "delay.h"
#include "Flash.h"
#include "driver_w25qxx.h"

int main(){
	
	// 读取W25QXX JEDEC ID并打印
	w25qxx_handle_t handle;
	uint8_t flash_buffer[20]={0};
	//SCB->VTOR = FLASH_BASE | 0x4000;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  //中断优先级分组 分2组
	USART1_Init(115200);
	LED_Init();
	delay_init();
	
	w25qxx_init_usr(&handle);
	w25qxx_write(&handle,0x1000, (uint8_t*)"Hello World", 12);
	w25qxx_read(&handle,0x1000, (uint8_t*)flash_buffer, 12);
	printf("flash_buffer: %s\r\n", flash_buffer);

	startTask();
	vTaskStartScheduler();
	while(1){
		
	}
}




