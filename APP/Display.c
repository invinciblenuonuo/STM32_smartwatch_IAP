#include "Display.h"

#define USE_HORIZONTAL 2  //���ú�������������ʾ 0��1Ϊ���� 2��3Ϊ����


void LCD_task(void *pvParameters){
	LCD_Init_os();
	LCD_Fill(0,0,LCD_W,LCD_H,WHITE);
	while(1){
		vTaskDelay(100);
	}
}
