#include "Display.h"

#define USE_HORIZONTAL 2  //设置横屏或者竖屏显示 0或1为竖屏 2或3为横屏


void LCD_task(void *pvParameters){
	LCD_Init_os();
	LCD_Fill(0,0,LCD_W,LCD_H,WHITE);
	while(1){
		vTaskDelay(100);
	}
}
