#ifndef _led_H
#define _led_H

#include "system.h"

/*  LED时钟端口、引脚定义 */
#define LED_PORT 			GPIOC   
#define LED_PIN 			GPIO_Pin_13
#define LED_PORT_RCC		RCC_APB2Periph_GPIOC


#define led1 PCout(13)
#define led2 PCout(1)
#define led3 PCout(2)


void LED_Init(void);


#endif
