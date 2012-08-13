/*
 * led.c
 *
 *  Created on: 13.06.2012
 *      Author: simon
 */

#include "led.h"
#include "stm32f10x_gpio.h"

void led_init(void){
	GPIO_InitTypeDef LED_InitStruc;

		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);


		//GPIO_StructInit(&LED_InitStruc);
		LED_InitStruc.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
		LED_InitStruc.GPIO_Mode = GPIO_Mode_Out_PP;
		LED_InitStruc.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOC, &LED_InitStruc);
}



