/*
 * servo.c
 *
 *  Created on: 21.06.2012
 *      Author: simon
 */

#include "servo.h"

#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"


void servo_init()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitTypeDef initStruct;
	GPIO_StructInit(&initStruct);

	initStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
	initStruct.GPIO_Speed = GPIO_Speed_50MHz;
	initStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &initStruct);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	TIM_TimeBaseInitTypeDef timInit;
	TIM_TimeBaseStructInit(&timInit);

	timInit.TIM_Prescaler = 11; // 24 MHz durch 11+1 = 500ns
	timInit.TIM_CounterMode = TIM_CounterMode_Up;
	timInit.TIM_Period = 39999; // 500 ns * 39999+1 = 20ms
	timInit.TIM_ClockDivision = TIM_CKD_DIV1;
	timInit.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM4, &timInit);

	TIM_OCInitTypeDef oc_init;
	TIM_OCStructInit(&oc_init);

	oc_init.TIM_OCMode = TIM_OCMode_PWM2;
	oc_init.TIM_OutputState = TIM_OutputState_Enable;
	oc_init.TIM_OCPolarity = TIM_OCPolarity_Low;
	oc_init.TIM_OCNPolarity = TIM_OCPolarity_High;
	oc_init.TIM_Pulse = 0;

	TIM_OC1Init(TIM4, &oc_init);
	TIM_OC2Init(TIM4, &oc_init);
	TIM_OC3Init(TIM4, &oc_init);
	TIM_OC4Init(TIM4, &oc_init);


	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);


	TIM_Cmd(TIM4, ENABLE);
}

void servo_set(const struct ServoSpeed * const servo_speed)
{
	int value[3];

	int i;
	for(i = 0; i < 3; i++)
	{
		value[i] = 3000 + servo_speed->v[i] * 400;
	}

	TIM_SetCompare1(TIM4, value[0]);
	TIM_SetCompare2(TIM4, value[1]);
	TIM_SetCompare3(TIM4, value[2]);
}
