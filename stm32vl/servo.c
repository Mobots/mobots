/*
 * servo.c
 *
 *  Created on: 21.06.2012
 *      Author: simon
 */

#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#include "servo.h"

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

	timInit.TIM_Prescaler = 11;
	timInit.TIM_CounterMode = TIM_CounterMode_Up;
	timInit.TIM_Period = 39999;
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

void servo_setAngle(enum Servos servo, int value)
{
	//1,3 ms <-> 1,7 ms

	assert_param(value >= -1000);
	assert_param(value <= 1000);

	int valueIntern = 3000 + value * 4 / 10;

	switch (servo)
	{
	case Servo_1:
		TIM_SetCompare1(TIM4, valueIntern);
		break;
	case Servo_2:
		TIM_SetCompare2(TIM4, valueIntern);
		break;
	case Servo_3:
		TIM_SetCompare3(TIM4, valueIntern);
		break;
	}
}

