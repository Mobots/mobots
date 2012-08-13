#include "stm32f10x_exti.h"
#include "stm32f10x_rcc.h"
#include "misc.h"
#include "spi_1.h"
#include "mousesensor.h"
#include "srom.h"
#include "stm32f10x.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_it.h"
#include "printf.h"
#include "stm32f10x_gpio.h"

//void DMA_init() {
//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
//	DMA_InitTypeDef DMA_InitStructure;
//
//	// DMA1 Channel2 Config must be used with SPI1_Rx Update
//	DMA_DeInit(DMA1_Channel2);
//
//	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32) &(SPI1->DR);
//	DMA_InitStructure.DMA_MemoryBaseAddr = (u32) mouse_data.motion;
//	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
//	DMA_InitStructure.DMA_BufferSize = sizeof(mouse_data);
//	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
//	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
//	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
//
//	DMA_Init(DMA1_Channel2, &DMA_InitStructure);
//
//	NVIC_InitTypeDef NVIC_InitStructure;
//
//	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//
//	DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);
//
//	////DMA1 Channel2 enable
//	DMA_Cmd(DMA1_Channel2, ENABLE);
//
//}

void SPI_init() {
	spi1_datastat = OUTDATED;
	spi2_datastat = OUTDATED;
	spi1_init();
}

void delay_us(uint32_t u_sec) {
	int TimingDelay = u_sec * 6;

	while (TimingDelay--)
		;
}

char FW_init() {

	spi1_DeselectChip();
	delay_us(2);


	spi1_WriteRegister(REG_Configuration_IV, 0x02);
	spi1_WriteRegister(REG_SROM_Enable, 0x1d);
	delay_us(800);
	spi1_WriteRegister(REG_SROM_Enable, 0x18);
	delay_us(150);


	spi1_SelectChip();
	delay_us(1);

	// Wait for aSPI Tx buffer empty
	while (!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE))
		;

	// Send aSPI data
	SPI_I2S_SendData(SPI1, (REG_SROM_Load_Burst | 0x80)); //Address of SROM_Load_Burst register // MSB bit set

	while (!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE))
		;
	delay_us(20); //delay >=15us after the address

	spi1_SendData((const unsigned char*) &firmware_3k, firmware_3k_length);
	delay_us(25); //tSCLK-NCS //20

	spi1_DeselectChip();
	delay_us(200); //soonest to read SROM_ID is 160uS
	char id = spi1_ReadRegister(REG_SROM_ID);
	return id;
}

void EXIT_init() {
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource3);

	EXTI_InitTypeDef EXTI_InitStructure;

	EXTI_InitStructure.EXTI_Line = EXTI_Line3;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	GPIO_SetBits(GPIOC, GPIO_Pin_8);
}

void Sensor_init() {

	SPI_init();
	EXIT_init();

	spi1_DeselectChip();
	spi1_SelectChip();
	spi1_DeselectChip();
	spi1_WriteRegister(REG_Power_Up_Reset, 0x5a);
	delay_us(100);

	spi1_ReadRegister(REG_Motion);
	spi1_ReadRegister(REG_Delta_X_L);
	spi1_ReadRegister(REG_Delta_X_H);
	spi1_ReadRegister(REG_Delta_Y_L);
	spi1_ReadRegister(REG_Delta_Y_H);

	FW_init();
	delay_us(20);

	char out = spi1_ReadRegister(REG_LASER_CTRL0);
	spi1_WriteRegister(REG_LASER_CTRL0, out & 0xf0);
}

void Sensor_ReadByBurst() {
	spi1_SelectChip();
	delay_us(2);
	while (!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE))
		;

	SPI_I2S_SendData(SPI1, REG_Motion_Burst);
	while (!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE))
		;
	delay_us(120);

	spi1_GetData((unsigned char *) &mouse_data, 14);
	spi1_DeselectChip();
	delay_us(15);
}



void EXTI15_10_IRQHandler(void) {
	if (EXTI_GetITStatus(EXTI_Line11) != RESET) {
		Sensor_ReadByBurst();
		spi2_datastat = UPDATED;
		EXTI_ClearITPendingBit(EXTI_Line11);
	}
}

void EXTI3_IRQHandler(void) {
	if (EXTI_GetITStatus(EXTI_Line3)) {

		Sensor_ReadByBurst();
		spi1_datastat = UPDATED;
		EXTI_ClearITPendingBit(EXTI_Line3);
	}
}



