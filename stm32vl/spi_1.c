/*
 * spi_1.c
 *
 *  Created on: 26.06.2012
 *      Author: simon
 */

#include "stm32f10x_spi.h"
#include "stm32f10x_it.h"
#include "mousesensor.h"

#define SPI_BUFFER_SIZE 256


struct SPI_Data {
	u8 RxBuffer[SPI_BUFFER_SIZE];
	u16 RxWritePointer;
	u16 RxReadPointer;

	u8 RxBufferFullError;
};

volatile struct SPI_Data SPI1_Data;

void spi1_init(void) {
	RCC_APB2PeriphClockCmd(
			RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1,
			ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_StructInit(&GPIO_InitStructure);

	// A5 SCK, A6 MISO, A7 MOSI
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

//	// A6 MISO IN FLOTING
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);

//Pin A4 SS
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	SPI1_Data.RxReadPointer = 0;
	SPI1_Data.RxWritePointer = 0;

//	NVIC_InitTypeDef NVIC_InitStructure;
//	NVIC_InitStructure.NVIC_IRQChannel = SPI1_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);

	SPI_InitTypeDef SPI_InitStructure;
	SPI_StructInit(&SPI_InitStructure);

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);

	//SPI_SSOutputCmd(SPI1, ENABLE);
	//SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, ENABLE);
	SPI_Cmd(SPI1, ENABLE);
	GPIO_SetBits(GPIOA, GPIO_Pin_4);
}

void spi1_SelectChip() {
	//enable clock by sending
	//pull down clock selcet
	GPIO_ResetBits(GPIOA, GPIO_Pin_4);
}

void spi1_DeselectChip() {
	// pull up clock select
	GPIO_SetBits(GPIOA, GPIO_Pin_4);
}

int spi1_SendData(const unsigned char *buffer, const unsigned int count) {

	u32 pos = 0;

	//write byte by byte in buffer
	for (pos = 0; pos < count; pos++) {

		SPI_I2S_SendData(SPI1, buffer[pos]);

		//Wait until data is send
		 while (!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE));
		delay_us(1);
	}

	return count;
}

int spi1_GetData(unsigned char *buffer, const unsigned int buff_length){

	u32 pos = 0;

	for (pos = 0 ; pos < buff_length; pos++) {
	        SPI_I2S_SendData(SPI1, 0); //Write a 0 to generate clock
	        while (!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)); //blocks until there is something to read
	        buffer[pos]=SPI_I2S_ReceiveData(SPI1);
	        //delay_us(0); //motion-burst -> no need to wait between bytes
	    }

	return pos;
}



unsigned char spi1_WriteRegister(unsigned char address, unsigned char data) {

   // address |= (1 << 8);
	address |=  0x80;
    while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE));

    spi1_SelectChip();
    delay_us(1); //delay tNCS-SCLK 120ns

    // Send SPI1 data
    SPI_I2S_SendData(SPI1, address);
    while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE));
    SPI_I2S_SendData(SPI1, data);

    while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE));
    delay_us(25);
    spi1_DeselectChip();
    delay_us(140);
    return data;
}

unsigned char spi1_ReadRegister(unsigned char address) {

	//address &= ~(1 << 8);
	address &=  0x7f;

	while (!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE))
		;
	spi1_SelectChip();
	delay_us(1);
	SPI_I2S_SendData(SPI1, address);
	while (!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE))
		;
	delay_us(120);

	unsigned char data = 0;
	data = SPI_I2S_ReceiveData(SPI1);
	SPI_I2S_SendData(SPI1, 0);
	while (!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE))
		;
	data = SPI_I2S_ReceiveData(SPI1);
	delay_us(25);
	spi1_DeselectChip();
	delay_us(25);
	return data;
}

//int spi1_GetData(unsigned char *buffer, const unsigned int buffer_length)
//{
//
//	u32 counter = 0;
//		while (counter < buffer_length
//				&& SPI1_Data.RxWritePointer != SPI1_Data.RxReadPointer)
//		{
//			buffer[counter] = SPI1_Data.RxBuffer[SPI1_Data.RxReadPointer];
//			counter++;
//			SPI1_Data.RxReadPointer = (SPI1_Data.RxReadPointer + 1)
//					% SPI_BUFFER_SIZE;
//		}
//
//		if (SPI1_Data.RxWritePointer == SPI1_Data.RxReadPointer
//				&& SPI1_Data.RxBufferFullError)
//		{
//			//clear error
//			SPI1_Data.RxBufferFullError = 0;
//			//reenable receive interrupt
//			SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, ENABLE);
//			//inform user space that we had an error
//			return -1;
//		}
//
//		return counter;
//}
//
//
//void SPI1_IRQHandler(void){
//	if (SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_RXNE) != RESET)
//		{
//			vu8 nextRxWritePointer = (SPI1_Data.RxWritePointer + 1) % SPI_BUFFER_SIZE;
//
//			//ringbuffer is full :-((
//			if (nextRxWritePointer == SPI1_Data.RxReadPointer)
//			{
//				//set error flag and disable receiving
//				SPI1_Data.RxBufferFullError = 1;
//				//Disable the USARTx Receive interrupt
//				SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, DISABLE);
//			}
//
//			// Read one byte from the receive data register
//			SPI1_Data.RxBuffer[SPI1_Data.RxWritePointer] = SPI_I2S_ReceiveData(SPI1);
//			SPI1_Data.RxWritePointer = nextRxWritePointer;
//		}
//}

