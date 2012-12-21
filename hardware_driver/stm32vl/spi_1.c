/*
 * spi_1.c
 *
 *  Created on: 26.06.2012
 *      Author: simon
 */

#include "stm32f10x_spi.h"
#include "stm32f10x_it.h"
#include "mousesensor.h"
#include "util.h"
#include "spi_1.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"

#define SPI_BUFFER_SIZE 256


struct SPI_Data {
	u8 RxBuffer[SPI_BUFFER_SIZE];
	u16 RxWritePointer;
	u16 RxReadPointer;

	u8 RxBufferFullError;
};

volatile struct SPI_Data SPI1_Data;

void spi_init(void) {
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
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_3;
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
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);

	//SPI_SSOutputCmd(SPI1, ENABLE);
	//SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, ENABLE);
	SPI_Cmd(SPI1, ENABLE);
	GPIO_SetBits(GPIOA, GPIO_Pin_4);
}

void spi_SelectChip(SPI spi) {
	//enable clock by sending
	//pull down clock selcet
	if(!spi){
		GPIO_ResetBits(GPIOA, GPIO_Pin_4); // spi1
	} else {
		GPIO_ResetBits(GPIOA, GPIO_Pin_3); // spi2
	}
}

void spi_DeselectChip(SPI spi) {
	// pull up clock select
	if (!spi) {
		GPIO_SetBits(GPIOA, GPIO_Pin_4); // spi1
	} else {
		GPIO_SetBits(GPIOA, GPIO_Pin_3); //spi 2
	}

}

int spi_SendData(const unsigned char *buffer, const unsigned int count) {

	u32 pos = 0;

	//write byte by byte in buffer
	for (pos = 0; pos < count; pos++) {

		SPI_I2S_SendData(SPI1, buffer[pos]);

		//Wait until data is send
		 while (!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE));
		delay_us(20);
	}

	return count;
}

int spi_GetData(unsigned char *buffer, const unsigned int buff_length){

	u32 pos = 0;

	for (pos = 0 ; pos < buff_length; pos++) {
	        SPI_I2S_SendData(SPI1, 0); //Write a 0 to generate clock
	        while (!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)); //blocks until there is something to read
	        buffer[pos]=SPI_I2S_ReceiveData(SPI1);
	        //delay_us(0); //motion-burst -> no need to wait between bytes
	    }

	return pos;
}



unsigned char spi_WriteRegister(unsigned char address, unsigned char data, SPI spi) {

	address |=  0x80; // MSB high
    while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE));

    spi_SelectChip(spi);
    delay_us(1); //delay tNCS-SCLK 120ns

    // Send SPI1 data
    SPI_I2S_SendData(SPI1, address);
    while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE));
    SPI_I2S_SendData(SPI1, data);

    while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE));
    delay_us(25);
    spi_DeselectChip(spi);
    delay_us(140);
    return data;
}

unsigned char spi_ReadRegister(unsigned char address, SPI spi) {

	//address &= ~(1 << 8);
	address &=  0x7f; // MSB low

	while (!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE))
		;
	spi_SelectChip(spi);
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
	delay_us(1);
	spi_DeselectChip(spi);
	delay_us(1);
	return data;
}

inline uint8_t set_bits(const uint8_t data, const uint8_t bitmask) {
    return (data | bitmask);
}

inline uint8_t clear_bits(const uint8_t data, const uint8_t bitmask) {
    return (data & ~(bitmask));
}

inline void set_bits_addr(const uint8_t address, const uint8_t bitmask, SPI spi) {
	spi_WriteRegister(address, set_bits(spi_ReadRegister(address, spi),bitmask), spi);
}
inline void clear_bits_addr(const uint8_t address, const uint8_t bitmask, SPI spi) {
	spi_WriteRegister(address, clear_bits(spi_ReadRegister(address, spi),bitmask), spi);
}


