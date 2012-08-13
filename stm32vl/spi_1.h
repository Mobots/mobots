/*
 * spi_1.h
 *
 *  Created on: 26.06.2012
 *      Author: simon
 */

#ifndef SPI_1_H_
#define SPI_1_H_

void spi1_init(void);
int spi1_SendData(const unsigned char *buffer, const unsigned int count);
int spi1_GetData(unsigned char *buffer, const unsigned int buffer_length);
void spi1_SelectChip();
void spi1_DeselectChip();
unsigned char spi1_WriteRegister(unsigned char address, unsigned char data);
unsigned char spi1_ReadRegister(unsigned char address);

#endif /* SPI_1_H_ */
