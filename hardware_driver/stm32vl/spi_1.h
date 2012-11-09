/*
 * spi_1.h
 *
 *  Created on: 26.06.2012
 *      Author: simon
 */

#ifndef SPI_1_H_
#define SPI_1_H_


typedef enum {
	SPI_1 = 0,
	SPI_2
}SPI;

void SPI_init();
void spi_init(void);
int spi_SendData(const unsigned char *buffer, const unsigned int count);
int spi_GetData(unsigned char *buffer, const unsigned int buffer_length);
void spi_SelectChip(SPI spi);
void spi_DeselectChip(SPI spi);
unsigned char spi_WriteRegister(unsigned char address, unsigned char data, SPI spi);
unsigned char spi_ReadRegister(unsigned char address, SPI spi);
inline void clear_bits_addr(const uint8_t address, const uint8_t bitmask, SPI spi);
inline void set_bits_addr(const uint8_t address, const uint8_t bitmask, SPI spi);
inline uint8_t set_bits(const uint8_t data, const uint8_t bitmask);
	inline uint8_t clear_bits(const uint8_t data, const uint8_t bitmask);

#endif /* SPI_1_H_ */
