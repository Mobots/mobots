#ifndef __USART_H
#define __USART_H


enum USART_MODE
{
    USART_POLL,
    USART_USE_INTERRUPTS,
};

/**
 * Initializes the USART 1 with 115200 Baud
 * 
 * The parameter mode defines wheather the driver will use 
 * an interrupt to send and receive the data.
 **/
void USART1_Init(enum USART_MODE mode);

/**
 * Deinitializes the USART1
 **/
void USART1_DeInit(void);

/**
 * Sends data out via USART1. 
 * 
 * Arguments are a pointer to the data and the size of the
 * data to be send.
 *
 * returns nr ob bytes send or -1 in case the data could not be sent.
 **/
signed int USART1_SendData(const unsigned char *data, const unsigned int size);

/**
 * Receives data via USART1
 *
 * Arguments are a pointer to an array were the received data
 * should be saved. Also the maximum number of bytes which may
 * be stored in the array, is given.
 *
 * Note this function will only work if the driver uses the 
 * interrupt mode.
 *
 * Returns the amount of received bytes. Or -1 in case that
 * the internal receive buffer overrun. 
 **/
signed int USART1_GetData (unsigned char *buffer, const unsigned int buffer_length);


#endif
