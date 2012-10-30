#ifndef _PROTOCOL_H__
#define _PROTOCOL_H__

#include "engine.h"
#include "stm32f10x.h"

enum PROTOCOL_IDS
{
    ACK = 0,
    NACK,
    SET_POINT,
    SET_TRAJEKTORY,
    REQUEST,
    WARNING,
    SensorData_All,
    SensorData_DeltaVal,
    NUM_IDS,
    Servo //zum debuggen
};

typedef enum{
	MouseData_All = 0,
	MouseData_DeltaVal
} REQUEST_TYP;

typedef enum{
	UnknownRequest = 0,
	UnknownPointFormat,
	UnknownTrajektory,
	UnknownWarning
} WARNING_TYP;

struct Request
{
	REQUEST_TYP req_typ;
};

struct ProtocolHeader
{
    unsigned char magicByte;
    unsigned short payloadSize;
    enum PROTOCOL_IDS id;
    unsigned short payloadCRC;
    unsigned short headerCRC;
}__attribute__((packed)) __attribute__((__may_alias__));




void protocol_init(bool crc);

/**
 * this function will check weather a packet can be read 
 * from usart 1. If this is the case the function will
 * block until the whole packet is received, call the 
 * registered packet handler and return 1.
 * Will return 0 if no packet can be received.
 */
unsigned char protocol_receiveData();
unsigned short protocol_calculateCRC(unsigned char *data, unsigned short size);

void protocol_sendData(enum PROTOCOL_IDS id, unsigned char *data, unsigned short size);


#define MAX_PACKET_SIZE (1024 + sizeof(struct ProtocolHeader))

#define MAGIC_BYTE 0x3f

/**
 * Registers a handler function for a special ID
 * */
void protocol_registerHandler(enum PROTOCOL_IDS id, void (*handler)(enum PROTOCOL_IDS id, unsigned char *data, unsigned short size));

#endif
