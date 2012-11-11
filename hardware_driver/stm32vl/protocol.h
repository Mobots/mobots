#ifndef _PROTOCOL_H__
#define _PROTOCOL_H__

#include "stm32f10x.h"

enum PROTOCOL_IDS
{
    VELOCITY,
    SERVO_SPEED,
    MOUSE_DATA,
    NUM_IDS // muss immer an letzter stelle stehen
};

struct Velocity
{
	float x;
	float y;
	float theta;
} __attribute__((packed)) __attribute__((__may_alias__));

extern volatile struct Velocity last_velocity_command;

struct ServoSpeed
{
	float v[3]; // v0, v1, v2
} __attribute__((packed)) __attribute__((__may_alias__));

struct MouseData
{
	float x;
	float y;
	float theta;
} __attribute__ ((packed)) __attribute__((__may_alias__));

struct ProtocolHeader
{
    unsigned char magicByte;
    unsigned short payloadSize;
    enum PROTOCOL_IDS id;
    unsigned short payloadCRC;
    unsigned short headerCRC;
} __attribute__((packed)) __attribute__((__may_alias__));


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
