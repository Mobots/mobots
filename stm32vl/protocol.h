#ifndef _PROTOCOL_H__
#define _PROTOCOL_H__


enum PROTOCOL_IDS
{
    ACK = 0,
    NACK,
    SET_STEP,
    SET_SEQUENCE,
    NUM_IDS,
};

struct ProtocolHeader
{
    unsigned char magicByte;
    unsigned short payloadSize;
    enum PROTOCOL_IDS id;
    unsigned short payloadCRC;
    unsigned short headerCRC;
} __attribute__ ((packed)) __attribute__((__may_alias__));



void protocol_init();
void protocol_receiveData();
unsigned short protocol_calculateCRC(unsigned char *data, unsigned short size);

void protocol_sendData(enum PROTOCOL_IDS id, unsigned char *data, unsigned short size);


#define MAX_PACKET_SIZE (1024 + sizeof(struct ProtocolHeader))

#define MAGIC_BYTE 0x3f

/**
 * Registers a handler function for a special ID
 * */
void protocol_registerHandler(enum PROTOCOL_IDS id, void (*handler)(enum PROTOCOL_IDS id, unsigned char *data, unsigned short size));

#endif
