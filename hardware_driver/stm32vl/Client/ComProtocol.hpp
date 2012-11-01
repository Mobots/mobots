#ifndef COMPROTOCOL_HPP
#define COMPROTOCOL_HPP

#include "Communication.hpp"
extern "C" {
#include "../protocol.h"
#include "../mousesensor.h"
#include "../crc.h"
#include "../servo.h"
}

class ComProtocol
{
private:
class Communication *com;
struct ProtocolHeader* protocol_receiveHeader();

public:
ComProtocol(class Communication *com);
void sendData(enum PROTOCOL_IDS id, unsigned char *data, unsigned short size);
void receiveData();
void protocol_init(void (*defaultHandler)(enum PROTOCOL_IDS id, unsigned char *data, unsigned short size, class Communication* com));
void protocol_registerHandler(enum PROTOCOL_IDS id, void (*handler)(enum PROTOCOL_IDS id, unsigned char *data, unsigned short size, class Communication* com));

};

#endif // COMPROTOCOL_HPP
