/*
 * protocol_handler.cpp
 *
 *  Created on: 28.10.2012
 *      Author: simon
 */

#include "ProtocolHandler.hpp"

void protocol_defaultHandler(enum PROTOCOL_IDS id, unsigned char *data,
		unsigned short size, class Communication* com) {
	//ßunsigned short idl = id;
	std::cout << "Warning, packet with id %hu was not handled \n" << std::flush;
}
