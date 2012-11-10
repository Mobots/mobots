/*
 * protocol_handler.hpp
 *
 *  Created on: 28.10.2012
 *      Author: simon
 */

#ifndef PROTOCOLHANDLER_HPP
#define PROTOCOLHANDLER_HPP
#include "Communication.hpp"
#include <iostream>
extern "C" {
#include "../protocol.h"
}


void protocol_defaultHandler(enum PROTOCOL_IDS id, unsigned char *data, unsigned short size, class Communication* com);

#endif /* PROTOCOL_HANDLER_HPP_ */
