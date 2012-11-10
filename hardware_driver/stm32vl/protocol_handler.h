/*
 * protocol_handler.h
 *
 *  Created on: 24.09.2012
 *      Author: simon
 */

#ifndef PROTOCOL_HANDLER_H_
#define PROTOCOL_HANDLER_H_

void protocol_handler_init();
void protocol_defaultHandler(enum PROTOCOL_IDS id, unsigned char *data, unsigned short size);

#endif /* PROTOCOL_HANDLER_H_ */
