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

/*
int i = 0;
int j = 0;
void setDataValHandler(enum PROTOCOL_IDS id, unsigned char *data,
		unsigned short size, Communication* com) {

	//std::cout <<"datavalHandler"<<std::endl;

	if (id != SensorData_DeltaVal) {
		std::cout << "Error, wrong ID\n" << std::endl;
		return;
	}

	if (size != sizeof(struct Mouse_Data_DeltaVal)) {
		std::cout << "Error, wrong size\n" << std::endl;
		return;
	}

	struct Mouse_Data_DeltaVal *delta_vals = (struct Mouse_Data_DeltaVal*) data;

	i += delta_vals->delta_x;
	j += delta_vals->delta_y;

	std::cout << "X: " << i << std::flush;
	std::cout << " Y: " << j << std::endl;
}
*/
