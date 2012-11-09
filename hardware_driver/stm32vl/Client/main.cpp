#include "ComProtocol.hpp"
#include "UARTCommunication.hpp"
#include <string.h>
#include <iostream>
#include <pthread.h>
#include <unistd.h>
#include <sys/types.h>

void readPrintfs(Communication* com) {
	unsigned char buf;
	while (com->read(&buf, sizeof(char)))
		std::cout << buf << std::flush;
}

int main() {

	Communication* com;
	com = new UARTCommunication();
	ComProtocol proto(com);
	proto.protocol_init();

	std::cout << "begin\n" << std::flush;

	while (1)
	{

		usleep(10000);
		//proto.receiveData();
		float input[3] = {0.0, 0.0, 0.0};

		for(int i = 0; i < 3; i++)
		{
			std::cout << "input[" << i << "]" << std::endl;
			std::cin >> input[i];
		}

		struct Velocity v_mobot = {input[0], input[1], input[2]};

		proto.sendData(VELOCITY, (unsigned char*) &v_mobot, sizeof(struct Velocity));

		readPrintfs(com);
	}

	return 0;
}

