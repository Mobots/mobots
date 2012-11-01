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
	std::cout << sizeof(struct ServoSpeed) << std::endl;
//	struct ServoSpeed sersp;
//	sersp.s1 = 0;
//	sersp.s2 = 0;
//	sersp.s3 = 0;

	while (1) {
		usleep(10000);
		proto.receiveData();
		//proto.sendData(Servo, (unsigned char*) &sersp, sizeof(struct ServoSpeed));
		readPrintfs(com);
	}
	return 0;
}

