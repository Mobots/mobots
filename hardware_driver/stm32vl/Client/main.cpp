#include "ComProtocol.hpp"
#include "UARTCommunication.hpp"
#include <string.h>
#include <iostream>
#include <pthread.h>
#include <unistd.h>
#include <sys/types.h>
#include <boost/lexical_cast.hpp>

void readPrintfs(Communication* com) {
	unsigned char buf;
	while (com->read(&buf, sizeof(char)))
		std::cout << buf << std::flush;
}

void defaultHandler(enum PROTOCOL_IDS id, unsigned char *data, unsigned short size, Communication* com) {
	std::cerr << "no handler specified for id: " << id << std::endl;
}

int main(int argc, char *argv[]) {

	Communication* com;
	com = new UARTCommunication();
	ComProtocol proto(com);
	proto.protocol_init(defaultHandler);

	struct Velocity v_mobot = {0, 0, 0};

	switch(argc)
	{
	case 1:
		v_mobot = {0, 0, 0};
		break;

	case 1+3:
		v_mobot = { boost::lexical_cast<float>(argv[1]), boost::lexical_cast<float>(argv[2]), boost::lexical_cast<float>(argv[3]) };
		break;

	default:
		std::cout << "Wrong number of arguments!" << std::endl;
		return EXIT_FAILURE;
	}

	std::cout << "sending: v_mobot = (" << v_mobot.x << ',' << v_mobot.y << ',' << v_mobot.theta << ")" << std::endl;
	proto.sendData(VELOCITY, (unsigned char*) &v_mobot, sizeof(struct Velocity));


	/*while (1)
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
		std::cout << "sending: (" << v_mobot.x << ',' << v_mobot.y << ',' << v_mobot.theta << " schnauze )" << std::endl;

		proto.sendData(VELOCITY, (unsigned char*) &v_mobot, sizeof(struct Velocity));

		//readPrintfs(com);
	}*/

	return EXIT_SUCCESS;
}

