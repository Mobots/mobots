#include "ComProtocol.hpp"
#include "UARTCommunication.hpp"
#include <string.h>
#include <iostream>
#include <pthread.h>
#include <unistd.h>
#include <sys/types.h>
#include <boost/lexical_cast.hpp>
#include <signal.h>

UARTCommunication com;
ComProtocol protocol(&com);
struct MouseData destination = {0, 0, 0};

void readPrintfs(Communication* com) {
	unsigned char buf;
	while (com->read(&buf, sizeof(char)))
		std::cout << buf << std::flush;
}

void defaultHandler(enum PROTOCOL_IDS id, unsigned char *data, unsigned short size, Communication* com) {
	std::cerr << "no handler specified for id: " << id << std::endl;
}


void mouseDataHandler(enum PROTOCOL_IDS id, unsigned char *data, unsigned short size, Communication* com) {
	if (id != MOUSE_DATA) {
		std::cout << "Error, wrong ID\n" << std::endl;
		return;
	}

	if (size != sizeof(struct MouseData)) {
		std::cout << "Error, wrong size\n" << std::endl;
		return;
	}

	struct MouseData *mouse = (struct MouseData*) data;

	std::cout << "mouse->x: " << mouse->x << "\tmouse->y: " << mouse->y << "\tmouse->theta:" << mouse->theta << std::endl;

    static struct MouseData integral = {0, 0, 0};
    integral.theta += mouse->theta;    
    integral.x += cos(integral.theta) * mouse->x - sin(integral.theta) * mouse->y;
	integral.y += sin(integral.theta) * mouse->x + cos(integral.theta) * mouse->y;

    std::cout << "integral.x: " << integral.x << "\tintegral.y: " << integral.y << "\tintegral.theta:" << integral.theta << std::endl;
    
	struct Velocity v_mobot = {destination.x - integral.x, destination.y - integral.y, 0};
    std::cout << "sending: v_mobot = (" << v_mobot.x << ',' << v_mobot.y << ',' << v_mobot.theta << ")" << std::endl << std::endl;
	protocol.sendData(VELOCITY, (unsigned char*) &v_mobot, sizeof(struct Velocity));

}

void sigint_handler(int signal) {
    struct Velocity v_mobot = {0, 0, 0};
    std::cout << "sending: v_mobot = (" << v_mobot.x << ',' << v_mobot.y << ',' << v_mobot.theta << ")" << std::endl;
	protocol.sendData(VELOCITY, (unsigned char*) &v_mobot, sizeof(struct Velocity));
    exit(EXIT_SUCCESS);
}


int main(int argc, char *argv[]) {
	protocol.protocol_init(defaultHandler);
	protocol.protocol_registerHandler(MOUSE_DATA, mouseDataHandler);

    signal(SIGINT, sigint_handler);

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
	protocol.sendData(VELOCITY, (unsigned char*) &v_mobot, sizeof(struct Velocity));


	while (1)
	{
		protocol.receiveData();
	}

	return EXIT_SUCCESS;
}


