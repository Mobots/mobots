#include "ComProtocol.hpp"
#include <iostream>
#include "ProtocolHandler.hpp"

struct ProtocolHeader headerInternal;
int receivedHeaderBytes = 0;

ComProtocol::ComProtocol(Communication* com) {
	this->com = com;
}

short unsigned int calculateCRC(unsigned char* data, short unsigned int size) {
	return crcSlow(data, size);
}

void (*protocolHandlers[NUM_IDS])(enum PROTOCOL_IDS id, unsigned char *data,
		unsigned short size, class Communication* com);

void ComProtocol::protocol_registerHandler(enum PROTOCOL_IDS id,
		void (*handler)(enum PROTOCOL_IDS id, unsigned char *data,
				unsigned short size, Communication* com)) {
	protocolHandlers[id] = handler;
}

void ComProtocol::protocolHandler_init() {
	protocol_registerHandler(SensorData_DeltaVal, setDataValHandler);
}



void ComProtocol::protocol_init() {
	receivedHeaderBytes = 0;
	int i = 0;
	for (i = 0; i < NUM_IDS; i++) {
		protocolHandlers[i] = protocol_defaultHandler;
	}
	protocolHandler_init();
}



struct ProtocolHeader* ComProtocol::protocol_receiveHeader() {
	unsigned char *buffer = (unsigned char *) &headerInternal;
//     print("Rec header\n");
	//std::cout <<"Rec header"<<std::endl;
	while (1) {
		int ret = com->read(buffer + receivedHeaderBytes,
				sizeof(struct ProtocolHeader) - receivedHeaderBytes);
		if (ret < 0) {
			std::cout << "Error, receive buffer ran over" << std::flush;
			//discard all data and start over
			receivedHeaderBytes = 0;
			continue;
		}
		receivedHeaderBytes += ret;

		//not more data there, try again later
		if (ret == 0)
			return 0;

		//search for the magic byte
		int i;
		unsigned char found_magic = 0;
		for (i = 0; i < receivedHeaderBytes; i++) {
			if (buffer[i] == MAGIC_BYTE) {
				found_magic = 1;
				//discard all data before the magic byte
				int j;
				for (j = i; j < receivedHeaderBytes; j++) {
					buffer[j - i] = buffer[j];
				}
				receivedHeaderBytes -= i;
				break;
			}
		}

		//no start marker found, discard buffer
		if (!found_magic) {
			receivedHeaderBytes = 0;
			continue;
		}

		//std::cout <<"MB found"<<std::endl;
		//wait until we received a full header
		if (receivedHeaderBytes < (int) sizeof(struct ProtocolHeader))
			continue;

		struct ProtocolHeader *header = &headerInternal;

		//check crc
		if (header->headerCRC
				!= calculateCRC(buffer,
						sizeof(struct ProtocolHeader)
								- sizeof(unsigned short))) {
			std::cout <<"Got wrong CRC"<<std::endl;
			//  print("Got wrong CRC");
			receivedHeaderBytes = 0;
			continue;
		}

		receivedHeaderBytes = 0;

		//std::cout <<"valid header "<<std::endl;
		return &headerInternal;
	}

	return 0;
}
;

void ComProtocol::receiveData() {
	// ----------- header überprüfen ------------
	struct ProtocolHeader *header;
	unsigned char buffer[MAX_PACKET_SIZE];

	int received = 0;

	while (1) {
		//receive header
		header = protocol_receiveHeader();

		//no header there
		if (!header)
			return;

		// ----------- header überprüfen ------------
		//----------------------------- Daten empfangen-----------
		//got a valid header, receive the rest of the packet
		while (received < header->payloadSize) {
			int ret = com->read(buffer + received,
					header->payloadSize - received);
			if (ret < 0) {
				std::cout << "Error, receive buffer ran over" << std::flush;
				//discard all data and start over
				received = 0;
			} else
				received += ret;
		}

		//check payload crc
		if (header->payloadCRC != calculateCRC(buffer, header->payloadSize)) {
			std::cout << "Wrong Payload CRC \n" << std::flush;
			received = 0;
			continue;
		}
		//----------------------------- Daten empfangen-----------

		//TODO ACK or Nack
		//---------------------------call handler-----------------
		// 	assert_param(header->id <= NUM_IDS);
		//std::cout <<"ProtcolHandler"<<std::endl;
		protocolHandlers[header->id](header->id, buffer, header->payloadSize, com);

		return;
	}

}

void ComProtocol::sendData(PROTOCOL_IDS id, unsigned char* data,
		short unsigned int size) {
	struct ProtocolHeader header;

	header.magicByte = MAGIC_BYTE;
	header.id = id;
	header.payloadSize = size;
	header.payloadCRC = calculateCRC(data, size);
	header.headerCRC = calculateCRC((unsigned char *) &header,
			sizeof(struct ProtocolHeader) - sizeof(unsigned short));

	//std::cout << header.headerCRC << std::endl;

	unsigned int sent = 0;
	while (sent < sizeof(struct ProtocolHeader)) {
		int ret = com->write(((unsigned char *) &header) + sent,
				sizeof(struct ProtocolHeader) - sent);
		if (ret < 0) {
			std::cout << "Error, sending failed" << std::endl;
			sent = 0;
		} else
			sent += ret;
	}

	sent = 0;
	while (sent < size) {
		int ret = com->write(data + sent, size - sent);
		if (ret < 0) {
			std::cout << "Error, sending failed" << std::endl;
			sent = 0;
		} else
			sent += ret;
	}
	//std::cout << (unsigned char*)&header << std::endl;
	//std::cout << (unsigned char*)data << std::endl;
}

