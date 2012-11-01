#include "protocol.h"
#include "usart.h"
#include "printf.h"
#include "crc.h"
#include "protocol_handler.h"

struct ProtocolHeader headerInternal;
int receivedHeaderBytes = 0;
bool useCrc;

void (*protocolHandlers[NUM_IDS])(enum PROTOCOL_IDS id, unsigned char *data,
		unsigned short size);


void protocol_init(bool crc) {

	useCrc = crc;
	receivedHeaderBytes = 0;
	int i = 0;
	for (i = 0; i < NUM_IDS; i++) {
		protocolHandlers[i] = protocol_defaultHandler;
	}
	protocol_handler_init();
}

void protocol_registerHandler(enum PROTOCOL_IDS id,
		void (*handler)(enum PROTOCOL_IDS id, unsigned char *data,
				unsigned short size)) {
	protocolHandlers[id] = handler;
}

struct ProtocolHeader *protocol_receiveHeader() {
	unsigned char *buffer = (unsigned char *) &headerInternal;
//     print("Rec header\n");
	while (1) {
		int ret = USART1_GetData(buffer + receivedHeaderBytes,
				sizeof(struct ProtocolHeader) - receivedHeaderBytes);
		if (ret < 0) {
			//print("Error, receive buffer ran over");
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

		//wait until we received a full header
		if (receivedHeaderBytes < sizeof(struct ProtocolHeader))
			continue;

		struct ProtocolHeader *header = &headerInternal;

		if (useCrc) {
			int crc = protocol_calculateCRC(buffer,
					sizeof(struct ProtocolHeader) - sizeof(unsigned short));
			//printf("%d\n", crc);
			//check crc
			if (header->headerCRC != crc) {
				//                                                                                    print("Got wrong CRC\n");
				receivedHeaderBytes = 0;
				continue;
			}
		}
		receivedHeaderBytes = 0;

		return &headerInternal;
	}

	return 0;
}
;

unsigned char protocol_receiveData() {

	struct ProtocolHeader *header;
	unsigned char buffer[MAX_PACKET_SIZE];

	int received = 0;

	while (1) {
		//receive header
		header = protocol_receiveHeader();

		//no header there
		if (!header)
			return 0;

		//got a valid header, receive the rest of the packet
		while (received < header->payloadSize) {
			int ret = USART1_GetData(buffer + received,
					header->payloadSize - received);
			if (ret < 0) {
				//print("Error, receive buffer ran over");
				//discard all data and start over
				received = 0;
			} else
				received += ret;
		}

		//check payload crc
		if (useCrc) {
			if (header->payloadCRC
					!= protocol_calculateCRC(buffer, header->payloadSize)) {
				//print("Wrong Payload CRC \n");
				received = 0;
				continue;
			}
		}

		//ACK or Nack

		//call handler
// 	assert_param(header->id <= NUM_IDS);
		//print("proHeaer");
		protocolHandlers[header->id](header->id, buffer, header->payloadSize);

		return 1;
	}
}

short unsigned int protocol_calculateCRC(unsigned char* data,
		short unsigned int size) {
	return crcSlow(data, size);;
}

void protocol_sendData(enum PROTOCOL_IDS id, unsigned char* data,
		short unsigned int size) {
	struct ProtocolHeader header;

	header.magicByte = MAGIC_BYTE;
	header.id = id;
	header.payloadSize = size;
	header.payloadCRC = protocol_calculateCRC(data, size);
	header.headerCRC = protocol_calculateCRC((unsigned char *) &header,
			sizeof(struct ProtocolHeader) - sizeof(unsigned short));

	int sent = 0;
	while (sent < sizeof(struct ProtocolHeader)) {
		int ret = USART1_SendData(((unsigned char *) &header) + sent,
				sizeof(struct ProtocolHeader) - sent);
		if (ret < 0) {
			//print("Error, sending failed");
			sent = 0;
		} else
			sent += ret;
	}

	sent = 0;
	while (sent < size) {
		int ret = USART1_SendData(data + sent, size - sent);
		if (ret < 0) {
			//print("Error, sending failed");
			sent = 0;
		} else
			sent += ret;
	}
}

