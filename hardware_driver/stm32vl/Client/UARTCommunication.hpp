#ifndef UARTCOMMUNICATION_HPP
#define UARTCOMMUNICATION_HPP

#include "Communication.hpp"
#include <termios.h>

class UARTCommunication : public Communication
{
private:
    int tty_fd;
    struct termios tio;
public:
    UARTCommunication();
    virtual ~UARTCommunication();
    virtual int write(unsigned char *data, size_t size);
    virtual int read(unsigned char *buffer, size_t buffersize);
};

#endif // UARTCOMMUNICATION_HPP
