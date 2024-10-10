//
// Created by phm on 10/10/24.
//

#include "Serial.h"

#include <filesystem>
#include <iostream>

#include <linux/types.h>
#include <cstdio>
#include <ctime>
#include <cstdlib>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <cerrno>
#include <cstring>
#include <linux/rtc.h>
#include <termios.h>

phm::witmotion::Serial * phm::witmotion::Serial::build(std::string & dev, Baudrate baud) {
    // Build an instance of Serial (builder design pattern).
    // Validate the passed arguments
    if (baud < 1 || !std::filesystem::exists(dev)) {
        return nullptr;
    }
    return new phm::witmotion::Serial(dev, baud);
}

phm::witmotion::Serial::Serial(std::string & dev, Baudrate baud) : baudrate(baud), devFile(dev) {
    // Empty body
}

phm::witmotion::Serial::~Serial() {
    if (devHandler >= 0) {
        end();
    }
}

void phm::witmotion::Serial::begin() {
    int fd = open(devFile.c_str(), O_RDWR | O_NOCTTY);
    if(isatty(STDIN_FILENO)==0) {
        std::cout << "The standard input was not a terminal device!" << std::endl;
    } else {
        std::cout << "The standard input was successful!" << std::endl;
    }
    struct termios newtio{},oldtio{};
    if (!tcgetattr(fd,&oldtio)) {
        std::cerr << "Setup Serial 1 : " << "tcgetattr( fd,&oldtio) ->" << tcgetattr( fd,&oldtio) << std::endl;
        return;
    }

    bzero( &newtio, sizeof( newtio ) );
    newtio.c_cflag  |=  CLOCAL | CREAD;
    newtio.c_cflag |= CS8;
    newtio.c_cflag &= ~PARENB;

    // Setting the baud rate
    switch(baudrate) {
        case 2400:
            cfsetispeed(&newtio, B2400);
            cfsetospeed(&newtio, B2400);
            break;
        case 4800:
            cfsetispeed(&newtio, B4800);
            cfsetospeed(&newtio, B4800);
            break;
        case 9600:
            cfsetispeed(&newtio, B9600);
            cfsetospeed(&newtio, B9600);
            break;
        case 115200:
            cfsetispeed(&newtio, B115200);
            cfsetospeed(&newtio, B115200);
            break;
        case 230400:
            cfsetispeed(&newtio, B230400);
            cfsetospeed(&newtio, B230400);
            break;
        case 460800:
            cfsetispeed(&newtio, B460800);
            cfsetospeed(&newtio, B460800);
            break;
        default:
            cfsetispeed(&newtio, B9600);
            cfsetospeed(&newtio, B9600);
            break;
    }

    newtio.c_cflag &=  ~CSTOPB;
    newtio.c_cc[VTIME]  = 0;
    newtio.c_cc[VMIN] = 0;
    tcflush(fd,TCIFLUSH);

    if(!tcsetattr(fd,TCSANOW,&newtio)) {
        std::cerr << "COM set error" << std::endl;
        return;
    }

    // Check if opening was successful
    if (fd < 0) {
        devHandler = fd;
    }
}

void phm::witmotion::Serial::end() {
    if (isOpened()) {
        close(devHandler);
        devHandler = -1;
    }
}

int phm::witmotion::Serial::receive(uint8_t * buffer, size_t length) {
    return read(devHandler, buffer, length);
}

int phm::witmotion::Serial::send(uint8_t * buffer, size_t length) {
    return write(devHandler, buffer, length * sizeof(uint8_t));
}