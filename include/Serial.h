//
// Created by phm on 10/10/24.
//

#ifndef FODCAMERADRIVER_SERIAL_H
#define FODCAMERADRIVER_SERIAL_H

#include <string>

typedef int FileHandler;
typedef unsigned int Baudrate;

namespace phm::witmotion {
    class Serial {
    private:
        std::string devFile;
        Baudrate baudrate;
        FileHandler devHandler = -1;
    private:
        Serial(std::string &, Baudrate);
    public:
        Serial() = delete;
        ~Serial();
        std::string getDevFile() {return devFile;}
        [[nodiscard]] Baudrate getBaudrate() const {return baudrate;}

        void begin();
        void end();
        int receive(uint8_t *, size_t length);
        int send(uint8_t *, size_t length);
        [[nodiscard]] bool isOpened() const { return devHandler >= 0; }

    public: // Static methods
        static Serial * build(std::string &, Baudrate baud);
    };

    typedef Serial * pSerial;
}

#endif //FODCAMERADRIVER_SERIAL_H
