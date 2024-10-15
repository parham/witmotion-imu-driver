//
// Created by phm on 14/10/24.
//

#ifndef FODCAMERADRIVER_IMU_H
#define FODCAMERADRIVER_IMU_H

#include <string>

#include <Serial.h>
#include <Message.h>

#define START_BYTE 0x55

namespace phm::witmotion {

    auto computeChecksum = [](uint8_t code, int8_t * body, int len) {
        int16_t checksum = 0x55 + (int16_t) (code & 0x00ff);
        for (int ind = 0; ind < len; ind++) {
            checksum += body[ind];
        }
        checksum &= 0xff;
        return (int8_t) checksum;
    };

    class IMUDriver {
    private: // Fields
        Baudrate baudrate;
        std::string devFile;
        Serial * devHandler = nullptr;
    public:
        IMUDriver(Baudrate, std::string);
        [[nodiscard]] Baudrate getBaudrate() const { return baudrate; }

        bool isOpened();
        void open();
        void close();

        void receive();

        void readPacket(DataPacket *);
    private:
        void initSerial();
        void deinitSerial();
    };

}

#endif //FODCAMERADRIVER_IMU_H
