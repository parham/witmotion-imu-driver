//
// Created by phm on 14/10/24.
//

#ifndef FODCAMERADRIVER_IMU_H
#define FODCAMERADRIVER_IMU_H

#include <cstdint>
#include <string>

#include <Serial.h>

#define DPACKET_BODY_ARRSIZE 4
#define DPACKET_BODY_SIZE 8

#define START_BYTE 0x55

namespace phm::witmotion {

    typedef enum {
        Time = 0x50,
        Acceleration = 0x51,
        AngularVelocity = 0x52,
        Angle = 0x53,
        Magnetic = 0x54,
        Location = 0x57,
        GPS = 0x58,
        Quaternion = 0x59,
        GpsAccuracy = 0x5A
    } DPCode;

    typedef struct {
        uint8_t code;
        uint8_t body[DPACKET_BODY_ARRSIZE];
        uint8_t crc;
    } DataPacket;

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

        void readPacket(DataPacket *);
    private:
        void initSerial();
        void deinitSerial();
    };

}

#endif //FODCAMERADRIVER_IMU_H
