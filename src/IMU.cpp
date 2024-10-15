//
// Created by phm on 14/10/24.
//

#include "IMU.h"

#include <utility>
#include <iostream>
#include <cstring>

phm::witmotion::IMUDriver::IMUDriver(Baudrate brate, std::string fdev)
    :baudrate(brate), devFile(std::move(fdev)) {
    // Empty body
}

void phm::witmotion::IMUDriver::initSerial() {
    deinitSerial();
    devHandler = Serial::build(devFile, baudrate);
    devHandler->begin();
}

void phm::witmotion::IMUDriver::deinitSerial() {
    if (devHandler != nullptr) {
        if (devHandler->isOpened())
            devHandler->end();
        delete devHandler;
    }
}

bool phm::witmotion::IMUDriver::isOpened() {
    return devHandler != nullptr && devHandler->isOpened();
}

void phm::witmotion::IMUDriver::open() {
    // Initialize the serial port
    initSerial();
}

void phm::witmotion::IMUDriver::close() {
    deinitSerial();
}

void phm::witmotion::IMUDriver::readPacket(DataPacket * packet) {
    while(isOpened()) {
        // Wait for start signal
        uint8_t dbyte = 0;
        if (devHandler->receive(&dbyte, 1) == 0) {
            continue;
        }
        if (dbyte != START_BYTE) {
            continue;
        }
        // The start flag is detected.
        // Getting the header
        if (devHandler->receive(&dbyte, 1) == 0) {
            continue;
        }
        // std::cout << "state: header -> payload, got code 0x" << std::hex << dbyte << std::endl;
        // Receiving the packet body & CRC
        int8_t pact[DPACKET_BODY_SIZE + 1];
        memset(pact, 0, DPACKET_BODY_SIZE + 1);
        auto rp = devHandler->receive(reinterpret_cast<uint8_t *>(pact), DPACKET_BODY_SIZE + 1);
        if (rp != 9) {
            continue;
        }
        int8_t owtCrc = pact[DPACKET_BODY_SIZE];
        int8_t crc = computeChecksum(dbyte, pact, DPACKET_BODY_SIZE);
        if (owtCrc != crc) {
            std::cout << "Invalid checksum. code : 0x" << std::hex << static_cast<int16_t>(dbyte)
                    << ", expected : 0x" << std::hex << static_cast<int16_t>(owtCrc)
                    << ", got : 0x" << std::hex <<  static_cast<int16_t>(crc) << std::endl;
            continue;
        }
        // Fill the packet
        packet->code = static_cast<DPCode>(dbyte);
        memcpy(packet->body, pact, DPACKET_BODY_SIZE);
        packet->crc = crc;
        break;
    }
}

void phm::witmotion::IMUDriver::receive() {
    DataPacket packet;
    // Read a packet from RX serial
    readPacket(&packet);
    ReceivePacket * msg = nullptr;
    build_ReceivePacket(static_cast<DPCode>(packet.code), &msg);
    if (msg == nullptr) {
        return;
    }
    msg->parse(&packet);
    std::cout << msg->toString() << std::endl;
//    std::cout << "Code: 0x" << std::hex << static_cast<int16_t>(packet.code) << ", CRC: " << static_cast<int16_t>(packet.crc) << std::endl;
    delete msg;
}




