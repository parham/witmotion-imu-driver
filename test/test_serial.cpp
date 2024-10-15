//
// Created by phm on 10/10/24.
//

#include <iostream>
#include <memory>
#include <unistd.h>
#include <string>

#include "Serial.h"
#include "IMU.h"

int main () {

    std::cout << "Test is done successfully." << std::endl;
    std::string devfile = "/dev/ttyUSB0";
    phm::witmotion::IMUDriver imu(230400, devfile);
    imu.open();
    usleep(1000);
    for (int index = 0; index < 200; index++) {
        phm::witmotion::DataPacket dp;
        imu.readPacket(&dp);
        std::cout << "Code: " << "Code: 0x" << std::hex << static_cast<int16_t>(dp.code) << ", CRC: " << static_cast<int16_t>(dp.crc) << std::endl;
    }

    imu.close();
}