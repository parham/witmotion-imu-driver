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
    phm::witmotion::NullIMUListener mlistener;
    imu.setListener(&mlistener);
    imu.open();
    usleep(1000);
    for (int index = 0; index < 200; index++) {
        imu.receive();
    }

    imu.close();
}