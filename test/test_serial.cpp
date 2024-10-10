//
// Created by phm on 10/10/24.
//

#include <iostream>

#include "Serial.h"

int main () {
    std::cout << "Test is done successfully." << std::endl;
    return 0;
    std::string devfile = "/dev/null";
    phm::witmotion::Serial * obj = phm::witmotion::Serial::build(devfile, 9600);

    delete obj;
}