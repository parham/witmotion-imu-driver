//
// Created by phm on 14/10/24.
//

#ifndef FODCAMERADRIVER_IMU_H
#define FODCAMERADRIVER_IMU_H

#include <string>
#include <iostream>

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

    class IMUListener {
    public:
        virtual void OnReceive_Time(TimePacket * msg) {};
        virtual void OnReceive_Acceleration(AccelerationPacket * msg) {};
        virtual void OnReceive_AngularVelocity(AngularVelocityPacket * msg) {};
        virtual void OnReceive_Angle(AnglePacket * msg) {};
        virtual void OnReceive_Magnetic(MagneticPacket * msg) {};
        virtual void OnReceive_BarometricAltitude(BarometricAltitudePacket * msg) {};
        virtual void OnReceive_Location(LocationPacket * msg) {};
        virtual void OnReceive_GPS(GPSPacket * msg) {};
        virtual void OnReceive_Quaternion(QuaternionPacket * msg) {};
        virtual void OnReceive_GpsAccuracy(GPSAccuracyPacket * msg) {};
    };

    class NullIMUListener : public IMUListener {
    public:
        void OnReceive_Time(TimePacket * msg) override  { std::cout << msg->toString() << std::endl; };
        void OnReceive_Acceleration(AccelerationPacket * msg) override { std::cout << msg->toString() << std::endl; };
        void OnReceive_AngularVelocity(AngularVelocityPacket * msg) override { std::cout << msg->toString() << std::endl; };
        void OnReceive_Angle(AnglePacket * msg) override { std::cout << msg->toString() << std::endl; };
        void OnReceive_Magnetic(MagneticPacket * msg) override { std::cout << msg->toString() << std::endl; };
        void OnReceive_BarometricAltitude(BarometricAltitudePacket * msg) override { std::cout << msg->toString() << std::endl; };
        void OnReceive_Location(LocationPacket * msg) override { std::cout << msg->toString() << std::endl; };
        void OnReceive_GPS(GPSPacket * msg) override { std::cout << msg->toString() << std::endl; };
        void OnReceive_Quaternion(QuaternionPacket * msg) override { std::cout << msg->toString() << std::endl; };
        void OnReceive_GpsAccuracy(GPSAccuracyPacket * msg) override { std::cout << msg->toString() << std::endl; };
    };

    class IMUDriver {
    private: // Fields
        Baudrate baudrate;
        std::string devFile;
        Serial * devHandler = nullptr;
        IMUListener * imuListener = nullptr;
    public:
        IMUDriver(Baudrate, std::string);
        [[nodiscard]] Baudrate getBaudrate() const { return baudrate; }

        void setListener(IMUListener * listener) { imuListener = listener; }

        bool isOpened();
        void open();
        void close();

        void receive();
    private:
        void initSerial();
        void deinitSerial();
        void readPacket(DataPacket *);
        void dispatch(ReceivePacket *);
    };

}

#endif //FODCAMERADRIVER_IMU_H
