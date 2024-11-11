//
// Created by phm on 14/10/24.
//

#include <cstdint>
#include <sstream>
#include <cmath>

#ifndef FODCAMERADRIVER_MESSAGE_H
#define FODCAMERADRIVER_MESSAGE_H

#define DPACKET_BODY_SIZE 8

typedef enum {
    Time = 0x50,
    Acceleration = 0x51,
    AngularVelocity = 0x52,
    Angle = 0x53,
    Magnetic = 0x54,
    Port = 0x55,
    BarometricAltitude = 0x56,
    Location = 0x57,
    GPS = 0x58,
    Quaternion = 0x59,
    GpsAccuracy = 0x5A
} DPCode;

typedef struct {
    uint8_t code;
    int8_t body[DPACKET_BODY_SIZE];
    uint8_t crc;
} DataPacket;

struct ReceivePacket {
    uint8_t code;

public:
    explicit ReceivePacket(uint8_t cd) : code(cd) { /* Empty body */ }
    virtual ~ReceivePacket() = default;
    virtual void parse(DataPacket * pd) = 0;
    virtual std::string toString() = 0;
};

struct TimePacket : ReceivePacket {
    uint8_t year{};
    uint8_t month{};
    uint8_t day{};
    uint8_t hour{};
    uint8_t minute{};
    uint8_t second{};
    uint16_t milisecond{};

public:
    TimePacket() : ReceivePacket(static_cast<uint8_t>(DPCode::Time)) {}
    void parse(DataPacket * pd) override;
    std::string toString() override {
        std::stringstream ss;
        ss << "[ " << static_cast<int16_t>(year)
            << "-" << static_cast<int16_t>(month)
            << "-" << static_cast<int16_t>(day)
            << "\t" << static_cast<int16_t>(hour)
            << ":" << static_cast<int16_t>(minute)
            << ":" << static_cast<int16_t>(second)
            << "-" << static_cast<int16_t>(milisecond) << "]";
        return ss.str();
    }
};

struct AccelerationPacket : ReceivePacket {
    double ac_x{};
    double ac_y{};
    double ac_z{};
    double temperature{};

public:
    AccelerationPacket() : ReceivePacket(static_cast<uint8_t>(DPCode::Acceleration)) {}
    void parse(DataPacket * pd) override;
    std::string toString() override {
        std::stringstream ss;
        ss << "ACC [" << ac_x << ", " << ac_y << ", " << ac_z << "], Temp = " << temperature;
        return ss.str();
    }
};

struct AngularVelocityPacket : ReceivePacket {
    double av_x{};
    double av_y{};
    double av_z{};
    double temperature{};

public:
    AngularVelocityPacket() : ReceivePacket(static_cast<uint8_t>(DPCode::AngularVelocity)) {}
    void parse(DataPacket * pd) override;
    std::string toString() override {
        std::stringstream ss;
        ss << "ANGV [" << av_x << ", " << av_y << ", " << av_z << "], Temp = " << temperature;
        return ss.str();
    }
};

struct AnglePacket : ReceivePacket {
    double roll{};
    double pitch{};
    double yaw{};
    int16_t version{};

public:
    AnglePacket() : ReceivePacket(static_cast<uint8_t>(DPCode::Angle)) {}
    void parse(DataPacket * pd) override;
    std::string toString() override {
        std::stringstream ss;
        ss << "ANGLE [" << roll << ", " << pitch << ", " << yaw << "], Version = " << version;
        return ss.str();
    }
};

struct MagneticPacket : ReceivePacket {
    int16_t mag_x{};
    int16_t mag_y{};
    int16_t mag_z{};
    int16_t temperature{};

public:
    MagneticPacket() : ReceivePacket(static_cast<uint8_t>(DPCode::Magnetic)) {}
    void parse(DataPacket * pd) override;
    std::string toString() override {
        std::stringstream ss;
        ss << "MAG [" << mag_x << ", " << mag_y << ", " << mag_z << "], Temp = " << temperature;
        return ss.str();
    }
};

struct BarometricAltitudePacket : ReceivePacket {
    int32_t airPressure{};
    int32_t altitude{};

public:
    BarometricAltitudePacket() : ReceivePacket(static_cast<uint8_t>(DPCode::BarometricAltitude)) {}
    void parse(DataPacket * pd) override;
    std::string toString() override {
        std::stringstream ss;
        ss << "Air Pressure : " << airPressure << ", Altitude : " << altitude;
        return ss.str();
    }
};

#define GPS_DECIMAL 10000000

struct LocationPacket : ReceivePacket {
    int32_t longitude{};
    int32_t latitude{};

public:
    LocationPacket() : ReceivePacket(static_cast<uint8_t>(DPCode::Location)) {}
    void parse(DataPacket * pd) override;
    std::string toString() override {
        std::stringstream ss;
        ss << "Longitude : " << longitude << ", Latitude = " << latitude;
        return ss.str();
    }
};

struct GPSPacket : ReceivePacket {
    double altitude{};
    double heading{};
    double groundSpeed{};

public:
    GPSPacket() : ReceivePacket(static_cast<uint8_t>(DPCode::GPS)) {}
    void parse(DataPacket * pd) override;
    std::string toString() override {
        std::stringstream ss;
        ss << "Altitude : " << altitude << ", Heading = " << heading << ", GSpeed = " << groundSpeed;
        return ss.str();
    }
};

struct GPSAccuracyPacket : ReceivePacket {
    int numSats{};
    double positionAccuracy{};
    double hPrecision{};
    double vPrecision{};

public:
    GPSAccuracyPacket() : ReceivePacket(static_cast<uint8_t>(DPCode::GpsAccuracy)) {}
    void parse(DataPacket * pd) override;
    std::string toString() override {
        std::stringstream ss;
        ss << "PAccuracy : " << positionAccuracy << ", Sats = " << numSats
            << ", hPrecision = " << hPrecision
            << ", vPrecision = " << vPrecision;
        return ss.str();
    }
};

struct QuaternionPacket : ReceivePacket {
    double q1{};
    double q2{};
    double q3{};
    double q4{};

public:
    QuaternionPacket() : ReceivePacket(static_cast<uint8_t>(DPCode::Quaternion)) {}
    void parse(DataPacket * pd) override;
    std::string toString() override {
        std::stringstream ss;
        ss << "Q1 = " << q1
           << ", Q2 = " << q2
           << ", Q3 = " << q3
           << ", Q4 = " << q4;
        return ss.str();
    }
};

void build_ReceivePacket (DPCode, ReceivePacket **);

#endif //FODCAMERADRIVER_MESSAGE_H
