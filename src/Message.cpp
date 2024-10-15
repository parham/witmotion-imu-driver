//
// Created by phm on 14/10/24.
//

#include <iostream>

#include "Message.h"

#define GRAVITY 9.8

void TimePacket::parse(DataPacket *pd) {
    year = pd->body[0];
    month = pd->body[1];
    day = pd->body[2];
    hour = pd->body[3];
    minute = pd->body[4];
    second = pd->body[5];
    auto msl = pd->body[6];
    auto msh = pd->body[7];
    milisecond = ((uint16_t) msh << 8) | msl;
}

void AccelerationPacket::parse(DataPacket *pd) {
    // X=((AxH<<8)|AxL)/32768*16g
    // AC X
    int ac = pd->body[1] << 8;
    ac += pd->body[0];
    ac_x = (ac / 32768.0f) * 16.0f * GRAVITY;
    // AC Y
    ac = pd->body[3] << 8;
    ac += pd->body[2];
    ac_y = (ac / 32768.0f) * 16.0f * GRAVITY;
    // AC Z
    ac = pd->body[5] << 8;
    ac += pd->body[4];
    ac_z = (ac / 32768.0f) * 16.0f * GRAVITY;
    // TEMP
    ac = pd->body[7] << 8;
    ac += pd->body[6];
    temperature = ac / 100.0f;
}

void AngularVelocityPacket::parse(DataPacket *pd) {
    // X=((WxH<<8)|WxL)/32768*2000°/s
    // Ag X
    int ac = pd->body[1] << 8;
    ac += pd->body[0];
    av_x = (ac / 32768.0f) * 2000.0f;
    // Ag Y
    ac = pd->body[3] << 8;
    ac += pd->body[2];
    av_y = (ac / 32768.0f) * 2000.0f;
    // Ag Z
    ac = pd->body[5] << 8;
    ac += pd->body[4];
    av_z = (ac / 32768.0f) * 2000.0f;
    // TEMP
    ac = pd->body[7] << 8;
    ac += pd->body[6];
    temperature = ac / 100.0f;
}

void AnglePacket::parse(DataPacket *pd) {
    // Ag X
    int ac = pd->body[1] << 8;
    ac += pd->body[0];
    roll = (ac / 32768.0f) * 180.0f;
    // Ag Y
    ac = pd->body[3] << 8;
    ac += pd->body[2];
    pitch = (ac / 32768.0f) * 180.0f;
    // Ag Z
    ac = pd->body[5] << 8;
    ac += pd->body[4];
    yaw = (ac / 32768.0f) * 180.0f;
    // VERSION
    ac = pd->body[7] << 8;
    ac += pd->body[6];
    version = ac;
}

void MagneticPacket::parse(DataPacket *pd) {
    mag_x = pd->body[1] << 8;
    mag_x += pd->body[0];
    mag_y = pd->body[3] << 8;
    mag_y += pd->body[2];
    mag_z = pd->body[5] << 8;
    mag_z += pd->body[4];
}

void BarometricAltitudePacket::parse(DataPacket *pd) {
    airPressure = 0;
    for (int i = 3; i >= 0; i--) {
        airPressure = (airPressure << 8) + pd->body[i];
    }
    altitude = 0;
    for (int i = 7; i >= 4; i--) {
        altitude = (altitude << 8) + pd->body[i];
    }
}

void LocationPacket::parse(DataPacket *pd) {
    longitude = 0;
    for (int i = 3; i >= 0; i--) {
        longitude = (longitude << 8) + pd->body[i];
    }
    latitude = 0;
    for (int i = 7; i >= 4; i--) {
        latitude = (latitude << 8) + pd->body[i];
    }
}

void GPSPacket::parse(DataPacket *pd) {
    int alt = pd->body[1] << 8;
    alt += pd->body[0];
    altitude = alt / 10.0;
    int hd = pd->body[3] << 8;
    hd += pd->body[2];
    heading = hd / 100.0;
    int gs = 0;
    for (int i = 7; i >= 4; i--) {
        gs = (gs << 8) + pd->body[i];
    }
    groundSpeed = gs;
}

void GPSAccuracyPacket::parse(DataPacket *pd) {
    numSats = pd->body[1] << 8;
    numSats += pd->body[0];

    int pa = pd->body[3] << 8;
    pa += pd->body[2];
    positionAccuracy = pa / 100.0;

    pa = pd->body[5] << 8;
    pa += pd->body[4];
    hPrecision = pa / 100.0;

    pa = pd->body[7] << 8;
    pa += pd->body[6];
    vPrecision = pa / 100.0;
}

void QuaternionPacket::parse(DataPacket *pd) {
    int q = pd->body[1] << 8;
    q += pd->body[0];
    q1 = q / 32768.0;

    q = pd->body[3] << 8;
    q += pd->body[2];
    q2 = q / 32768.0;

    q = pd->body[5] << 8;
    q += pd->body[4];
    q3 = q / 32768.0;

    q = pd->body[7] << 8;
    q += pd->body[6];
    q4 = q / 32768.0;
}

void build_ReceivePacket (DPCode code, ReceivePacket ** msg) {
    switch (code) {
        case DPCode::Time:
            *msg = new TimePacket(); break;
        case DPCode::Acceleration:
            *msg = new AccelerationPacket(); break;
        case DPCode::AngularVelocity:
            *msg = new AngularVelocityPacket(); break;
        case DPCode::Angle:
            *msg = new AnglePacket(); break;
        case DPCode::Magnetic:
            *msg = new MagneticPacket(); break;
        case DPCode::BarometricAltitude:
            *msg = new BarometricAltitudePacket(); break;
        case DPCode::Location:
            *msg = new LocationPacket(); break;
        case DPCode::GPS:
            *msg = new GPSPacket(); break;
        case DPCode::Quaternion:
            *msg = new QuaternionPacket(); break;
            break;
        case DPCode::GpsAccuracy:
            *msg = new GPSAccuracyPacket(); break;
        default:
            std::cerr << "Invalid code >> " << std::hex << static_cast<int16_t>(code) << std::endl;
            break;
    }
}


