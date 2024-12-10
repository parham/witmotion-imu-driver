//
// Created by phm on 14/10/24.
//

#include <iostream>

#include "Message.h"

#define GRAVITY 9.8

void TimePacket::parse(DataPacket *pd) {
    this->year = pd->body[0];
    this->month = pd->body[1];
    this->day = pd->body[2];
    this->hour = pd->body[3];
    this->minute = pd->body[4];
    this->second = pd->body[5];
    auto msl = pd->body[6];
    auto msh = pd->body[7];
    this->milisecond = ((uint16_t) msh << 8) | msl;
}

void AccelerationPacket::parse(DataPacket *pd) {
    // X=((AxH<<8)|AxL)/32768*16g
    // AC X
    int ac = pd->body[1] << 8;
    ac += pd->body[0];
    this->ac_x = (ac / 32768.0f) * 16.0f * GRAVITY;
    // AC Y
    ac = pd->body[3] << 8;
    ac += pd->body[2];
    this->ac_y = (ac / 32768.0f) * 16.0f * GRAVITY;
    // AC Z
    ac = pd->body[5] << 8;
    ac += pd->body[4];
    this->ac_z = (ac / 32768.0f) * 16.0f * GRAVITY;
    // TEMP
    ac = pd->body[7] << 8;
    ac += pd->body[6];
    this->temperature = ac / 100.0f;
}

void AngularVelocityPacket::parse(DataPacket *pd) {
    // X=((WxH<<8)|WxL)/32768*2000°/s
    // Ag X
    int ac = pd->body[1] << 8;
    ac += pd->body[0];
    this->av_x = (ac / 32768.0f) * 2000.0f;
    // Ag Y
    ac = pd->body[3] << 8;
    ac += pd->body[2];
    this->av_y = (ac / 32768.0f) * 2000.0f;
    // Ag Z
    ac = pd->body[5] << 8;
    ac += pd->body[4];
    this->av_z = (ac / 32768.0f) * 2000.0f;
    // TEMP
    ac = pd->body[7] << 8;
    ac += pd->body[6];
    this->temperature = ac / 100.0f;
}

void AnglePacket::parse(DataPacket *pd) {
    // Ag X
    int ac = pd->body[1] << 8;
    ac += pd->body[0];
    this->roll = (ac / 32768.0f) * 180.0f;
    // Ag Y
    ac = pd->body[3] << 8;
    ac += pd->body[2];
    this->pitch = (ac / 32768.0f) * 180.0f;
    // Ag Z
    ac = pd->body[5] << 8;
    ac += pd->body[4];
    this->yaw = (ac / 32768.0f) * 180.0f;
    // VERSION
    ac = pd->body[7] << 8;
    ac += pd->body[6];
    this->version = ac;
}

void MagneticPacket::parse(DataPacket *pd) {
    this->mag_x = pd->body[1] << 8;
    this->mag_x += pd->body[0];
    this->mag_y = pd->body[3] << 8;
    this->mag_y += pd->body[2];
    this->mag_z = pd->body[5] << 8;
    this->mag_z += pd->body[4];
}

void BarometricAltitudePacket::parse(DataPacket *pd) {
    this->airPressure = 0;
    for (int i = 3; i >= 0; i--) {
        this->airPressure = (this->airPressure << 8) + pd->body[i];
    }
    this->altitude = 0;
    for (int i = 7; i >= 4; i--) {
        this->altitude = (this->altitude << 8) + pd->body[i];
    }
}

void LocationPacket::parse(DataPacket *pd) {
    this->longitude = 0;
    for (int i = 3; i >= 0; i--) {
        this->longitude = (this->longitude << 8) + pd->body[i];
    }
    this->latitude = 0;
    for (int i = 7; i >= 4; i--) {
        this->latitude = (this->latitude << 8) + pd->body[i];
    }
}

void GPSPacket::parse(DataPacket *pd) {
    int alt = pd->body[1] << 8;
    alt += pd->body[0];
    this->altitude = alt / 10.0;
    int hd = pd->body[3] << 8;
    hd += pd->body[2];
    this->heading = hd / 100.0;
    int gs = 0;
    for (int i = 7; i >= 4; i--) {
        gs = (gs << 8) + pd->body[i];
    }
    this->groundSpeed = gs;
}

void GPSAccuracyPacket::parse(DataPacket *pd) {
    this->numSats = pd->body[1] << 8;
    this->numSats += pd->body[0];

    int pa = pd->body[3] << 8;
    pa += pd->body[2];
    this->positionAccuracy = pa / 100.0;

    pa = pd->body[5] << 8;
    pa += pd->body[4];
    this->hPrecision = pa / 100.0;

    pa = pd->body[7] << 8;
    pa += pd->body[6];
    this->vPrecision = pa / 100.0;
}

void QuaternionPacket::parse(DataPacket *pd) {
    int q = pd->body[1] << 8;
    q += pd->body[0];
    this->q1 = q / 32768.0;

    q = pd->body[3] << 8;
    q += pd->body[2];
    this->q2 = q / 32768.0;

    q = pd->body[5] << 8;
    q += pd->body[4];
    this->q3 = q / 32768.0;

    q = pd->body[7] << 8;
    q += pd->body[6];
    this->q4 = q / 32768.0;
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
        case DPCode::GpsAccuracy:
            *msg = new GPSAccuracyPacket(); break;
        default:
            std::cerr << "Invalid code >> " << std::hex << static_cast<int16_t>(code) << std::endl;
            break;
    }
}


