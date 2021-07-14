#pragma once
#include <Arduino.h>

class SensorInterface {
public:

    float accX, accY, accZ;
    float gyroX, gyroY, gyroZ;
    float magX, magY, magZ;
    float gpsLat, gpsLng, gpsAltitude;
    float preassure;

    bool accChanged = false;
    bool gyroChanged = false;
    bool magChanged = false;
    bool gpsChanged = false;
    bool preassureChanged = false;

    virtual void begin() = 0;
    virtual void handle() = 0;
};