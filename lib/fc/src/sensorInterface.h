#pragma once
#include <Arduino.h>

// essential
struct Gyroscope {
    float x, y, z; //degrees
    uint64_t lastChange;
};

// essential
struct Accelerometer {
    float x, y, z; // G's
    uint64_t lastChange;
};

// optional
struct Magnetometer {
    bool connected = false;
    float x, y, z;
    uint64_t lastChange;
};

// optional
struct Barometer {
    bool connected = false;
    float altitude;     //Meters
    float preassure;    //atmospheres
    float temperature;  //celcius
    uint64_t lastChange;
};

// optional
struct GPSSensor {
    bool connected = false;

    bool locationValid = false;
    float lat, lng; //degrees

    //date
    bool dateValid = false;
    int year;
    int month;
    int day;

    //time
    bool timeValid = false;
    int hour;
    int minute;
    int second;
    int centisecond;

    //speed
    float speed; //km/h
    bool speedValid = false;

    //course
    float course; //degrees
    bool courseValid = false;

    //altitude
    float altitude; //Meters
    bool altitudeValid = false;

    //satelites
    int satelites;

    //precision
    float hdop; // hdop < 2 = good, hdop < 8 = ok

    uint64_t lastChange;
};

/**
 * Abstract class for interfacing with all sensors.
 * stores only raw sensor data
 */
class SensorInterface {
public:
    Accelerometer acc;
    Gyroscope gyro;
    Magnetometer mag;

    Barometer baro;

    GPSSensor gps;

    virtual void begin() = 0;
    virtual void handle() = 0;
};