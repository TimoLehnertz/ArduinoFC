#pragma once
#include <error.h>
#include <flightModes.h>
#include <maths.h>

/**
 * General data type for all sensors on board
 *      last change: last time the readings got updated in micro seconds
 *      error: 0 => OK, 1 => Warning(typically missing calibration but should be possible to fly), 2 => Critical error
 */
struct Sensor {
    uint64_t lastChange;
    Error::Error_t error;
    FlightMode::FlightMode_t minFlightMode;

    Sensor(FlightMode::FlightMode_t minFlightMode) : minFlightMode(minFlightMode) {}

    virtual void checkError() = 0;
};

struct Vec3Sensor : public Sensor {
    float x, y, z, lastX, lastY, lastZ;
    int similarCount;

    Vec3Sensor(FlightMode::FlightMode_t minFlightMode) : Sensor(minFlightMode), similarCount(0) {}

    void update(float x1, float y1, float z1) {
        if(x1 != x || y1 != y || z1 != z) {
            lastChange = micros();
            x = x1;
            y = y1;
            z = z1;
        }
    }

    /**
     * check if sensor readings change and if not for 100 readings raise CRITICAL_ERROR
     */
    void checkError() {
        if(x == lastX && y == lastY && z == lastZ) {
            similarCount++;
            if(similarCount >= 100) {
                similarCount = 100;
                error = Error::CRITICAL_ERROR;
            }
        } else {
            similarCount = 0;
            lastX = x;
            lastY = y;
            lastZ = z;
            error = Error::NO_ERROR;
        }
    }
};

struct Accelerometer : public Vec3Sensor {
    Accelerometer() : Vec3Sensor(FlightMode::level) {}
};

struct Gyroscope : public Vec3Sensor {
    Gyroscope() : Vec3Sensor(FlightMode::rate) {}
};

struct Magnetometer : public Vec3Sensor {
    Magnetometer() : Vec3Sensor(FlightMode::gpsHold) {}
};


struct Barometer : public Sensor{
    float altitude, lastAltitude;     //Meters
    float preassure;    //atmospheres
    float temperature;  //celcius

    int similarCount;

    Barometer() : Sensor(FlightMode::altitudeHold), similarCount(0) {}

    void checkError() {
        if(altitude == lastAltitude) {
            similarCount++;
            if(similarCount >= 100) {
                similarCount = 100;
                error = Error::CRITICAL_ERROR;
            }
        } else {
            similarCount = 0;
            lastAltitude = altitude;
            error = Error::NO_ERROR;
        }
    }
};

struct GPS : public Sensor{
    bool locationValid = false;
    float lat, lng, lastLat, lastLng; //degrees
    int similarCount;

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

    GPS() : Sensor(FlightMode::gpsHold) {}

    void checkError() {
        if(micros() - lastChange > 2000000) { // 2 sek timeout
            error = Error::CRITICAL_ERROR;
        } else {
            error = Error::NO_ERROR;
        }
    }
};

struct Battery : public Sensor {
    float vBat; // Volts
    float vCell; // Volts
    byte cellCount;

    Battery() : Sensor(FlightMode::dreaming) {}

    void checkError() {} // do nothing
};

/**
 * Abstract class for interfacing with all sensors.
 * stores only raw sensor data
 */
class SensorInterface {
public:

    bool useAcc = true;
    bool useMag = true;

    static constexpr byte sensorCount = 6;

    Sensor* sensors[sensorCount];

    Accelerometer acc;
    Gyroscope gyro;
    Magnetometer mag;

    Barometer baro;

    GPS gps;

    Battery bat;

    float batLpf = 0.001;
    float vBatMul = 11.8;

    SensorInterface() {
        sensors[0] = &acc;
        sensors[1] = &gyro;
        sensors[2] = &mag;
        sensors[3] = &baro;
        sensors[4] = &gps;
        sensors[5] = &bat;
    }

    virtual void begin() = 0;
    virtual void handle() = 0;

    virtual void setAccCal (Vec3 gVecOffset, Vec3 scale)  = 0;
    virtual void setGyroCal(Vec3 degVecOffset) = 0;
    virtual void setMagCal (Vec3 offset, Vec3 scale)  = 0;

    virtual void calibrateAcc() = 0;
    virtual void calibrateGyro() = 0;
    virtual void calibrateMag() = 0;

    virtual Vec3 getAccOffset() = 0;
    virtual Vec3 getAccScale() = 0;
    virtual Vec3 getGyroOffset() = 0;
    virtual Vec3 getMagOffset() = 0;
    virtual Vec3 getMagScale() = 0;

    virtual void calibrateBat(float actualVoltage) = 0;

    /**
     * Find the highest Flight mode that is possible to achieve with the sensors with an equal or lower error that maxError
     */
    FlightMode::FlightMode_t getHighestFM(Error::Error_t maxError = Error::WARNING) {
        FlightMode::FlightMode_t fm = FlightMode::gpsHold;
        for (size_t i = 0; i < sensorCount; i++) {
            if(sensors[i]->error > maxError) {
                fm = min(fm, sensors[i]->minFlightMode);
            }
        }
        return fm;
    }
};