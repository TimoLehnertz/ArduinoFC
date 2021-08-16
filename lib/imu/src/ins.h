#pragma once

#include <maths.h>
#include <sensorInterface.h>

class INS {
public:

    INS(SensorInterface* sensors) : sensors(sensors) {}

    void updateAcc(double, double, double);
    void updateGyro(double, double, double);
    void updateMag(double, double, double);

    void handle();


//  Calibration
    void requestCalibration();

    Vec3 const getAccOffset() const;
    Vec3 const getGyroOffset() const;

    void calibrateAcc(bool = true);
    void calibrateGyro(bool = true);
    void calibrateMag(bool = true);

    void reset();

//  Getters
    double getRoll();
    double getRollRate();
    double getPitch();
    double getPitchRate();
    double getYaw();
    double getYawRate();
    long getReadingVersion();
    Vec3 getPosition() const;
    Vec3 getVelocity() const;
    EulerRotation getEulerRotationZYX() const;
    Quaternion getQuaternionRotation() const;

    Vec3 getAccMul() {return accMul;}
    Vec3 getGyroMul() {return gyroMul;}
    Vec3 getMagMul() {return magMul;}

    Vec3 getAccOffset() {return accOffset;}
    Vec3 getGyroOffset() {return gyroOffset;}
    Vec3 getMagOffset() {return magOffset;}

    float getAccLowpassFilter() {return accLowpassFilter;}
    float getGyroLowpassFilter() {return gyroLowpassFilter;}

    void setAccLowpassFilter(float accLowpassFilter)  { this->accLowpassFilter = accLowpassFilter; }
    void setGyroLowpassFilter(float gyroLowpassFilter) { this->gyroLowpassFilter = gyroLowpassFilter; }

    void setAccMul(Vec3 mul) {accMul = mul;}
    void setGyroMul(Vec3 mul) {gyroMul = mul;}
    void setMagMul(Vec3 mul) {magMul = mul;}

    void setAccOffset(Vec3 offset) {accOffset = offset;}
    void setGyroOffset(Vec3 offset) {gyroOffset = offset;}
    void setMagOffset(Vec3 offset) {magOffset = offset;}

    Vec3 getLastFilteredAcc() { return lastFilteredAcc; }
    Vec3 getLastFilteredGyro() { return lastFilteredGyro; }
    Vec3 getLastFilteredMag() { return lastFilteredMag; }

    float getAccInfluence() { return accInfluence; }
    void setAccInfluence(float accInfluence) { this->accInfluence = accInfluence; }

private:
    SensorInterface* sensors;
    long readingVersion = 0; //counter that gets incremented everytime a an update occours
//  Processing
    void processFilteredAcc(const Vec3&);
    void processFilteredGyro(const Vec3&);
    void processFilteredMag(const Vec3&);

//  Calibration
    Vec3 accMul  {Vec3(1,1,1)};
    Vec3 gyroMul {Vec3(1,1,1)};
    Vec3 magMul  {Vec3(1,1,1)};

    Vec3 accOffset  {Vec3(0,0,0)};
    Vec3 gyroOffset {Vec3(0,0,0)};
    Vec3 magOffset  {Vec3(0,0,0)};

    float accLowpassFilter = 0.5;
    float gyroLowpassFilter = 1;

    bool accCalibrationInQue  = false;
    bool gyroCalibrationInQue = false;
    bool magCalibrationInQue  = false;

    bool accBufferFull = false;
    bool gyroBufferFull = false;
    bool magBufferFull = false;

    float accInfluence = 0.01;

//  saved raw measurements
    unsigned long lastAccTime = 0;
    unsigned long lastGyroTime = 0;
    unsigned long lastMagTime = 0;

    static const long saveCounts = 30; //also minimum amount of measurements needed for calibration of each sensor
    Vec3 lastRawAccs[saveCounts] {Vec3(0, 0, 1)};
    Vec3 lastRawGyros[saveCounts] {Vec3(0, 0, 0)};
    Vec3 lastRawMags[saveCounts] {Vec3(0, 0, 0)};
    int lastRawAccCount = 0;
    int lastRawGyroCount = 0;
    int lastRawMagCount = 0;

//  last filtered measurements
    Vec3 lastFilteredAcc;
    Vec3 lastFilteredGyro;
    Vec3 lastFilteredMag;

    Vec3 lastRawAcc;
    Vec3 lastRawGyro;
    Vec3 lastRawMag;

//  States
    Vec3 loc {Vec3(0, 0, 0)};
    Vec3 vel {Vec3(0, 0, 0)};
    Quaternion rot {Quaternion()};

//  Filtering
    Vec3 prevFilteredAcc = Vec3(0, 0, 0);
    Vec3 filterAcc(const Vec3&);
    Vec3 filterMag(const Vec3&);
    Vec3 filterGyro(const Vec3&);

    static Vec3 lowPassFilter(const Vec3 &prevFiltered, const Vec3 &now, double smoothing) {
		return now * smoothing + prevFiltered * (1 - smoothing);
	}
};