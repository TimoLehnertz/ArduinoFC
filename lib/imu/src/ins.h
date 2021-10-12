#pragma once

#include <maths.h>
#include <sensorInterface.h>

#define INS_MAX_G_ERROR 0.2
#define G 9.807

class INS {
public:

    SensorInterface* sensors;

    INS(SensorInterface* sensors) : sensors(sensors) {}

    void updateAcc(double, double, double, uint32_t);
    void updateGyro(double, double, double, uint32_t);
    void updateMag(double, double, double, uint32_t);
    void updateBaroAltitude(double, uint32_t);

    void begin();
    void handle();

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
    float getGForce();
    Vec3 getLocation() const {return loc;}
    Vec3 getVelocity() const {return vel;}
    Vec3 getAcceleration() const {return accel;}
    EulerRotation getEulerRotationZYX() const;
    Quaternion getQuaternionRotation() const;
    double getBaroAltSpd() const {return baroAltSpeed;}

    Matrix3 getAccMul() {return accMul;}
    Vec3 getGyroMul() {return gyroMul;}
    Matrix3 getMagSoftIron() {return magSoftIron;}

    Vec3 getAccOffset() {return accOffset;}
    Vec3 getGyroOffset() {return gyroOffset;}
    Vec3 getMagHardIron() {return magHardIron;}

    float getAccLowpassFilter() {return accLowpassFilter;}
    float getGyroLowpassFilter() {return gyroLowpassFilter;}

    void setAccLowpassFilter(float accLowpassFilter)  { this->accLowpassFilter = accLowpassFilter; }
    void setGyroLowpassFilter(float gyroLowpassFilter) { this->gyroLowpassFilter = gyroLowpassFilter; }

    void setAccMul(Matrix3 mul) {accMul = mul;}
    void setGyroMul(Vec3 mul) {gyroMul = mul;}
    void setMagSoftIron(Matrix3 mul) {magSoftIron = mul;}

    void setAccOffset(Vec3 offset) {accOffset = offset;}
    void setGyroOffset(Vec3 offset) {gyroOffset = offset;}
    void setMagHardIron(Vec3 offset) {magHardIron = offset;}

    Vec3 getLastFilteredAcc() { return lastFilteredAcc; }
    Vec3 getLastFilteredGyro() { return lastFilteredGyro; }
    Vec3 getLastFilteredMag() { return lastFilteredMag; }
    double getLastFilteredBaroAltitude() { return baroAltitude; }

    float getAccInfluence() { return accInfluence; }
    float gatMagInfluence() { return magInfluence; }
    void setAccInfluence(float accInfluence) { this->accInfluence = accInfluence; }
    void setMagInfluence(float magInfluence) { this->magInfluence = magInfluence; }

    void setMaxGError(float maxGError) { this->maxGError = maxGError; }
    float getMaxGError() { return this->maxGError; }

    void setUseDroneOptimization(bool flag) {useDroneOptimization = flag;}
    bool isUseDroneOptimization() {return useDroneOptimization;}

    void setAccAngleOffset();
    void setAccAngleOffset(Quaternion quat);
    Quaternion getAccAngleOffset() {return accAngleOffset;}

private:
    bool useIMUFiltering = false;

    long readingVersion = 0; //counter that gets incremented everytime a an update occours
//  Processing
    void processFilteredAcc(const Vec3&, uint32_t);
    void processFilteredGyro(const Vec3&, uint32_t);
    void processFilteredMag(const Vec3&);
    void processFilteredBaroAltitude(const double, uint32_t);

//  Calibration
    // Vec3 accMul  {Vec3(1,1,1)};
    Matrix3 accMul {Matrix3(1,1,1,
                            1,1,1,
                            1,1,1)};
    Vec3 gyroMul {Vec3(1,1,1)};
    Matrix3 magSoftIron  {Matrix3(  1,1,1,
                                    1,1,1,
                                    1,1,1)};

    Vec3 accOffset  {Vec3()};
    Quaternion accAngleOffset  {Quaternion()};
    Vec3 gyroOffset {Vec3()};
    Vec3 magHardIron  {Vec3()};

    bool headDown = false;

    bool useDroneOptimization = true;

    float altitudeOffset = 0;

    float maxGError = INS_MAX_G_ERROR;

    float accLowpassFilter = 0.5;
    float gyroLowpassFilter = 1;

    bool accCalibrationInQue  = false;
    bool gyroCalibrationInQue = false;
    bool magCalibrationInQue  = false;

    bool accBufferFull = false;
    bool gyroBufferFull = false;
    bool magBufferFull = false;

    float accInfluence = 0.01;
    float magInfluence = 0.1;

//  saved raw measurements
    unsigned long lastAccTime = 0;
    unsigned long lastGyroTime = 0;
    unsigned long lastMagTime = 0;
    unsigned long lastBaroTime = 0;

    static const int saveCounts = 1500; //also minimum amount of measurements needed for calibration of each sensor
    Vec3 lastRawAccs[saveCounts] {Vec3(0, 0, 1)};
    Vec3 lastRawGyros[saveCounts] {Vec3(0, 0, 0)};
    int lastRawAccCount = 0;
    int lastRawGyroCount = 0;
    int lastRawMagCount = 0;


//  last filtered measurements
    Vec3 lastFilteredAcc;
    Vec3 lastFilteredGyro;
    Vec3 lastFilteredMag;
    double lastFilteredBaroAltitude;

    Vec3 lastRawAcc;
    Vec3 lastRawGyro;
    Vec3 lastRawMag;
    double lastRawBaroAltitude = 0.0;

//  States
    Vec3 loc {Vec3(0, 0, 0)};
    Vec3 vel {Vec3(0, 0, 0)};
    Vec3 accel {Vec3(0, 0, 0)};
    Quaternion rot {Quaternion()};
    double baroAltitude = 0.0;
    double baroAltSpeed = 0.0;

//  Filtering
    Vec3 prevFilteredAcc = Vec3(0, 0, 0);
    Vec3 prevFilteredGyro = Vec3(0, 0, 0);
    Vec3 filterAcc(const Vec3&);
    Vec3 filterMag(const Vec3&);
    Vec3 filterGyro(const Vec3&);
    double filterBaroAltitude(const double);

    static Vec3 lowPassFilter(const Vec3 &prevFiltered, const Vec3 &now, double smoothing) {
		return now * smoothing + prevFiltered * (1 - smoothing);
	}

    static double lowPassFilter(double prevFiltered, double now, double smoothing) {
		return now * smoothing + prevFiltered * (1 - smoothing);
	}
};