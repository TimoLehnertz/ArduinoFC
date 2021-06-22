#ifndef INS
#define INS_DEF

#define INS_DEBUG

#include <maths.h>

class INS {
public:

    void updateAcc(double, double, double);
    void updateGyro(double, double, double);
    void updateMag(double, double, double);

//  Getters
    Vec3 getPosition() const;
    Vec3 getVelocity() const;
    EulerRotation getEulerRotationZYX() const;
    Quaternion getQuaternionRotation() const;

//  Calibration
    void requestCalibration();

    Vec3 const getAccOffset() const;
    Vec3 const getGyroOffset() const;

    void calibrateAcc(bool = true);
    void calibrateGyro(bool = true);
    void calibrateMag(bool = true);

private:
//  Processing
    void processFilteredAcc(const Vec3&);
    void processFilteredGyro(const Vec3&);
    void processFilteredMag(const Vec3&);

//  Calibration
    Vec3 accMul  {Vec3(1,1,1)};
    Vec3 magMul  {Vec3(1,1,1)};
    Vec3 gyroMul {Vec3(-1,-1,-1)};
    bool accCalibrationInQue  = false;
    bool gyroCalibrationInQue = false;
    bool magCalibrationInQue  = false;

    bool accBufferFull = false;
    bool gyroBufferFull = false;
    bool magBufferFull = false;

    Vec3 accOffset {Vec3(0, 0, 0)};
    Vec3 gyroOffset {Vec3(0, 0, 0)};

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

#endif