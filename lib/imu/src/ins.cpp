#include <Arduino.h>
#include "ins.h"

// INS::INS() {
//     accMul = Vec3(1, 1, 1);
//     magMul = Vec3(1, 1, 1);
//     gyroMul = Vec3(1, 1, 1);
// }

Vec3 const INS::getAccOffset() const {
    return accOffset;
}

Vec3 const INS::getGyroOffset() const {
    return gyroOffset;
}

void INS::handle() {
    /**
     * Accelerometer
     */
    if(sensors->acc.lastChange != lastAccTime) {
        updateAcc(sensors->acc.x, sensors->acc.y, sensors->acc.z);
        lastAccTime = sensors->acc.lastChange;
    }
    /**
     * Gyroscope
     */
    if(sensors->gyro.lastChange != lastGyroTime) {
        updateGyro(sensors->gyro.x, sensors->gyro.y, sensors->gyro.z);
        lastGyroTime = sensors->gyro.lastChange;
    }
    /**
     * Magnetometer
     */
    if(sensors->mag.lastChange != lastMagTime) {
        updateMag(sensors->mag.x, sensors->mag.y, sensors->mag.z);
        lastMagTime = sensors->mag.lastChange;
    }
}

void INS::updateAcc(double x, double y, double z) {
    lastRawAccs[lastRawAccCount] = Vec3(x, y, z);
    if(lastRawAccCount + 1 == saveCounts) {
        accBufferFull = true;
    }
    lastRawAcc = lastRawAccs[lastRawAccCount];
    lastRawAccCount = (lastRawAccCount + 1) % saveCounts;

    lastFilteredAcc = filterAcc(lastRawAcc);

    processFilteredAcc(lastFilteredAcc);

    if(accBufferFull && accCalibrationInQue) calibrateAcc();
}

void INS::updateMag(double x, double y, double z) {
    
}

void INS::updateGyro(double x, double y, double z) {
    lastRawGyros[lastRawGyroCount] = Vec3(x, y, z);
    if(lastRawGyroCount + 1 == saveCounts) {
        gyroBufferFull = true;
    }
    lastRawGyro = lastRawGyros[lastRawGyroCount];
    lastRawGyroCount = (lastRawGyroCount + 1) % saveCounts;

    lastFilteredGyro = filterGyro(lastRawGyro);

    // Serial.println(lastFilteredGyro.toString());

    processFilteredGyro(lastFilteredGyro);

    // if(gyroBufferFull && gyroCalibrationInQue) calibrateGyro();
}

void INS::processFilteredAcc(const Vec3 &acc) {
    // const unsigned long now = millis();
    // const unsigned long diff = now - lastAccTime;
    // lastAccTime = now;
    //rotation
    // return;
    if(acc.getLength() < 1.2 && acc.getLength() > 0.8) { //check if movement is too strong
        double roll = atan2(acc.y, acc.z);
        double pitch = atan2(-acc.x, sqrt(acc.y*acc.y + acc.z*acc.z));
        Quaternion accRot(EulerRotation(roll, pitch, -rot.toEulerZYX().z));
        rot.normalize();
        rot.calibrate();
        accRot.calibrate();
        double len = acc.getLength();
        double limitRad = PI / 5;
        double limitG = 0.2;
        // pitch = rot.toEulerZYX().y;
        if(len > 1 - limitG && len < 1 + limitG && pitch > -limitRad && pitch < limitRad) {
            rot = Quaternion::lerp(accRot, rot, 1 - accInfluence); // 5% confidence
            // rot = Quaternion::lerp(accRot, rot, 0.95); // 5% confidence
        } else {
            // Serial.print(len);
            // Serial.println(" | no acc");
        }
    }
    readingVersion++;
    //velocity

    //position
}

void INS::processFilteredGyro(const Vec3 &gyro) {
    const uint64_t now = micros();
    if(lastGyroTime == 0 ) {
        lastGyroTime = now;
        return; //waiting for next measurement
    }
    const uint32_t diff = now - lastGyroTime;
    lastGyroTime = now;
    float elapsedSeconds = diff / 1000000.0f;

    static double degX = 0;
    degX += gyro.x * elapsedSeconds;

    //rotation
    // gyro.x = 90;
    // gyro.y = 0;
    // gyro.z = 0;
    EulerRotation localRotEuler((gyro * elapsedSeconds).toRad(), ZYX_EULER);
    Quaternion localRotQ(localRotEuler);
    // rot = localRotQ.
    rot.normalize();
    rot *= localRotQ;
    readingVersion++;
}


void INS::reset() {
    rot.setFromEuler(EulerRotation(0, 0, 0));
}

void INS::requestCalibration() {
    gyroCalibrationInQue = true;
    accCalibrationInQue =  true;
    magCalibrationInQue =  true;
}

void INS::calibrateAcc(bool retry) {
    Vec3 sum(0, 0, 0);
    int i = lastRawAccCount;
    do {
        i = (i + 1) % saveCounts;
        sum += lastRawAccs[i] * accMul;
    } while(i != lastRawAccCount);
    Vec3 avg = sum / (double) saveCounts;
    //testing succsess
    const double maxOff = 10;
    Vec3 offset = Vec3(0, 0, 1) - avg; //offset from target
    const double diff = offset.absSum();
    if(diff < maxOff && diff > -maxOff) {
        accOffset = offset;
        accCalibrationInQue = false;//succeeded
        Serial.print("Succsessfully calibrated acc :) Offset is: ");
        Serial.println(accOffset.toString());
    } else if(retry){
        accCalibrationInQue = true;
        //not succeeded requesting new attempt
        Serial.println(":( calibrating acc failed. values where outside the range. retrying...");
    }
}

void INS::calibrateGyro(bool retry) {
    Vec3 sum(0, 0, 0);
    int i = lastRawGyroCount;
    do {
        i = (i + 1) % saveCounts;
        sum += lastRawGyros[i];
    } while(i != lastRawGyroCount);
    Vec3 avg = sum / (double) saveCounts;
    //testing succsess
    const double maxOff = 15;
    Vec3 offset = Vec3(0, 0, 0) - avg; //offset from target
    const double diff = offset.absSum();
    if(diff < maxOff && diff > -maxOff) {
        gyroOffset = offset;
        gyroCalibrationInQue = false;//succeeded
        Serial.print("Succsessfully calibrated gyro :) Offset is: ");
        Serial.println(gyroOffset.toString());
    } else if(retry) {
        Serial.println(":( calibrating gyro failed retrying...");
        Serial.println(diff);
        gyroCalibrationInQue = true;
        //not succeeded requesting new attempt
    }
}

Vec3 INS::filterAcc(const Vec3 &acc) {
    Vec3 filtered = acc * accMul;
    filtered += accOffset;
    filtered = lowPassFilter(prevFilteredAcc, filtered, 0.5);
    prevFilteredAcc = filtered.clone();
    return filtered;
}

Vec3 INS::filterGyro(const Vec3 &gyro) {
    Vec3 filtered = gyro * gyroMul;
    filtered += gyroOffset;
    return filtered;
}

EulerRotation INS::getEulerRotationZYX() const {
    return rot.toEulerZYX();
}

Quaternion INS::getQuaternionRotation() const {
    return rot;
}

double INS::getPitch() {
    return rot.toEulerZYX().getPitch();
}

double INS::getRoll() {
    return rot.toEulerZYX().getRoll();
}

double INS::getYaw() {
    return rot.toEulerZYX().getYaw();
}

double INS::getPitchRate() {
    return lastFilteredGyro.y;
}

double INS::getRollRate() {
    return lastFilteredGyro.x;
}

double INS::getYawRate() {
    return lastFilteredGyro.z;
}

long INS::getReadingVersion() {
    return readingVersion;
}