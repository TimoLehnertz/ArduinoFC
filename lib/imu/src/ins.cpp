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
            rot = Quaternion::lerp(accRot, rot, 0.95); // 5% confidence
        } else {
            Serial.print(len);
            Serial.println(" | no acc");
        }
    }
    //velocity

    //position
}

void INS::processFilteredGyro(const Vec3 &gyro) {
    const unsigned long now = millis();
    if(lastGyroTime == 0 ) {
        lastGyroTime = now;
        return; //waiting for next measurement
    }
    const unsigned long diff = now - lastGyroTime;
    lastGyroTime = now;
    //rotation
    double elapsedSeconds = diff / 1000.0;
    // zSum += gyro.z * elapsedSeconds;
    // xSum += gyro.x * elapsedSeconds;
    // Serial.println(int(xSum));
    EulerRotation localRotEuler((gyro * elapsedSeconds).toRad(), ZYX_EULER);
    Quaternion localRotQ(localRotEuler);
    rot.normalize();
    rot *= localRotQ;
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
        sum += lastRawAccs[i];
    } while(i != lastRawAccCount);
    Vec3 avg = sum / saveCounts;
    //testing succsess
    const double maxOff = 0.3;
    Vec3 offset = Vec3(0, 0, 1) - avg; //offset from target
    const double diff = offset.absSum();
    if(diff < maxOff && diff > -maxOff) {
        accOffset = offset;
        accCalibrationInQue = false;//succeeded
        #ifdef INS_DEBUG
        Serial.println(":) succsessfully calibrated acc. Offset is:");
        Serial.println(accOffset.toString());
        #endif
    } else if(retry){
        accCalibrationInQue = true;
        //not succeeded requesting new attempt
         #ifdef INS_DEBUG
        // Serial.println(":( calibrating acc failed. retrying...");
        #endif
    }
}

void INS::calibrateGyro(bool retry) {
    Vec3 sum(0, 0, 0);
    int i = lastRawGyroCount;
    do {
        i = (i + 1) % saveCounts;
        sum += lastRawGyros[i];
    } while(i != lastRawGyroCount);
    Vec3 avg = sum / saveCounts;
    //testing succsess
    const double maxOff = 5;
    Vec3 offset = Vec3(0, 0, 0) - avg; //offset from target
    const double diff = offset.absSum();
    if(diff < maxOff && diff > -maxOff) {
        gyroOffset = offset;
        gyroCalibrationInQue = false;//succeeded
        #ifdef INS_DEBUG
        Serial.println(":) succsessfully calibrated gyro. Offset is:");
        Serial.println(gyroOffset.toString());
        #endif
    } else if(retry) {
        #ifdef INS_DEBUG
        Serial.println(":( calibrating gyro failed retrying...");
        Serial.println(diff);
        #endif
        gyroCalibrationInQue = true;
        //not succeeded requesting new attempt
    }
}

Vec3 INS::filterAcc(const Vec3 &acc) {
    Vec3 filtered = acc + accOffset;
    filtered *= accMul;
    filtered = lowPassFilter(prevFilteredAcc, filtered, 0.5);
    prevFilteredAcc = filtered.clone();
    return filtered;
}

Vec3 INS::filterGyro(const Vec3 &gyro) {
    Vec3 filtered = gyro + gyroOffset;
    filtered *= gyroMul;
    return filtered;
}

EulerRotation INS::getEulerRotationZYX() const {
    return rot.toEulerZYX();
}

Quaternion INS::getQuaternionRotation() const {
    return rot;
}