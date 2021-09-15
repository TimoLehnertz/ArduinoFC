#include <Arduino.h>
#include "ins.h"
#include "../storage/Storage.h"

Vec3 const INS::getAccOffset() const {
    return accOffset;
}

Vec3 const INS::getGyroOffset() const {
    return gyroOffset;
}

void INS::begin() {
    accMul = Storage::read(Vec3Values::accMul);
    gyroMul = Storage::read(Vec3Values::gyroMul);
    
    accOffset = Storage::read(Vec3Values::accOffset);
    gyroOffset = Storage::read(Vec3Values::gyroOffset);
    
    accLowpassFilter = Storage::read(FloatValues::accLPF);
    gyroLowpassFilter = Storage::read(FloatValues::gyroLPF);
}

void INS::handle() {
    const u_int32_t now = micros();
    /**
     * Accelerometer
     */
    if(sensors->acc.lastChange != lastAccTime) {
        const u_int32_t diff = now - lastAccTime;
        lastAccTime = now;
        updateAcc(sensors->acc.x, sensors->acc.y, sensors->acc.z, diff);
        lastAccTime = sensors->acc.lastChange;
    }
    /**
     * Gyroscope
     */
    if(sensors->gyro.lastChange != lastGyroTime) {
        const u_int32_t diff = now - lastGyroTime;
        lastGyroTime = now;
        updateGyro(sensors->gyro.x, sensors->gyro.y, sensors->gyro.z, diff);
        lastGyroTime = sensors->gyro.lastChange;
    }
    /**
     * Magnetometer
     */
    if(sensors->mag.lastChange != lastMagTime) {
        const u_int32_t diff = now - lastMagTime;
        lastMagTime = now;
        updateMag(sensors->mag.x, sensors->mag.y, sensors->mag.z, diff);
        lastMagTime = sensors->mag.lastChange;
    }
}

void INS::updateAcc(double x, double y, double z, uint32_t deltaT) {
    lastRawAccs[lastRawAccCount] = Vec3(x, y, z);
    if(lastRawAccCount + 1 == saveCounts) {
        accBufferFull = true;
    }
    lastRawAcc = lastRawAccs[lastRawAccCount];
    lastRawAccCount = (lastRawAccCount + 1) % saveCounts;

    lastFilteredAcc = filterAcc(lastRawAcc);

    processFilteredAcc(lastFilteredAcc, deltaT);

    if(accBufferFull && accCalibrationInQue) calibrateAcc();
}

void INS::updateMag(double x, double y, double z, uint32_t deltaT) {
    lastRawMag = Vec3(x, y, z);
    lastFilteredMag = filterMag(lastRawMag);
    processFilteredMag(lastFilteredMag);
}

Vec3 INS::filterMag(const Vec3 &mag) {
    return magSoftIron * (mag - magHardIron);
}

void INS::updateGyro(double x, double y, double z, uint32_t deltaT) {
    lastRawGyros[lastRawGyroCount] = Vec3(x, y, z);
    if(lastRawGyroCount + 1 == saveCounts) {
        gyroBufferFull = true;
    }
    lastRawGyro = lastRawGyros[lastRawGyroCount];
    lastRawGyroCount = (lastRawGyroCount + 1) % saveCounts;

    lastFilteredGyro = filterGyro(lastRawGyro);

    processFilteredGyro(lastFilteredGyro, deltaT);
}

void INS::processFilteredAcc(const Vec3 &acc, uint32_t deltaT) {
    float elapsedSeconds = deltaT / 1000000.0f;
    //rotation
    if(acc.getLength() < 1.2 && acc.getLength() > 0.8) { //check if movement is too strong
        double roll = -atan2(acc.y, acc.z); //minus because not beeing aligned with sticks otherwise
        double pitch = atan2(-acc.x, sqrt(acc.y*acc.y + acc.z*acc.z));
        Quaternion accRot(EulerRotation(roll, pitch, -rot.toEulerZYX().z));
        rot.normalize();
        rot.calibrate();
        accRot.calibrate();
        double len = acc.getLength();
        double limitRad = PI / 3.5;
        double limitG = 0.2;
        pitch = rot.toEulerZYX().y;
        if(len > 1 - limitG && len < 1 + limitG && pitch > -limitRad && pitch < limitRad) {
            rot = Quaternion::lerp(accRot, rot, 1 - accInfluence);
        }
    }
    readingVersion++;
    //acceleration
    // Vec3 accel = Vec3(0,0,1.005);
    Vec3 accel = acc * 9.807;
    // accel.y *= -1;
    accel.x *= -1;
    //rotate acceleration
    float len = accel.getLength();
    if(len > 0) {
        accel /= len;
        rot.rotateReverse(accel);
        accel *= len;
        // if(millis() % 10 == 0) {
        //     Serial.println(accel.toString());
        // }
        //adding gravity
        accel -= Vec3(0, 0, 9.807);
        vel += accel * elapsedSeconds;
        //position
        loc += vel * elapsedSeconds;
    }
}

void INS::processFilteredGyro(const Vec3 &gyro, uint32_t deltaT) {
    if(lastGyroTime == 0 ) {
        lastGyroTime = micros();
        return; //waiting for next measurement
    }
    Vec3 gyroConf = gyro.clone();
    gyroConf.x *= -1; //align with sticks
    float elapsedSeconds = deltaT / 1000000.0f;

    //rotation
    EulerRotation localRotEuler((gyroConf * elapsedSeconds).toRad(), ZYX_EULER);
    Quaternion localRotQ(localRotEuler);
    rot.normalize();
    rot *= localRotQ;
    readingVersion++;
}

void INS::processFilteredMag(const Vec3 &mag) {
    float pitch = rot.toEulerZYX().y;
    float roll = rot.toEulerZYX().x;
    if(pitch < PI / 4 && pitch > -PI / 4 && roll < PI / 4 && roll > -PI / 4) {
        Quaternion magRot(EulerRotation(roll, pitch, atan2(mag.y, mag.x)));
        rot = Quaternion::lerp(magRot, rot, 1 - magInfluence);
    }
}


void INS::reset() {
    rot.setFromEuler(EulerRotation(0, 0, atan2(lastFilteredMag.y, lastFilteredMag.x)));
    vel = Vec3();
    loc = Vec3();
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
        sum += lastRawAccs[i];// * accMul;
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
    
    Vec3 offset = Vec3(0, 0, 0) - avg; //offset from target
    gyroOffset = offset;
    Serial.print("Succsessfully calibrated gyro :) Offset is: ");
    Serial.println(gyroOffset.toString());
}

Vec3 INS::filterAcc(const Vec3 &acc) {
    Vec3 filtered = (acc + accOffset) * accMul;
    filtered = lowPassFilter(prevFilteredAcc, filtered, accLowpassFilter);
    prevFilteredAcc = filtered.clone();
    return filtered;
}

Vec3 INS::filterGyro(const Vec3 &gyro) {
    Vec3 filtered = (gyro + gyroOffset) * gyroMul;
    filtered = lowPassFilter(prevFilteredGyro, filtered, gyroLowpassFilter);
    prevFilteredGyro = filtered.clone();
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