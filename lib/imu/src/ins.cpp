#include <Arduino.h>
#include "ins.h"
#include "../storage/Storage.h"

Vec3 const INS::getAccOffset() const {
    return accOffset;
}

Vec3 const INS::getGyroOffset() const {
    return gyroOffset;
}

// void INS::begin() {
//     accMul = Storage::read(Matrix3Values::accMul);
//     gyroMul = Storage::read(Vec3Values::gyroMul);
    
//     accOffset = Storage::read(Vec3Values::accOffset);
//     gyroOffset = Storage::read(Vec3Values::gyroOffset);
    
//     accLowpassFilter = Storage::read(FloatValues::accLPF);
//     gyroLowpassFilter = Storage::read(FloatValues::gyroLPF);
// }

void INS::handle() {
    const u_int32_t now = micros();
    /**
     * Accelerometer
     */
    if(sensors->acc.lastChange != lastAccTime) {
        const u_int32_t diff = now - lastAccTime;
        updateAcc(sensors->acc.x, sensors->acc.y, sensors->acc.z, diff);
        lastAccTime = sensors->acc.lastChange;
    }
    /**
     * Gyroscope
     */
    if(sensors->gyro.lastChange != lastGyroTime) {
        const u_int32_t diff = now - lastGyroTime;
        updateGyro(sensors->gyro.x, sensors->gyro.y, sensors->gyro.z, diff);
        lastGyroTime = sensors->gyro.lastChange;
    }
    /**
     * Magnetometer
     */
    if(sensors->mag.lastChange != lastMagTime) {
        const u_int32_t diff = now - lastMagTime;
        updateMag(sensors->mag.x, sensors->mag.y, sensors->mag.z, diff);
        lastMagTime = sensors->mag.lastChange;
    }
    /**
     * Barometer
     */
    if(sensors->baro.lastChange != lastBaroTime) {
        const u_int32_t diff = now - lastBaroTime;
        updateBaroAltitude(sensors->baro.altitude, diff);
        lastBaroTime = sensors->baro.lastChange;
    }
}

void INS::updateAcc(double x, double y, double z, uint32_t deltaT) {
    lastRawAccs[lastRawAccCount] = Vec3(x, y, z);
    if(lastRawAccCount + 1 == saveCounts) {
        accBufferFull = true;
    }
    lastRawAcc = lastRawAccs[lastRawAccCount];
    lastRawAccCount = (lastRawAccCount + 1) % saveCounts;

    if(useIMUFiltering) {
        lastFilteredAcc = filterAcc(lastRawAcc);
    } else {
        lastFilteredAcc = lastRawAcc.clone();
    }

    processFilteredAcc(lastFilteredAcc, deltaT);

    if(accBufferFull && accCalibrationInQue) calibrateAcc();
}

void INS::updateMag(double x, double y, double z, uint32_t deltaT) {
    lastRawMag = Vec3(x, y, z);
    if(useIMUFiltering) {
        lastFilteredMag = filterMag(lastRawMag);
    } else {
        lastFilteredMag = lastRawMag.clone();
    }
    processFilteredMag(lastFilteredMag);
}

void INS::updateBaroAltitude(double altitude, uint32_t deltaT) {
    lastRawBaroAltitude = altitude;
    lastFilteredBaroAltitude = filterBaroAltitude(altitude);
    processFilteredBaroAltitude(lastFilteredBaroAltitude, deltaT);
}

void INS::updateGyro(double x, double y, double z, uint32_t deltaT) {
    lastRawGyros[lastRawGyroCount] = Vec3(x, y, z);
    if(lastRawGyroCount + 1 == saveCounts) {
        gyroBufferFull = true;
    }
    lastRawGyro = lastRawGyros[lastRawGyroCount];
    lastRawGyroCount = (lastRawGyroCount + 1) % saveCounts;
    if(useIMUFiltering) {
        lastFilteredGyro = filterGyro(lastRawGyro);
    } else {
        lastFilteredGyro = lastRawGyro.clone();
    }

    processFilteredGyro(lastFilteredGyro, deltaT);
}

void INS::processFilteredAcc(const Vec3 &acc, uint32_t deltaT) {
    float elapsedSeconds = deltaT / 1000000.0f;
    //rotation
    double limitRad = PI / 3.5;
    double g = acc.getLength();
    double pitch = rot.toEulerZYX().y;
    float minG = 0.8;
    if(pitch > -limitRad && pitch < limitRad && g > minG) { //check if movement is too strong or gimbal lock could interfere
        Vec3 accCorrected = acc;
        if(useDroneOptimization && !headDown) {
            accCorrected = Vec3(acc.x, acc.y, sqrt(-pow(acc.x, 2) -pow(acc.y, 2) + 1)); // calculating z assuming that G-Force is equal to 1 => eliminating propeller lift
        }
        if(accCorrected.z == accCorrected.z) { // NaN check
            double accRoll = atan2(accCorrected.y, accCorrected.z); //minus because not beeing aligned with sticks otherwise
            double accPitch = -atan2(accCorrected.x, sqrt(accCorrected.y*accCorrected.y + accCorrected.z*accCorrected.z));
            Quaternion accRot(EulerRotation(accRoll, accPitch, -rot.toEulerZYX().z));
            accRot *= accAngleOffset;
            rot.normalize();
            rot.calibrate();
            accRot.calibrate();

            rot = Quaternion::lerp(accRot, rot, 1 - accInfluence);
        }
    }
    readingVersion++;
    Vec3 accel = acc * G;
    //rotate acceleration
    rot.rotate(accel);
    //subtracting gravity
    accel -= Vec3(0, 0, 1);
    accel /= 2.0;//strange but works
    vel += accel * elapsedSeconds;
    
    double minOff = 2;
    float baroAltitudeInfl = 0.0005;
    // float baroAltitudeSpdInfl = 0.001;
    if(baroAltitude < loc.z - minOff || baroAltitude > loc.z + minOff) {
        baroAltitudeInfl = 0.01;
    }

    //location
    loc += vel * elapsedSeconds;

    loc.x = 0;
    loc.y = 0;

    loc.z = loc.z * (1 - baroAltitudeInfl) + baroAltitude * baroAltitudeInfl;
    vel.z = vel.z * (1 - baroAltitudeInfl) + baroAltSpeed * baroAltitudeInfl;
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


    /**
     * head down
     */
    Vec3 facing(0,0,1);
    rot.rotate(facing);
    headDown = facing.z < 0.1;
    // loc = facing;
    // if(millis() % 100 == 0) {
    //     Serial.println(facing.toString());
    // }
    // if(headDown && millis() % 100 == 0) {
    //     Serial.println("flipped");
    // }
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

void INS::processFilteredBaroAltitude(const double altitude, uint32_t deltaT) {
    baroAltitude = altitude;
    float elapsedSeconds = deltaT / 1000000.0f;
    static double lastBaroAlt = 0.0;
    if(lastBaroAlt == 0.0) {
        lastBaroAlt = altitude;
    }
    double tmpBaroSpd = (altitude - lastBaroAlt) / elapsedSeconds;
    baroAltSpeed = lowPassFilter(baroAltSpeed, tmpBaroSpd, 0.1);
    lastBaroAlt = altitude;
    // Serial.println(baroAltSpeed);
}


void INS::reset() {
    rot.setFromEuler(EulerRotation(0, 0, 0));
    // rot.setFromEuler(EulerRotation(0, 0, atan2(lastFilteredMag.y, lastFilteredMag.x)));
    vel = Vec3();
    loc = Vec3();
    altitudeOffset = -lastRawBaroAltitude;
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
    accOffset = offset * -1.0;
    Serial.print("Succsessfully calibrated acc :) Offset is: ");
    Serial.println(accOffset.toString());
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
    Vec3 filtered = accMul * (acc - accOffset);
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

Vec3 INS::filterMag(const Vec3 &mag) {
    return magSoftIron * (mag - magHardIron);
}

double INS::filterBaroAltitude(const double altitude) {
    if(altitudeOffset == 0.0) {
        altitudeOffset = -lastRawBaroAltitude;
    }
    return altitude + altitudeOffset;
}

void INS::setAccAngleOffset() {
    EulerRotation eulerRot = getEulerRotationZYX();
    eulerRot.z = 0;
    eulerRot.x *= 1;
    eulerRot.y *= 1;
    accAngleOffset = Quaternion(eulerRot);
    Serial.print("set acc angle offset to: ");
    Serial.println(accAngleOffset.toString());
}

void INS::setAccAngleOffset(Quaternion quat) {
    accAngleOffset = quat;
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

float INS::getGForce() {
    return lastFilteredAcc.getLength();;
}