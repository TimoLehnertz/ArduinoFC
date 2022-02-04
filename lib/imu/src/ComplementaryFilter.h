/**
 * @file ComplementaryFilter.h
 * @author Timo Lehnertz
 * @brief 
 * @version 0.1
 * @date 2022-01-01
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once
#include "SensorFusion.h"
#include <sensorInterface.h>
#include <maths.h>

#define G 9.807
#define EARTH_CIRCUM 40075000.0 //meters

class ComplementaryFilter : public SensorFusion {
public:
    ComplementaryFilter(SensorInterface* sensors) : SensorFusion(sensors){}

    float accInfluence = 0.002;
    float magInfluence = 0.002;
    double baroAltSpeed = 0;
    double magZOffsetDeg = 0.0;
    double baroAltitude = 0;
    // GPS
    double centerLat = 0;
    double centerLng = 0;

    void begin() {
        // nothing to do
    }

    void handle() {
        if(!sensors->acc.isError() && sensors->acc.lastChange != lastAcc) {
            processAcc(sensors->acc.getVec3(), micros() - lastAccProcessed);
            lastAcc = sensors->acc.lastChange;
            lastAccProcessed = micros();
        }
        if(!sensors->gyro.isError() && sensors->gyro.lastChange != lastGyro) {
            processGyro(sensors->gyro.getVec3(), micros() - lastGyroProcessed);
            lastGyro = sensors->gyro.lastChange;
            lastGyroProcessed = micros();
        }
        if(!sensors->mag.isError() && sensors->mag.lastChange != lastMag) {
            processMag(sensors->mag.getVec3());
            lastMag = sensors->mag.lastChange;
        }
        if(!sensors->baro.isError() && sensors->baro.lastChange != lastBaro) {
            processBaroAltitude(sensors->baro.altitude, micros() - lastBaroProcessed);
            lastBaro = sensors->baro.lastChange;
            lastBaroProcessed = micros();
        }
        if(!sensors->gps.isError() && sensors->gps.lastChange != lastGPS) {
            processGPS(sensors->gps, micros() - lastGPSProcessed);
            lastGPS = sensors->gps.lastChange;
            lastGPSProcessed = micros();
        }
    }

    bool isLocationValid() {
        return sensors->gps.error != Error::CRITICAL_ERROR;
    }

    bool isVelocityValid() {
        return sensors->gps.error != Error::CRITICAL_ERROR;
    }

    bool isHeightValid() {
        return sensors->baro.error != Error::CRITICAL_ERROR;
    }

    void calibrateAttitude() {
        // DO nothing
    }

    void setMagZOffset(double deg) {
        magZOffsetDeg = deg;
    }

    double getMagZOffset() {
        return magZOffsetDeg;
    }

private:
    uint64_t lastAcc = 0;
    uint64_t lastAccProcessed = 0;
    uint64_t lastGyro = 0;
    uint64_t lastGyroProcessed = 0;
    uint64_t lastMag = 0;
    uint64_t lastMagProcessed = 0;
    uint64_t lastBaro = 0;
    uint64_t lastBaroProcessed = 0;
    uint64_t lastGPS = 0;
    uint64_t lastGPSProcessed = 0;

    Vec3 lastMagF = Vec3();
    uint64_t magCounter = 0;

    double baroOffset = 0;
    double lastRawBaroAltitude = 0;
    double lastBaroAlt = 0;

    bool useDroneOptimization = true;
    bool headDown = false;
    
    // GPS
    double lastLat = 0;
    double lastLng = 0;


    void processAcc(const Vec3 acc, const uint32_t deltaT) {
        float elapsedSeconds = deltaT / 1000000.0f;
        //rotation
        double limitRad = 70 * DEG_TO_RAD;
        double g = acc.getLength();
        double pitch = rot.toEulerZYX().y;
        double roll = rot.toEulerZYX().x;
        float minG = 0.5;
        float maxG = 1.5;
        if(g > minG && g < maxG) { //check if movement is too strong or gimbal lock could interfere
        // if(acc.z > 0 && g > minG && g < maxG) { //check if movement is too strong or gimbal lock could interfere
        // if(pitch > -limitRad && pitch < limitRad && roll > -limitRad && roll < limitRad && g > minG && g < maxG) { //check if movement is too strong or gimbal lock could interfere
            Vec3 accCorrected = acc;
            if(useDroneOptimization && !headDown) {
                accCorrected = Vec3(acc.x, acc.y, sqrt(-pow(acc.x, 2) -pow(acc.y, 2) + 1)); // calculating z assuming that G-Force is equal to 1 => eliminating propeller lift
            }
            if(accCorrected.z == accCorrected.z) { // NaN check
                double accRoll = atan2(accCorrected.y, accCorrected.z); //minus because not beeing aligned with sticks otherwise
                double accPitch = -atan2(accCorrected.x, sqrt(accCorrected.y*accCorrected.y + accCorrected.z*accCorrected.z));
                Quaternion accRot(EulerRotation(accRoll, accPitch, -rot.toEulerZYX().z));
                rot.normalize();
                rot.calibrate();
                accRot.calibrate();

                rot = Quaternion::lerp(accRot, rot, 1 - accInfluence);
            }
        }
        //  subtracting gravity
        Vec3 accel = acc * G;
        //rotate acceleration
        rot.rotate(accel);
        accel -= Vec3(0, 0, G);
        vel.z += accel.z * elapsedSeconds;
        
        // float baroAltitudeInfl = abs(baroAltitude - loc.z) * 0.005;

        // Velocity relaxing
        // vel.x *= 0.9999;
        // vel.y *= 0.9999;

        //location
        // loc += vel * elapsedSeconds;

        // loc.x = 0;
        // loc.y = 0;

        double baroAltSpdInf = 0.0001;
        // double baroAltSpdInf = 0.1;
        double baroAltInf = 0.005;

        loc.z = loc.z * (1 - baroAltInf) + baroAltitude * baroAltInf;
        vel.z = vel.z * (1 - baroAltSpdInf) + baroAltSpeed * baroAltSpdInf;
    }

    void processGyro(const Vec3 &gyro, uint32_t deltaT) {
        Vec3 gyroConf = gyro.clone();
        gyroConf.x *= -1; //align with sticks
        float elapsedSeconds = deltaT / 1000000.0f;

        //rotation
        EulerRotation localRotEuler((gyroConf * elapsedSeconds).toRad(), ZYX_EULER);
        Quaternion localRotQ(localRotEuler);
        rot.normalize();
        rot *= localRotQ;

        // Head down
        Vec3 facing(0,0,1);
        rot.rotate(facing);
        headDown = facing.z < 0.1;
    }

    void processMag(const Vec3 &mag1) {
        Vec3 mag = mag1.clone();
        float roll = rot.toEulerZYX().x * RAD_TO_DEG;
        float pitch = rot.toEulerZYX().y * RAD_TO_DEG;

        // @Todo: Tilt compensation
        // Vec3 magFiltered = Vec3();
        // magFiltered.y = mag.x * cos(pitch) + mag.y * sin(roll) * sin(pitch) - mag.z * cos(roll) * sin(pitch);
        // magFiltered.x = mag.y * cos(roll) + mag.z *sin(roll);

        double limDeg = 25;
        if(pitch < limDeg && pitch > -limDeg && roll < limDeg && roll > -limDeg) {
            mag.toUnitLength();
            EulerRotation euler(rot.toEulerZYX().y, -rot.toEulerZYX().x, 0);
            euler.rotate(mag);

            // Serial.print("FC_S MAG(t);X;");Serial.println(mag.x);
            // Serial.print("FC_S MAG(t);Y;");Serial.println(mag.y);
            // Serial.print("FC_S MAG(t);Z;");Serial.println(mag.z);
            

            double magRotRad = atan2(mag.y, mag.x);

            magRotRad += magZOffsetDeg * DEG_TO_RAD;

            Quaternion magRot(EulerRotation(rot.toEulerZYX().x, -rot.toEulerZYX().y, magRotRad));
            rot = Quaternion::lerp(magRot, rot, magCounter == 10 ? 0.0 : 1 - magInfluence);
            // magCounter++;
        }
    }

    void processBaroAltitude(const double altitude, const uint32_t deltaT) {
        float elapsedSeconds = deltaT / 1000000.0f;
        lastRawBaroAltitude = altitude;
        if(baroOffset == 0.0) {
            baroOffset = altitude;
        }
        const double altitudeFiltered = altitude - baroOffset;
        
        double lpf = (max(0.1, min(10, abs(baroAltitude - altitudeFiltered))) * 0.04) - 0.001;
        baroAltitude = lowPassFilter(baroAltitude, altitudeFiltered, lpf);
        
        lpf = (max(0.1, min(10, abs(baroAltitude - altitudeFiltered))) * 0.01);
        baroAltitude = lowPassFilter(baroAltitude, altitudeFiltered, lpf);
        // baroAltitude = altitudeFiltered;

        if(lastBaroAlt == 0.0) {
            lastBaroAlt = altitudeFiltered;
        }
        double tmpBaroSpd = (baroAltitude - lastBaroAlt) / elapsedSeconds;
        lpf = (max(0, min(10, abs(baroAltSpeed - tmpBaroSpd))) * 0.001) + 0.01;
        baroAltSpeed = lowPassFilter(baroAltSpeed, tmpBaroSpd, lpf);
        // baroAltSpeed = tmpBaroSpd;
        lastBaroAlt = baroAltitude;
    }

    void processGPS(GPS gps, uint64_t deltaT) {
        static Vec3 lastGPSloc = Vec3();
        if(gps.locationValid) {
            if(centerLat == 0) {
                centerLat = gps.lat;
                centerLng = gps.lng;
            }
            // double gpsInf = 0.05;
            loc.y = -(gps.lat - centerLat) * (EARTH_CIRCUM / 360.0);
            loc.x = (gps.lng - centerLng) * (EARTH_CIRCUM / 360.0) * cos(gps.lng * DEG_TO_RAD);
            lastLat = gps.lat;
            lastLng = gps.lng;
            if(lastGPSloc.getLength() != 0) {
                // gpsInf *= 2; // reduce amount heavy of acc drift
                double elapsedSeconds = deltaT / 1000000.0;
                Vec3 gpsVel = (loc - lastGPSloc) / elapsedSeconds;
                vel.x = gpsVel.x;
                vel.y = gpsVel.y;
                // skip z
                // vel.x = 0;
                // vel.y = 0;
            }
            lastGPSloc = loc.clone();
        }
        // double course = -160;
        // EulerRotation eulerRot = rot.toEulerZYX();
        // double course = normalizeDeg(gps.course + 180);
        // // Serial.println(course);

        // double lpf = 0.1;

        // // double zRot = eulerRot.z * RAD_TO_DEG + course / 2;

        // double zRot = lerpDeg(eulerRot.z * RAD_TO_DEG, course, lpf);

        // rot = Quaternion(EulerRotation(eulerRot.x, -eulerRot.y, -zRot * DEG_TO_RAD));
        // Quaternion courseRot(EulerRotation(eulerRot.x, -eulerRot.y, course * DEG_TO_RAD));
        // rot = Quaternion::lerp(rot, courseRot, 0.3);
        // Serial.println("gps");
    }

    double lerpDeg(double from, double to, double fact) {

        from = normalizeDeg(from) * DEG_TO_RAD;
        to = normalizeDeg(to) * DEG_TO_RAD;
        // from *= DEG_TO_RAD;
        // to *= DEG_TO_RAD;

        double deg = atan2(sin(from-to), cos(from-to)) * RAD_TO_DEG;

        // Serial.println(deg);
        // return from * RAD_TO_DEG + deg * fact;
        return normalizeDeg(from * RAD_TO_DEG - deg * fact);
    }

    double normalizeDeg(double deg) {
        while(deg > 180) {
            deg -= 360;
        }
        while(deg < -180) {
            deg += 360;
        }
        return deg;
    }

    void reset() {
        rot = Quaternion();
        magCounter = 0; // resetts rotation to compass after 100 readings
        loc = Vec3();
        vel = Vec3();
        resetAltitude();
    }

    void resetAltitude() {
        baroOffset = lastRawBaroAltitude;
        lastBaroAlt = baroAltitude;
        baroAltSpeed = 0;
        centerLat = lastLat;
        centerLng = lastLng;
        loc.z = 0;
        vel.z = 0;
    }

    static double lowPassFilter(double prevFiltered, double now, double smoothing) {
        if(smoothing < 0) smoothing = 0;
        if(smoothing > 1) smoothing = 1;
		return now * smoothing + prevFiltered * (1 - smoothing);
	}
};