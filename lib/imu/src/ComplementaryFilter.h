#pragma once
#include "SensorFusion.h"
#include <sensorInterface.h>
#include <maths.h>

#define G 9.807

class ComplementaryFilter : public SensorFusion {
public:
    ComplementaryFilter(SensorInterface* sensors) : SensorFusion(sensors){}

    float accInfluence = 0.002;
    float magInfluence = 0.002;

    void begin() {
        // nothing to do
    }

    void handle() {
        if(!sensors->acc.isError() && sensors->acc.lastChange != lastAcc) {
            processAcc(sensors->acc.getVec3(), micros() - sensors->acc.lastChange);
            lastAcc = sensors->acc.lastChange;
        }
        if(!sensors->gyro.isError() && sensors->gyro.lastChange != lastGyro) {
            processAcc(sensors->gyro.getVec3(), micros() - sensors->gyro.lastChange);
            lastGyro = sensors->gyro.lastChange;
        }
        if(!sensors->mag.isError() && sensors->mag.lastChange != lastMag) {
            processMag(sensors->mag.getVec3());
            lastMag = sensors->mag.lastChange;
        }
        if(!sensors->baro.isError() && sensors->baro.lastChange != lastBaro) {
            processBaroAltitude(sensors->baro.altitude, micros() - sensors->baro.lastChange);
            lastBaro = sensors->baro.lastChange;
        }
        if(!sensors->gps.isError() && sensors->gps.lastChange != lastGPS) {
            processGPS(sensors->gps);
            lastGPS = sensors->gps.lastChange;
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

private:
    uint64_t lastAcc = 0;
    uint64_t lastGyro = 0;
    uint64_t lastMag = 0;
    uint64_t lastBaro = 0;
    uint64_t lastGPS = 0;

    float baroAltitude = 0;
    float lastBaroAlt = 0;
    float baroAltSpeed = 0;

    bool useDroneOptimization = true;
    bool headDown = false;

    void processAcc(const Vec3 acc, const uint32_t deltaT) {
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
                rot.normalize();
                rot.calibrate();
                accRot.calibrate();

                rot = Quaternion::lerp(accRot, rot, 1 - accInfluence);
            }
        }
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

    void processMag(const Vec3 &mag) {
        float pitch = rot.toEulerZYX().y;
        float roll = rot.toEulerZYX().x;
        if(pitch < PI / 4 && pitch > -PI / 4 && roll < PI / 4 && roll > -PI / 4) {
            Quaternion magRot(EulerRotation(roll, pitch, -atan2(mag.y, mag.x)));
            rot = Quaternion::lerp(magRot, rot, 1 - magInfluence);
        }
    }

    void processBaroAltitude(const double altitude, const uint32_t deltaT) {
        baroAltitude = altitude;
        float elapsedSeconds = deltaT / 1000000.0f;
        static double lastBaroAlt = 0.0;
        if(lastBaroAlt == 0.0) {
            lastBaroAlt = altitude;
        }
        double tmpBaroSpd = (altitude - lastBaroAlt) / elapsedSeconds;
        baroAltSpeed = lowPassFilter(baroAltSpeed, tmpBaroSpd, 0.1);
        lastBaroAlt = altitude;
    }

    void processGPS(GPS gps) {
        /**
         * Todo
         */
    }

    static double lowPassFilter(double prevFiltered, double now, double smoothing) {
		return now * smoothing + prevFiltered * (1 - smoothing);
	}
};