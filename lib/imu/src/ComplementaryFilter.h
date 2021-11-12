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

    double baroAltitude = 0;
    double baroOffset = 0;
    double lastRawBaroAltitude = 0;
    double lastBaroAlt = 0;

    bool useDroneOptimization = true;
    bool headDown = false;
    
    // GPS
    double centerLat = 0;
    double centerLng = 0;
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
        if(pitch > -limitRad && pitch < limitRad && roll > -limitRad && roll < limitRad && g > minG && g < maxG) { //check if movement is too strong or gimbal lock could interfere
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
         //subtracting gravity
        Vec3 accel = acc * G;
        //rotate acceleration
        rot.rotate(accel);
        accel -= Vec3(0, 0, G);
        vel += accel * elapsedSeconds;
        
        float baroAltitudeInfl = abs(baroAltitude - loc.z) * 0.0001;

        // vel.x *= 0.999;
        // vel.y *= 0.999;

        //location
        loc += vel * elapsedSeconds;

        // loc.x = 0;
        // loc.y = 0;

        loc.z = loc.z * (1 - baroAltitudeInfl) + baroAltitude * baroAltitudeInfl;
        vel.z = vel.z * (1 - baroAltitudeInfl) + baroAltSpeed * baroAltitudeInfl;
        // loc = accel.clone();
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
        float roll = rot.toEulerZYX().x * RAD_TO_DEG;
        float pitch = rot.toEulerZYX().y * RAD_TO_DEG;

        // Vec3 magFiltered = Vec3();
        // magFiltered.y = mag.x * cos(pitch) + mag.y * sin(roll) * sin(pitch) - mag.z * cos(roll) * sin(pitch);
        // magFiltered.x = mag.y * cos(roll) + mag.z *sin(roll);

        double limDeg = 5;
        if(pitch < limDeg && pitch > -limDeg && roll < limDeg && roll > -limDeg) {
            Quaternion magRot(EulerRotation(rot.toEulerZYX().x, -rot.toEulerZYX().y, -atan2(mag.y, mag.x)));
            rot = Quaternion::lerp(magRot, rot, 1 - magInfluence);
        }
    }

    void processBaroAltitude(const double altitude, const uint32_t deltaT) {
        float elapsedSeconds = deltaT / 1000000.0f;
        lastRawBaroAltitude = altitude;
        if(baroOffset == 0.0) {
            baroOffset = altitude;
        }
        const double altitudeFiltered = altitude - baroOffset;
        baroAltitude = altitudeFiltered;

        if(lastBaroAlt == 0.0) {
            lastBaroAlt = altitudeFiltered;
        }
        double tmpBaroSpd = (baroAltitude - lastBaroAlt) / elapsedSeconds;
        baroAltSpeed = lowPassFilter(baroAltSpeed, tmpBaroSpd, 0.0000001);
        lastBaroAlt = baroAltitude;
    }

    void processGPS(GPS gps, uint64_t deltaT) {
        static Vec3 lastGPSloc = Vec3();
        if(gps.locationValid) {
            if(centerLat == 0) {
                centerLat = gps.lat;
                centerLng = gps.lng;
            }
            loc.y = (gps.lat - centerLat) * (EARTH_CIRCUM / 360.0);
            loc.x = (gps.lng - centerLng) * (EARTH_CIRCUM / 360.0) * cos(gps.lng * DEG_TO_RAD);
            lastLat = gps.lat;
            lastLng = gps.lng;
            lastGPSloc = loc.clone();
            if(lastGPSloc.getLength() != 0) {
                vel = (lastGPSloc - loc) * (deltaT / 1000000.0);
            }
        }
    }

    void reset() {
        rot = Quaternion();
        loc = Vec3();
        vel = Vec3();
        centerLat = lastLat;
        centerLng = lastLng;
        resetAltitude();
    }

    void resetAltitude() {
        baroOffset = lastRawBaroAltitude;
        lastBaroAlt = baroAltitude;
        baroAltSpeed = 0;
        loc.z = 0;
    }

    static double lowPassFilter(double prevFiltered, double now, double smoothing) {
		return now * smoothing + prevFiltered * (1 - smoothing);
	}
};