#pragma once
#include <SensorInterface.h>
#include <math.h>

/**
 * Abstract class for idfferent aproaches to Sensor Fusion
 * 
 * Required input:
 *      accelerometer, gyroscope,
 * Optional input:
 *      Magnetometer, Barometer, GPS
 * 
 * Output:
 *      attitude(Euler and quaternion)
 *      velocity, position
 */
class SensorFusion {
public:

    /**
     * All available Sensor fusion Algorythms
     */
    enum FusionAlgorythm {
        ComplementaryFilter = 0,
        MagdwickFilter = 1,
    };

    SensorFusion(SensorInterface* sensors) : sensors(sensors) {}

    virtual ~SensorFusion() {}

    virtual void begin() = 0;
    virtual void handle() = 0;
    virtual void resetAltitude() = 0;
    
    virtual void reset() = 0;

    Quaternion getAttitude() {
        return rot;
    }

    EulerRotation getEulerAttitudeZYX() {
        return rot.toEulerZYX();
    }

    double getRoll() {
        return rot.toEulerZYX().x;
    }

    double getPitch() {
        return rot.toEulerZYX().y;
    }

    double getYaw() {
        return rot.toEulerZYX().z;
    }

    Vec3 getVelocity() {
        return vel;
    }

    Vec3 getLocalVelocity() {
        Vec3 velGlobal = vel.clone();
        EulerRotation zRot = EulerRotation(0,0, getYaw());
        zRot.rotateReverse(velGlobal);
        return velGlobal;//(Local)
    }

    Vec3 getLocation() {
        return loc;
    }

    virtual bool isLocationValid() = 0;
    virtual bool isVelocityValid() = 0;
    virtual bool isHeightValid() = 0;

protected:
    SensorInterface* sensors;
    Quaternion rot;
    Vec3 vel;
    Vec3 loc;
};