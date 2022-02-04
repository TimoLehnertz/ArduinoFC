/**
 * @file ins.h
 * @author Timo Lehnertz
 * @brief 
 * @version 0.1
 * @date 2022-01-01
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once

#include <maths.h>
#include <sensorInterface.h>
#include "SensorFusion.h"
#include "ComplementaryFilter.h"
#include "Magdwick.h"

class INS {
public:

    SensorInterface* sensors;
    ComplementaryFilter complementaryFilter;
    MagdwickFilter magdwickFilter;

    INS(SensorInterface* sensors) : sensors(sensors), complementaryFilter(sensors), magdwickFilter(sensors) {}

    void begin() {getSensorFusion()->begin();}
    void handle() {getSensorFusion()->handle();}

    void reset() {getSensorFusion()->reset();}
    void resetAltitude() {getSensorFusion()->resetAltitude();}

    bool isAngleSmallerThanDeg(double deg) {
        float roll = getRoll() * RAD_TO_DEG;
        float pitch = getPitch() * RAD_TO_DEG;
        return roll > -deg && roll < deg && pitch > -deg && pitch < deg;
    }

    double getMaxAngleDeg() {
        double roll = getRoll() * RAD_TO_DEG;
        double pitch = getPitch() * RAD_TO_DEG;
        return max(abs(roll), abs(pitch));
    }

    /**
     * Getters /Setters
     */
    SensorFusion::FusionAlgorythm getFusionAlgorythm() {return sensorFusionType;}
    void setFusionAlgorythm(SensorFusion::FusionAlgorythm algorythm) {sensorFusionType = algorythm; begin();}
    double getRoll()        {return getEulerRotationZYX().getRoll();}
    double getRollRate()    {return sensors->gyro.x;}
    double getMaxRate()     {return max(abs(sensors->gyro.x), max(abs(sensors->gyro.y), abs(sensors->gyro.z)));}
    double getMaxRateExceptYaw()     {return max(abs(sensors->gyro.x), abs(sensors->gyro.y));}
    double getPitch()       {return getEulerRotationZYX().getPitch();}
    double getPitchRate()   {return sensors->gyro.y;}
    double getYaw()         {return getEulerRotationZYX().getYaw();}
    double getYawRate()     {return sensors->gyro.z;}
    float getGForce()       {return sensors->acc.getVec3().getLength();}
    double getMagZOffset()  {return getSensorFusion()->getMagZOffset();}
    void setMagZOffset(double deg)  {return getSensorFusion()->setMagZOffset(deg);}
    Vec3 getLocation()      {return getSensorFusion()->getLocation();}
    Vec3 getVelocity()      {return getSensorFusion()->getVelocity();}
    Vec3 getLocalVelocity()      {return getSensorFusion()->getLocalVelocity();}
    EulerRotation getEulerRotationZYX() {return getSensorFusion()->getEulerAttitudeZYX();}
    Quaternion getQuaternionRotation() {return getSensorFusion()->getAttitude();}

private:
    SensorFusion::FusionAlgorythm sensorFusionType;

    SensorFusion* getSensorFusion() {
        switch(sensorFusionType) {
            case SensorFusion::ComplementaryFilter: return &complementaryFilter;
            case SensorFusion::MagdwickFilter: return &magdwickFilter;
            default: return &complementaryFilter;
        }
    };
};