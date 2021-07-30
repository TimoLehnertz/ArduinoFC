#pragma once
#include "FC.h"
#include "utils.h"
#include <ins.h>
#include "Motor.h"
#include <pid.h>
#include <Arduino.h>

#define P_RP 0.2f // Roll and pitch
#define I_RP 0.3f
#define D_RP 0.05f

#define P_y 0.2f //yaw
#define I_RY 0.05f
#define D_RY 0.00015f

class LevelFC : public FC {
public:
    
    float maxAngle = 20; //maximum roll / pitch angle
    float maxYawSpeed = 180; // max 30 deg/s

    LevelFC(INS* ins, Motor* motorFL, Motor* motorFR, Motor* motorBL, Motor* motorBR) :
        FC(ins, motorFL, motorFR, motorBL, motorBR),

        rollPID (P_RP, I_RP, D_RP),
        pitchPID(P_RP, I_RP, D_RP),
        yawPID  (P_y, I_RY, D_RY) {
            // ins->requestCalibration();
        }
    
    void begin() {
        mFL->begin();
        mFR->begin();
        mBL->begin();
        mBR->begin();
    }


    void startFailsafe() {
        disarm();
    }

    void stopFailsafe() {
        //do nothing
    }
private: 

    PID rollPID;
    PID pitchPID;
    PID yawPID;

    long imuReadingVerions = -1;
    long count = 0;
    uint32_t t = 0;

    float yawInfluence = 0.3; //multiplicator for yaw adjustments. 1 is fully aggressive
    float rollSpeed = 0.2;
    float maxRateChange = 0.3; // maximum percentage of thrust that is beeing used for pitch / yaw / roll

    void handlePrivate() {
        // if(imuReadingVerions == ins->getReadingVersion()) {
        //     return; //nothing changed
        // }
        // imuReadingVerions = ins->getReadingVersion();
        float roll = -ins->getRoll() * RAD_TO_DEG;
        float rollRate = -ins->getRollRate();// is already in degrees
        float pitch = ins->getPitch() * RAD_TO_DEG;
        float pitchRate = -ins->getPitchRate();// is already in degrees
        float yawRate = -ins->getYawRate(); // is already in degrees

        // bool useIntegrator = chanels.throttle > 0.1;

        // float rollRateAdjust = rollPID.compute(roll, chanels.roll * maxAngle, ins->getRollRate(), useIntegrator);
        // float pitchRateAdjust = pitchPID.compute(pitch, chanels.pitch * maxAngle, ins->getPitchRate(), useIntegrator);
        // float yawRateAdjust = yawPID.compute(yawRate, chanels.yaw * maxYawSpeed, ins->getYawRate(), useIntegrator, true);
        
        /**
         * Roll
         */
        float rollErrorInfluence = 1;//deg/s per error
        float rollInfluence = 360;//deg/s per to -1 to 1 | divider

        float desRoll = chanels.roll * maxAngle;
        float errRoll = desRoll - roll;
        float desRollRate = errRoll * rollErrorInfluence;

        // if((desRollRate > 0 && rollRate > desRollRate) || (desRollRate < 0 && rollRate > desRollRate)) {
        //     rollRate *= 1.2;
        // }
        float rollRateAdjust = ((desRollRate - rollRate) / rollInfluence) * (chanels.chanel9 + 1);
        // rollRateAdjust = 0;
        /**
         * Pitch
         */
        float desPitch = chanels.pitch * maxAngle;
        float errPitch = desPitch - pitch;
        float desPitchRate = errPitch * rollErrorInfluence;

        // if(desPitchRate > 0 && pitchRate > desPitchRate || desPitchRate < 0 && pitchRate < desPitchRate) {
        //     pitchRate *= 0.4;
        // }


        float pitchRateAdjust = ((desPitchRate - pitchRate) / rollInfluence) * (chanels.chanel9 + 1);
        // pitchRateAdjust = 0;

        // float pid_roll_setpoint = chanels.roll * maxAngle;

        // float rollRateAdjust = -rollRate / 1000 + chanels.roll / 10;
        // float pitchRateAdjust = -pitchRate / 1000 + chanels.pitch / 10;
        /**
         * Yaw
         */
        float yawRateAdjust = (-(yawRate / maxYawSpeed) + chanels.yaw - 0.3) * yawInfluence;

        uint32_t now = micros();
        uint32_t elapsed = now - t;
        int hz = 1000000 / elapsed;
        t = now;
        count++;

        crop(rollRate, maxRateChange);
        crop(pitchRateAdjust, maxRateChange);
        crop(yawRateAdjust, maxRateChange);

        if(count % 50 == 0) {
            // Serial.printf("%ihz, roll rc: %f, roll: %f, pitch rc: %f, pitch: %f, yaw rc: %f, yawRate: %f throttle: %f\n", hz, chanels.roll, roll, chanels.pitch, pitch, chanels.yaw, yawRate, chanels.throttle);
            // Serial.printf("rollRateAdjust: %f, pitchRateAdjust: %f, yawRateAdjust: %f\n", rollRateAdjust * 100, pitchRateAdjust * 100, yawRateAdjust * 100);
        }

        // rollRateAdjust  /= 360 * 1; //convert degrees to motor value from 0 to 1
        // pitchRateAdjust /= 360 * 1;
        // yawRateAdjust /= 360 * 1;
        // // yawRateAdjust = 0;

        // if(yawRateAdjust > 0.1) {
        //     yawRateAdjust = 0.1;
        // }
        // if(yawRateAdjust < -0.1) {
        //     yawRateAdjust = -0.1;
        // }
        mFL->write(chanels.throttle + rollRateAdjust - pitchRateAdjust + yawRateAdjust);
        mFR->write(chanels.throttle - rollRateAdjust - pitchRateAdjust - yawRateAdjust);
        mBL->write(chanels.throttle + rollRateAdjust + pitchRateAdjust - yawRateAdjust);
        mBR->write(chanels.throttle - rollRateAdjust + pitchRateAdjust + yawRateAdjust);
    }

    void crop(float& val, float lim) {
        return crop(val, -lim, lim);
    }

    void crop(float& val, float min, float max) {
        if(val < min) val = min;
        if(val > max) val = max;
    }
};