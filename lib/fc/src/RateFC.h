#pragma once
#include "FC.h"
#include "utils.h"
#include <ins.h>
#include "Motor.h"
#include <pid.h>
#include <Arduino.h>

#define P_RP 0.09f // Roll and pitch
#define I_RP 0.1f
#define D_RP 0.10f

#define P_Y 0.4f //yaw
#define I_Y 0.8f
#define D_Y 0.0015f

class RateFC : public FC {
public:
    
    float maxPitchRate = 360;
    float maxRollRate = 360;
    float maxYawRate = 360;

    RateFC(INS* ins, Motor* motorFL, Motor* motorFR, Motor* motorBL, Motor* motorBR) :
        FC(ins, motorFL, motorFR, motorBL, motorBR),

        rollPID (P_RP, I_RP, D_RP),
        pitchPID(P_RP, I_RP, D_RP),
        yawPID  (P_Y, I_Y, D_Y) {}
    
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

    float maxYawAdjust = 0.3;

    long count = 0;
    uint64_t t = 0;

    float yawInfluence = 0.3; //multiplicator for yaw adjustments. 1 is fully aggressive
    float rollSpeed = 0.2;
    float maxRateChange = 0.3; // maximum percentage of thrust that is beeing used for pitch / yaw / roll

    void handlePrivate() {

        float p = map(chanels.chanel10, -1, 1, 0, 2);
        float i = map(chanels.chanel11, -1, 1, 0, 2);
        float d = map(chanels.chanel12, -1, 1, 0, 2);

        rollPID.p = P_RP * p;
        rollPID.i = I_RP * i;
        rollPID.d = D_RP * d;

        pitchPID.p = P_RP * p;
        pitchPID.i = I_RP * i;
        pitchPID.d = D_RP * d;

        float rollRate = ins->getRollRate();// is already in degrees
        float pitchRate = ins->getPitchRate();// is already in degrees
        float yawRate = ins->getYawRate(); // is already in degrees

        float rollRateAdjust = rollPID.compute(rollRate, chanels.roll * maxRollRate, ins->getRollRate());
        float pitchRateAdjust = pitchPID.compute(pitchRate, chanels.pitch * maxPitchRate, ins->getPitchRate());
        float yawRateAdjust = yawPID.compute(yawRate, chanels.yaw * maxYawRate, ins->getYawRate());

        rollRateAdjust  /= 360; //convert degrees to motor value from 0 to 1
        pitchRateAdjust /= 360;
        yawRateAdjust /= 360;

        crop(yawRateAdjust, maxYawAdjust);


        float mFLVal = chanels.throttle + rollRateAdjust - pitchRateAdjust - yawRateAdjust;
        float mFRVal = chanels.throttle - rollRateAdjust - pitchRateAdjust + yawRateAdjust;
        float mBLVal = chanels.throttle + rollRateAdjust + pitchRateAdjust + yawRateAdjust;
        float mBRVal = chanels.throttle - rollRateAdjust + pitchRateAdjust - yawRateAdjust;

        float minMotor = min(mFLVal, min(mFRVal, min(mBLVal, mBRVal)));
        if(minMotor < 0) {
            mFLVal += -minMotor;
            mFRVal += -minMotor;
            mBLVal += -minMotor;
            mBRVal += -minMotor;
        }

        float maxMotor = max(mFLVal, max(mFRVal, max(mBLVal, mBRVal)));
        if(maxMotor > 1) {
            mFLVal += 1 - maxMotor;
            mFRVal += 1 - maxMotor;
            mBLVal += 1 - maxMotor;
            mBRVal += 1 - maxMotor;
        }

        mFL->write(mFLVal);
        mFR->write(mFRVal);
        mBL->write(mBLVal);
        mBR->write(mBRVal);
        if(count % 1 == 0) {
            // Serial.print("rollPID.p: "); Serial.println(rollPID.p);
            // Serial.print("rollPID.i: "); Serial.println(rollPID.i);
            // Serial.print("rollPID.d: "); Serial.println(rollPID.d);
        }

        count++;
    }

    void crop(float& val, float lim) {
        return crop(val, -lim, lim);
    }

    void crop(float& val, float min, float max) {
        if(val < min) val = min;
        if(val > max) val = max;
    }

    void reset() {
        pitchPID.reset();
        rollPID.reset();
        yawPID.reset();
    }
};