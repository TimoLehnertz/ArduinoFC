#pragma once
#include <Arduino.h>
// #include <ins.h>
#include "../../imu/src/ins.h"
#include "Motor.h"
#include "../../crossfire/src/crossfire.h"
// #include <pid.h>
#include "../../pid/pid.h"
#include "flightModes.h"

/**
 * Default pids
 */
//rates
#define RATE_PID_RP                 0.00050f //Roll
#define RATE_PID_RI                 0.00100f
#define RATE_PID_RD                 0.00010f
#define RATE_PID_RD_LPF             0.95000f
#define RATE_PID_R_MAX              1.00000f

#define RATE_PID_PP                 0.00050f //pitch
#define RATE_PID_PI                 0.00100f
#define RATE_PID_PD                 0.00010f
#define RATE_PID_PD_LPF             0.95000f
#define RATE_PID_P_MAX              1.00000f

#define RATE_PID_YP                 0.00080f //yaw
#define RATE_PID_YI                 0.00400f
#define RATE_PID_YD                 0.00000f
#define RATE_PID_YD_LPF             1.00000f
#define RATE_PID_Y_MAX              1.00000f

//levels
#define LEVEL_PID_RP                0.00148f//Roll
#define LEVEL_PID_RI                0.00000f
#define LEVEL_PID_RD                0.00058f
#define LEVEL_PID_RD_LPF            1.00000f
#define LEVEL_PID_R_MAX             1.00000f

#define LEVEL_PID_PP                0.00148f//pitch
#define LEVEL_PID_PI                0.00000f
#define LEVEL_PID_PD                0.00058f
#define LEVEL_PID_PD_LPF            1.00000f
#define LEVEL_PID_P_MAX             1.00000f

#define LEVEL_PID_YP                0.00000f//yaw
#define LEVEL_PID_YI                0.00000f
#define LEVEL_PID_YD                0.00000f
#define LEVEL_PID_YD_LPF            1.00000f
#define LEVEL_PID_Y_MAX             1.00000f

#define I_RELAX_MIN_RATE            30

class FC {
public:

    FlightMode::FlightMode_t flightMode = FlightMode::rate;
    FlightMode::FlightMode_t overwriteFlightMode = FlightMode::none;

    float maxAngle = 20;
    bool propsIn = true;

    /**
     * Rates
     */
    float rollRateRC = 1.0f;
    float rollRateSuper = 0.7f;
    float rollRateRCExpo = 0.0f;

    float pitchRateRC = 1.0f;
    float pitchRateSuper = 0.7f;
    float pitchRateRCExpo = 0.0f;

    float yawRateRC = 1.0f;
    float yawRateSuper = 0.7f;
    float yawRateRCExpo = 0.0f;

    /**
     * Anti gravity
     */
    bool useAntiGravity = true;
    float antiGravityMul = 1.0f;
    float boostLpf = 0.005;
    float boostSpeed = 40;
    float iBoost = 0;

    int iRelaxMinRate = I_RELAX_MIN_RATE;

    INS* ins;
    PID rateRollPID;
    PID ratePitchPID;
    PID rateYawPID;

    PID levelRollPID;
    PID levelPitchPID;
    PID levelYawPID;

    CRSF_TxChanels_Converted chanels;
    CRSF_TxChanels chanelsRaw;

    float rollRateAdjust = 0.0f;
    float pitchRateAdjust = 0.0f;
    float yawRateAdjust = 0.0f;

    /**
     * Statistics
     */
    float gForce = 1.0f;
    float maxGForce = 1.0f;
    
    FC(INS* ins, Motor* mFL, Motor* mFR, Motor* mBL, Motor* mBR) :
        ins(ins),
        rateRollPID     (RATE_PID_RP,   RATE_PID_RI,  RATE_PID_RD,  RATE_PID_RD_LPF,  RATE_PID_R_MAX),
        ratePitchPID    (RATE_PID_PP,   RATE_PID_PI,  RATE_PID_PD,  RATE_PID_PD_LPF,  RATE_PID_P_MAX),
        rateYawPID      (RATE_PID_YP,   RATE_PID_YI,  RATE_PID_YD,  RATE_PID_YD_LPF,  RATE_PID_Y_MAX),
        levelRollPID    (LEVEL_PID_RP,  LEVEL_PID_RI, LEVEL_PID_RD, LEVEL_PID_RD_LPF, LEVEL_PID_R_MAX),
        levelPitchPID   (LEVEL_PID_PP,  LEVEL_PID_PI, LEVEL_PID_PD, LEVEL_PID_PD_LPF, LEVEL_PID_P_MAX),
        levelYawPID     (LEVEL_PID_YP,  LEVEL_PID_YI, LEVEL_PID_YD, LEVEL_PID_YD_LPF, LEVEL_PID_Y_MAX),
        mFL(mFL), mFR(mFR), mBL(mBL), mBR(mBR) {}

    void begin() {
        mFL->begin();
        mFR->begin();
        mBL->begin();
        mBR->begin();
    }

    void handle() {
        handleArm();
        handleFlightMode();
        handleI();
        handleAntiGravity();
        handleStatistics();

        PID::updateAux(chanels.aux6, chanels.aux7, chanels.aux8);

        // levelRollPID.pMul  = map(chanels.aux6, -1, 1, 0, 2);
        // levelRollPID.iMul  = map(chanels.aux7, -1, 1, 0, 2);
        // levelRollPID.dMul  = map(chanels.aux8, -1, 1, 0, 2);

        // levelPitchPID.pMul = map(chanels.aux6, -1, 1, 0, 2);
        // levelPitchPID.iMul = map(chanels.aux7, -1, 1, 0, 2);
        // levelPitchPID.dMul = map(chanels.aux8, -1, 1, 0, 2);

        float desYawRate = stickToRate(chanels.yaw,   yawRateRC,   yawRateSuper,   yawRateRCExpo);

        switch(flightMode) {
            case FlightMode::rate: {
                float desRollRate  = stickToRate(chanels.roll,  rollRateRC,  rollRateSuper,  rollRateRCExpo);
                float desPitchRate = stickToRate(chanels.pitch, pitchRateRC, pitchRateSuper, pitchRateRCExpo);
                controllRate(rollRateAdjust, pitchRateAdjust, yawRateAdjust, desRollRate, desPitchRate, desYawRate);
                break;
            }
            case FlightMode::level: {
                float desRollAngle = chanels.roll * 45.0f;
                float desPitchAngle = chanels.pitch * 45.0f;
                controllAngle(rollRateAdjust, pitchRateAdjust, yawRateAdjust, desRollAngle, desPitchAngle, desYawRate);
                break;
            }
            default: {}
        }
        controllMotors(chanels.throttle, rollRateAdjust, pitchRateAdjust, yawRateAdjust);

        mFL->handle();
        mFR->handle();
        mBL->handle();
        mBR->handle();
    }

    void arm() {
        Serial.println("arming");
        Serial2.println("arming");
        mFL->arm();
        mFR->arm();
        mBL->arm();
        mBR->arm();
    };

    void disarm() {
        mFL->disarm();
        mFR->disarm();
        mBL->disarm();
        mBR->disarm();
    };

    void startFailsafe() {
        disarm();
    }

    void stopFailsafe() {
        //do nothing
    }

    bool isArmed() {
        return mFL->isArmed() || mFR->isArmed() || mBL->isArmed() || mBR->isArmed();
    }

    void updateRcChanels(CRSF_TxChanels_Converted& chanels, CRSF_TxChanels& chanelsRaw) {
        this->chanels = chanels;
        this->chanelsRaw = chanelsRaw;
    }

private:

    bool airborne = false;
    bool armed = false;

    float lastThrottle = 0;
    float lastBoost = 0;

    Motor* mFL; // front left:  motor1
    Motor* mFR; // front rigt:  motor2
    Motor* mBL; // back left:   motor3
    Motor* mBR; // back right:  motor4

    long imuReadingVerions = -1;
    long count = 0;
    uint32_t lastHandle = 0;

    float rollRates = 300;
    float pitchRates = 300;
    float yawRates = 300;

    uint64_t antiGravityStart = 0;

    float maxRateChange = 1.0f; // maximum percentage of thrust that is beeing used for pitch / yaw / roll

    void controllAngle(float &rollRateAdjust, float &pitchRateAdjust, float &yawRateAdjust, float desRollAngle, float desPitchAngle, float desYawRate) {
        // float roll = -((ins->getRoll() * 4068) / 71 + desRollAngle);
        // float pitch = -((ins->getPitch() * 4068) / 71 + desPitchAngle);
        // float desRollRate = levelRollPID.compute(roll, desRollAngle, ins->getRollRate());
        // float desPitchRate = levelPitchPID.compute(pitch, desPitchAngle, ins->getPitchRate());
        // controllRate(rollRateAdjust, pitchRateAdjust, yawRateAdjust, desRollRate, desPitchRate, desYawRate);

        float roll = (-ins->getRoll() * 4068) / 71;
        float pitch = (-ins->getPitch() * 4068) / 71;
        rollRateAdjust = levelRollPID.compute(roll, desRollAngle, ins->getRollRate());
        pitchRateAdjust = levelPitchPID.compute(pitch, desPitchAngle, ins->getPitchRate());
        yawRateAdjust = rateYawPID.compute(ins->getYawRate(), desYawRate);
    }

    void controllRate(float &rollRateAdjust, float &pitchRateAdjust, float &yawRateAdjust, float desRollRate, float desPitchRate, float desYawRate) {
        rollRateAdjust = rateRollPID.compute(ins->getRollRate(), desRollRate);
        pitchRateAdjust = ratePitchPID.compute(ins->getPitchRate(), desPitchRate);
        yawRateAdjust = rateYawPID.compute(ins->getYawRate(), desYawRate);
    }

    void reset() {
        ratePitchPID.reset();
        rateYawPID.reset();
        rateRollPID.reset();

        levelPitchPID.reset();
        levelYawPID.reset();
        levelRollPID.reset();
    }

    void crop(float& val, float lim) {
        return crop(val, -lim, lim);
    }

    void crop(float& val, float min, float max) {
        if(val < min) val = min;
        if(val > max) val = max;
    }

    /**
     * Betaflight rates
     * r = RC_RATE
     * b = s_Rate
     * c = expo
     * Desmos.com: f\left(x\right)=\left(200\cdot\ \left(\left(x^{4}\ \cdot\ c\right)\ +\ x\ \cdot\ \left(1-c\right)\right)\ \cdot\ a\right)\cdot\left(\frac{1}{\left(1-\left(x\cdot b\right)\right)}\right)\left\{-1<x<1\right\}
     */
    float stickToRate(float x, float rc, float super, float expo) {
        float mul = x > 0 ? 1 : -1;
        x = abs(x);
        return (200.0f * ((pow(x, 4.0f) * expo) + x * (1.0f - expo)) * rc) * (1.0f / (1.0f - (x * super))) * mul;
    }

    void handleFlightMode() {
        if(overwriteFlightMode != FlightMode::none) {
            flightMode = overwriteFlightMode;
        } else if(chanels.aux2 > 0.75) {
            flightMode = FlightMode::level;
        } else {
            flightMode = FlightMode::rate;
        }

        switch(flightMode) {
            case FlightMode::rate: {
                ins->sensors->useAcc = true;
                ins->sensors->useMag = true;
                break;
            }
            case FlightMode::level: {
                ins->sensors->useAcc = true;
                ins->sensors->useMag = true;
            }
            default: {
                ins->sensors->useAcc = true;
                ins->sensors->useMag = true;
            }
        }
    }

    void handleArm() {
        if(chanels.aux1 > 0.9 && !armed) {
            arm();
            reset();
            armed = true;
        } else if(chanels.aux1 < 0.9) {
            disarm();
            armed = false;
        }
    }

    void controllMotors(float throttle, float rollRateAdjust, float pitchRateAdjust, float yawRateAdjust) {
        crop(yawRateAdjust, maxRateChange);
        crop(rollRateAdjust, maxRateChange);
        crop(pitchRateAdjust, maxRateChange);
        float mFLThrottle = chanels.throttle + rollRateAdjust - pitchRateAdjust + (propsIn ? +yawRateAdjust : -yawRateAdjust);
        float mFRThrottle = chanels.throttle - rollRateAdjust - pitchRateAdjust + (propsIn ? -yawRateAdjust : +yawRateAdjust);
        float mBLThrottle = chanels.throttle + rollRateAdjust + pitchRateAdjust + (propsIn ? -yawRateAdjust : +yawRateAdjust);
        float mBRThrottle = chanels.throttle - rollRateAdjust + pitchRateAdjust + (propsIn ? +yawRateAdjust : -yawRateAdjust);

        float minThrottle = min(mFLThrottle, min(mFRThrottle, min(mBLThrottle, mBRThrottle)));
        if(minThrottle < 0) {
            mFLThrottle -= minThrottle;
            mFRThrottle -= minThrottle;
            mBLThrottle -= minThrottle;
            mBRThrottle -= minThrottle;
        }

        float maxThrottle = max(mFLThrottle, max(mFRThrottle, max(mBLThrottle, mBRThrottle)));
        if(maxThrottle > 1) {
            mFLThrottle += 1 - maxThrottle;
            mFRThrottle += 1 - maxThrottle;
            mBLThrottle += 1 - maxThrottle;
            mBRThrottle += 1 - maxThrottle;
        }

        mFL->write(mFLThrottle);
        mFR->write(mFRThrottle);
        mBL->write(mBLThrottle);
        mBR->write(mBRThrottle);
    }

    uint32_t lastRollIReset = 0;
    uint32_t lastPitchIReset = 0;
    uint32_t lastYawIReset = 0;

    void handleI() {
        if(armed && chanels.throttle > 0.3) {
            airborne = true;
        }
        if(!armed) {
            airborne = false;
        }
        airborne = true;
        //relax
        rateRollPID.lockI   = abs(ins->getRollRate()) > iRelaxMinRate;
        ratePitchPID.lockI  = abs(ins->getPitchRate()) > iRelaxMinRate;
        rateYawPID.lockI    = abs(ins->getYawRate()) > iRelaxMinRate;

        rateRollPID.iEnabled  = airborne;
        ratePitchPID.iEnabled = airborne;
        rateYawPID.iEnabled   = airborne;
        levelRollPID.iEnabled = airborne;
        levelPitchPID.iEnabled= airborne;
        levelYawPID.iEnabled  = airborne;
    }

    float getMaxAbsAttitudeChanel() {
        return max(abs(chanels.roll), max(abs(chanels.pitch), abs(chanels.yaw)));
    }

    void handleAntiGravity() {
        if(!useAntiGravity) return;
        iBoost = abs(chanels.throttle - lastThrottle) * boostSpeed;
        if(iBoost < lastBoost) {
            iBoost = boostLpf * iBoost + (1 - boostLpf) * lastBoost;
        }
        if(iBoost > 1) iBoost = 1;
        if(iBoost < 0) iBoost = 0;
        rateRollPID.iBoost  = 1 + (antiGravityMul - 1) * iBoost;
        ratePitchPID.iBoost = 1 + (antiGravityMul - 1) * iBoost;
        rateYawPID.iBoost   = 1 + (antiGravityMul - 1) * iBoost;
        lastThrottle = chanels.throttle;
        lastBoost = iBoost;
    }

    void handleStatistics() {
        if(chanels.aux3 > 0.5) {
            maxGForce = 0;
        }
        gForce = ins->getGForce();
        maxGForce = max(maxGForce, gForce);
    }
};