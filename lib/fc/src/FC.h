#pragma once
#include <Arduino.h>
// #include <ins.h>
#include "../../imu/src/ins.h"
#include "Motor.h"
#include "../../crossfire/src/crossfire.h"
// #include <pid.h>
#include "../../pid/pid.h"
#include "flightModes.h"
#include "error.h"

/**
 * Default pids
 */
//rates
#define RATE_PID_RP                 0.00130f//Roll
#define RATE_PID_RI                 0.00150f
#define RATE_PID_RD                 0.00010f
#define RATE_PID_RD_LPF             0.95000f
#define RATE_PID_R_MAX              1.00000f

#define RATE_PID_PP                 0.00130f//pitch
#define RATE_PID_PI                 0.00150f
#define RATE_PID_PD                 0.00010f
#define RATE_PID_PD_LPF             0.95000f
#define RATE_PID_P_MAX              1.00000f

#define RATE_PID_YP                 0.00170f//yaw
#define RATE_PID_YI                 0.00350f
#define RATE_PID_YD                 0.00000f
#define RATE_PID_YD_LPF             1.00000f
#define RATE_PID_Y_MAX              1.00000f

//levels
#define LEVEL_PID_RP                0.00250f//Roll
#define LEVEL_PID_RI                0.00500f
#define LEVEL_PID_RD                0.00050f
#define LEVEL_PID_RD_LPF            1.00000f
#define LEVEL_PID_R_MAX             1.00000f

#define LEVEL_PID_PP                0.00250f//pitch
#define LEVEL_PID_PI                0.00500f
#define LEVEL_PID_PD                0.00050f
#define LEVEL_PID_PD_LPF            1.00000f
#define LEVEL_PID_P_MAX             1.00000f

#define LEVEL_PID_YP                0.00000f//Yaw
#define LEVEL_PID_YI                0.00000f
#define LEVEL_PID_YD                0.00000f
#define LEVEL_PID_YD_LPF            1.00000f
#define LEVEL_PID_Y_MAX             1.00000f

#define ATITUDE_PID_P               0.20000f
#define ATITUDE_PID_I               0.30000f
#define ATITUDE_PID_D               0.10000f
#define ATITUDE_PID_D_LPF           0.10000f
#define ATITUDE_PID_MAX             0.50000f

#define VEL_PID_P                   10.00000f
#define VEL_PID_I                   0.01000f
#define VEL_PID_D                   0.00000f
#define VEL_PID_D_LPF               1.00000f
#define VEL_PID_MAX                 20.00000f


#define I_RELAX_MIN_RATE            15//min rate at wich i gets disabled

#define ANGLE_MODE_MAX_ANGLE        20//deg

#define GPS_MAX_SPEED_HORIZONTAL    5//m/s
#define GPS_MAX_SPEED_VERTICAL      3//m/s

#define HOVER_THROTTLE              0.3// only used when switching into an automode mid flight

#define LAUNCH_I_BOOST_SECONDS      4.0//Seconds after launch where i terms get boosted

#define LAUNCH_I_BOOST_LEVEL        5// multiplies the i term from level pids
#define LAUNCH_I_BOOST_ALTITUDE     1// multiplies the i term from altitude pid

#define RATE_RC                     1.0
#define RATE_SUPER                  0.7
#define RATE_RC_EXPO                0.0



struct Rates : public Vec3 {

    Rates(double rc, double super, double rcExpo) : Vec3(rc, super, rcExpo) {}
    Rates(char* str) : Vec3(str) {}
    Rates(Vec3 v) : Vec3(v.x, v.y, v.z) {}

    double getRC() {
        return x;
    }
    double getSuper() {
        return y;
    }
    double getRCExpo() {
        return z;
    }

    void setRc(double rc) {
        x = rc;
    }
    void setSuper(double super) {
        y = super;
    }
    void setRCExpo(double rcExpo) {
        z = rcExpo;
    }

    Vec3 toVec3() {
        return Vec3(x, y, z);
    }
};

class FC {
public:

    FlightMode::FlightMode_t flightMode = FlightMode::rate;
    FlightMode::FlightMode_t lastFlightMode = FlightMode::none;
    FlightMode::FlightMode_t overwriteFlightMode = FlightMode::none;

    bool propsIn = true;

    Vec3 wayPoint = Vec3(0,0,20); // 20 Meters above home
    bool wayPointReached = false;

    /**
     * Rate
     */
    Rates rollRate   = Rates(RATE_RC, RATE_SUPER, RATE_RC_EXPO);
    Rates pitchRate  = Rates(RATE_RC, RATE_SUPER, RATE_RC_EXPO);
    Rates yawRate    = Rates(RATE_RC, RATE_SUPER, RATE_RC_EXPO);

    /**
     * Anti gravity
     */
    bool useAntiGravity = true;
    float antiGravityMul = 1.0f;
    float boostLpf = 0.005;
    float boostSpeed = 40;
    float iBoost = 0;

    double angleModeMaxAngle = ANGLE_MODE_MAX_ANGLE;

    double gpsMaxSpeedVertical = GPS_MAX_SPEED_VERTICAL;
    double gpsMaxSpeedHorizontal = GPS_MAX_SPEED_HORIZONTAL;

    double iRelaxMinRate = I_RELAX_MIN_RATE;

    double launchIBoostSeconds = LAUNCH_I_BOOST_SECONDS;

    double launchIBoostLevel = LAUNCH_I_BOOST_LEVEL;
    double launchIBoostAltitude = LAUNCH_I_BOOST_ALTITUDE;

    double throttleMul4S = 1.0;
    double throttleMul6S = 1.0;

    INS* ins;
    PID rateRollPID;
    PID ratePitchPID;
    PID rateYawPID;

    PID levelRollPID;
    PID levelPitchPID;
    PID levelYawPID;

    PID altitudePID;

    PID velPIDx;
    PID velPIDy;

    PID groundPID;

    Crossfire* crsf;
    CRSF_TxChanels_Converted chanels;
    CRSF_TxChanels chanelsRaw;

    float rollRateAdjust = 0.0f;
    float pitchRateAdjust = 0.0f;
    float yawRateAdjust = 0.0f;

    double hoverThrottle = HOVER_THROTTLE;

    /**
     * Statistics
     */
    float gForce = 1.0f;
    float maxGForce = 1.0f;
    
    FC(INS* ins, Motor* mFL, Motor* mFR, Motor* mBL, Motor* mBR, Crossfire* crsf) :
        ins(ins),
        rateRollPID     (RATE_PID_RP,       RATE_PID_RI,    RATE_PID_RD,    RATE_PID_RD_LPF,    RATE_PID_R_MAX),
        ratePitchPID    (RATE_PID_PP,       RATE_PID_PI,    RATE_PID_PD,    RATE_PID_PD_LPF,    RATE_PID_P_MAX),
        rateYawPID      (RATE_PID_YP,       RATE_PID_YI,    RATE_PID_YD,    RATE_PID_YD_LPF,    RATE_PID_Y_MAX),
        levelRollPID    (LEVEL_PID_RP,      LEVEL_PID_RI,   LEVEL_PID_RD,   LEVEL_PID_RD_LPF,   LEVEL_PID_R_MAX),
        levelPitchPID   (LEVEL_PID_PP,      LEVEL_PID_PI,   LEVEL_PID_PD,   LEVEL_PID_PD_LPF,   LEVEL_PID_P_MAX),
        levelYawPID     (LEVEL_PID_YP,      LEVEL_PID_YI,   LEVEL_PID_YD,   LEVEL_PID_YD_LPF,   LEVEL_PID_Y_MAX),
        altitudePID     (ATITUDE_PID_P,     ATITUDE_PID_I,  ATITUDE_PID_D,  ATITUDE_PID_D_LPF,  ATITUDE_PID_MAX),
        velPIDx         (VEL_PID_P,         VEL_PID_I,      VEL_PID_D,      VEL_PID_D_LPF,      VEL_PID_MAX),
        velPIDy         (VEL_PID_P,         VEL_PID_I,      VEL_PID_D,      VEL_PID_D_LPF,      VEL_PID_MAX),
        // groundPID       ()
        crsf(crsf),
        mFL(mFL), mFR(mFR), mBL(mBL), mBR(mBR) {
        }

    void begin() {
        mFL->begin();
        mFR->begin();
        mBL->begin();
        mBR->begin();
    }

    void handle() {
        altitudePID.minOut = 0;
        handleArm();
        handleFlightMode();
        handleI();
        // handleAntiGravity();
        handleWaypoint();
        handleStatistics();

        float desYawRate = stickToRate(chanels.yaw, yawRate.getSuper(), yawRate.getSuper(), yawRate.getRCExpo());
        float throttle = chanels.throttle;
        float desRollAngle = chanels.roll * angleModeMaxAngle;
        float desPitchAngle = chanels.pitch * angleModeMaxAngle;

        float climbRate = map(throttle, 0.0, 1.0, -gpsMaxSpeedVertical, gpsMaxSpeedVertical);

        Vec3 velLocalDesired = Vec3(map(chanels.pitch, -1, 1, -gpsMaxSpeedHorizontal, gpsMaxSpeedHorizontal), map(chanels.roll, -1, 1, -gpsMaxSpeedHorizontal, gpsMaxSpeedHorizontal), 0);
        // if(velLocalDesired.getLength() > gpsMaxSpeedVertical) velLocalDesired.setLength(gpsMaxSpeedVertical);//constrain to max
        ins->getQuaternionRotation().rotate(velLocalDesired);//rotate to Global
        // Vec3 velGlobalDes = velLocalDesired;

        switch(flightMode) {
            case FlightMode::wayPoint: {
                Vec3 vecToPoint = ins->getLocation() - wayPoint;
                // if(vecToPoint.getLength() < 2) { //inside a radius of 2 meters
                //     wayPointReached = true;
                // } else {
                    // face waypoint
                    double yaw = ins->getYaw();
                    double desYaw = atan2(vecToPoint.x, vecToPoint.y);
                    desYawRate = yaw - desYaw;
                    desYawRate = ((int) desYawRate + 180) % 360 - 180;

                    //adjust altitude
                    double maxWaypointClimbRate = 0.3;
                    climbRate = max(min(vecToPoint.z, maxWaypointClimbRate), -maxWaypointClimbRate);
                    if(vecToPoint.z < 2 && desYawRate < 20) { // above 2 meters under target && less 20 20 deg yaw error
                        desPitchAngle = min(vecToPoint.getLength(), 20); // deg
                    } else {
                        desPitchAngle = 0;
                    }
                    desRollAngle = 0;
                    desYawRate /= 3;
                // }
            }
            case FlightMode::gpsHold: {
                // Vec3 axisPitch = Vec3();
                // axisPitch.x = velPIDx.compute(ins->getVelocity().x, velGlobalDes.x);
                // axisPitch.y = velPIDy.compute(ins->getVelocity().y, velGlobalDes.y);
                
                // ins->getQuaternionRotation().rotateReverse(axisPitch);//rotate to local
                // desRollAngle = axisPitch.y;
                // desPitchAngle = axisPitch.x;
            }
            case FlightMode::altitudeHold: {
                // if(!autoLiftoff && throttle < 0.9) climbRate = 0;
                // if(throttle > 0.9) autoLiftoff = true;
                // if(ins->sensors->ultrasonic.connected) {
                //     Ultrasonic ultrasonic = ins->sensors->ultrasonic;
                //     double height = ultrasonic.distance;
                //     if(height > 0.5 && airborne && micros() - airBornTime > 1000) launched = true;
                //     double minHeight = 0.5;
                //     if(height < minHeight && chanels.throttle > 0.1 && launched) { // landing
                //         climbRate = 0.5 - min(0, ultrasonic.speed) * 1;
                //     }
                //     if(height < minHeight && chanels.throttle < 0.1) {
                //         climbRate = -(height / minHeight) / 5 - min(0, ultrasonic.speed);
                //     } else if(!ultrasonic.outOfRange && ultrasonic.speed < 0) {
                //         climbRate = max(climbRate, -(height - minHeight) / 3 * 0.5 + abs(ultrasonic.speed) * 1);
                //     }
                //     if(!launched && height < 0.5) {
                //         climbRate = chanels.throttle > 0.9 ? 0.2 : -0.2;
                //     }
                // }
                throttle = altitudePID.compute((float) ins->getVelocity().z, climbRate);
                if(ins->sensors->ultrasonic.connected && !launched && chanels.throttle < 0.9) {
                    throttle = 0;
                }
            }
            case FlightMode::level: {
                rollRateAdjust = levelRollPID.compute(-ins->getRoll() * RAD_TO_DEG, desRollAngle, ins->getRollRate());
                pitchRateAdjust = levelPitchPID.compute(-ins->getPitch() * RAD_TO_DEG, desPitchAngle, ins->getPitchRate());
                yawRateAdjust = rateYawPID.compute(ins->getYawRate(), desYawRate);
                break;
            }
            case FlightMode::rate: {
                float desRollRate  = stickToRate(chanels.roll,  rollRate.getRC(),  rollRate.getSuper(),  rollRate.getRCExpo());
                float desPitchRate = stickToRate(chanels.pitch, pitchRate.getRC(), pitchRate.getSuper(), pitchRate.getRCExpo());
                rollRateAdjust  = rateRollPID.compute (ins->getRollRate(),  desRollRate);
                pitchRateAdjust = ratePitchPID.compute(ins->getPitchRate(), desPitchRate);
                yawRateAdjust   = rateYawPID.compute  (ins->getYawRate(),   desYawRate);
                break;
            }
            default: {}
        }
        controllMotors(throttle, rollRateAdjust, pitchRateAdjust, yawRateAdjust);

        mFL->handle();
        mFR->handle();
        mBL->handle();
        mBR->handle();
    }

    double getThrottleMul() {

        if(ins->sensors->bat.cellCount < 6) {
            return throttleMul4S;
        } else {
            return throttleMul6S;
        }
    }

    // Motor from 1 to 4
    void setMotorPin(int motor, int pin) {
        switch(motor) {
            case 1: mFL->setPin(pin); break;
            case 2: mFR->setPin(pin); break;
            case 3: mBL->setPin(pin); break;
            case 4: mBR->setPin(pin); break;
        }
    }

    // Motor from 1 to 4
    int getMotorPin(int motor) {
        switch(motor) {
            case 1: return mFL->getPin();
            case 2: return mFR->getPin();
            case 3: return mBL->getPin();
            case 4: return mBR->getPin();
            default: return -1;
        }
    }

    void arm() {
        armTime = millis();
        reset();
        mFL->arm();
        mFR->arm();
        mBL->arm();
        mBR->arm();
        armed = true;
        autoLiftoff = false;
        launched = false;
    };

    void disarm() {
        if(isArmed()) {
            mFL->disarm();
            mFR->disarm();
            mBL->disarm();
            mBR->disarm();
            armed = false;
            autoLiftoff = false;
        }
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
    bool launched = false;

    bool autoLiftoff = false;
    uint64_t autoLiftoffTime = 0;

    bool airborne = false;
    bool armed = false;

    uint64_t armTime = 0;
    uint64_t airBornTime = 0;
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

    float maxRateChange = 0.5f; // maximum percentage of thrust that is beeing used for pitch / yaw / roll

    void controllAngle(float &rollRateAdjust, float &pitchRateAdjust, float &yawRateAdjust, float desRollAngle, float desPitchAngle, float desYawRate) {
        float roll = (-ins->getRoll() * 4068) / 71;
        float pitch = (-ins->getPitch() * 4068) / 71;
        rollRateAdjust = levelRollPID.compute(roll, desRollAngle, ins->getRollRate());
        pitchRateAdjust = levelPitchPID.compute(pitch, desPitchAngle, ins->getPitchRate());
        yawRateAdjust = rateYawPID.compute(ins->getYawRate(), desYawRate);
    }

    void crop(float& val, float lim) {
        crop(val, -lim, lim);
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
         if(chanels.aux2 > 0.75) {
            flightMode = FlightMode::altitudeHold;
        } else if(chanels.aux2 > -0.3){
            flightMode = FlightMode::level;
        } else {
            flightMode = FlightMode::rate;
        }
        if(chanels.aux4 > -0.8) {
            flightMode = FlightMode::wayPoint;
        }

        /**
         * Never use flight mode with unsufficient sensors
         */
        // flightMode = FlightMode::FlightMode_t(min(flightMode, ins->sensors->getHighestFM(Error::WARNING)));

        /**
         * Overwrite All
         */
        if(overwriteFlightMode != FlightMode::none) {
            flightMode = overwriteFlightMode;
        }
        if(flightMode != lastFlightMode) {
            initFlightMode(flightMode);
        }
        lastFlightMode = flightMode;
    }

    void handleWaypoint() {
        if(flightMode != FlightMode::wayPoint) return;
        if(wayPointReached) {
            wayPoint = Vec3(0,0,-1000);
        }
        if(ins->getGForce() > 3) { // disarm on crash
            disarm();
        }
    }

    void initFlightMode(FlightMode::FlightMode_t fm) {
        if(fm >= FlightMode::altitudeHold) {
            if(altitudePID.integrator == 0 && airborne) {
                altitudePID.integrator = hoverThrottle;
            }
        }
        if(fm == FlightMode::wayPoint) {
            wayPoint = Vec3(0,0,max(50, ins->getLocation().z + 20));
        }
    }

    bool rcWasDisarmed = false;

    /**
     * Only Arm when rc was connected for more than 1 second
     * && the rc send a disarm command before
     * && maximum roll, pitch angle is less than 5 deg
     */
    void handleArm() {
        if(!armed) {
            if(crsf->isRcConnected() && crsf->timeSinceRcConnect() > 1000) {
                if(!rcWasDisarmed && chanels.aux1 <= 0.9) {
                    rcWasDisarmed = true;
                } else if(rcWasDisarmed && chanels.aux1 > 0.9) {
                    if(!ins->isAngleSmallerThanDeg(5)) {
                        rcWasDisarmed = false;
                    }
                    if(chanels.throttle > 0.1) {
                        rcWasDisarmed = false;
                    }
                    if(rcWasDisarmed) {
                        rcWasDisarmed = false;
                        arm();
                    }
                }
            }
        }

        // if(ins->getGForce() < 0.4 && chanels.aux1 <= 0.9) {
        //     arm();
        // }

        if(isArmed() && chanels.aux1 < 0.9) {
            disarm();
        }
    }

    void controllMotors(float throttle, float rollRateAdjust, float pitchRateAdjust, float yawRateAdjust) {
        crop(yawRateAdjust, maxRateChange);
        crop(rollRateAdjust, maxRateChange);
        crop(pitchRateAdjust, maxRateChange);

        // Mixer
        float mFLThrottle = throttle + rollRateAdjust - pitchRateAdjust + (propsIn ? +yawRateAdjust : -yawRateAdjust);
        float mFRThrottle = throttle - rollRateAdjust - pitchRateAdjust + (propsIn ? -yawRateAdjust : +yawRateAdjust);
        float mBLThrottle = throttle + rollRateAdjust + pitchRateAdjust + (propsIn ? -yawRateAdjust : +yawRateAdjust);
        float mBRThrottle = throttle - rollRateAdjust + pitchRateAdjust + (propsIn ? +yawRateAdjust : -yawRateAdjust);

        // Keeping controll authority
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

        // Write
        double throttleMul = getThrottleMul();
        mFL->write(mFLThrottle * throttleMul);
        mFR->write(mFRThrottle * throttleMul);
        mBL->write(mBLThrottle * throttleMul);
        mBR->write(mBRThrottle * throttleMul);
    }

    uint32_t lastRollIReset = 0;
    uint32_t lastPitchIReset = 0;
    uint32_t lastYawIReset = 0;

    void handleI() {
        bool autoFm = flightMode >= FlightMode::altitudeHold;
        if(armed && chanels.throttle > (autoFm ? 0.5 : 0.3)) {
            if(!airborne) {
                airBornTime = micros();
            }
            airborne = true;
        }
        if(!armed) {
            airborne = false;
        }
        // airborne = true;
        //relax
        rateRollPID.lockI   = abs(ins->getRollRate()) > iRelaxMinRate;
        ratePitchPID.lockI  = abs(ins->getPitchRate()) > iRelaxMinRate;
        rateYawPID.lockI    = abs(ins->getYawRate()) > iRelaxMinRate;

        levelRollPID.lockI   = abs(ins->getRollRate()) > iRelaxMinRate;
        levelPitchPID.lockI  = abs(ins->getPitchRate()) > iRelaxMinRate;
        levelYawPID.lockI    = abs(ins->getYawRate()) > iRelaxMinRate;

        rateRollPID.iEnabled  = airborne;
        ratePitchPID.iEnabled = airborne;
        rateYawPID.iEnabled   = airborne;
        levelRollPID.iEnabled = airborne;
        levelPitchPID.iEnabled= airborne;
        levelYawPID.iEnabled  = airborne;
        altitudePID.iEnabled  = airborne;
        if(airborne && micros() - airBornTime > launchIBoostSeconds * 1000000) { // 5 Seconds after arm
            levelRollPID.iMul = launchIBoostLevel;
            levelPitchPID.iMul = launchIBoostLevel;
            levelYawPID.iMul = launchIBoostLevel;

            altitudePID.iMul = launchIBoostAltitude;
            if(altitudePID.integrator >= hoverThrottle) {
                altitudePID.iMul = 1;
            }
        } else {
            levelRollPID.iMul = 1;
            levelPitchPID.iMul = 1;
            levelYawPID.iMul = 1;

            altitudePID.iMul = 1;
        }
        if(!airborne) {
            altitudePID.pMul = 0;
            altitudePID.iMul = 0;
            altitudePID.dMul = 0;
        } else {
            altitudePID.pMul = 1;
            altitudePID.iMul = 1;
            altitudePID.dMul = 1;
        }
    }

    void reset() {
        rateRollPID.reset();
        ratePitchPID.reset();
        rateYawPID.reset();
        levelRollPID.reset();
        levelPitchPID.reset();
        levelYawPID.reset();
        altitudePID.reset();
        velPIDx.reset();
        velPIDy.reset();
        ins->resetAltitude();
    }

    float getMaxAbsAttitudeChanel() {
        return max(abs(chanels.roll), max(abs(chanels.pitch), abs(chanels.yaw)));
    }

    // void handleAntiGravity() {
    //     if(!useAntiGravity) return;
    //     iBoost = abs(chanels.throttle - lastThrottle) * boostSpeed;
    //     if(iBoost < lastBoost) {
    //         iBoost = boostLpf * iBoost + (1 - boostLpf) * lastBoost;
    //     }
    //     if(iBoost > 1) iBoost = 1;
    //     if(iBoost < 0) iBoost = 0;
    //     rateRollPID.iBoost  = 1 + (antiGravityMul - 1) * iBoost;
    //     ratePitchPID.iBoost = 1 + (antiGravityMul - 1) * iBoost;
    //     rateYawPID.iBoost   = 1 + (antiGravityMul - 1) * iBoost;
    //     lastThrottle = chanels.throttle;
    //     lastBoost = iBoost;
    // }

    void handleStatistics() {
        // if(chanels.aux3 > 0.5) {
        //     maxGForce = 0;
        // }
        gForce = ins->getGForce();
        maxGForce = max(maxGForce, gForce);
    }

    double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
};