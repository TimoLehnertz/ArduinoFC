#pragma once

#include <crossfire.h>
#include <ins.h>

// Super classs for all FC's
class FC {
public:

    INS* ins;

    FC(INS* ins, Motor* motorFL, Motor* motorFR, Motor* motorBL, Motor* motorBR) : ins(ins), mFL(motorFL), mFR(motorFR), mBL(motorBL), mBR(motorBR) {}

    void arm() {
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

    bool isArmed() {
        return mFL->isArmed() || mFR->isArmed() || mBL->isArmed() || mBR->isArmed();
    }

    bool armed = false;

    virtual void begin() = 0;
    void handle() {
        if(chanels.armed3 && !armed) {
            arm();
            armed = true;
            reset();
        } else if(!chanels.armed3){
            disarm();
            armed = false;
        }
        handlePrivate();
        mFL->handle();
        mFR->handle();
        mBL->handle();
        mBR->handle();
    };


    void updateRcChanels(CRSF_TxChanels_Converted& chanels) {
        this->chanels = chanels;
    }

    virtual void reset();

    virtual void startFailsafe() = 0;
    virtual void stopFailsafe() = 0;

protected:
    virtual void handlePrivate() = 0;
    CRSF_TxChanels_Converted chanels;

    Motor* mFL; // front left:  motor1
    Motor* mFR; // front rigt:  motor2
    Motor* mBL; // back left:   motor3
    Motor* mBR; // back right:  motor4
};