/**
 * @file Motor.h
 * @author Timo Lehnertz
 * @brief 
 * @version 0.1
 * @date 2022-01-01
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once
#include <Arduino.h>
// #include "../../Servo/src/Servo.h"

enum MotorProtocol {
    PWM,
    ONESHOT125,
};

class Motor {
public:

    int pin;

    Motor(int pin) : pin(pin) {};

    virtual void begin() = 0;
    virtual void handle() = 0;
    virtual void end() = 0;

    virtual void arm() = 0;
    virtual void disarm() = 0;

    virtual void writeRaw(float percentage) = 0; //write a percentage from 0 to 1

    void write(float percentage) {
        if(!armed) return;
        if(percentage < 0) percentage = 0;
        if(percentage > 1) percentage = 1;
        writeRaw(map(percentage, 0.0f, 1.0f, minThrottle, maxThrottle));
    }

    virtual bool isArmed() = 0;

    void setMinThrottle(float min) { minThrottle = min; };
    void setMaxThrottle(float max) { maxThrottle = max; };

    int getPin() { return pin; }
    virtual void setPin(int pin) = 0;

protected:
    bool armed;

    float minThrottle = 0.06; //mapping values
    float maxThrottle = 1;

    double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
};