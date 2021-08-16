#pragma once
#include <Arduino.h>
#include <Servo.h>

enum MotorProtocol {
    PWM
};

class Motor {
public:

    Motor(int pin, MotorProtocol protocol = PWM) : pin(pin), protocol(protocol) {};

    void begin();
    void handle();
    void end();

    void arm();
    void disarm();

    void write(float percentage); //write a percentage from 0 to 1

    bool calibrate(); //only for pwm

    bool isArmed();

    void setMinThrottle(float min);
    void setMaxThrottle(float min);

    void setMinPWM(float min);
    void setMaxPWM(float min);

private:
    int pin;
    bool armed;
    MotorProtocol protocol;

    float minThrottle = 0.06; //mapping values
    float maxThrottle = 1;

    int minPWM = 1000;
    int maxPWM = 2000;
    long armTime = 0;

    bool pwmArmingDone = false;

    bool firstWriteAfterArm = true;

    Servo servo;

    void beginPWM();
    void endPWM();

    void writePWM(float percentage);
    void armPWM();//write 0 for 1 second then throttle up to minThrottle

    void handlePWM();

    double map(double x, double in_min, double in_max, double out_min, double out_max);
};