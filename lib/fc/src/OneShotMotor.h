/**
 * @file OneShotMotor.h
 * @author Timo Lehnertz
 * @brief 
 * @version 0.1
 * @date 2022-01-01
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <Arduino.h>
#include <Motor.h>
// #include "../../Servo/src/Servo.h"

#define ONE_SHOT_MIN_US 125
#define ONE_SHOT_MAX_US 250

class OneShotTimer {
public:
    IntervalTimer timer;
    uint8_t pin;
    volatile float speed;
    int chanel;
    volatile int phase;

    OneShotTimer(uint8_t pin, int chanel) : pin(pin), chanel(chanel), phase(0) {}
};

class OneShotMotor;

void timerFunc(OneShotTimer* timer);
void updateOneShot(int chanel, float speed);
int registerOneShotMotor(OneShotMotor* m);
void changeOneShotPin(int chanel, int pin);

class OneShotMotor : public Motor {
public:
    OneShotMotor() {};

    void begin(uint8_t pin) {
        oneShotChanel = registerOneShotMotor(this);
        pinMode(pin, OUTPUT);
    }



    void handle() {
        // if(!armed && arming && millis() - armStart > 2000) {
        //     armed = true;
        //     arming = false;
        // }
    }

    void end() {}

    void sendComand(uint8_t cmd, uint8_t cycles) {}

    void writeRaw(float percentage) {
        updateOneShot(oneShotChanel, percentage);
    }

    bool isArmed() {
        return armed || arming;
    }

    void arm() {
        writeRaw(0.0);
        armed = true;
        // arming = true;
        // armStart = millis();
    }

    void disarm() {
        writeRaw(0.00);
        armed = false;
        // arming = false;
    }

    // volatile float speed = 0.0f;
    float lastSpeed = 0.0f;

    void setPin(uint8_t pin) {
        if(pin < 2 || pin > 5) {
            pin = 255;
            return;
        }
        digitalWrite(this->pin, LOW);
        pinMode(pin, OUTPUT);
        changeOneShotPin(oneShotChanel, pin);
        this->pin = pin;
    }

    uint8_t getPin() {
        return pin;
    }

private:
    // Servo servo;
    bool armed = false;
    uint8_t pin;
    bool arming = false;

    int minPWM = 1000;
    int maxPWM = 2000;

    uint64_t armStart = 0;

    int oneShotChanel = 0;
};

OneShotTimer* timers[4];
size_t timerCount = 0;

void beginOneShot() {
    // pinMode(21, OUTPUT);
}

void timerFunc1() {
    timerFunc(timers[0]);
}

void timerFunc2() {
    timerFunc(timers[1]);
}

void timerFunc3() {
    timerFunc(timers[2]);
}

void timerFunc4() {
    timerFunc(timers[3]);
}

void timerFunc(OneShotTimer* timer) {
    timer->phase = !timer->phase;
    float time;
    if(timer->phase) {//ON
        time = map(timer->speed, 0.0f, 1.0f, ONE_SHOT_MIN_US, ONE_SHOT_MAX_US);
    } else { //OFF
        time = 100;
    }
    digitalWrite(timer->pin, timer->phase);
    switch(timer->chanel) {
        case 0: timer->timer.begin(timerFunc1, time); break;
        case 1: timer->timer.begin(timerFunc2, time); break;
        case 2: timer->timer.begin(timerFunc3, time); break;
        case 3: timer->timer.begin(timerFunc4, time); break;
    }
}

int registerOneShotMotor(OneShotMotor* m) {
    if(timerCount >= 4) return -1;
    if(timerCount == 0) {
        beginOneShot();
    }
    OneShotTimer* timer = new OneShotTimer(m->getPin(), timerCount);

    timer->speed = 0;
    int initDelay = 10;

    switch(timerCount) {
        case 0: timer->timer.begin(timerFunc1, initDelay + 00); break;
        case 1: timer->timer.begin(timerFunc2, initDelay + 10); break;
        case 2: timer->timer.begin(timerFunc3, initDelay + 20); break;
        case 3: timer->timer.begin(timerFunc4, initDelay + 30); break;
    }

    timers[timerCount] = timer;
    timerCount++;
    return timerCount - 1;
}

void updateOneShot(int chanel, float speed) {
    if(chanel < 0) return;
    noInterrupts();
    timers[chanel]->speed = speed;
    interrupts();
}

void changeOneShotPin(int chanel, int pin) {
    if(chanel < 0) return;
    noInterrupts();
    timers[chanel]->pin = pin;
    interrupts();
}