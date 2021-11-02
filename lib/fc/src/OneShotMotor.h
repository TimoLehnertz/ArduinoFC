#include <Arduino.h>
#include <Motor.h>
#include "../../Servo/src/Servo.h"

#define ONE_SHOT_MIN_US 125
#define ONE_SHOT_MAX_US 250

class OneShotTimer {
public:
    IntervalTimer timer;
    int pin;
    volatile float speed;
    int chanel;
    volatile int phase;

    OneShotTimer(int pin, int chanel) : pin(pin), chanel(chanel), phase(0) {}
};

class OneShotMotor;

void timerFunc(OneShotTimer* timer);
void updateOneShot(int chanel, float speed);
int registerOneShotMotor(OneShotMotor* m);

class OneShotMotor : public Motor {
public:
    OneShotMotor(int pin) : Motor(pin) {};

    void begin() {
        oneShotChanel = registerOneShotMotor(this);
        pinMode(pin, OUTPUT);
    }

    void end() {}

    void handle() {
        if(!armed && millis() - armStart > 1000) {
            armed = true;
            // Serial.println("arming");
        }
    }

    void writeRaw(float percentage) {
        updateOneShot(oneShotChanel, percentage);
    }

    void arm() {
        writeRaw(-.2);
        armStart = millis();
    }

    void disarm() {
        writeRaw(0.00);
        armed = false;
    }

    volatile float speed = 0.0f;
    float lastSpeed = 0.0f;

private:
    Servo servo;

    int minPWM = 1000;
    int maxPWM = 2000;

    uint64_t armStart = 0;

    int oneShotChanel = 0;
};

OneShotTimer* timers[4];
size_t timerCount = 0;

void beginOneShot() {
    pinMode(21, OUTPUT);
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
    OneShotTimer* timer = new OneShotTimer(m->pin, timerCount);

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
    noInterrupts();
    timers[chanel]->speed = speed;
    interrupts();
}