#include <Arduino.h>
#include "Motor.h"
// #include "utils.h"

void Motor::begin() {
    switch(protocol) {
    case PWM: beginPWM(); break;
    }
}

void Motor::end() {
    switch(protocol) {
    case PWM: endPWM(); break;
    }
}

void Motor::beginPWM() {
    servo.attach(pin, minPWM, maxPWM);
    servo.write(0);
}

void Motor::endPWM() {
    servo.detach();
}

void Motor::arm() {
    if(armed || firstWriteAfterArm) return;
    armTime = millis();
    firstWriteAfterArm = true;
    switch(protocol) {
    case PWM: armPWM(); break;
    }
}

void Motor::armPWM() {
    servo.write(0);
    // pwmArmingDone = false;
    armed = true;
    // delay(10);
}

void Motor::disarm() {
    // Serial.println("disarming");
    servo.write(0);
    armed = false;
}

void Motor::writeRaw(float percentage) {
    switch(protocol) {
    case PWM: servo.write(map(percentage, 0, 1, minPWM, maxPWM)); break;
    }
}

void Motor::write(float percentage) {
    if(!armed) return;
    if(firstWriteAfterArm && percentage > 0.5) return; //enshuring no motor is started with full throttle
    if(percentage < 0) percentage = 0;
    if(percentage > 1) percentage = 1;
    switch(protocol) {
    case PWM: writePWM(percentage); break;
    }
    firstWriteAfterArm = false;
}

void Motor::writePWM(float percentage) {
    percentage = map(percentage, 0, 1, minThrottle, maxThrottle);
    servo.write(map(percentage, 0, 1, minPWM, maxPWM));
}

void Motor::handle() {
    switch(protocol) {
    case PWM: handlePWM(); break;
    }
}

void Motor::handlePWM() {
    // arming
    if(millis() - armTime > 1000 && !pwmArmingDone) {
        pwmArmingDone = true;
        armed = true;
        write(0);
    }
}

bool Motor::calibrate() {
    if(isArmed() || protocol != PWM) {
        return false;
    }
    write(1);
    delay(100);
    return true;
}

/**
 * Getters / Setters
 */
void Motor::setMinThrottle(float min) {
    minThrottle = min;
}
void Motor::setMaxThrottle(float max) {
    maxThrottle = max;
}

void Motor::setMinPWM(float min) {
    minPWM = min;
}
void Motor::setMaxPWM(float max) {
    maxPWM = max;
}

bool Motor::isArmed() {
    return armed;
}

double Motor::map(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}