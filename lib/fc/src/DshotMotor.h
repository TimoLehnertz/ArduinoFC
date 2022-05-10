#pragma once
#include <Arduino.h>
#include <DShot.h>
#include <Motor.h>
class DShotMotor : public Motor, DShot {
public:
    DShotMotor() {}

    void begin(uint8_t pin) {
        DShot::begin(pin);
    }

    void handle() {}

    void end() {
        DShot::end();
    }

    void arm() {
        DShot::arm();
    }

    void disarm() {
        DShot::disarm();
    }

    bool isArmed() {
        return DShot::isArmed();
    }

    void writeRaw(float percentage) {
        DShot::writeThrottle(percentage);
    }

    void sendComand(uint8_t cmd, uint8_t cycles) {
        DShot::sendCommand(cmd, cycles);
    }

    u_int8_t getPin() {
        return DShot::getPin();
    }

    void setPin(uint8_t pin) {
        DShot::setPin(pin);
    }

private:
    uint8_t pin;
};