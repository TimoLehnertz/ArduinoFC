#include <Arduino.h>
#include "DShot.h"

uint8_t DShot::dShotCount = 0;
volatile uint8_t DShot::phase = 0;
DShot* DShot::dShots[MAX_DSHOT_COUNT];
IntervalTimer DShot::timer;

bool DShot::begin(uint8_t pin) {
    if(DShot::dShotCount >= MAX_DSHOT_COUNT) return false;
    this->pin = pin;
    pinMode(pin, OUTPUT);

    DShot::dShots[DShot::dShotCount] = this;
    DShot::dShotCount++;
    if(DShot::dShotCount == 1) { // first DShot
        DShot::timer.begin(DShot::timerFunc, 1);
    }
    begun = true;
    return true;
}

void DShot::end() {}

void DShot::writeThrottle(float throttle) {
    if(!armed) return;
    if(throttle > 1) throttle = 1;
    if(throttle < 0) throttle = 0;
    this->throttle = (uint16_t) map(throttle, 0.0, 1.0, 48.0, 2047.0);
    // if(millis() % 100 == 0) {
    //     Serial.println(this->throttle);
    // }
    calcPayload();
}

void DShot::sendCommand(uint8_t cmd, uint8_t cycles) {
    this->cmd = cmd;
    this->cmdCycles = cycles;
    calcPayload();
}

void DShot::arm() {
    sendCommand(DIGITAL_CMD_MOTOR_STOP);
    armed = true;
}

void DShot::disarm() {
    throttle = DIGITAL_CMD_MOTOR_STOP;
    calcPayload();
    armed = false;
}

void DShot::calcPayload() {
    if(cmdCycles > 0) {
        payload = cmd << 1;
    } else {
        payload = throttle << 1;
    }
    if(false) { // telemetry request
        payload |= 1;
    }
    uint16_t crc = (payload ^ (payload >> 4) ^ (payload >> 8)) & 0x0F;
    payload <<= 4;
    payload |= MASK_CRC & crc;
}

void DShot::timerFunc() {
    for (size_t i = 0; i < dShotCount; i++) {
        const DShot* dShot = dShots[i];
        if(phase < 3 * 16) {
            uint8_t subPhase = phase % 3;
            bool bit = (dShot->payload) & (MASK_MSB >> (phase / 3));
            if(subPhase == 0) digitalWrite(dShot->pin, HIGH);
            if(subPhase == 1) digitalWrite(dShot->pin, bit);
            if(subPhase == 2) digitalWrite(dShot->pin, LOW);
        }
    }
    phase++;
    if(phase == 3 * 16 * 3) {
        phase = 0;
        for (size_t i = 0; i < dShotCount; i++) {
            DShot* dShot = dShots[i];
            if(dShot->cmdCycles == 0) continue;
            dShot->cmdCycles--;
            if(dShot->cmdCycles == 0) dShot->calcPayload();
        }
    }
}

void DShot::setPin(uint8_t pin) {
    this->pin = pin;
    pinMode(pin, OUTPUT);
}