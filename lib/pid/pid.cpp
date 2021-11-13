#include <Arduino.h>
#include "pid.h"



PID::PID() : p(0), i(0), d(0), dlpf(0), maxOut(0), useAuxTuning(false) {registerPID(this);}

PID::PID(float p, float i, float d) : p(p), i(i), d(d), dlpf(1.0f), maxOut(1.0), useAuxTuning(false), minOut(-1.0) {registerPID(this);}
PID::PID(float p, float i, float d, float dlpf) : p(p), i(i), d(d), dlpf(dlpf), maxOut(1.0), useAuxTuning(false), minOut(-1.0) {registerPID(this);}
PID::PID(float p, float i, float d, float dlpf, float maxOut) : p(p), i(i), d(d), dlpf(dlpf), maxOut(maxOut), useAuxTuning(false), minOut(-maxOut) { registerPID(this); }

PID::~PID() { unregisterPID(this); }

PID::PID(char* str) {
    if(strlen(str) < 7) return;
    int delims[7];
    delims[0] = 0;
    bool succsess = true;
    for (size_t i = 0; i < 7; i++) {
        int start = i ? delims[i - 1] + 1 : 0;
        delims[i] = strpos2(str, ',', start);
        if(delims[i] < 0) {
            succsess = false;
            break;
        }
        str[delims[i]] = 0;
    }
    if(succsess) {
        p            = atof(str + delims[0] + 1);
        i            = atof(str + delims[1] + 1);
        d            = atof(str + delims[2] + 1);
        dlpf         = atof(str + delims[3] + 1);
        maxOut       = atof(str + delims[4] + 1);
        useAuxTuning = atof(str + delims[5] + 1) > 0;
    } else {
        Serial.println("PID String was formatted wrongly");
    }
    registerPID(this);
}

float PID::compute(float measurement, float setpoint) {
    return compute(measurement, setpoint, -1000000.0f);
}

float PID::compute(float measurement, float setpoint, float gyro) {
    uint64_t now = micros();
    if(dlpf > 1.0f) dlpf = 1.0f;
    if(dlpf < 0.0f) dlpf = 0.0f;
    float t = ((double) (now - prevTime)) / 1000000.0f;

    float p = this->p * pMul;
    float i = this->i * iMul;
    float d = this->d * dMul;
    
    /**
     *  Error
     */
    float error = setpoint - measurement;
    
    /**
     *  Integral
     */
    if(!lockI) {
        integrator += i * iBoost * error * t;
    }

    if(!iEnabled) integrator = 0;

    if(integrator > maxOut) integrator = maxOut;
    if(integrator < minOut) integrator = minOut;

    /**
     * derivative on measurement
     */
    float derivative;
    if(gyro == -1000000.0f) {
        derivative = (measurement - prevMeasurement) * dlpf + prevD * (1.0f - dlpf);
    } else {
        derivative = gyro * dlpf + prevD * (1 - dlpf);
    }
    derivative *= d;
    // if(derivative > dMax) derivative = dMax;
    // if(derivative < -dMax) derivative = -dMax;

    /**
     * Compute
     */
    float out = p * error + integrator - derivative;
    if(out > maxOut) out = maxOut;
    if(out < minOut) out = minOut;

    /**
     * Remember variables
     */
    prevMeasurement = measurement;
    prevD = derivative;
    prevTime = now;
    // Debug
    prevP = p * error;
    prevI = integrator;
    prevOut = out;

    return out;
}


void PID::reset() {
    integrator = 0;
    prevD = 0;
}

size_t PID::registeredCount = 0;
bool PID::registryFull = false;
PID* PID::registeredPIDs[100];

void PID::registerPID(PID* pid) {
    if(registryFull) {
        Serial.println("PID registry full!");
        return;
    }
    PID::registeredPIDs[registeredCount] = pid;
    registeredCount++;
    registryFull = registeredCount >= maxPids;
}

void PID::unregisterPID(PID* pid) {
    for (size_t i = 0; i < registeredCount; i++) {
        if(registeredPIDs[i] == pid) {
            registeredPIDs[i] = nullptr;
            if(i == registeredCount - 1) registeredCount--;
        }
    }
}

void PID::updateAux(float aux1, float aux2, float aux3) {
    for (size_t i = 0; i < registeredCount; i++) {
        if(registeredPIDs[i] == nullptr || (*(PID::registeredPIDs[i])).useAuxTuning) continue;
        PID pid = *PID::registeredPIDs[i];
        pid.pMul  = map(aux1, -1, 1, 0, 2);
        pid.iMul  = map(aux2, -1, 1, 0, 2);
        pid.dMul  = map(aux3, -1, 1, 0, 2);
    }
}