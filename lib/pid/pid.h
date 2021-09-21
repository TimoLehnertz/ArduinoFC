#pragma once

#include <Arduino.h>

class PID {
public:

    /**
     * Storage start
     */
    float p = 0;
    float i = 0;
    float d = 0;
    float dlpf = 1.0f;
    float maxOut = 1;
    bool useAuxTuning = false;
    /**
     * Storage end
     */

    float pMul = 1.0f;
    float iMul = 1.0f;
    float dMul = 1.0f;

    bool lockI = false;


    bool iEnabled = true;

    float iBoost = 1.0f;

    float integrator = 0;
    float prevP = 0.0f;
    float prevD = 0.0f;
    float prevMeasurement = 0;
    uint64_t prevTime = 0;
    float differentiator = 0;
    float derivative = 0;

    /**
     * Debug values
     */
    float prevI = 0;
    float prevOut = 0;

    PID() : p(0), i(0), d(0), dlpf(0), maxOut(0), useAuxTuning(false) {}

    PID(char* str) {
        if(strlen(str) < 7) return;
        int delims[6];
        delims[0] = 0;
        bool succsess = true;
        for (size_t i = 0; i < 6; i++) {
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
    }

    PID(float p, float i, float d) : p(p), i(i), d(d), dlpf(1.0f), maxOut(1.0), useAuxTuning(false) {}
    PID(float p, float i, float d, float dlpf) : p(p), i(i), d(d), dlpf(dlpf), maxOut(1.0), useAuxTuning(false) {}
    PID(float p, float i, float d, float dlpf, float maxOut) : p(p), i(i), d(d), dlpf(dlpf), maxOut(maxOut), useAuxTuning(false) {}

    float compute(float measurement, float setpoint) {
        return compute(measurement, setpoint, -1000000.0f);
    }

    float compute(float measurement, float setpoint, float gyro) {
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
        if(integrator < -maxOut) integrator = -maxOut;

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
        if(out < -maxOut) out = -maxOut;

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

    void reset() {
        integrator = 0;
        prevD = 0;
    }

private:

    int strpos2(const char* haystack, const char needle, int start = 0) {
        if(start == -2) return -2;
        for(int i = start; i < 100; i++) {
            if(haystack[i] == 0) return -2;
            if(haystack[i] == needle) return i;
        }
        return -1;
    }
};