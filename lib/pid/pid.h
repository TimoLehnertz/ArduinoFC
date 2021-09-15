#pragma once

#include <Arduino.h>

class PID {
public:

    /**
     * Storage start
     */
    float p;
    float i;
    float d;
    float dlpf;

    float iMul = 0.1f;
    float iMaxBuildup = 1.0f;
    float iMaxOut = 0.1f;

    float dMax = 0.2;
    /**
     * Storage end
     */

    bool lockI = false;

    float lastP = 0;
    float lastI = 0;
    float lastD = 0;

    float lastDLpf = 0.0f;

    bool iEnabled = true;

    float iBoost = 1.0f;

    float integrator = 0;
    float prevError = 0;
    float prevSetpoint = 0;
    float prevMeasurement = 0;
    uint64_t prevTime = 0;
    float differentiator = 0;
    float derivative = 0;

    PID() : p(0), i(0), d(0) {}

    PID(char* str) {
        if(strlen(str) < 7) return;
        int delim1 = strpos2(str, ',');
        str[delim1] = 0;
        int delim2 = strpos2(str, ',', delim1 + 1);
        str[delim2] = 0;
        int delim3 = strpos2(str, ',', delim2 + 1);
        str[delim3] = 0;
        int delim4 = strpos2(str, ',', delim3 + 1);
        str[delim4] = 0;
        int delim5 = strpos2(str, ',', delim4 + 1);
        str[delim5] = 0;
        int delim6 = strpos2(str, ',', delim5 + 1);
        str[delim6] = 0;
        int delim7 = strpos2(str, ',', delim6 + 1);
        str[delim7] = 0;
        int delim8 = strpos2(str, ',', delim7 + 1);
        str[delim8] = 0;

        if(delim7 == -2) {
            Serial.println("PID String was formatted wrongly");
        } else {
            p           = atof(str + delim1 + 1);
            i           = atof(str + delim2 + 1);
            d           = atof(str + delim3 + 1);
            dlpf        = atof(str + delim4 + 1);
            iMul        = atof(str + delim5 + 1);
            iMaxBuildup = atof(str + delim6 + 1);
            iMaxOut     = atof(str + delim7 + 1);
            dMax        = atof(str + delim8 + 1);
        }
    }

    PID(float p, float i, float d) : p(p), i(i), d(d), dlpf(1.0f) {}
    PID(float p, float i, float d, float dlpf) : p(p), i(i), d(d), dlpf(dlpf) {}
    PID(float p, float i, float d, float dlpf, float iMaxBuildup, float iMaxOut, float dMax) : p(p), i(i), d(d), dlpf(dlpf), iMaxBuildup(iMaxBuildup), iMaxOut(iMaxOut), dMax(dMax) {}

    float compute(float measurement, float setpoint) {
        return compute(measurement, setpoint, -1000000.0f);
    }

    float compute(float measurement, float setpoint, float gyro) {
        uint64_t now = micros();
        if(dlpf > 1.0f) dlpf = 1.0f;
        if(dlpf < 0.0f) dlpf = 0.0f;
        float t = ((double) (now - prevTime)) / 1000000.0f;
        /**
         *  Error
         */
        float error = setpoint - measurement;
        float proportional = p * error;
        /**
         *  Integral
         */
        if(!lockI) {
            integrator = integrator + error * t * iBoost;
        }
        // integrator = integrator + 0.5f * t * (error + prevError);

        /**
         * @ToDo dynamic integrator clamping
         */
        float iLimBuildupCalc = (iMaxBuildup * iBoost) / (i * iMul);
        if(integrator > iLimBuildupCalc) integrator = iLimBuildupCalc;
        if(integrator < -iLimBuildupCalc) integrator = -iLimBuildupCalc;

        if(!iEnabled) integrator = 0;

        float iOut = i * iMul * integrator;
        if(iOut > iMaxOut) iOut = iMaxOut;
        if(iOut < -iMaxOut) iOut = -iMaxOut;

        /**
         * derivative
         * 
         */
        float derivative;
        if(gyro == -1000000.0f) {
            derivative = ((measurement - setpoint) - (prevMeasurement - prevSetpoint)) * dlpf + lastDLpf * (1.0f - dlpf);
            // Serial.println(derivative);
        } else {
            derivative = gyro * dlpf + lastDLpf * (1 - dlpf);
        }
        lastDLpf = derivative;
        derivative *= d;
        if(derivative > dMax) derivative = dMax;
        if(derivative < -dMax) derivative = -dMax;

        float out = proportional + iOut - derivative;

        lastP = proportional;
        lastI = iOut;
        lastD = derivative;

        prevError = error;
        prevMeasurement = measurement;
        prevTime = now;
        prevSetpoint = setpoint;

        return out;
    }

    void reset() {
        integrator = 0;
        prevError = 0;
        lastD = 0;
    }

    void resetI() {
        integrator = 0;
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