#pragma once

#include <Arduino.h>

class PID {
public:

    float p;
    float i;
    float d;

    float integrator = 0;
    float prevError = 0;
    float prevMeasurement = 0;
    uint64_t prevTime = 0;
    float differentiator = 0;

    float i_limit = 20.0;

    PID(float p, float i, float d) : p(p), i(i), d(d) {}

    float compute(float measurement, float setpoint, float gyro) {
        uint64_t now = micros();
        float t = ((double) (now - prevTime)) / 1000000.0f;
        /**
         *  Error
         */
        float error = setpoint - measurement;
        /**
         *  Integral
         */
        // integrator = integrator + 0.5f * i * t * (error + prevError);
        integrator = integrator + error * t;

        if(integrator > i_limit) integrator = i_limit;
        if(integrator < -i_limit) integrator = -i_limit;


        float derivative = gyro;

        float out = (p * error + (i * integrator) - (d * derivative));

        prevError = error;
        prevMeasurement = measurement;
        prevTime = now;
        return out;
    }

    void reset() {
        integrator = 0;
        prevError = 0;
    }
};