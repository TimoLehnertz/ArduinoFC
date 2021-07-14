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
    uint32_t prevTime = 0;
    float differentiator = 0;

    float tau = 0.02f;

    float limMax = 360;
    float limMin = -360;
    float i_limit = 25.0;

    PID(float p, float i, float d) : p(p), i(i), d(d) {}

    float compute(float measurement, float setpoint, float gyro, bool useIntegrator = true, bool derivativeYaw = false) {
        uint32_t now = micros();
        double t = ((double) (now - prevTime)) / 1000000.0;
        /**
         *  Error
         */
        float error = setpoint - measurement;
        /**
         *  Proportional
         */
        float proportional = p * error;
        /**
         *  Integral
         */
        // integrator = integrator + 0.5f * i * t * (error + prevError);
        integrator = integrator + error * t;
        if(!useIntegrator) {
            integrator = 0;
        }
        if(integrator > i_limit) integrator = i_limit;
        if(integrator < -i_limit) integrator = -i_limit;


        float derivative = gyro;
        if(derivativeYaw) {
            derivative = (error - prevError) / t;
        }

        float out = 0.01*(proportional + i*integrator - d*derivative);

        prevError = error;
        prevMeasurement = measurement;
        prevTime = now;
        return out;
    }

    // float compute(float measurement, float setpoint) {
    //     uint32_t now = micros();
    //     uint32_t t = (now - prevTime) / 1000000;
    //     /**
    //      *  Error
    //      */
    //     float error = setpoint - measurement;
    //     /**
    //      *  Proportional
    //      */
    //     float proportional = p * error;
    //     /**
    //      *  Integral
    //      */
    //     integrator = integrator + 0.5f * i * t * (error + prevError);

    //     // Anti-wind-up via dynamic integrator clamping
    //     float limMinInt, limMaxInt;
        
    //     // Compute integrator limits
    //     if(limMax > proportional) {
    //         limMaxInt = limMax - proportional;
    //     } else {
    //         limMaxInt = 0.0f;
    //     }
    //     if(limMin < proportional) {
    //         limMinInt = limMin - proportional;
    //     } else {
    //         limMinInt = 0.0f;
    //     }

    //     // clamp integrator
    //     if(integrator > limMaxInt) {
    //         integrator = limMaxInt;
    //     }
    //     if(integrator < limMinInt) {
    //         integrator = limMinInt;
    //     }

    //     /**
    //      * Derivative (band-limited difference)
    //      */
    //     differentiator = -(2.0f * d * (measurement - prevMeasurement)	/* Note: derivative on measurement, therefore minus sign in front of equation! */
    //                     + (2.0f * tau - t) * differentiator)
    //                     / (2.0f * tau + t);

    //     /**
    //      *  Compute output and apply limits
    //      */
    //     float out = proportional + integrator + differentiator;
    //     if(out > limMax) {
    //         out = limMax;
    //     }
    //     if(out < limMin) {
    //         out = limMin;
    //     }

    //     prevError = error;
    //     prevMeasurement = measurement;
    //     prevTime = now;

    //     return out;
    // }
};