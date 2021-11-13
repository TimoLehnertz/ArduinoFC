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

    float minOut = -100000000;

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

    PID();

    PID(char* str);

    PID(float p, float i, float d);
    PID(float p, float i, float d, float dlpf);
    PID(float p, float i, float d, float dlpf, float maxOut);

    ~PID();

    float compute(float measurement, float setpoint);
    float compute(float measurement, float setpoint, float gyro);

    void reset();

    static void updateAux(float aux1, float aux2, float aux3);

private:
    /**
     * Aux tuning
     */
    static constexpr int maxPids = 100;
    static PID* registeredPIDs[maxPids];
    static size_t registeredCount;
    static bool registryFull;

    static void registerPID(PID* pid);
    static void unregisterPID(PID* pid);

    int strpos2(const char* haystack, const char needle, int start = 0) {
        if(start == -2) return -2;
        for(int i = start; i < 100; i++) {
            if(haystack[i] == 0) return -2;
            if(haystack[i] == needle) return i;
        }
        return -1;
    }
};