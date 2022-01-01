/**
 * @file Magdwick.h
 * @author Timo Lehnertz
 * @brief 
 * @version 0.1
 * @date 2022-01-01
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once
#include "SensorFusion.h"
#include <sensorInterface.h>
#include <error.h>

class MagdwickFilter : public SensorFusion {
public:
    MagdwickFilter(SensorInterface* sensors) : SensorFusion(sensors) {}

    /**
     * blocking min 0.5 seconds
     */
    void begin() {
        for (int i = 0; i <= 1000; i++) {
            sensors->handle();
            handle();
            delayMicroseconds(500); // not more than 2Khz
        }
    }

    void handle() {
        //DESCRIPTION: Attitude estimation through sensor fusion - 9DOF
        /*
        * This function fuses the accelerometer gyro, and magnetometer readings AccX, AccY, AccZ, GyroX, GyroY, GyroZ, MagX, MagY, and MagZ for attitude estimation.
        * Don't worry about the math. There is a tunable parameter B_madgwick in the user specified variable section which basically
        * adjusts the weight of gyro data in the state estimate. Higher beta leads to noisier estimate, lower 
        * beta leads to slower to respond estimate. It is currently tuned for 2kHz loop rate. This function updates the roll_IMU,
        * pitch_IMU, and yaw_IMU variables which are in degrees. If magnetometer data is not available, this function calls Madgwick6DOF() instead.
        */
        float dt = (micros() - lastUpdate) / 1000000.0;
        float recipNorm;
        float s0, s1, s2, s3;
        float qDot1, qDot2, qDot3, qDot4;
        float hx, hy;
        float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
        // float mholder;

        float ax = sensors->acc.x;
        float ay = sensors->acc.y;
        float az = sensors->acc.z;

        //Convert gyroscope degrees/sec to radians/sec
        float gx = sensors->gyro.x * 0.0174533f;
        float gy = sensors->gyro.y * 0.0174533f;
        float gz = sensors->gyro.z * 0.0174533f;

        float mx = sensors->mag.x;
        float my = sensors->mag.y;
        float mz = sensors->mag.z;

        //use 6DOF algorithm if MPU6050 is being used
        if(sensors->mag.error == Error::CRITICAL_ERROR) {
            // Madgwick6DOF(gx, gy, gz, ax, ay, az, dt);
            return;
        }
        //Use 6DOF algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
        if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
            // Madgwick6DOF(gx, gy, gz, ax, ay, az, dt);
            return;
        }

        //Rate of change of quaternion from gyroscope
        qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
        qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
        qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
        qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

        //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

            //Normalise accelerometer measurement
            recipNorm = invSqrt(ax * ax + ay * ay + az * az);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            //Normalise magnetometer measurement
            recipNorm = invSqrt(mx * mx + my * my + mz * mz);
            mx *= recipNorm;
            my *= recipNorm;
            mz *= recipNorm;

            //Auxiliary variables to avoid repeated arithmetic
            _2q0mx = 2.0f * q0 * mx;
            _2q0my = 2.0f * q0 * my;
            _2q0mz = 2.0f * q0 * mz;
            _2q1mx = 2.0f * q1 * mx;
            _2q0 = 2.0f * q0;
            _2q1 = 2.0f * q1;
            _2q2 = 2.0f * q2;
            _2q3 = 2.0f * q3;
            _2q0q2 = 2.0f * q0 * q2;
            _2q2q3 = 2.0f * q2 * q3;
            q0q0 = q0 * q0;
            q0q1 = q0 * q1;
            q0q2 = q0 * q2;
            q0q3 = q0 * q3;
            q1q1 = q1 * q1;
            q1q2 = q1 * q2;
            q1q3 = q1 * q3;
            q2q2 = q2 * q2;
            q2q3 = q2 * q3;
            q3q3 = q3 * q3;

            //Reference direction of Earth's magnetic field
            hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
            hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
            _2bx = sqrtf(hx * hx + hy * hy);
            _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
            _4bx = 2.0f * _2bx;
            _4bz = 2.0f * _2bz;

            //Gradient decent algorithm corrective step
            s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
            s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
            s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
            s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
            recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
            s0 *= recipNorm;
            s1 *= recipNorm;
            s2 *= recipNorm;
            s3 *= recipNorm;

            //Apply feedback step
            qDot1 -= B_madgwick * s0;
            qDot2 -= B_madgwick * s1;
            qDot3 -= B_madgwick * s2;
            qDot4 -= B_madgwick * s3;
        }

        //Integrate rate of change of quaternion to yield quaternion
        q0 += qDot1 * dt;
        q1 += qDot2 * dt;
        q2 += qDot3 * dt;
        q3 += qDot4 * dt;

        //Normalise quaternion
        recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;
        
        //compute angles - NWU
        // roll_IMU = atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)*57.29577951; //degrees
        // pitch_IMU = -asin(-2.0f * (q1*q3 - q0*q2))*57.29577951; //degrees
        // yaw_IMU = -atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)*57.29577951; //degrees

        rot.w = q0;
        rot.x = q1;
        rot.y = q2;
        rot.z = q3;

        lastUpdate = micros();
    }

    void reset() {
        
    }

    void resetAltitude() {
        loc.z = 0;
    }

    bool isLocationValid() {
        return sensors->gps.error != Error::CRITICAL_ERROR;
    }

    bool isVelocityValid() {
        return sensors->gps.error != Error::CRITICAL_ERROR;
    }

    bool isHeightValid() {
        return sensors->baro.error != Error::CRITICAL_ERROR;
    }

    void setMagZOffset(double deg) {
        magZOffsetDeg = deg;
    }

    double getMagZOffset() {
        return magZOffsetDeg;
    }


private:
    uint64_t lastUpdate = 0;

    //Filter parameters - Defaults tuned for 2kHz loop rate; Do not touch unless you know what you are doing:
    float B_madgwick = 0.04;  //Madgwick filter parameter
    float B_accel = 0.14;     //Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
    float B_gyro = 0.1;       //Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)
    float B_mag = 1.0;        //Magnetometer LP filter parameter

    float q0 = 1.0f; //initialize quaternion for madgwick filter
    float q1 = 0.0f;
    float q2 = 0.0f;
    float q3 = 0.0f;
    double magZOffsetDeg = 0.0;

    float invSqrt(float x) {
        //Fast inverse sqrt for madgwick filter
        /*
        float halfx = 0.5f * x;
        float y = x;
        long i = *(long*)&y;
        i = 0x5f3759df - (i>>1);
        y = *(float*)&i;
        y = y * (1.5f - (halfx * y * y));
        y = y * (1.5f - (halfx * y * y));
        return y;
        */
        //alternate form:
        // unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
        // float tmp = *(float*)&i;
        // float y = tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
        // return y;
        return 0;
    }
};