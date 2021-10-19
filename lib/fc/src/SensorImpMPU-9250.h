#pragma once
#include <Arduino.h>
#include "sensorInterface.h"
#include <Adafruit_BMP280.h>
#include <TinyGPS++.h>
#include <MPU9250.h>
#include <error.h>
#include <maths.h>

#define G 9.807

/**
 * VCC MPU9250                  : 5V
 * GND MPU9250                  : GND
 * SCL MPU9250                  : Pin 13
 * SDA MPU9250                  : Pin 11
 * AD0 MPU9250                  : Pin 12
 * NCS MPU9250                  : Pin 10
 * 
 * SCL bmp280                   : Pin 16
 * SDA bmp280                   : Pin 17
 * 
 * RX GPS                       : Pin 20
 * TX GPS                       : Pin 21
 * 
 * Bat (voltage divider)        : Pin 22
 **/
class MPU9250Sensor : public SensorInterface {
public:

    // MPU9250_asukiaaa imu;
    Adafruit_BMP280 bmp280;
    TinyGPSPlus gpsSensor;

    float altitudeOffset = 0;
    bool firstHeightMeasured = false;

    int baroHz = 50;
    uint32_t lastBaro = 0;

    MPU9250 mpu9250;

    MPU9250Sensor() : mpu9250(SPI, 10) {}
    
    /**
     * Begin function can becalled as many times as wanted
     * Note: this function cal have a significant delay when waiting for sensors
     */
    void begin() {
        initMPU9250();

        initBmp280();

        initGPS();

        initBattery();
    }

    void initBattery() {
        pinMode(22, INPUT);
    }

    void initGPS() {
        Serial5.begin(9600);
    }

    void initBmp280() {
        // if (!bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID)) {
        Wire1.setClock(500000);
        bmp280 = Adafruit_BMP280(&Wire1);
        bool succsess = bmp280.begin(BMP280_ADDRESS_ALT);
        if (succsess) {
            bmp280.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                            Adafruit_BMP280::SAMPLING_X1,     /* Temp. oversampling */
                            Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                            Adafruit_BMP280::FILTER_X8,       /* Filtering. */
                            Adafruit_BMP280::STANDBY_MS_1);   /* Standby time. */
            baro.error = Error::CRITICAL_ERROR;
        } else {
            Serial.println(F("Could not find a valid BMP280 sensor. Wiring: SCL bmp280: Pin 16, SDA bmp280: Pin 17"));
            baro.error = Error::NO_ERROR;
        }
    }

    void initMPU9250() {
        int status = mpu9250.begin();
        if (status < 0) {
            if(!mpuErrorPrinted) {
                Serial.println("MPU9250 initialization unsuccessful");
                Serial.println("Check MPU9250 wiring");
                Serial.print("Status: ");
                Serial.println(status);
                mpuErrorPrinted = true;
            }
            acc.error  = Error::CRITICAL_ERROR;
            gyro.error = Error::CRITICAL_ERROR;
            mag.error  = Error::CRITICAL_ERROR;
        } else {
            mpu9250.setGyroRange(mpu9250.GYRO_RANGE_1000DPS);
            mpu9250.setAccelRange(mpu9250.ACCEL_RANGE_8G);
            mpu9250.setMagCalX(0, 1);
            mpu9250.setMagCalY(0, 1);
            mpu9250.setMagCalZ(0, 1);
            mpu9250.setSrd(0); //sets gyro and accel read to 1khz, magnetometer read to 100hz
            acc.error  = Error::NO_ERROR;
            gyro.error = Error::NO_ERROR;
            mag.error  = Error::NO_ERROR;
            Serial.println("Succsessfully initiated IMU9250");
        }
    }

    void setAccCal(Vec3 gVecOffset, Vec3 scale) {
        mpu9250.setAccelCalX(gVecOffset.x * G, scale.x);
        mpu9250.setAccelCalY(gVecOffset.y * G, scale.y);
        mpu9250.setAccelCalZ(gVecOffset.z * G, scale.z);
    }

    void setGyroCal(Vec3 degVecOffset) {
        degVecOffset.toRad();
        mpu9250.setGyroBiasX_rads(degVecOffset.x);
        mpu9250.setGyroBiasY_rads(degVecOffset.y);
        mpu9250.setGyroBiasZ_rads(degVecOffset.z);
    }

     void setMagCal(Vec3 offset, Vec3 scale) {
        mpu9250.setMagCalX(offset.x, scale.x);
        mpu9250.setMagCalY(offset.y, scale.y);
        mpu9250.setMagCalZ(offset.z, scale.z);
    }

    Vec3 getAccOffset() {
        return Vec3(mpu9250.getAccelBiasX_mss() / G, mpu9250.getAccelBiasY_mss() / G, mpu9250.getAccelBiasZ_mss() / G);
    }

    Vec3 getAccScale() {
        return Vec3(mpu9250.getAccelScaleFactorX(), mpu9250.getAccelScaleFactorY(), mpu9250.getAccelScaleFactorZ());
    }

    Vec3 getGyroOffset() {
        return Vec3(mpu9250.getGyroBiasX_rads() * RAD_TO_DEG, mpu9250.getGyroBiasY_rads() * RAD_TO_DEG, mpu9250.getGyroBiasZ_rads() * RAD_TO_DEG);
    }

     Vec3 getMagOffset() {
        return Vec3(mpu9250.getMagBiasX_uT(), mpu9250.getMagBiasY_uT(), mpu9250.getMagBiasZ_uT());
     }

     Vec3 getMagScale() {
        return Vec3(mpu9250.getMagScaleFactorX(), mpu9250.getMagScaleFactorY(), mpu9250.getMagScaleFactorZ());
     }

    void handle() {
        /**
         * MPU9250
         */
        mpu9250.readSensor();
        acc.update (mpu9250.getAccelX_G(), mpu9250.getAccelY_G(), -mpu9250.getAccelZ_G());
        gyro.update(mpu9250.getGyroX_rads() * RAD_TO_DEG, -mpu9250.getGyroY_rads() * RAD_TO_DEG, mpu9250.getGyroZ_rads() * RAD_TO_DEG);
        mag.update (mpu9250.getMagX_uT(), mpu9250.getMagY_uT(), mpu9250.getMagZ_uT());

        /**
         * BMP280
         */
        if(millis() > lastBaro + (1000 / baroHz)) {
            float temperature = bmp280.readTemperature();
            if(temperature != baro.temperature) {
                baro.temperature = temperature;
                baro.lastChange = micros();
            }
            baro.preassure = bmp280.readPressure() / 100000.0f;
            baro.altitude = bmp280.readAltitude(1028.6);
            lastBaro = millis();
        }
        // /**
        //  * GPS
        //  */
        while (Serial5.available()) {
            if (gpsSensor.encode(Serial5.read())) {
                gps.timeValid = gpsSensor.location.isValid();
                if(gps.timeValid) {
                    gps.lat = gpsSensor.location.lat();
                    gps.lng = gpsSensor.location.lng();
                }
                gps.dateValid = gpsSensor.date.isValid();
                if(gps.timeValid) {
                    gps.year = gpsSensor.date.year();
                    gps.month = gpsSensor.date.month();
                    gps.day = gpsSensor.date.day();
                }
                gps.timeValid = gpsSensor.time.isValid();
                if(gps.timeValid) {
                    gps.hour = gpsSensor.time.hour();
                    gps.minute = gpsSensor.time.minute();
                    gps.second = gpsSensor.time.second();
                    gps.centisecond = gpsSensor.time.centisecond();
                }
                gps.satelites = gpsSensor.satellites.value();
                gps.courseValid = gpsSensor.course.isValid();
                if(gps.courseValid) {
                    gps.course = gpsSensor.course.deg();
                }
                gps.speedValid = gpsSensor.speed.isValid();
                if(gps.speedValid) {
                    gps.speed = gpsSensor.speed.kmph();
                }
                gps.altitudeValid = gpsSensor.altitude.isValid();
                if(gps.altitudeValid) {
                    gps.altitude = gpsSensor.altitude.value();
                }
                gps.lastChange = micros();
            }
        }
        
        // /**
        //  * vBat
        //  * Resolution 10 Bit(0 to 1023)
        //  * Range 0 to 3.3 Volts
        //  * 
        //  * Voltage divider:
        //  *  R1: 5640000 Ohms
        //  *  R2: 477000  Ohms
        //  */
        int analog = analogRead(22);
        vMeasured = (analog * 3.3) / 1023.0;
        bat.vBat = bat.vBat * (1 - batLpf) + batLpf * vMeasured * vBatMul; // lpf

        bat.cellCount = max(bat.cellCount, (int) (bat.vBat / 3.65));

        bat.vCell = bat.vBat / bat.cellCount;
        bat.lastChange = micros();
    }

    /**
     * System has to be perfectly level!
     * Sets the acc scale to one and only calibrates the offset to level
     */
    void calibrateAccQuick() {
        Vec3 avg = getAccAvg(100, 20);
        Vec3 offset = avg - Vec3(0, 0, -G);
        mpu9250.setAccelCalX(offset.x, 1);
        mpu9250.setAccelCalY(offset.y, 1);
        mpu9250.setAccelCalZ(offset.z, 1);
    }

    void calibrateAccSide(Side side) {
        accSideAvgs[side] = getAccAvg(100, 10);
        sideCals++;
        switch(side) {
            case top: Serial.println("top calibrated"); break;
            case bottom: Serial.println("bottom calibrated"); break;
            case left: Serial.println("left calibrated"); break;
            case right: Serial.println("right calibrated"); break;
            case front: Serial.println("front calibrated"); break;
            case back: Serial.println("back calibrated"); break;
        }
    }

    void calibrateAcc() {
        if(sideCals < 6) {
            Serial.println("Not all sides have been calibrated. Try again!");
            sideCals = 0;
            return;
        }
        Vec3 scale;
        scale.x = 1 / ((accSideAvgs[back].x   - accSideAvgs[front].x)  / (2.0 * G)); // 1 / ((max - min) / 2G)
        scale.y = 1 / ((accSideAvgs[left].y   - accSideAvgs[right].y)  / (2.0 * G));
        scale.z = 1 / ((accSideAvgs[top].z    - accSideAvgs[bottom].z) / (2.0 * G));

        Vec3 offset = (accSideAvgs[bottom] * scale) - Vec3(0, 0, -G);

        float error = 0;
        float topError  = abs((accSideAvgs[top]   - Vec3(0, 0, G)).getValue());
        float rightError= abs((accSideAvgs[right] - Vec3(0,-G, 0)).getValue());
        float leftError = abs((accSideAvgs[left]  - Vec3(0, G, 0)).getValue());
        float frontError= abs((accSideAvgs[front] - Vec3(-G,0, 0)).getValue());
        float backError = abs((accSideAvgs[back]  - Vec3(G, 0, 0)).getValue());

        error = topError + rightError + leftError + frontError + backError;

        Serial.println("Acc calibration errors:");
        Serial.print("top: ");
        Serial.print(topError);
        Serial.print(", right: ");
        Serial.print(rightError);
        Serial.print(", left: ");
        Serial.print(leftError);
        Serial.print(", front: ");
        Serial.print(frontError);
        Serial.print(", back: ");
        Serial.println(backError);

        Serial.print("Total error: ");
        Serial.print(error);
        Serial.println("Mss(9.807 = 1G)");

        mpu9250.setAccelCalX(offset.x, scale.x);
        mpu9250.setAccelCalY(offset.y, scale.y);
        mpu9250.setAccelCalZ(offset.z, scale.z);

        sideCals = 0;
    }

    Vec3 getAccAvg(uint8_t samples, int delayMs) {
        Vec3 avg = Vec3();
        for (size_t i = 0; i < samples; i++) {
            mpu9250.readSensor();
            avg += Vec3(mpu9250.getAccelX_mss(), mpu9250.getAccelY_mss(), mpu9250.getAccelZ_mss()) / (double) samples;
            delay(20);
        }
        return avg;
    }

    /**
     * Blocks for 2 seconds
     */
    void calibrateGyro() {
        int error = mpu9250.calibrateGyro();
        if(error < 1) {
            Serial.print("Failed to calibrate Gyro. Error: ");
            Serial.println(error);
        }
    }

    /**
     * Blocks for 20 seconds
     */
    void calibrateMag() {
        int error = mpu9250.calibrateMag();
        if(error < 1) {
            Serial.print("Failed to calibrate Mag. Error: ");
            Serial.println(error);
        }
    }

    void calibrateBat(float actualVoltage) {
        vBatMul = actualVoltage / vMeasured;
    }

private:

    bool mpuErrorPrinted = false;
    float vMeasured = 1;

    Vec3 accSideAvgs[6];
    int sideCals = 0;
};