#pragma once
#include <Arduino.h>
#include "sensorInterface.h"
#include <TinyGPS++.h>
#include <MPU9250.h>
#include <error.h>
#include <maths.h>

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <MechaQMC5883.h>

#define G 9.807

// #define BME_SCK 13
// #define BME_MISO 12
// #define BME_MOSI 11
#define BMP_CS 9

#define SEALEVELPRESSURE_HPA (1013.25)


/**
 * VIN MPU9250                  : 5V
 * GND MPU9250                  : GND
 * SCL MPU9250                  : Pin 13
 * SDA MPU9250                  : Pin 11
 * SDO/SAO MPU9250              : Pin 12
 * NCS MPU9250                  : Pin 10
 * 
 * SCL SPI Same as MPU
 * SDA Chip select              : Pin 9
 * 
 * RX GPS                       : Pin 20
 * TX GPS                       : Pin 21
 * 
 * Bat (voltage divider)        : Pin 22
 **/
class MPU9250Sensor : public SensorInterface {
public:

    // Adafruit_BMP280 bmp;
    TinyGPSPlus gpsSensor;

    float altitudeOffset = 0;
    bool firstHeightMeasured = false;

    float baroHz = 50;
    uint32_t lastBaro = 0;

    float magHz = 200;
    uint32_t lastMag = 0;

    MPU9250 mpu9250;
    Adafruit_BMP280 bmp;

    MechaQMC5883 qmc; // I2C Address: 0x0D

    MPU9250Sensor() : mpu9250(SPI, 10), bmp(BMP_CS) {}
    
    /**
     * Begin function can becalled as many times as wanted
     * Note: this function cal have a significant delay when waiting for sensors
     */
    void begin() {
        initMPU9250();

        initBmp280();

        initGPS();

        initBattery();
        
        initMag();
    }

    void initMag() {
        Wire.begin();
        Wire.setClock(1000000);
        Serial.println("qmc init");
        qmc.init();
    }

    void initBattery() {
        pinMode(22, INPUT);
    }

    void initGPS() {
        Serial1.begin(9600);
        delay(200);
        char disable_GPGSV[11] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15};
        Serial1.write(disable_GPGSV, 11);
        delay(200);
        char setTo5Hz[14] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8/*200ms*/, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A};
        Serial1.write(setTo5Hz, 14);
        delay(200);
        char setTo57kbs[28] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0xE1, 0x00, 0x00, 0x23, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFA, 0xA9};
        Serial1.write(setTo57kbs, 28);
        Serial1.end();
        Serial1.begin(57600);
    }

    void initBmp280() {
        bool succsess = bmp.begin();
        if (succsess) {
            bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X1,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X1,     /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X4,       /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1);   /* Standby time. */
            baro.error = Error::NO_ERROR;
            Serial.println("Succsessfully initiated BMP280");
        } else {
            Serial.println("Could not find a valid BMP280 sensor, check wiring, address, sensor ID!");
            baro.error = Error::CRITICAL_ERROR;
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
        mag.lpf = 0.1;
    }

    void setAccCal(Vec3 gVecOffset, Vec3 scale) {
        accOffset = gVecOffset.clone();
    }

    void setGyroCal(Vec3 degVecOffset) {
        gyroOffset = degVecOffset.clone();
    }

     void setMagCal(Vec3 offset, Vec3 scale) {
        magOffset = offset;
        magScale = scale;
    }

    Vec3 getAccOffset() {
        return accOffset;
    }

    Vec3 getAccScale() {
        return Vec3(mpu9250.getAccelScaleFactorX(), mpu9250.getAccelScaleFactorY(), mpu9250.getAccelScaleFactorZ());
    }

    Vec3 getGyroOffset() {
        return gyroOffset;
    }

    Vec3 getMagOffset() {
        return magOffset;
    }

    Vec3 getMagScale() {
        return magScale;
    }

    Vec3 getAccRaw() {
        return Vec3(mpu9250.getAccelY_G(), -mpu9250.getAccelX_G(), -mpu9250.getAccelZ_G());
    }

    Vec3 getMagRaw() {
        int x,y,z;
        qmc.read(&x, &y, &z);
        return Vec3(x, y, z);
    }

    Vec3 getGyrocRaw() {
        return Vec3(mpu9250.getGyroY_rads(), mpu9250.getGyroX_rads(), mpu9250.getGyroZ_rads());
    }

    void handle() {
        uint64_t timeTmp = micros();
        /**
         * BMP280
         */
        if(baroHz > 0 && millis() > lastBaro + (1000.0f / baroHz)) {
            timeTmp = micros();
            float altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
            if(altitude != baro.altitude) {
                baro.altitude = altitude;
                baro.lastChange = micros();
            }
            lastBaro = millis();
            baro.lastPollTime = micros() - timeTmp;
        }
        /**
         * MPU9250
         */
        mpu9250.readSensor();
        Vec3 accRaw = getAccRaw();
        if(accRaw.getLength() != 0) {
            acc.update (accRaw - accOffset);
        }
        acc.lastPollTime = micros() - timeTmp;
        timeTmp = micros();
        Vec3 gyroRaw = getGyrocRaw();
        gyro.update(gyroRaw.toDeg() - gyroOffset);
        gyro.lastPollTime = micros() - timeTmp;
        timeTmp = micros();
        if(magHz > 0 && millis() > lastMag + (1000.0f / magHz)) {
            mag.update ((getMagRaw() + magOffset) * magScale);
            mag.lastPollTime = micros() - timeTmp;
            lastMag = millis();
        }
        timeTmp = micros();

        // /**
        //  * GPS
        //  */
        while (Serial1.available()) {
            char c = Serial1.read();
            // Serial.write(c);
            if (gpsSensor.encode(c)) {
                timeTmp = micros();
                gps.locationValid = gpsSensor.location.isValid();
                if(gps.locationValid) {
                    gps.lat = gpsSensor.location.lat();
                    gps.lng = gpsSensor.location.lng();
                }
                gps.dateValid = gpsSensor.date.isValid();
                if(gps.dateValid) {
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
                    gps.speed = gpsSensor.speed.mps();
                }
                gps.altitudeValid = gpsSensor.altitude.isValid();
                if(gps.altitudeValid) {
                    gps.altitude = gpsSensor.altitude.value();
                }
                if(micros() - gps.lastChange > 180000) {
                    gps.lastChange = micros();
                }
                gps.lastPollTime = micros() - timeTmp;
            }
        }
        
        /**
         * vBat
         * Resolution 10 Bit(0 to 1023)
         * Range 0 to 3.3 Volts
         */
        int analog = analogRead(22);
        vMeasured = (analog * 3.3) / 1023.0;
        bat.vBat = bat.vBat * (1 - batLpf) + batLpf * vMeasured * vBatMul; // lpf

        bat.cellCount = max(bat.cellCount, (int) ceil((bat.vBat - 0.2) / 4.2));

        bat.vCell = bat.vBat / bat.cellCount;
        bat.lastChange = micros();

        /**
         * Error handling
         */
        acc.checkError();
        gyro.checkError();
        mag.checkError();
        baro.checkError();
        gps.checkError();
    }

    /**
     * System has to be perfectly level!
     * Sets the acc scale to one and only calibrates the offset to level
     */
    void calibrateAcc() {
        Vec3 avg = getAccAvg(100, 20);
        accOffset = (avg - Vec3(0, 0, 1));
    }

    Vec3 getAccAvg(uint8_t samples, int delayMs) {
        Vec3 avg = Vec3();
        for (size_t i = 0; i < samples; i++) {
            mpu9250.readSensor();
            avg += getAccRaw() / (double) samples;
            delay(20);
        }
        return avg;
    }

    /**
     * Blocks for 2 seconds
     */
    void calibrateGyro() {
        Vec3 avg = Vec3();
        const int sampleCount = 100;
        for (size_t i = 0; i < sampleCount; i++) {
            mpu9250.readSensor();
            Vec3 raw = getGyrocRaw().toDeg();
            avg += raw;
            // if(raw.getLength() == 0) {
            //     Serial.println("Gy");
            // }
            delay(20);
        }
        gyroOffset = (avg / (double) sampleCount);
    }

    /**
     * Blocks for 20 seconds
     */
    void calibrateMag() {
        Vec3 min = Vec3();
        Vec3 max = Vec3();
        double seconds = 20;
        size_t sampleCount = seconds / (1.0 / magHz);
        uint32_t lastPrint = 0;
        for (size_t i = 0; i < sampleCount; i++) {
            Vec3 magRaw = getMagRaw();
            min = Vec3::min(min, magRaw);
            max = Vec3::max(max, magRaw);
            delay(1000.0f / magHz);
            if(millis() - lastPrint > 30) {
                Serial.print("FC_POST_SENSOR MAG;X;");
                Serial.println(magRaw.x, 5);
                Serial.print("FC_POST_SENSOR MAG;Y;");
                Serial.println(magRaw.y, 5);
                Serial.print("FC_POST_SENSOR MAG;Z;");
                Serial.println(magRaw.z, 5);
                lastPrint = millis();
            }
        }
        magOffset = (max + min) / -2;
        min += magOffset;
        max += magOffset;
        Vec3 targetRadius = (max.x + max.y + max.z) / 3.0;
        magScale = targetRadius / max;
    }

    void calibrateBat(float actualVoltage) {
        vBatMul = actualVoltage / vMeasured;
    }

private:

    Vec3 gyroOffset = Vec3();//in degrees
    Vec3 accOffset = Vec3();//in G

    Vec3 magOffset = Vec3();
    Vec3 magScale = Vec3();

    bool mpuErrorPrinted = false;
    float vMeasured = 1;

    Vec3 accSideAvgs[6];
    int sideCals = 0;
};