#pragma once
#include <Arduino.h>
#include "sensorInterface.h"
#include <MPU9250_asukiaaa.h>
#include <Adafruit_BMP280.h>
#include <Wire.h>
#include <TinyGPS++.h>

/**
 * SCL MPU9250 : Pin 19
 * SDA MPU9250 : Pin 18
 * 
 * SCL bmp280  : Pin 16
 * SDA bmp280  : Pin 17
 * 
 * RX GPS      : Pin 20
 * TX GPS      : Pin 21
 **/
class MPU9250Sensor : public SensorInterface {
public:

    MPU9250_asukiaaa imu;
    Adafruit_BMP280 bmp;
    TinyGPSPlus gpsSensor;

    MPU9250Sensor() {}

    void begin() {
        /**
         * MPU9250
         */
        imu.beginAccel(ACC_FULL_SCALE_8_G);
        imu.beginGyro(GYRO_FULL_SCALE_1000_DPS);
        imu.beginMag(MAG_MODE_CONTINUOUS_8HZ);
        /**
         * BMP280
         */
        bmp = Adafruit_BMP280(&Wire1);

        //if (!bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID)) {
        if (!bmp.begin(0x76)) {
            Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                            "try a different address!"));
            while (1) delay(10);
        }

        /* Default settings from datasheet. */
        bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                        Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                        Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                        Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                        Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
        /**
         * GPS
         */
        Serial5.begin(9600);
    }

    void handle() {
        /**
         * MPU9250
         */
        if (imu.accelUpdate() == 0) {
            acc.y = -imu.accelX() + 0.3297;// swapped x and y because of alignement
            acc.x = -imu.accelY() + -0.0290;
            acc.z = -imu.accelZ() + 0.2768;
            acc.lastChange = micros();
        }
        if (imu.gyroUpdate() == 0) {
            gyro.y = imu.gyroX() * 0.6 + 0.7544;// swapped x and y because of alignement
            gyro.x = imu.gyroY() * -0.6 + -0.2399;
            gyro.z = imu.gyroZ() * 0.6 + -0.3522;
            gyro.lastChange = micros();
        }
        if (imu.magUpdate() == 0) {
            mag.connected = true;
            mag.y = imu.magX();
            mag.x = imu.magY();
            mag.z = imu.magZ();
            mag.lastChange = micros();
        }
        /**
         * BMP280
         */
        float temperature = bmp.readTemperature();
        if(temperature != baro.temperature) {
            baro.temperature = temperature;
            baro.connected = true;
        }
        float pa = bmp.readPressure();
        float bar = pa / 100000.0f;
        if(bar != baro.preassure) {
            baro.preassure = bar;
        }
        float altitude = bmp.readAltitude();
        if(altitude != baro.altitude) {
            baro.altitude = altitude;
            baro.lastChange = micros();
        }
        /**
         * GPS
         */
        while (Serial5.available()) {
            gps.connected = true;
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
    }
};