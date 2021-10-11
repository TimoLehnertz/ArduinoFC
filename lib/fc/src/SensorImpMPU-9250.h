#pragma once
#include <Arduino.h>
#include "sensorInterface.h"
#include <Adafruit_BMP280.h>
#include <TinyGPS++.h>
#include <MPU9250.h>
#include <error.h>

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
            Serial.println("MPU9250 initialization unsuccessful");
            Serial.println("Check MPU9250 wiring");
            Serial.print("Status: ");
            Serial.println(status);
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

    void handle() {
        /**
         * MPU9250
         */
        mpu9250.readSensor();
        acc.update (mpu9250.getAccelX_G(), mpu9250.getAccelY_G(), -mpu9250.getAccelZ_G());
        gyro.update(mpu9250.getGyroX_rads() * RAD_TO_DEG, mpu9250.getGyroY_rads() * RAD_TO_DEG, mpu9250.getGyroZ_rads() * RAD_TO_DEG);
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

    void calibrateBat(float actualVoltage) {
        vBatMul = actualVoltage / vMeasured;
    }

private:

    float vMeasured = 1;
};