#pragma once
#include <Arduino.h>
#include "sensorInterface.h"
#include "utils.h"

class SerialArduinoSensors : public SensorInterface {
public:

    HardwareSerial uart;

    SerialArduinoSensors(HardwareSerial& uart) : uart(uart) {}

    void begin() {
        uart.begin(115200);
    }

    void handle() {
        while(uart.available()) {
            char c = uart.read();
            if(c == '\n') {
                s2buff[sBuf2Count] = 0;
                processS2Buff();
                sBuf2Count = 0;
            } else {
                s2buff[sBuf2Count] = c;
                sBuf2Count++;
            }
        }
    }
private:
    int rxPin = 34;//Serial imu data
    int txPin = 35;//not used

    char s2buff[100];
    int sBuf2Count = 0;

    void processS2Buff() {
        if(sBuf2Count < 5) return;

        int comma1 = strpos1(s2buff, ',');
        int comma2 = comma1 + strpos1(&s2buff[comma1 + 1], ',') + 1;
        int comma3 = comma2 + strpos1(&s2buff[comma2 + 1], ',') + 1;
        int comma4 = comma3 + strpos1(&s2buff[comma3 + 1], ',') + 1;
        s2buff[comma1] = 0;
        s2buff[comma2] = 0;
        s2buff[comma3] = 0;
        s2buff[comma4] = 0;

        // String strx(&s2buff[comma1 + 1]);
        // double x = strx.toDouble();
        double x = atof(&s2buff[comma1 + 1]);

        // String stry(&s2buff[comma2 + 1]);
        // double y = stry.toDouble();
        double y = atof(&s2buff[comma2 + 1]);

        // String strz(&s2buff[comma3 + 1]);
        // double z = strz.toDouble();
        double z = atof(&s2buff[comma3 + 1]);
        if(s2buff[0] == 'a') { //accelerometer
            accX = x;
            accY = y;
            accZ = z;
            accChanged = true;
        } else if(s2buff[0] == 'g') { //gyroscope
            gyroX = x;
            gyroY = y;
            gyroZ = z;
            gyroChanged = true;
        }
    }
};