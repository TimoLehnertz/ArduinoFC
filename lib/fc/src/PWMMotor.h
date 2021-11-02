#include <Arduino.h>
#include <Motor.h>
#include "../../Servo/src/Servo.h"

class PWMMotor : public Motor {
public:
    PWMMotor(int pin) : Motor(pin) {};

    void begin() {
        servo.attach(pin, minPWM, maxPWM);
        servo.write(0);
    }

    void end() {
        servo.detach();
    }

    void handle() {
        if(millis() - armStart > 1000 && !armed) {
            armed = true;
        }
    }

    void writeRaw(float percentage) {
        servo.write(map(percentage, 0, 1, minPWM, maxPWM));
    }

    void arm() {
        servo.write(0);
        armStart = millis();
    }

    void disarm() {
        servo.write(0);
        armed = false;
    }

private:
    Servo servo;

    int minPWM = 1000;
    int maxPWM = 2000;

    uint64_t armStart = 0;
};