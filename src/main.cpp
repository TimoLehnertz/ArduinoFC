#include <Arduino.h>
#include <Motor.h>
#include <SensorImpMPU-9250.h>
#include <crossfire.h>
#include <RateFC.h>
#include <Comunicator.h>
#include "setup.h"
// #include <Servo.h>

// Servo testServo;

uint8_t sensorId;
float mDirection, mX, mY, mZ;

MPU9250Sensor sensors;
INS ins(&sensors);

Motor mFL(MOTOR_1);
Motor mFR(MOTOR_2);
Motor mBL(MOTOR_3);
Motor mBR(MOTOR_4);

RateFC flightController(&ins, &mFL, &mFR, &mBL, &mBR);

Comunicator com(&ins, &sensors);

Crossfire crsf(CRSF_SERIAL_PORT);

void setup() {
  Serial.begin(115200);
  delay(2000);
  com.begin(); //start comunication over serial / usb
  crsf.begin(); //start Crossfire
  sensors.begin();
  flightController.begin();

  // testServo.attach(5, 1000, 2000);
}

void loop() {
  crsf.handle(); //handle remote controll comunication
  sensors.handle();
  ins.handle();
  com.handle();
  if(crsf.isFailsafe()) {
    flightController.startFailsafe();
  } else {
    flightController.stopFailsafe();
  }

  CRSF_TxChanels_Converted chanels = crsf.getChanelsCoverted();
  flightController.updateRcChanels(chanels);

  flightController.handle(); //also handles motors

  
  // if(chanels.armed3) {
  //   testServo.write(map(chanels.throttle, 0, 1, 1000, 2000));
  //   Serial.println(map(chanels.throttle, 0, 1, 1000, 2000));
  // } else {
  //   testServo.write(1000);
  // }
  // mFL.handle();
}