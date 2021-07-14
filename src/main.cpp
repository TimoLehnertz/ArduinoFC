#include <Arduino.h>
// #include "ota.h"
#include <Motor.h>
#include <SensorImpSerialArduino.h>
#include <crossfire.h>
#include <LevelFc.h>
#include <Comunicator.h>
#include "setup.h"

SerialArduinoSensors sensors(IMU_SERIAL_PORT);
INS ins(sensors);

Motor mFL(MOTOR_1);
Motor mFR(MOTOR_2);
Motor mBL(MOTOR_3);
Motor mBR(MOTOR_4);

LevelFC flightController(&ins, &mFL, &mFR, &mBL, &mBR);

Comunicator com(&ins);

Crossfire crsf(CRSF_SERIAL_PORT);

void setup() {
  delay(2000);
  com.begin(); //start comunication over serial / usb
  crsf.begin(); //start Crossfire
  sensors.begin();
  flightController.begin();
}

void loop() {
  crsf.handle(); //handle remote controll comunication
  if(crsf.isFailsafe()) {
    flightController.startFailsafe();
  } else {
    flightController.stopFailsafe();
  }

  sensors.handle();
  ins.handle();

  CRSF_TxChanels_Converted chanels = crsf.getChanelsCoverted();
  flightController.updateRcChanels(chanels);

  flightController.handle(); //also handles motors
}