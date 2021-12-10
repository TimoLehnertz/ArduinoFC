#include <Arduino.h>
#include <OneShotMotor.h>
#include <SensorImpMPU-9250.h>
#include <crossfire.h>
#include <FC.h>
#include <Comunicator.h>
#include <Storage.h>
#include "setup.h"
#include <EEPROM.h>
#include <Adafruit_NeoPixel.h>

MPU9250Sensor sensors;

INS ins(&sensors);

Crossfire crsf(CRSF_SERIAL_PORT);

OneShotMotor mFL(MOTOR_1);
OneShotMotor mFR(MOTOR_2);
OneShotMotor mBL(MOTOR_3);
OneShotMotor mBR(MOTOR_4);

FC fc(&ins, &mFL, &mFR, &mBL, &mBR, &crsf);

#define NUMPIXELS 10
#define PIXEL_PIN 20

Adafruit_NeoPixel pixels(NUMPIXELS, PIXEL_PIN, NEO_GRB + NEO_KHZ800);


Comunicator com(&ins, &sensors, &fc, &crsf, &pixels);

uint32_t bootTime;

uint32_t lastLoop = 0;

void handleLoopFreq() {
  int freq = 1000;
  switch(fc.flightMode) {
    case FlightMode::rate:  freq = com.loopFreqRate; break;
    case FlightMode::level: freq = com.loopFreqLevel; break;
    default:                freq = com.loopFreqLevel;
  }
  float microT = 1000000.0f / freq;
  com.cpuLoad = ((float)(com.loopEnd - com.loopStart) / microT) * 100.0f;
  com.loopTimeUs = com.loopEnd - com.loopStart;
  delayMicroseconds(microT - (com.loopEnd - com.loopStart));
  com.actualFreq = 1000000.0f / (micros() - com.loopStart);
}

void setup() {
  Serial.begin (921600);
  Serial2.begin(115200);
  Storage::begin();
  com.begin();
  crsf.begin();
  sensors.begin();
  fc.begin();
  com.readEEPROM();
  ins.begin();
  bootTime = millis();
  Serial.print("Booting time: ");
  Serial.println(bootTime);
}

void loop() {
  // Serial.println("test");
  // delay(1000);
  // return;
  uint64_t now = micros();
  com.loopStart = now;
  crsf.handle();
  com.crsfTime = micros();
  sensors.handle();
  com.sensorsTime = micros();
  ins.handle();
  com.insTime = micros();
  if(crsf.isFailsafe()) {
    fc.startFailsafe();
  } else {
    fc.stopFailsafe();
  }

  CRSF_TxChanels_Converted chanelsConv = crsf.getChanelsCoverted();
  CRSF_TxChanels chanels = crsf.getChanels();
  fc.updateRcChanels(chanelsConv, chanels);
  com.handleCRSFTelem();

  com.chanelsTime = micros();
  if(!com.motorOverwrite) {
    fc.handle(); //also handles motors
  } else {
    mFL.writeRaw(((float) com.motorFL) / 100);
    mFR.writeRaw(((float) com.motorFR) / 100);
    mBL.writeRaw(((float) com.motorBL) / 100);
    mBR.writeRaw(((float) com.motorBR) / 100);
  }
  com.fcTime = micros();
  com.loopEnd = micros();
  handleLoopFreq();
  com.handle();
}