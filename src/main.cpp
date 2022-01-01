/**
 * @file main.cpp
 * @author Timo Lehnertz
 * @brief 
 * @version 0.1
 * @date 2022-01-01
 * 
 * @copyright Copyright (c) 2022
 * 
 */
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

MPU9250Sensor sensors;// Sensor interface to interface with all sensors on board

INS ins(&sensors);// Inertial navigation system to convert sensor reading into position, velocity, rotation, rotational rates

Crossfire crsf(CRSF_SERIAL_PORT); // Crossfire implementation to get radio commands from pilot and send telemetry

// One shot implementations to talk to ESCs
OneShotMotor mFL(MOTOR_1); 
OneShotMotor mFR(MOTOR_2);
OneShotMotor mBL(MOTOR_3);
OneShotMotor mBR(MOTOR_4);

FC fc(&ins, &mFL, &mFR, &mBL, &mBR, &crsf); // Flight controller

#define NUMPIXELS 10 // maximum number of pixels controlled
#define PIXEL_PIN 20 // Digital pin from LED Strip
Adafruit_NeoPixel pixels(NUMPIXELS, PIXEL_PIN, NEO_GRB + NEO_KHZ800); // LED libary

Comunicator com(&ins, &sensors, &fc, &crsf, &pixels); // Comunicator to talk to gui, storage, dji air unit / caddx vista and for sending telemetry

/**
 * @brief Waits until next loop starts to keep looptimes consitent
 * 
 */
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
  Serial.begin (921600);// Gui over usb
  Serial2.begin(115200);// Gui over air(not used)
  Serial4.begin(115200);// DJI MSP
  Storage::begin();     // EEPROM storage to save settings
  com.begin();
  crsf.begin();
  sensors.begin();      // Initiate all sensors (takes some seconds)
  fc.begin();
  com.readEEPROM();     // Read Settings from EEPROM
  ins.begin();
  Serial.print("Boot time: ");
  Serial.println(millis());
}

void loop() {
  uint64_t now = micros();
  com.loopStart = now;// timing statistics available in GUI
  crsf.handle(); // talk to radio controller
  com.crsfTime = micros();
  sensors.handle(); // schedule and get sensor readings
  com.sensorsTime = micros();
  ins.handle(); // convert readings to navigation data
  com.insTime = micros();
  if(crsf.isFailsafe()) fc.startFailsafe(); else fc.stopFailsafe();// handle failsafe

  CRSF_TxChanels_Converted chanelsConv = crsf.getChanelsCoverted();
  CRSF_TxChanels chanels = crsf.getChanels();
  fc.updateRcChanels(chanelsConv, chanels);

  com.chanelsTime = micros();
  if(!com.motorOverwrite) { // available in GUI
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