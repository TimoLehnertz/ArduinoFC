#include <Arduino.h>
#include <Motor.h>
#include <SensorImpMPU-9250.h>
#include <crossfire.h>
#include <FC.h>
#include <Comunicator.h>
#include <Storage.h>
#include "setup.h"
#include <EEPROM.h>

MPU9250Sensor sensors;
INS ins(&sensors);

Motor mFL(MOTOR_1);
Motor mFR(MOTOR_2);
Motor mBL(MOTOR_3);
Motor mBR(MOTOR_4);

FC fc(&ins, &mFL, &mFR, &mBL, &mBR);

Crossfire crsf(CRSF_SERIAL_PORT);

Comunicator com(&ins, &sensors, &fc, &crsf);

uint32_t bootTime;

uint32_t lastLoop = 0;

void loopFrequ(int freq) {
  int t = 1.0f / freq * 1000000;
  delayMicroseconds(t  - micros() % t);
}


void handleLoopFreq() {
  int freq = 100;
  switch(fc.flightMode) {
    case FlightMode::rate:  freq = com.loopFreqRate; break;
    case FlightMode::level: freq = com.loopFreqLevel; break;
    default:                freq = com.loopFreqLevel;
  }
  float microT = 1000000 / freq;
  com.cpuLoad = ((float)(com.loopEnd - com.loopStart) / microT) * 100.0f;
  loopFrequ(freq);
  com.actualFreq = 1000000 / (micros() - com.loopStart);
}

// void handleLeds() {
//     if(error > 0) {
//       Serial.print("error! Code:");
//       Serial.println(error);
//       if(millis() % 2000 > 1500) {
//         analogWrite(LED_1, ((millis() % 100) > 50) * 255);
//         analogWrite(LED_2, ((millis() % 100) > 50) * 255);
//         analogWrite(LED_4, ((millis() % 100) > 50) * 255);
//         analogWrite(LED_3, ((millis() % 100) > 50) * 255);
//       } else {
//         analogWrite(LED_1, 0);
//         analogWrite(LED_2, 0);
//         analogWrite(LED_4, 0);
//         analogWrite(LED_3, 0);
//         switch(error) {
//           case 1: {
//             analogWrite(LED_1, 255);
//             break;
//           }
//           case 2: {
//             analogWrite(LED_2, 255);
//             break;
//           }
//           case 3: {
//             analogWrite(LED_3, 255);
//             break;
//           }
//           case 4: {
//             analogWrite(LED_4, 255);
//             break;
//           }
//         }
//       }
//       return;
//     }
//   if(!com.useLeds) {
//     analogWrite(LED_1, 0);
//     analogWrite(LED_2, 0);
//     analogWrite(LED_4, 0);
//     analogWrite(LED_3, 0);
//     return;
//   };
//   int cycletime = 1000;
//   float progress = (millis() % cycletime) / (float) cycletime;
//   if(millis() < bootTime + 1000) {
//     analogWrite(LED_1, (millis() - bootTime > 0) * 255);
//     analogWrite(LED_2, (millis() - bootTime > 250) * 255);
//     analogWrite(LED_4, (millis() - bootTime > 500) * 255);
//     analogWrite(LED_3, (millis() - bootTime > 750) * 255);
//   } else if(millis() < bootTime + 1600) {
//     analogWrite(LED_1, ((millis() % 200) < 100) * 255);
//     analogWrite(LED_2, ((millis() % 200) < 100) * 255);
//     analogWrite(LED_4, ((millis() % 200) < 100) * 255);
//     analogWrite(LED_3, ((millis() % 200) < 100) * 255);
//   } else if(!fc.isArmed()) {
//     analogWrite(LED_1, (progress > 0.00f && progress < 0.25f) * 255);
//     analogWrite(LED_2, (progress > 0.25f && progress < 0.50f) * 255);
//     analogWrite(LED_4, (progress > 0.50f && progress < 0.75f) * 255);
//     analogWrite(LED_3, (progress > 0.75f && progress < 1.00f) * 255);
//   } else if(fc.chanels.throttle > 0.02f) {
//     analogWrite(LED_1, (fc.chanels.throttle + 0.1) * 230);
//     analogWrite(LED_2, (fc.chanels.throttle + 0.1) * 230);
//     analogWrite(LED_4, (fc.chanels.throttle + 0.1) * 230);
//     analogWrite(LED_3, (fc.chanels.throttle + 0.1) * 230);
//   } else {
//     analogWrite(LED_1, (sin(millis() / 300.0f + 0.0f)      / 2 + 0.2) * 255);
//     analogWrite(LED_2, (sin(millis() / 300.0f + HALF_PI)   / 2 + 0.2) * 255);
//     analogWrite(LED_4, (sin(millis() / 300.0f + PI)        / 2 + 0.2) * 255);
//     analogWrite(LED_3, (sin(millis() / 300.0f + PI * 1.5f) / 2 + 0.2) * 255);
//   }
// }

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  // delay(2000);
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
  // if(millis() % 100 == 0) {
  //   Serial.println(millis());
  // }
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

  com.chanelsTime = micros();
  com.handleCRSFTelem();
  if(!com.motorOverwrite) {
    fc.handle(); //also handles motors
  } else {
    mFL.writeRaw(((float) com.motorFL) / 100);
    mFR.writeRaw(((float) com.motorFR) / 100);
    mBL.writeRaw(((float) com.motorBL) / 100);
    mBR.writeRaw(((float) com.motorBR) / 100);
  }
  com.fcTime = micros();
  com.handle();
  // watchDog();
  com.loopEnd = micros();
  handleLoopFreq();
}