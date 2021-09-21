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

Comunicator com(&ins, &sensors, &fc);

Crossfire crsf(CRSF_SERIAL_PORT);

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

 /**
 * 0 => No error
 * 
 * 2 => critical error
 */
int error = 0;

Vec3 lastGyro = Vec3();
int sameGyros = 0;
void watchDog() {
  error = 0;
  /**
   * check if raw imu data is changing and if not for 100 cycles raise critical error
   */
  if(sensors.gyro.x == lastGyro.x && sensors.gyro.y == lastGyro.y && sensors.gyro.z == lastGyro.z) {
    sameGyros++;
    if(sameGyros >= 100) {
      sameGyros = 100;
      error = 2;
    }
  } else {
    lastGyro = Vec3(sensors.gyro.x, sensors.gyro.y, sensors.gyro.z);
    sameGyros = 0;
  }
}

void handleLeds() {
  if(error == 2) {
    // Serial.println("error");
    analogWrite(LED_1, ((millis() % 150) > 80) * 100);
    analogWrite(LED_2, ((millis() % 150) > 80) * 100);
    analogWrite(LED_4, ((millis() % 150) > 80) * 100);
    analogWrite(LED_3, ((millis() % 150) > 80) * 100);
    return;
  }
  if(!com.useLeds) {
    analogWrite(LED_1, 0);
    analogWrite(LED_2, 0);
    analogWrite(LED_4, 0);
    analogWrite(LED_3, 0);
    return;
  };
  int cycletime = 1000;
  float progress = (millis() % cycletime) / (float) cycletime;
  if(millis() < 3000) {
    analogWrite(LED_1, ((millis() % 100) > 80) * 100);
    analogWrite(LED_2, ((millis() % 100) > 80) * 100);
    analogWrite(LED_4, ((millis() % 100) > 80) * 100);
    analogWrite(LED_3, ((millis() % 100) > 80) * 100);
  } else if(!fc.isArmed()) {
    analogWrite(LED_1, (progress > 0.00f && progress < 0.25f) * 255);
    analogWrite(LED_2, (progress > 0.25f && progress < 0.50f) * 255);
    analogWrite(LED_4, (progress > 0.50f && progress < 0.75f) * 255);
    analogWrite(LED_3, (progress > 0.75f && progress < 1.00f) * 255);
  } else if(fc.chanels.throttle > 0.02f) {
    analogWrite(LED_1, (fc.chanels.throttle + 0.1) * 230);
    analogWrite(LED_2, (fc.chanels.throttle + 0.1) * 230);
    analogWrite(LED_4, (fc.chanels.throttle + 0.1) * 230);
    analogWrite(LED_3, (fc.chanels.throttle + 0.1) * 230);
  } else {
    analogWrite(LED_1, (sin(millis() / 300.0f + 0.0f)      / 2 + 0.2) * 255);
    analogWrite(LED_2, (sin(millis() / 300.0f + HALF_PI)   / 2 + 0.2) * 255);
    analogWrite(LED_4, (sin(millis() / 300.0f + PI)        / 2 + 0.2) * 255);
    analogWrite(LED_3, (sin(millis() / 300.0f + PI * 1.5f) / 2 + 0.2) * 255);
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  // delay(3000);
  Storage::begin();
  com.begin();
  crsf.begin();
  sensors.begin();
  fc.begin();

  com.readEEPROM();
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(LED_3, OUTPUT);
  pinMode(LED_4, OUTPUT);
}

void handleCRSFTelem() {
  // vBat, current, mahDraw, remaining Percent
  // crsf.updateTelemetryBattery(0.2, 5, 10, 88);
  crsf.updateTelemetryBattery(0.0, fc.gForce, fc.maxGForce, 0);
  // if(millis() % 100 == 0) {
    // Serial.println(fc.gForce);
  // }
  crsf.updateTelemetryAttitude(-ins.getRoll(), -ins.getPitch(), ins.getYaw());
  switch(fc.flightMode) {
    case FlightMode::none: {
      crsf.updateTelemetryFlightMode("None"); break;
    }
    case FlightMode::rate: {
      crsf.updateTelemetryFlightMode("Rate"); break;
    }
    case FlightMode::level: {
      crsf.updateTelemetryFlightMode("Lvl"); break;
    }
    case FlightMode::FlightModeSize: break;
  }
}

void loop() {
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
  handleCRSFTelem();
  if(!com.motorOverwrite) {
    fc.handle(); //also handles motors
  } else {
    mFL.writeRaw(((float) com.motorFL) / 100);
    mFR.writeRaw(((float) com.motorFR) / 100);
    mBL.writeRaw(((float) com.motorBL) / 100);
    mBR.writeRaw(((float) com.motorBR) / 100);
  }
  com.fcTime = micros();
  handleLeds();
  com.handle();
  watchDog();
  com.loopEnd = micros();
  handleLoopFreq();
}