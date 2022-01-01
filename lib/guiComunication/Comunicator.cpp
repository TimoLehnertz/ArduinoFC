/**
 * @file Comunicator.cpp
 * @author Timo Lehnertz
 * @brief 
 * @version 0.1
 * @date 2022-01-01
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <Arduino.h>
#include <EEPROM.h>
#include "Comunicator.h"
#include <maths.h>
#include <error.h>
#include <SensorFusion.h>
#include <Adafruit_NeoPixel.h>

int strpos3(const char* haystack, const char needle, int start = 0) {
  for(int i = start; i < 100; i++) {
    if(haystack[i] == needle) return i;
  }
  return -1;
}

void Comunicator::begin() {
    Serial.println("FC comunication initiated");
    Serial2.println("FC comunication initiated");
    Serial4.begin(115200); // MSP
    msp.begin(Serial4);
    pixels->begin();
}

void Comunicator::handle() {
  while(Serial.available() || Serial2.available()) {
    char c = Serial.available() ? Serial.read() : Serial2.read();
    if(c == '\n') {
      buffer[bufferCount] = 0;
      processSerialLine();
      bufferCount = 0;
      return;
    } else {
      buffer[bufferCount] = c;
    }
    bufferCount++;
    if(bufferCount == 255)
      bufferCount = 0;
  }
  scheduleTelemetry();
  handleLED();
  handleMsp();
  handleStickCommands();
  handleCRSFTelem();
}

void Comunicator::handleStickCommands() {
  if(fc->isArmed()) return;
  if(fc->chanels.roll < -0.9 && fc->chanels.pitch < -0.9 && fc->chanels.throttle < 0.05 && fc->chanels.yaw > 0.9) {
    if(scMagCalibStart == 0) {
      scMagCalibStart = millis();
    }
    if(millis() > scMagCalibStart + 3000) {
      uint32_t color = pixels->Color(0,0,100);
      pixels->fill(color, 0, 5);
      pixels->show();
      sensors->calibrateMag();
      scMagCalibStart = 0;
    }
  } else {
    scMagCalibStart = 0;
  }
}

void Comunicator::handleLED() {
  if(!useLeds) return;
  if(millis() - (1000 / ledFreq) > lastLED) {
    pixels->clear();
    // if(fc->isArmed()) {
      // double angle = min(ins->getMaxAngleDeg(), 5);
      double angle = angleFromCoordinate(ins->sensors->gps.lat, ins->sensors->gps.lng, ins->complementaryFilter.centerLat, ins->complementaryFilter.centerLng);
      // Serial.println(ins->getYaw() * RAD_TO_DEG);
      // Serial.println(angleFromTo(360, 0));
      // Serial.println(angle);
      // Serial.println(angleFromTo(ins->getYaw() * RAD_TO_DEG, 360));
      angle = min(90, abs(angleFromTo(ins->getYaw() * RAD_TO_DEG, angle)));
      // double angle = min(90, abs(angleFromTo(ins->getYaw() * RAD_TO_DEG, 0)));

      
      double brightness = 100;

      double yaw = min(90, abs(ins->getYaw() * RAD_TO_DEG));
      
      double prog = (yaw / 90);
      uint32_t color = pixels->Color(prog * brightness, (1 - prog) * brightness * 2, 0);
      pixels->fill(color, 1, 1);
      
      prog = (angle / 90);
      color = pixels->Color(prog * brightness, (1 - prog) * brightness * 2, 0);
      pixels->fill(color, 2, 1);

      prog = !sensors->gps.locationValid;
      color = pixels->Color(prog * brightness, (1 - prog) * brightness * 2, 0);
      pixels->fill(color, 0, 1);
      
    // } else {
    //   drawLedIdle();
    // }
    pixels->show();
    lastLED = millis();
  }
}

void Comunicator::drawLedIdle() {
  uint32_t color;
  int c = (int) (sin(millis() / 500.0f) * 70);
  if(c > 0) {
    color = pixels->Color(c, 0 ,0);
  } else {
    color = pixels->Color(0, -c / 2 ,-c);
  }
  pixels->fill(color, 0, 10);
}

void Comunicator::scheduleTelemetry() {
  uint64_t now = micros();
  if(now - lastTelem > telemUs) {
    postTelemetry();
    lastTelem = now;
  }
}

// #define MSP_FC_VERSION 3
#define MSP_OSD_CONFIG 10
#define MSP_PID_ADVANCED 94
#define MSP_SET_PID_ADVANCED 95
#define MSP_SET_RC_TUNING 204
#define MSP_RC_TUNING 111

void Comunicator::handleMsp() {
  static char pos = 0;
  static uint8_t mId = 0;
  static uint8_t mSize = 0;
  bool gotData = false;
  static uint8_t mspBuff[100];
  static uint8_t mspBuffSize = 0;
  static uint8_t checksum = 0;

  while(Serial4.available()) {
    char c = Serial4.read();
    // Serial.write(c);
    if(c == '$') {
      pos = 0;
    }
    if(pos == 1 && c != 'M') {
      pos = 0;
      // Serial.println("invalid preamble");
    }
    if(pos == 2 && c != '<') {
      pos = 0;
      // Serial.println("invalid direction");
    }
    if(pos == 3) {
      mSize = c;
      // Serial.print("size:");
      // Serial.println(mSize);
    }
    if(pos == 4) {
      mId = c;
      // Serial.print("id:");
      // Serial.println(mId);
      checksum = mSize ^ mId;
      mspBuffSize = 0;
    }
    if(pos > 4 && pos - 5 < mSize) {
      mspBuff[mspBuffSize] = c;
      
      mspBuffSize++;
      checksum ^= c;
      if(mspBuffSize >= 100) {
        mspBuffSize = 0;
        Serial.println("MSP overflow");
      }
    }
    if(pos == mSize + 5) {
      if(checksum == c) {
        gotData = true;
      } else {
        Serial.print("Invalid checksum ");
        // Serial.println(checksum);
        // Serial.print("$M<,");
        // Serial.print(mSize);
        // Serial.print(",");
        // Serial.print(mId);
        // Serial.print(",");
        // for (size_t i = 0; i < mSize; i++) {
        //   Serial.print(mspBuff[i]);
        //   Serial.print(",");
        // }
        Serial.println((uint8_t)c);
      }
    }
    pos++;
  }
  
  // handle request
  if(gotData) {
    // Serial.println(mId);
    if(mId != 84 && mId != 101 && mId != 105 && mId != 106 && mId != 107 && mId != 108 && mId != 109 &&mId != 134 &&mId != 247 &&  mId != 110 && mId != 111 && mId != 112 && mId != 130 && mId != 150 && mId != 3 && mId != 10 && mId != 92 && mId != 94) {
      Serial.println(mId);
    }
    switch(mId) {
      // case MSP_RAW_GPS: {
      //   // Serial.println("writing GPS");
      //   uint8_t payload[30];
      //   payload[0] = sensors->gps.satelites > 10;
      //   payload[1] = sensors->gps.satelites;
      //   *(uint32_t*) &payload[2]  = sensors->gps.lat * 10000000;
      //   *(uint32_t*) &payload[6]  = sensors->gps.lng * 10000000;
      //   *(uint16_t*) &payload[10] = (int) ins->getLocation().z;
      //   *(uint16_t*) &payload[12] = (int) ins->getVelocity().getLength2D();
      //   *(uint16_t*) &payload[14] = (int) sensors->gps.speed * 100;
      //   *(uint16_t*) &payload[16] = (int) sensors->gps.course * 10;
      //   msp.send(MSP_RAW_GPS, payload, 30);
      //   break;
      // }
      case MSP_PID: {
        uint8_t payload[30];
        if(fc->flightMode == FlightMode::rate) {
          payload[0] = fc->rateRollPID.p * 10000.0;
          payload[1] = fc->rateRollPID.i * 10000.0;
          payload[2] = fc->rateRollPID.d * 10000.0;
          payload[3] = fc->ratePitchPID.p * 10000.0;
          payload[4] = fc->ratePitchPID.i * 10000.0;
          payload[5] = fc->ratePitchPID.d * 10000.0;
        } else if(fc->flightMode == FlightMode::level) {
          payload[0] = fc->levelRollPID.p * 10000.0;
          payload[1] = fc->levelRollPID.i * 10000.0;
          payload[2] = fc->levelRollPID.d * 10000.0;
          payload[3] = fc->levelPitchPID.p * 10000.0;
          payload[4] = fc->levelPitchPID.i * 10000.0;
          payload[5] = fc->levelPitchPID.d * 10000.0;
        } else if(fc->flightMode == FlightMode::altitudeHold) {
          payload[0] = fc->altitudePID.p * 10000.0;
          payload[1] = fc->altitudePID.i * 10000.0;
          payload[2] = fc->altitudePID.d * 10000.0;

          payload[3] = 0;
          payload[4] = 0;
          payload[5] = 0;
        } else if(fc->flightMode == FlightMode::gpsHold) {
          payload[0] = fc->velPIDx.p * 10000.0;
          payload[1] = fc->velPIDx.i * 10000.0;
          payload[2] = fc->velPIDx.d * 10000.0;

          payload[3] = 0;
          payload[4] = 0;
          payload[5] = 0;
        }

        payload[6] = fc->rateYawPID.p * 10000.0;
        payload[7] = fc->rateYawPID.i * 10000.0;
        payload[8] = fc->rateYawPID.d * 10000.0;
        
        msp.send(MSP_PID, payload, 30);
        break;
      }
      case MSP_SET_PID: {
        Serial.println("Setting PID:");
        for (size_t i = 0; i < mSize; i++) {
          Serial.println(mspBuff[i]);
        }

        if(fc->flightMode == FlightMode::rate) {
          fc->rateRollPID.p = mspBuff[0] / 10000.0;
          fc->rateRollPID.i = mspBuff[1] / 10000.0;
          fc->rateRollPID.d = mspBuff[2] / 10000.0;
          fc->ratePitchPID.p = mspBuff[3] / 10000.0;
          fc->ratePitchPID.i = mspBuff[4] / 10000.0;
          fc->ratePitchPID.d = mspBuff[5] / 10000.0;
        } else if(fc->flightMode == FlightMode::level) {
          fc->levelRollPID.p = mspBuff[0] / 10000.0;
          fc->levelRollPID.i = mspBuff[1] / 10000.0;
          fc->levelRollPID.d = mspBuff[2] / 10000.0;
          fc->levelPitchPID.p = mspBuff[3] / 10000.0;
          fc->levelPitchPID.i = mspBuff[4] / 10000.0;
          fc->levelPitchPID.d = mspBuff[5] / 10000.0;
        } else if(fc->flightMode == FlightMode::altitudeHold) {
          fc->altitudePID.p = mspBuff[0] / 10000.0;
          fc->altitudePID.i = mspBuff[1] / 10000.0;
          fc->altitudePID.d = mspBuff[2] / 10000.0;
        } else if(fc->flightMode == FlightMode::gpsHold) {
          fc->velPIDx.p = mspBuff[0] / 10000.0;
          fc->velPIDx.i = mspBuff[1] / 10000.0;
          fc->velPIDx.d = mspBuff[2] / 10000.0;

          fc->velPIDy.p = mspBuff[0] / 10000.0;
          fc->velPIDy.i = mspBuff[1] / 10000.0;
          fc->velPIDy.d = mspBuff[2] / 10000.0;
        }


        fc->rateYawPID.p = mspBuff[6] / 10000.0;
        fc->rateYawPID.i = mspBuff[7] / 10000.0;
        fc->rateYawPID.d = mspBuff[8] / 10000.0;
        break;
      }
      case MSP_SET_PID_ADVANCED: {
        Serial.println("Setting PID advanced:");
        for (size_t i = 0; i < mSize; i++) {
          Serial.println(mspBuff[i]);
        }
        fc->rateRollPID.dlpf  = mspBuff[32] / 10000.0;
        fc->ratePitchPID.dlpf = mspBuff[34] / 10000.0;
        fc->rateYawPID.dlpf   = mspBuff[36] / 10000.0;
        fc->antiGravityMul = mspBuff[21] / 100.0;
        break;
      }
      case MSP_PID_ADVANCED: {
        uint8_t payload[47];
        for (size_t i = 0; i < 47; i++)
        {
          payload[i] = 0;
        }
        
        payload[32] = fc->rateRollPID.dlpf * 10000.0;
        payload[34] = fc->ratePitchPID.dlpf * 10000.0;
        payload[36] = fc->rateYawPID.dlpf * 10000.0;
        payload[21] = fc->antiGravityMul * 100.0;
        msp.send(MSP_PID_ADVANCED, payload, 47);
        break;
      }
      case MSP_ANALOG: {
        uint8_t payload[7];
        if(useVCell) {
          payload[0] = (uint8_t) ((int) (sensors->bat.vCell * 10));
        } else {
          payload[0] = (uint8_t) ((int) (sensors->bat.vBat * 10));
        }
        msp.send(MSP_ANALOG, payload, 7);
        break;
      }
      case MSP_RC_TUNING: {
        uint8_t payload[14];

        payload[0] = round(fc->rollRate.getRC() * 100);
        payload[1] = round(fc->rollRate.getRCExpo() * 100);
        payload[2] = round(fc->rollRate.getSuper() * 100);

        payload[12] = round(fc->pitchRate.getRC() * 100);
        payload[13] = round(fc->pitchRate.getRCExpo() * 100);
        payload[3] =  round(fc->pitchRate.getSuper() * 100);

        payload[11] = round(fc->yawRate.getRC() * 100);
        payload[10] = round(fc->yawRate.getRCExpo() * 100);
        payload[4] =  round(fc->yawRate.getSuper() * 100);

        msp.send(MSP_RC_TUNING, payload, 14);
        break;
      }
      case MSP_SET_RC_TUNING: {
        Serial.println("Setting rate:");
        for (size_t i = 0; i < mSize; i++) {
          Serial.println(mspBuff[i]);
        }
        fc->rollRate.setRc (mspBuff[0] / 100.0);
        fc->rollRate.setRCExpo(mspBuff[1] / 100.0);
        fc->rollRate.setSuper(mspBuff[2] / 100.0);

        fc->pitchRate.setRc (mspBuff[12] / 100.0);
        fc->pitchRate.setRCExpo(mspBuff[13] / 100.0);
        fc->pitchRate.setSuper(mspBuff[3] / 100.0);

        fc->yawRate.setRc (mspBuff[11] / 100.0);
        fc->yawRate.setRCExpo(mspBuff[10] / 100.0);
        fc->yawRate.setSuper(mspBuff[4] / 100.0);
        break;
      }
      case MSP_STATUS: {
        // Serial.println("sending status");
        uint8_t payload[11];
        // *(uint16_t*) &payload[0] = loopTimeUs;
        // *(uint16_t*) &payload[2] = 0; // I2C Error count
        // *(uint16_t*) &payload[4] = 0; // sensors

        for (size_t i = 0; i < 11; i++) {
          payload[i] = fc->isArmed() ? 0xFF : 0;
        }
        msp.send(MSP_STATUS, payload, 11);
        break;
      }
      case 250: {
        saveEEPROM();
        break;
      }
    }
    gotData = false;
  }
  mspRoundRobin = (mspRoundRobin + 1) % 1;
}

void Comunicator::postTelemetry() {
  if(useAttiTelem) {
    postSensorData("ATTI", "Pitch", ins->getPitch());
    postSensorData("ATTI", "Roll", ins->getRoll());
    postSensorData("ATTI", "Yaw", ins->getYaw());
  }
  if(useLocTelem) {
    postSensorData("LOC", "X", ins->getLocation().x);
    postSensorData("LOC", "Y", ins->getLocation().y);
    postSensorData("LOC", "Z", ins->getLocation().z);
  }

  if(useVelTelem) {
    postSensorData("VEL", "X", ins->getVelocity().x);
    postSensorData("VEL", "Y", ins->getVelocity().y);
    postSensorData("VEL", "Z", ins->getVelocity().z);

    postSensorData("VEL(Local)", "X", ins->getLocalVelocity().x);
    postSensorData("VEL(Local)", "Y", ins->getLocalVelocity().y);
    postSensorData("VEL(Local)", "Z", ins->getLocalVelocity().z);
  }

  if(useQuatTelem) {
    postSensorData("Q", "W", ins->getQuaternionRotation().w);
    postSensorData("Q", "X", ins->getQuaternionRotation().x);
    postSensorData("Q", "Y", ins->getQuaternionRotation().y);
    postSensorData("Q", "Z", ins->getQuaternionRotation().z);
  }

  if(useGyroTelem) {
    postSensorData("GYRO", "X", sensors->gyro.x);
    postSensorData("GYRO", "Y", sensors->gyro.y);
    postSensorData("GYRO", "Z", sensors->gyro.z);
  }

  if(useAccTelem) {
    postSensorData("ACC", "X", sensors->acc.x);
    postSensorData("ACC", "Y", sensors->acc.y);
    postSensorData("ACC", "Z", sensors->acc.z);
  }

  if(useMagTelem) {
    postSensorData("MAG", "X", sensors->mag.x);
    postSensorData("MAG", "Y", sensors->mag.y);
    postSensorData("MAG", "Z", sensors->mag.z);
  }

  if(useBaroTelem) {
    postSensorData("BARO", "Alt", ins->complementaryFilter.baroAltitude);
    // postSensorData("BARO(f)", "Alt", ins->getLastFilteredBaroAltitude());
    postSensorData("BARO(speed m/s)", "Alt", ins->complementaryFilter.baroAltSpeed);
    postSensorData("BARO(raw)", "Alt", sensors->baro.altitude);
  }

  if(useGpsTelem) {
    if(sensors->gps.locationValid) {
      postSensorData("GPS LAT", "LAT", sensors->gps.lat);
      postSensorData("GPS LNG", "LNG", sensors->gps.lng);
    }
    if(sensors->gps.altitudeValid) {
      postSensorData("GPS", "alt", sensors->gps.altitude);
    }
    if(sensors->gps.speedValid) {
      postSensorData("GPS", "spd", sensors->gps.speed);
    }
    postSensorDataInt("GPS", "Sat", sensors->gps.satelites);
  }
  if(useTimingTelem) {
    maxLoopTime = max(maxLoopTime, loopTimeUs);

    postSensorData("FREQ", "loopHz", actualFreq);
    postSensorDataInt("TIME", "CRSF Us", crsfTime - loopStart);
    postSensorDataInt("TIME", "Sens Us", sensorsTime - crsfTime);
    postSensorDataInt("TIME", "INS Us", insTime - sensorsTime);
    postSensorDataInt("TIME", "Chan Us", chanelsTime - insTime);
    postSensorDataInt("TIME", "FC Us", fcTime - chanelsTime);
    postSensorData("CPU Load", "", cpuLoad);
    postSensorDataInt("Loop time Us", "", loopTimeUs);
    postSensorDataInt("Sensor Poll Us", "Acc", sensors->acc.lastPollTime);
    postSensorDataInt("Sensor Poll Us", "Gyro", sensors->gyro.lastPollTime);
    postSensorDataInt("Sensor Poll Us", "Mag", sensors->mag.lastPollTime);
    postSensorDataInt("Sensor Poll Us", "Baro", sensors->baro.lastPollTime);
    postSensorDataInt("Sensor Poll Us", "GPS", sensors->gps.lastPollTime);
    postSensorDataInt("Max Loop Time", "Us", maxLoopTime);
    postSensorDataInt("Min Freq", "Hz", 1000000.0f / maxLoopTime);
  }
  if(useRCTelem) {
    postSensorDataInt("RC", "CH1", fc->chanelsRaw.chanels[0]);
    postSensorDataInt("RC", "CH2", fc->chanelsRaw.chanels[1]);
    postSensorDataInt("RC", "CH3", fc->chanelsRaw.chanels[2]);
    postSensorDataInt("RC", "CH4", fc->chanelsRaw.chanels[3]);
    postSensorDataInt("RC", "CH5", fc->chanelsRaw.chanels[4]);
    postSensorDataInt("RC", "CH6", fc->chanelsRaw.chanels[5]);
    postSensorDataInt("RC", "CH7", fc->chanelsRaw.chanels[6]);
    postSensorDataInt("RC", "CH8", fc->chanelsRaw.chanels[7]);
    postSensorDataInt("RC", "CH9", fc->chanelsRaw.chanels[8]);
    postSensorDataInt("RC", "CH10", fc->chanelsRaw.chanels[9]);
    postSensorDataInt("RC", "CH11", fc->chanelsRaw.chanels[10]);
    postSensorDataInt("RC", "CH12", fc->chanelsRaw.chanels[11]);
  }
  if(useFCTelem) {
    // postSensorData("RateAdj", "Roll", fc->rollRateAdjust);
    // postSensorData("RateAdj", "Pitch", fc->pitchRateAdjust);
    // postSensorData("RateAdj", "Yaw", fc->yawRateAdjust);
    postSensorData("Rate PID Roll", fc->rateRollPID);
    postSensorData("Rate PID Pitch", fc->ratePitchPID);
    // // postSensorData("Rate PID Yaw", fc->rateYawPID);
    // postSensorData("Level PID Roll", fc->levelRollPID);
    // // postSensorData("Level PID Pitch", fc->levelPitchPID);
    // postSensorData("Altitude PID", fc->altitudePID);
    // postSensorData("Vel PIDx", fc->velPIDx);
    // postSensorData("Vel PIDy", fc->velPIDy);
    // postSensorData("Anti Gravity", "boost", fc->iBoost);
    postSensorDataInt("Flight mode", "Mode", fc->flightMode);
    postSensorData("Alti PID", fc->altitudePID);
  }
  if(useBatTelem) {
    postSensorData("vBat", "Voltage", sensors->bat.vBat);
    postSensorData("vCell", "Voltage", sensors->bat.vCell);
    postSensorDataInt("Cell count", "count", sensors->bat.cellCount);
  }
  if(useUltrasonicTelem) {
    postSensorData("Ultrasonic", "distance", sensors->ultrasonic.distance);
    postSensorData("Ultrasonic", "speed(ms)", sensors->ultrasonic.speed);
  }
}

void Comunicator::end() {
  Serial.println("ending comunication");
  Serial.end();

  Serial2.println("ending comunication");
  Serial2.end();
}

void Comunicator::postSensorDataInt(const char* sensorName, const char* subType, uint64_t value) {
  Serial.print("FC_S");
  Serial.print(' ');
  Serial.print(sensorName);
  Serial.print(';');
  Serial.print(subType);
  Serial.print(';');
  Serial.println(value, 10);

  Serial2.print("FC_S");
  Serial2.print(' ');
  Serial2.print(sensorName);
  Serial2.print(';');
  Serial2.print(subType);
  Serial2.print(';');
  Serial2.println(value, 10);
}

void Comunicator::postSensorDataDouble(const char* sensorName, const char* subType, double value) {
  Serial.print("FC_S");
  Serial.print(' ');
  Serial.print(sensorName);
  Serial.print(';');
  Serial.print(subType);
  Serial.print(';');
  Serial.println(value, 10);

  Serial2.print("FC_S");
  Serial2.print(' ');
  Serial2.print(sensorName);
  Serial2.print(';');
  Serial2.print(subType);
  Serial2.print(';');
  Serial2.println(value, 10);
}

void Comunicator::postSensorData(const char* sensorName, const char* subType, float value) {
  Serial.print("FC_S");
  Serial.print(' ');
  Serial.print(sensorName);
  Serial.print(';');
  Serial.print(subType);
  Serial.print(';');
  Serial.println(value, 5);

  Serial2.print("FC_S");
  Serial2.print(' ');
  Serial2.print(sensorName);
  Serial2.print(';');
  Serial2.print(subType);
  Serial2.print(';');
  Serial2.println(value, 5);
}

void Comunicator::postSensorData(const char* sensorName, PID pid) {
  postSensorData(sensorName, "P", pid.prevP);
  postSensorData(sensorName, "I", pid.prevI);
  postSensorData(sensorName, "D", pid.prevD);
  postSensorData(sensorName, "SUM", pid.prevOut);
}

void Comunicator::post(const char* command, const char* value) {
  Serial.print(command);
  Serial.print(' ');
  Serial.println(value);

  Serial2.print(command);
  Serial2.print(' ');
  Serial2.println(value);
}

void Comunicator::processSerialLine() {
  char* command;
  if(bufferCount > 7 && strncmp("FC_GET_", buffer, 7) == 0) {
    command = buffer + 7;
    char* uid = nullptr;
    for(int i = 8; i < bufferCount; i++) {
      if(buffer[i] == ' ') {
        uid = buffer + i + 1;
        break;
      }
    }
    if(uid == nullptr) {
      Serial.println(F("Invalid request. No id"));
      Serial2.println(F("Invalid request. No id"));
      return;
    }

  // FC_GET
    if(strncmp("0", command, 1) == 0) {
      postResponse(uid, "0");
    }
    if(strncmp("ACC_OFFSET", command, 10) == 0) {
      postResponse(uid, sensors->getAccOffset());
    }
    if(strncmp("GYRO_OFFSET", command, 11) == 0) {
      postResponse(uid, sensors->getGyroOffset());
    }
    if(strncmp("GYRO_SCALE", command, 10) == 0) {
      postResponse(uid, sensors->getGyroScale());
    }
    if(strncmp("ACC_LPF", command,7) == 0) {
      postResponse(uid, sensors->acc.lpf);
    }
    if(strncmp("GYRO_LPF", command,7) == 0) {
      postResponse(uid, sensors->gyro.lpf);
    }
    if(strncmp("COMPLEMENTARY_ACC_INF", command, 21) == 0) {
      postResponse(uid, ins->complementaryFilter.accInfluence);
    }
    if(strncmp("USE_ACC_TELEM", command, 13) == 0) {
      postResponse(uid, useAccTelem);
    }
    if(strncmp("USE_GYRO_TELEM", command, 14) == 0) {
      postResponse(uid, useGyroTelem);
    }
    if(strncmp("USE_MAG_TELEM", command, 13) == 0) {
      postResponse(uid, useMagTelem);
    }
    if(strncmp("USE_BARO_TELEM", command, 14) == 0) {
      postResponse(uid, useBaroTelem);
    }
    if(strncmp("USE_GPS_TELEM", command, 13) == 0) {
      postResponse(uid, useGpsTelem);
    }
    if(strncmp("USE_ATTI_TELEM", command, 14) == 0) {
      postResponse(uid, useAttiTelem);
    }
    if(strncmp("USE_QUAT_TELEM", command, 14) == 0) {
      postResponse(uid, useQuatTelem);
    }
    if(strncmp("USE_VEL_TELEM", command, 13) == 0) {
      postResponse(uid, useVelTelem);
    }
    if(strncmp("USE_LOC_TELEM", command, 13) == 0) {
      postResponse(uid, useLocTelem);
    }
    if(strncmp("USE_BAT_TELEM", command, 13) == 0) {
      postResponse(uid, useBatTelem);
    }
    if(strncmp("BAT_LPF", command, 7) == 0) {
      postResponse(uid, sensors->batLpf);
    }
    if(strncmp("OVERWRITE_MOTORS", command, 16) == 0) {
      postResponse(uid, motorOverwrite);
    }
    if(strncmp("M1_OVERWRITE", command, 12) == 0) {
      postResponse(uid, motorFL);
    }
    if(strncmp("M2_OVERWRITE", command, 12) == 0) {
      postResponse(uid, motorFR);
    }
    if(strncmp("M3_OVERWRITE", command, 12) == 0) {
      postResponse(uid, motorBL);
    }
    if(strncmp("M4_OVERWRITE", command, 12) == 0) {
      postResponse(uid, motorBR);
    }
    if(strncmp("PROPS_IN", command, 8) == 0) {
      postResponse(uid, fc->propsIn);
    }

    if(strncmp("USE_TIMING", command, 10) == 0) {
      postResponse(uid, useTimingTelem);
    }
    if(strncmp("USE_RC_TELEM", command, 12) == 0) {
      postResponse(uid, useRCTelem);
    }
    if(strncmp("USE_FC_TELEM", command, 12) == 0) {
      postResponse(uid, useFCTelem);
    }
    if(strncmp("LOOP_FREQ_RATE", command, 14) == 0) {
      postResponse(uid, loopFreqRate);
    }
    if(strncmp("LOOP_FREQ_LEVEL", command, 15) == 0) {
      postResponse(uid, loopFreqLevel);
    }
    if(strncmp("COMPLEMENTARY_MAG_INF", command, 21) == 0) {
      postResponse(uid, ins->complementaryFilter.magInfluence);
    }
    if(strncmp("USE_LEDS", command, 8) == 0) {
      postResponse(uid, useLeds);
    }

    if(strncmp("RATE_PID_R", command, 10) == 0) {
      postResponse(uid, fc->rateRollPID);
    }
    if(strncmp("RATE_PID_P", command, 10) == 0) {
      postResponse(uid, fc->ratePitchPID);
    }
    if(strncmp("RATE_PID_Y", command, 10) == 0) {
      postResponse(uid, fc->rateYawPID);
    }
    
    if(strncmp("LEVEL_PID_R", command, 11) == 0) {
      postResponse(uid, fc->levelRollPID);
    }
    if(strncmp("LEVEL_PID_P", command, 11) == 0) {
      postResponse(uid, fc->levelPitchPID);
    }
    if(strncmp("LEVEL_PID_Y", command, 11) == 0) {
      postResponse(uid, fc->levelYawPID);
    }

    if(strncmp("ALTITUDE_PID", command, 12) == 0) {
      postResponse(uid, fc->altitudePID);
    }

    if(strncmp("VEL_PID", command, 7) == 0) {
      postResponse(uid, fc->velPIDx);
    }

    if(strncmp("I_RELAX_MIN_RATE", command, 16) == 0) {
      postResponse(uid, fc->iRelaxMinRate);
    }

    if(strncmp("USE_ANTI_GRAVITY", command, 16) == 0) {
      postResponse(uid, fc->useAntiGravity);
    }
    if(strncmp("ANTI_GRAVITY_MULTIPLICATOR", command, 26) == 0) {
      postResponse(uid, fc->antiGravityMul);
    }
    if(strncmp("ANTI_GRAVITY_SPEED", command, 18) == 0) {
      postResponse(uid, fc->boostSpeed);
    }
    if(strncmp("ANTI_GRAVITY_LPF", command, 16) == 0) {
      postResponse(uid, fc->boostLpf);
    }
    if(strncmp("OVERWRITE_FM", command, 12) == 0) {
      postResponse(uid, fc->overwriteFlightMode);
    }
    if(strncmp("USE_VCELL", command, 9) == 0) {
      postResponse(uid, useCellVoltage);
    }
    if(strncmp("ACC_SCALE", command, 9) == 0) {
      postResponse(uid, sensors->getAccScale());
    }
    if(strncmp("MAG_OFFSET", command, 10) == 0) {
      postResponse(uid, sensors->getMagOffset());
    }
    if(strncmp("MAG_SCALE", command, 9) == 0) {
      postResponse(uid, sensors->getMagScale());
    }
    if(strncmp("SENSOR_FUSION", command, 13) == 0) {
      postResponse(uid, ins->getFusionAlgorythm());
    }
    if(strncmp("ANGLE_MODE_MAX_ANGLE", command, 20) == 0) {
      postResponse(uid, fc->angleModeMaxAngle);
    }
    if(strncmp("GPS_MAX_SPEED_HORIZONTAL", command, 24) == 0) {
      postResponse(uid, fc->gpsMaxSpeedHorizontal);
    }
    if(strncmp("GPS_MAX_SPEED_VERTICAL", command, 22) == 0) {
      postResponse(uid, fc->gpsMaxSpeedVertical);
    }
    if(strncmp("HOVER_THROTTLE", command, 14) == 0) {
      postResponse(uid, fc->hoverThrottle);
    }
    if(strncmp("LAUNCH_I_BOOST_SECONDS", command, 22) == 0) {
      postResponse(uid, fc->launchIBoostSeconds);
    }
    if(strncmp("LAUNCH_I_BOOST_LEVEL", command, 20) == 0) {
      postResponse(uid, fc->launchIBoostLevel);
    }
    if(strncmp("LAUNCH_I_BOOST_ALTITUDE", command, 23) == 0) {
      postResponse(uid, fc->launchIBoostAltitude);
    }
    if(strncmp("LAUNCH_I_BOOST_ALTITUDE", command, 23) == 0) {
      postResponse(uid, fc->launchIBoostAltitude);
    }
    if(strncmp("ROLL_RATES", command, 10) == 0) {
      postResponse(uid, fc->rollRate.toVec3());
    }
    if(strncmp("PITCH_RATES", command, 11) == 0) {
      postResponse(uid, fc->pitchRate.toVec3());
    }
    if(strncmp("YAW_RATES", command, 9) == 0) {
      postResponse(uid, fc->yawRate.toVec3());
    }
    if(strncmp("MAG_Z_OFFSET", command, 12) == 0) {
      postResponse(uid, ins->getMagZOffset());
    }
    if(strncmp("USE_ULTRASONIC_TELEM", command, 20) == 0) {
      postResponse(uid, useUltrasonicTelem);
    }
    if(strncmp("M1_PIN", command, 6) == 0) {
      postResponse(uid, fc->getMotorPin(1));
    }
    if(strncmp("M2_PIN", command, 6) == 0) {
      postResponse(uid, fc->getMotorPin(2));
    }
    if(strncmp("M3_PIN", command, 6) == 0) {
      postResponse(uid, fc->getMotorPin(3));
    }
    if(strncmp("M4_PIN", command, 6) == 0) {
      postResponse(uid, fc->getMotorPin(4));
    }
    if(strncmp("THROTTLE_MUL_4S", command,15) == 0) {
      postResponse(uid, fc->throttleMul4S);
    }
    if(strncmp("THROTTLE_MUL_6S", command,15) == 0) {
      postResponse(uid, fc->throttleMul6S);
    }
  }

  // FC_DO
  if(bufferCount > 6 && strncmp("FC_DO_", buffer, 6) == 0) {
    command = buffer + 6;
    if(strncmp("ACC_CALIB_QUICK", command, 15) == 0) {
      Serial.println("Calibrating Accelerometer(Quick)");
      sensors->calibrateAcc();
      Serial.print("Done! Offset: ");
      sensors->getAccOffset().println();
      Serial.print(", Scale: ");
      sensors->getAccScale().println();
    }
    if(strncmp("ACC_CALIB", command, 9) == 0) {
      Serial.println("Calibrating Accelerometer(full)");
      sensors->calibrateAcc();
      Serial.print("Done! Offset: ");
      sensors->getAccOffset().println();
      Serial.print(", Scale: ");
      sensors->getAccScale().println();
    }
    if(strncmp("GYRO_CALIB_OFFSET", command, 17) == 0) {
      Serial.println("Calibrating Gyroscope");
      sensors->calibrateGyroOffset();
      Serial.println("Done!");
    }
    if(strncmp("GYRO_CALIB_SCALE", command, 16) == 0) {
      Serial.println("Calibrating Gyroscope");
      sensors->calibrateGyroScale();
      Serial.println("Done!");
    }
    if(strncmp("MAG_CALIB", command, 9) == 0) {
        Serial.println("Calibrating magnetometer");
        delay(100);
        sensors->calibrateMag();
        Serial.print("Done! Offset: ");
        sensors->getMagOffset().println();
        Serial.print(", Scale: ");
        sensors->getMagScale().println();
    }
    if(strncmp("STOP_MAG_CALIB", command, 14) == 0) {
        useMagTelem = false;
    }
    if(strncmp("RESET_INS", command, 9) == 0) {
      Serial.println("resetting INS");
      Serial2.println("resetting INS");
      ins->reset();
    }
    if(strncmp("SAVE_EEPROM", command, 11) == 0) {
      saveEEPROM();
      Serial.println("Successfully saved settings to EEPROM");
      Serial2.println("Successfully saved settings to EEPROM");
    }
    if(strncmp("ERASE_EEPROM", command, 12) == 0) {
      Storage::writeDefaults();
    }
    if(strncmp("REBOOT", command, 12) == 0) {
      SCB_AIRCR = 0x05FA0004;
    }
  }

  //FC_POST
  // if(bufferCount > 8 && strncmp("FC_POST_", buffer, 8) == 0) {
  //   command = buffer + 8;
  //   int valueStart = strpos3(buffer, ' ', 8) + 1;
  //   if(valueStart == -1) return;
  //   char* value = buffer + valueStart;
  //   /**
  //    * POST HERE
  //    */
  // }
  // FC_SET
  if(bufferCount > 7 && strncmp("FC_SET_", buffer, 7) == 0) {
    command = buffer + 7;
    int uidStart = strpos3(buffer, ' ', 7) + 1;
    if(uidStart == -1) {
      Serial.println("Invalid SET. No id");
      Serial2.println("Invalid SET. No id");
      return;
    }
    char* uid = buffer + uidStart;
    int valueStart = strpos3(buffer, ' ', uidStart + 1) + 1;
    buffer[valueStart - 1] = 0;
    if(valueStart == -1) {
      Serial.println("Invalid SET. No value");
      Serial2.println("Invalid SET. No value");
      return;
    }
    char* value = buffer + valueStart;
    if(strncmp("GYRO_OFFSET", command, 11) == 0) {
      postResponse(uid, value);
      sensors->setGyroCal(Vec3(value), sensors->getGyroScale());
      // ins->setGyroOffset(Vec3(value));
    }
    if(strncmp("GYRO_SCALE", command, 10) == 0) {
      postResponse(uid, value);
      sensors->setGyroCal(sensors->getGyroOffset(), Vec3(value));
      // ins->setGyroOffset(Vec3(value));
    }
    if(strncmp("ACC_LPF", command, 7) == 0) {
      postResponse(uid, value);
      sensors->acc.lpf = atof(value);
    }
    if(strncmp("GYRO_LPF", command, 8) == 0) {
      postResponse(uid, value);
      sensors->gyro.lpf = atof(value);
    }
    if(strncmp("COMPLEMENTARY_ACC_INF", command, 21) == 0) {
      postResponse(uid, value);
      ins->complementaryFilter.accInfluence = atof(value);
    }
    if(strncmp("ACC_TELEM", command, 9) == 0) {
      postResponse(uid, value);
      useAccTelem = value[0] == 't';
    }
    if(strncmp("GYRO_TELEM", command, 10) == 0) {
      postResponse(uid, value);
      useGyroTelem = value[0] == 't';
    }
    if(strncmp("MAG_TELEM", command, 9) == 0) {
      postResponse(uid, value);
      useMagTelem = value[0] == 't';
    }
    if(strncmp("BARO_TELEM", command, 10) == 0) {
      postResponse(uid, value);
      useBaroTelem = value[0] == 't';
    }
    if(strncmp("GPS_TELEM", command, 9) == 0) {
      postResponse(uid, value);
      useGpsTelem = value[0] == 't';
    }
    if(strncmp("VEL_TELEM", command, 9) == 0) {
      postResponse(uid, value);
      useVelTelem = value[0] == 't';
    }
    if(strncmp("LOC_TELEM", command, 9) == 0) {
      postResponse(uid, value);
      useLocTelem = value[0] == 't';
    }
    if(strncmp("LOC_TELEM", command, 9) == 0) {
      postResponse(uid, value);
      useLocTelem = value[0] == 't';
    }
    if(strncmp("ATTI_TELEM", command, 10) == 0) {
      postResponse(uid, value);
      useAttiTelem = value[0] == 't';
    }
    if(strncmp("QUAT_TELEM", command, 10) == 0) {
      postResponse(uid, value);
      useQuatTelem = value[0] == 't';
    }
    if(strncmp("BAT_TELEM", command, 9) == 0) {
      postResponse(uid, value);
      useBatTelem = value[0] == 't';
    }
    if(strncmp("BAT_LPF", command, 7) == 0) {
      postResponse(uid, value);
      sensors->batLpf = atof(value);
    }
    if(strncmp("OVERWRITE_MOTORS", command, 16) == 0) {
      postResponse(uid, value);
      motorOverwrite = value[0] == 't';
    }
    if(strncmp("M1_OVERWRITE", command, 12) == 0) {
      postResponse(uid, value);
      motorFL = atoi(value);
    }
    if(strncmp("M2_OVERWRITE", command, 12) == 0) {
      postResponse(uid, value);
      motorFR = atoi(value);
    }    
    if(strncmp("M3_OVERWRITE", command, 12) == 0) {
      postResponse(uid, value);
      motorBL = atoi(value);
    }
    if(strncmp("M4_OVERWRITE", command, 12) == 0) {
      postResponse(uid, value);
      motorBR = atoi(value);
    }
    if(strncmp("PROPS_IN", command, 8) == 0) {
      postResponse(uid, value);
      fc->propsIn = value[0] == 't';
    }

    if(strncmp("USE_TIMING", command, 10) == 0) {
      postResponse(uid, value);
      useTimingTelem = value[0] == 't';
    }
    if(strncmp("USE_RC_TELEM", command, 12) == 0) {
      postResponse(uid, value);
      useRCTelem = value[0] == 't';
    }
    if(strncmp("USE_FC_TELEM", command, 12) == 0) {
      postResponse(uid, value);
      useFCTelem = value[0] == 't';
    }
    if(strncmp("LOOP_FREQ_RATE", command, 14) == 0) {
      postResponse(uid, value);
      loopFreqRate = atoi(value);
      if(loopFreqRate < 10) loopFreqRate = 10;
    }
    if(strncmp("LOOP_FREQ_LEVEL", command, 15) == 0) {
      postResponse(uid, value);
      loopFreqLevel = atoi(value);
      if(loopFreqLevel < 10) loopFreqLevel = 10;
    }
    if(strncmp("COMPLEMENTARY_MAG_INF", command, 21) == 0) {
      postResponse(uid, value);
      ins->complementaryFilter.magInfluence = atof(value);
    }
    if(strncmp("USE_LEDS", command, 8) == 0) {
      postResponse(uid, value);
      useLeds = value[0] == 't';
    }

    if(strncmp("RATE_PID_R", command, 10) == 0) {
      postResponse(uid, value);
      fc->rateRollPID = PID(value);
    }
    if(strncmp("RATE_PID_P", command, 10) == 0) {
      postResponse(uid, value);
      fc->ratePitchPID = PID(value);
    }
    if(strncmp("RATE_PID_Y", command, 10) == 0) {
      postResponse(uid, value);
      fc->rateYawPID = PID(value);
    }

    if(strncmp("LEVEL_PID_R", command, 11) == 0) {
      postResponse(uid, value);
      fc->levelRollPID = PID(value);
    }
    if(strncmp("LEVEL_PID_P", command, 11) == 0) {
      postResponse(uid, value);
      fc->levelPitchPID = PID(value);
    }
    if(strncmp("LEVEL_PID_Y", command, 11) == 0) {
      postResponse(uid, value);
      fc->levelYawPID = PID(value);
    }

    if(strncmp("ALTITUDE_PID", command, 12) == 0) {
      postResponse(uid, value);
      fc->altitudePID = PID(value);
    }

    if(strncmp("VEL_PID", command, 7) == 0) {
      postResponse(uid, value);
      fc->velPIDx = PID(value);
      fc->velPIDy = PID(value);
    }


    if(strncmp("I_RELAX_MIN_RATE", command, 16) == 0) {
      postResponse(uid, value);
      fc->iRelaxMinRate = atof(value);
    }

    if(strncmp("USE_ANTI_GRAVITY", command, 16) == 0) {
      postResponse(uid, value);
      fc->useAntiGravity = value[0] == 't';
    }
    if(strncmp("ANTI_GRAVITY_MULTIPLICATOR", command, 26) == 0) {
      postResponse(uid, value);
      fc->antiGravityMul = atof(value);
    }
    if(strncmp("ANTI_GRAVITY_SPEED", command, 18) == 0) {
      postResponse(uid, value);
      fc->boostSpeed = atof(value);
    }
    if(strncmp("ANTI_GRAVITY_LPF", command, 16) == 0) {
      postResponse(uid, value);
      fc->boostLpf = atof(value);
    }
    if(strncmp("OVERWRITE_FM", command, 12) == 0) {
      int fm = atoi(value);
      if(fm >= 0 && fm < FlightMode::FlightModeSize) {
        postResponse(uid, value);
        fc->overwriteFlightMode = FlightMode::FlightMode_t(fm);
      } else {
        fc->overwriteFlightMode = FlightMode::none;
      }
    }
    if(strncmp("VOLTAGE_CALIB", command, 13) == 0) {
      postResponse(uid, value);
      sensors->calibrateBat(atof(value));
    }
    if(strncmp("USE_VCELL", command, 9) == 0) {
      postResponse(uid, value);
      useCellVoltage = value[0] == 't';
    }
    if(strncmp("ACC_OFFSET", command, 10) == 0) {
      postResponse(uid, value);
      sensors->setAccCal(Vec3(value), sensors->getAccScale());
    }
    if(strncmp("ACC_SCALE", command, 9) == 0) {
      postResponse(uid, value);
      sensors->setAccCal(sensors->getAccOffset(), Vec3(value));
    }
    if(strncmp("MAG_OFFSET", command, 10) == 0) {
      postResponse(uid, value);
      sensors->setMagCal(Vec3(value), sensors->getMagScale());
    }
    if(strncmp("MAG_SCALE", command, 9) == 0) {
      postResponse(uid, value);
      sensors->setMagCal(sensors->getMagOffset(), Vec3(value));
    }
    if(strncmp("SENSOR_FUSION", command, 13) == 0) {
      postResponse(uid, value);
      ins->setFusionAlgorythm(SensorFusion::FusionAlgorythm(int(value)));
    }
    if(strncmp("ANGLE_MODE_MAX_ANGLE", command, 20) == 0) {
      postResponse(uid, value);
      fc->angleModeMaxAngle = atof(value);
    }
    if(strncmp("GPS_MAX_SPEED_HORIZONTAL", command, 24) == 0) {
      postResponse(uid, value);
      fc->gpsMaxSpeedHorizontal = atof(value);
    }
    if(strncmp("GPS_MAX_SPEED_VERTICAL", command, 22) == 0) {
      postResponse(uid, value);
      fc->gpsMaxSpeedVertical = atof(value);
    }
    if(strncmp("HOVER_THROTTLE", command, 14) == 0) {
      postResponse(uid, value);
      fc->hoverThrottle = atof(value);
    }
    if(strncmp("LAUNCH_I_BOOST_SECONDS", command, 22) == 0) {
      postResponse(uid, value);
      fc->launchIBoostSeconds = atof(value);
    }
    if(strncmp("LAUNCH_I_BOOST_LEVEL", command, 20) == 0) {
      postResponse(uid, value);
      fc->launchIBoostLevel = atof(value);
    }
    if(strncmp("LAUNCH_I_BOOST_ALTITUDE", command, 23) == 0) {
      postResponse(uid, value);
      fc->launchIBoostAltitude = atof(value);
    }
    if(strncmp("ROLL_RATES", command, 10) == 0) {
      postResponse(uid, value);
      fc->rollRate = Rates(value);
    }
    if(strncmp("PITCH_RATES", command, 11) == 0) {
      postResponse(uid, value);
      fc->pitchRate = Rates(value);
    }
    if(strncmp("YAW_RATES", command, 9) == 0) {
      postResponse(uid, value);
      fc->yawRate = Rates(value);
    }
    if(strncmp("MAG_Z_OFFSET", command, 12) == 0) {
      postResponse(uid, value);
      ins->setMagZOffset(atof(value));
    }
    if(strncmp("USE_ULTRASONIC_TELEM", command, 20) == 0) {
      postResponse(uid, value);
      useUltrasonicTelem = value[0] == 't';
    }
    if(strncmp("M1_PIN", command, 6) == 0) {
      postResponse(uid, value);
      fc->setMotorPin(1, atoi(value));
    }
    if(strncmp("M2_PIN", command, 6) == 0) {
      postResponse(uid, value);
      fc->setMotorPin(2, atoi(value));
    }
    if(strncmp("M3_PIN", command, 6) == 0) {
      postResponse(uid, value);
      fc->setMotorPin(3, atoi(value));
    }
    if(strncmp("M4_PIN", command, 6) == 0) {
      postResponse(uid, value);
      fc->setMotorPin(4, atoi(value));
    }
    if(strncmp("THROTTLE_MUL_4S", command, 15) == 0) {
      postResponse(uid, value);
      fc->throttleMul4S = atof(value);
    }
    if(strncmp("THROTTLE_MUL_6S", command, 15) == 0) {
      postResponse(uid, value);
      fc->throttleMul6S = atof(value);
    }
  }
}

void Comunicator::postResponse(char* uid, Vec3 vec) {
  Serial.print("FC_RES ");
  Serial.print(uid);
  Serial.print(" ");
  Serial.print(",");
  Serial.print(vec.x, 5);
  Serial.print(",");
  Serial.print(vec.y, 5);
  Serial.print(",");
  Serial.print(vec.z, 5);
  Serial.println(",");

  Serial2.print("FC_RES ");
  Serial2.print(uid);
  Serial2.print(" ");
  Serial2.print(",");
  Serial2.print(vec.x, 5);
  Serial2.print(",");
  Serial2.print(vec.y, 5);
  Serial2.print(",");
  Serial2.print(vec.z, 5);
  Serial2.println(",");
}

void Comunicator::postResponse(char* uid, Matrix3 mat) {
  Serial.print("FC_RES ");
  Serial.print(uid);
  Serial.print(" ");
  for (size_t i = 0; i < 9; i++) {
    Serial.print(mat.m[i], 5);
    Serial.print(",");
  }
  Serial.println();
  
  Serial2.print("FC_RES ");
  Serial2.print(uid);
  Serial2.print(" ");
  for (size_t i = 0; i < 9; i++) {
    Serial2.print(mat.m[i], 5);
    Serial2.print(",");
  }
  Serial2.println();
}

void Comunicator::postResponse(char* uid, const char* body) {
  Serial.print("FC_RES ");
  Serial.print(uid);
  Serial.print(" ");
  Serial.println(body);

  Serial2.print("FC_RES ");
  Serial2.print(uid);
  Serial2.print(" ");
  Serial2.println(body);
}

void Comunicator::postResponse(char* uid, float num) {
  Serial.print("FC_RES ");
  Serial.print(uid);
  Serial.print(" ");
  Serial.println(num, 5);

  Serial2.print("FC_RES ");
  Serial2.print(uid);
  Serial2.print(" ");
  Serial2.println(num, 5);
}

void Comunicator::postResponse(char* uid, double num) {
  Serial.print("FC_RES ");
  Serial.print(uid);
  Serial.print(" ");
  Serial.println(num, 5);

  Serial2.print("FC_RES ");
  Serial2.print(uid);
  Serial2.print(" ");
  Serial2.println(num, 5);
}

void Comunicator::postResponse(char* uid, bool val) {
  Serial.print("FC_RES ");
  Serial.print(uid);
  Serial.print(" ");

  Serial2.print("FC_RES ");
  Serial2.print(uid);
  Serial2.print(" ");
  if(val) {
    Serial.println("true");
    Serial2.println("true");
  } else {
    Serial.println("false");
    Serial2.println("false");
  }
}

void Comunicator::postResponse(char* uid, int val) {
  Serial.print("FC_RES ");
  Serial.print(uid);
  Serial.print(" ");
  Serial.println(val);

  Serial2.print("FC_RES ");
  Serial2.print(uid);
  Serial2.print(" ");
  Serial2.println(val);
}

void Comunicator::postResponse(char* uid, PID pid) {
  Serial.print("FC_RES ");
  Serial.print(uid);
  Serial.print(" ");
  Serial.print(",");
  Serial.print(pid.p, 5);
  Serial.print(",");
  Serial.print(pid.i, 5);
  Serial.print(",");
  Serial.print(pid.d, 5);
  Serial.print(",");
  Serial.print(pid.dlpf, 5);
  Serial.print(",");
  Serial.print(pid.maxOut, 5);
  Serial.print(",");
  Serial.print(pid.useAuxTuning ? "1" : "0");
  Serial.println(",");

  Serial2.print(uid);
  Serial2.print(" ");
  Serial2.print(",");
  Serial2.print(pid.p, 5);
  Serial2.print(",");
  Serial2.print(pid.i, 5);
  Serial2.print(",");
  Serial2.print(pid.d, 5);
  Serial2.print(",");
  Serial2.print(pid.dlpf, 5);
  Serial2.print(",");
  Serial2.print(pid.maxOut, 5);
  Serial2.print(",");
  Serial2.print(pid.useAuxTuning ? "1" : "0");
  Serial2.println(",");
}

void Comunicator::saveEEPROM() {
  Storage::write(Vec3Values::accOffset,  sensors->getAccOffset());
  Storage::write(Vec3Values::accScale,  sensors->getAccScale());
  Storage::write(Vec3Values::gyroOffset, sensors->getGyroOffset());
  Storage::write(Vec3Values::gyroScale, sensors->getGyroScale());
  Storage::write(Vec3Values::magOffset, sensors->getMagOffset());
  Storage::write(Vec3Values::magScale, sensors->getMagScale());

  Storage::write(FloatValues::m1Pin, fc->getMotorPin(1));
  Storage::write(FloatValues::m2Pin, fc->getMotorPin(2));
  Storage::write(FloatValues::m3Pin, fc->getMotorPin(3));
  Storage::write(FloatValues::m4Pin, fc->getMotorPin(4));

  Storage::write(FloatValues::throttleMul4S, fc->throttleMul4S);
  Storage::write(FloatValues::throttleMul6S, fc->throttleMul6S);

  Storage::write(FloatValues::accInsInf, ins->complementaryFilter.accInfluence);
  Storage::write(FloatValues::magInsInf, ins->complementaryFilter.magInfluence);
  Storage::write(FloatValues::accLPF, sensors->acc.lpf);
  Storage::write(FloatValues::gyroLPF, sensors->gyro.lpf);

  Storage::write(FloatValues::insSensorFusion, ins->getFusionAlgorythm());
  Storage::write(FloatValues::magZOffset, ins->getMagZOffset());

  Storage::write(FloatValues::batLpf, sensors->batLpf);
  Storage::write(FloatValues::batMul, sensors->vBatMul);
  Storage::write(BoolValues::useVCell, useCellVoltage);

  Storage::write(BoolValues::propsIn, fc->propsIn);
  Storage::write(BoolValues::useLeds, useLeds);
  Storage::write(BoolValues::useAntiGravity, fc->useAntiGravity);
  
  //FC
  Storage::write(FloatValues::angleModeMaxAngle, fc->angleModeMaxAngle);
  Storage::write(FloatValues::gpsMaxSpeedHorizontal, fc->gpsMaxSpeedHorizontal);
  Storage::write(FloatValues::gpsMaxSpeedVertical, fc->gpsMaxSpeedVertical);
  Storage::write(FloatValues::hoverThrottle, fc->hoverThrottle);
  Storage::write(FloatValues::launchIBoostSeconds, fc->launchIBoostSeconds);
  Storage::write(FloatValues::launchIBoostLevel, fc->launchIBoostLevel);
  Storage::write(FloatValues::launchIBoostAltitude, fc->launchIBoostAltitude);

  Storage::write(Vec3Values::rateR, fc->rollRate.toVec3());
  Storage::write(Vec3Values::rateP, fc->pitchRate.toVec3());
  Storage::write(Vec3Values::rateY, fc->yawRate.toVec3());

 //PIDs
  Storage::write(PidValues::ratePidR,   fc->rateRollPID);
  Storage::write(PidValues::ratePidP,   fc->ratePitchPID);
  Storage::write(PidValues::ratePidY,   fc->rateYawPID);

  Storage::write(PidValues::altitudePid,fc->altitudePID);
  Storage::write(PidValues::velPid,     fc->velPIDx);

  Storage::write(PidValues::levelPidR,  fc->levelRollPID);
  Storage::write(PidValues::levelPidP,  fc->levelPitchPID);
  Storage::write(PidValues::levelPidY,  fc->levelYawPID);

  Storage::write(FloatValues::loopFreqRate, loopFreqRate);
  Storage::write(FloatValues::loopFreqLevel, loopFreqLevel);
  Storage::write(FloatValues::iRelaxMinRate, fc->iRelaxMinRate);

  Storage::write(FloatValues::antiGravityMul, fc->antiGravityMul);
  Storage::write(FloatValues::boostLpf, fc->boostLpf);
  Storage::write(FloatValues::boostSpeed, fc->boostSpeed);
}

void Comunicator::readEEPROM() {
  Serial.println("Reading from EEPROM");
  Serial2.println("Reading from EEPROM");

  // Sensor Interface calibration
  sensors->setAccCal (Storage::read(Vec3Values::accOffset), Storage::read(Vec3Values::accScale));
  sensors->setGyroCal(Storage::read(Vec3Values::gyroOffset), Storage::read(Vec3Values::gyroScale));
  sensors->setMagCal (Storage::read(Vec3Values::magOffset), Storage::read(Vec3Values::magScale));

  ins->complementaryFilter.accInfluence = Storage::read(FloatValues::accInsInf);
  ins->complementaryFilter.magInfluence = Storage::read(FloatValues::magInsInf);
  sensors->acc.lpf = Storage::read(FloatValues::accLPF);
  sensors->gyro.lpf = Storage::read(FloatValues::gyroLPF);

  ins->setFusionAlgorythm(SensorFusion::FusionAlgorythm(Storage::read(FloatValues::insSensorFusion)));
  ins->setMagZOffset(Storage::read(FloatValues::magZOffset));

  sensors->batLpf = Storage::read(FloatValues::batLpf);
  sensors->vBatMul = Storage::read(FloatValues::batMul);

  fc->propsIn = Storage::read(BoolValues::propsIn);
  fc->useAntiGravity = Storage::read(BoolValues::useAntiGravity);
  useLeds = Storage::read(BoolValues::useLeds);
  useCellVoltage = Storage::read(BoolValues::useVCell);

  //FC
  fc->angleModeMaxAngle   = Storage::read(FloatValues::angleModeMaxAngle);
  fc->gpsMaxSpeedHorizontal= Storage::read(FloatValues::gpsMaxSpeedHorizontal);
  fc->gpsMaxSpeedVertical = Storage::read(FloatValues::gpsMaxSpeedVertical);
  fc->hoverThrottle       = Storage::read(FloatValues::hoverThrottle);
  fc->launchIBoostSeconds = Storage::read(FloatValues::launchIBoostSeconds);
  fc->launchIBoostLevel   = Storage::read(FloatValues::launchIBoostLevel);
  fc->launchIBoostAltitude= Storage::read(FloatValues::launchIBoostAltitude);
  fc->rollRate            = Rates(Storage::read(Vec3Values::rateR));
  fc->pitchRate           = Rates(Storage::read(Vec3Values::rateP));
  fc->yawRate             = Rates(Storage::read(Vec3Values::rateY));

  // PIDs
  fc->rateRollPID     = Storage::read(PidValues::ratePidR);
  fc->ratePitchPID    = Storage::read(PidValues::ratePidP);
  fc->rateYawPID      = Storage::read(PidValues::ratePidY);

  fc->levelRollPID    = Storage::read(PidValues::levelPidR);
  fc->levelPitchPID   = Storage::read(PidValues::levelPidP);
  fc->levelYawPID     = Storage::read(PidValues::levelPidY);

  fc->altitudePID     = Storage::read(PidValues::altitudePid);

  fc->velPIDx         = Storage::read(PidValues::velPid);
  fc->velPIDy         = Storage::read(PidValues::velPid);

  fc->setMotorPin(1, (int) Storage::read(FloatValues::m1Pin));
  fc->setMotorPin(2, (int) Storage::read(FloatValues::m2Pin));
  fc->setMotorPin(3, (int) Storage::read(FloatValues::m3Pin));
  fc->setMotorPin(4, (int) Storage::read(FloatValues::m4Pin));

  fc->throttleMul4S =  Storage::read(FloatValues::throttleMul4S);
  fc->throttleMul6S =  Storage::read(FloatValues::throttleMul6S);

  loopFreqRate = Storage::read(FloatValues::loopFreqRate);
  loopFreqLevel = Storage::read(FloatValues::loopFreqLevel);
  fc->iRelaxMinRate = Storage::read(FloatValues::iRelaxMinRate);

  fc->antiGravityMul = Storage::read(FloatValues::antiGravityMul);
  fc->boostSpeed = Storage::read(FloatValues::boostSpeed);
  fc->boostLpf = Storage::read(FloatValues::boostLpf);

  Serial.print("EEPROM size: ");
  Serial.println(Storage::size());
}

void Comunicator::handleCRSFTelem() {
  // vBat, current, mahDraw, remaining Percent

  crsf->updateTelemetryBattery(useCellVoltage ? sensors->bat.vCell : sensors->bat.vBat, fc->gForce, fc->maxGForce, max(0, ((sensors->bat.vCell - 3.3) / 0.9) * 100));
  crsf->updateTelemetryAttitude(-ins->getRoll(), -ins->getPitch(), ins->getYaw());
  crsf->updateTelemetryGPS(sensors->gps.lat, sensors->gps.lng, ins->getVelocity().getLength2D(), ins->getYaw(), ins->getLocation().getLength2D(), sensors->gps.satelites);
  // crsf->updateTelemetryGPS(sensors->gps.lat, sensors->gps.lng, ins->getVelocity().getLength2D(), ins->getYaw(), ins->getLocation().z, sensors->gps.satelites);
  switch(fc->flightMode) {
    case FlightMode::none: {
      crsf->updateTelemetryFlightMode("None"); break;
    }
    case FlightMode::rate: {
      crsf->updateTelemetryFlightMode("Rate"); break;
    }
    case FlightMode::level: {
      crsf->updateTelemetryFlightMode("Lvl"); break;
    }
    case FlightMode::altitudeHold: {
      crsf->updateTelemetryFlightMode("Alt"); break;
    }
    case FlightMode::gpsHold: {
      crsf->updateTelemetryFlightMode("GPS"); break;
    }
    case FlightMode::wayPoint: {
      crsf->updateTelemetryFlightMode("wpt"); break;
    }
    case FlightMode::dreaming: crsf->updateTelemetryFlightMode("Err"); break;
    case FlightMode::FlightModeSize: crsf->updateTelemetryFlightMode("Err"); break;
  }
}