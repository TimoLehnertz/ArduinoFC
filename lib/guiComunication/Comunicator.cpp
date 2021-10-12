#include <Arduino.h>
#include <EEPROM.h>
#include "Comunicator.h"
#include <maths.h>
#include <error.h>

int strpos3(const char* haystack, const char needle, int start = 0) {
  for(int i = start; i < 100; i++) {
    if(haystack[i] == needle) return i;
  }
  return -1;
}

void Comunicator::begin() {
    Serial.println("FC comunication initiated");
    Serial2.println("FC comunication initiated");
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
}

void Comunicator::scheduleTelemetry() {
  uint64_t now = micros();
  if(now - lastTelem > telemUs) {
    postTelemetry();
    lastTelem = now;
  }
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
  }

  if(useQuatTelem) {
    postSensorData("QUAT", "W", ins->getQuaternionRotation().w);
    postSensorData("QUAT", "X", ins->getQuaternionRotation().x);
    postSensorData("QUAT", "Y", ins->getQuaternionRotation().y);
    postSensorData("QUAT", "Z", ins->getQuaternionRotation().z);
  }

  if(useGyroTelem) {
    postSensorData("GYRO", "X", sensors->gyro.x);
    postSensorData("GYRO", "Y", sensors->gyro.y);
    postSensorData("GYRO", "Z", sensors->gyro.z);

    postSensorData("GYRO(f)", "X", ins->getLastFilteredGyro().x);
    postSensorData("GYRO(f)", "Y", ins->getLastFilteredGyro().y);
    postSensorData("GYRO(f)", "Z", ins->getLastFilteredGyro().z);
  }

  if(useAccTelem) {
    postSensorData("ACC", "X", sensors->acc.x);
    postSensorData("ACC", "Y", sensors->acc.y);
    postSensorData("ACC", "Z", sensors->acc.z);

    postSensorData("ACC(f)", "X", ins->getLastFilteredAcc().x);
    postSensorData("ACC(f)", "Y", ins->getLastFilteredAcc().y);
    postSensorData("ACC(f)", "Z", ins->getLastFilteredAcc().z);
  }

  if(useMagTelem) {
    postSensorData("MAG", "X", sensors->mag.x);
    postSensorData("MAG", "Y", sensors->mag.y);
    postSensorData("MAG", "Z", sensors->mag.z);

    postSensorData("MAG(f)", "X", ins->getLastFilteredMag().x);
    postSensorData("MAG(f)", "Y", ins->getLastFilteredMag().y);
    postSensorData("MAG(f)", "Z", ins->getLastFilteredMag().z);
  }

  if(useBaroTelem) {
    postSensorData("BARO", "Alt", sensors->baro.altitude);
    postSensorData("BARO(f)", "Alt", ins->getLastFilteredBaroAltitude());
    postSensorData("BARO(speed)", "Alt", ins->getBaroAltSpd());
  }

  if(useGpsTelem) {
    if(sensors->gps.locationValid) {
      postSensorData("GPS", "LAT", sensors->gps.lat);
      postSensorData("GPS", "LNG", sensors->gps.lng);
    }
    if(sensors->gps.altitudeValid) {
      postSensorData("GPS", "alt", sensors->gps.altitude);
    }
    if(sensors->gps.speedValid) {
      postSensorData("GPS", "spd", sensors->gps.speed);
    }
    postSensorData("GPS", "Sat", sensors->gps.satelites);
  }
  if(useTimingTelem) {
    postSensorData("FREQ", "loopHz", actualFreq);
    postSensorData("TIME", "CRSF Us", crsfTime - loopStart);
    postSensorData("TIME", "Sens Us", sensorsTime - crsfTime);
    postSensorData("TIME", "INS Us", insTime - sensorsTime);
    postSensorData("TIME", "Chan Us", chanelsTime - insTime);
    postSensorData("TIME", "FC Us", fcTime - chanelsTime);
    // postSensorData("TIME", "Sum", loopEnd - loopStart); // overflow
    postSensorData("CPU Load", "", cpuLoad);
  }
  if(useRCTelem) {
    postSensorData("RC", "CH1", fc->chanelsRaw.chanels[0]);
    postSensorData("RC", "CH2", fc->chanelsRaw.chanels[1]);
    postSensorData("RC", "CH3", fc->chanelsRaw.chanels[2]);
    postSensorData("RC", "CH4", fc->chanelsRaw.chanels[3]);
    postSensorData("RC", "CH5", fc->chanelsRaw.chanels[4]);
    postSensorData("RC", "CH6", fc->chanelsRaw.chanels[5]);
    postSensorData("RC", "CH7", fc->chanelsRaw.chanels[6]);
    postSensorData("RC", "CH8", fc->chanelsRaw.chanels[7]);
    postSensorData("RC", "CH9", fc->chanelsRaw.chanels[8]);
    postSensorData("RC", "CH10", fc->chanelsRaw.chanels[9]);
    postSensorData("RC", "CH11", fc->chanelsRaw.chanels[10]);
    postSensorData("RC", "CH12", fc->chanelsRaw.chanels[11]);
  }
  if(useFCTelem) {
    postSensorData("RateAdj", "Roll", fc->rollRateAdjust);
    postSensorData("RateAdj", "Pitch", fc->pitchRateAdjust);
    postSensorData("RateAdj", "Yaw", fc->yawRateAdjust);
    postSensorData("Rate PID Roll", fc->rateRollPID);
    postSensorData("Rate PID Pitch", fc->ratePitchPID);
    postSensorData("Rate PID Yaw", fc->rateYawPID);
    postSensorData("Level PID Roll", fc->levelRollPID);
    postSensorData("Level PID Pitch", fc->levelPitchPID);
    postSensorData("Anti Gravity", "boost", fc->iBoost);
  }
  if(useBatTelem) {
    postSensorData("vBat", "Voltage", sensors->bat.vBat);
    postSensorData("vCell", "Voltage", sensors->bat.vCell);
    postSensorData("Cell count", "count", sensors->bat.cellCount);
  }
}

void Comunicator::end() {
  Serial.println("ending comunication");
  Serial.end();

  Serial2.println("ending comunication");
  Serial2.end();
}

void Comunicator::postSensorData(const char* sensorName, const char* subType, float value) {
  Serial.print("FC_POST_SENSOR");
  Serial.print(' ');
  Serial.print(sensorName);
  Serial.print(';');
  Serial.print(subType);
  Serial.print(';');
  Serial.println(value, 10);

  Serial2.print("FC_POST_SENSOR");
  Serial2.print(' ');
  Serial2.print(sensorName);
  Serial2.print(';');
  Serial2.print(subType);
  Serial2.print(';');
  Serial2.println(value, 10);
}

void Comunicator::postSensorData(const char* sensorName, PID pid) {
  postSensorData(sensorName, "P", pid.prevP);
  postSensorData(sensorName, "I", pid.prevI);
  postSensorData(sensorName, "D", pid.prevD);
  postSensorData(sensorName, "SUM", pid.prevOut);
}


void Comunicator::post(const char* command, String value) {
  Serial.print(command);
  Serial.print(' ');
  Serial.println(value);

  Serial2.print(command);
  Serial2.print(' ');
  Serial2.println(value);
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
    if(strncmp("ACC_OFFSET", command, 10) == 0) {
      postResponse(uid, ins->getAccOffset().toString());
    }
    if(strncmp("GYRO_OFFSET", command, 11) == 0) {
      postResponse(uid, ins->getGyroOffset().toString());
    }
    if(strncmp("MAG_HARD_IRON", command, 13) == 0) {
      postResponse(uid, ins->getMagHardIron().toString());
    }
    if(strncmp("ACC_MUL", command, 7) == 0) {
      postResponse(uid, ins->getAccMul());
    }
    if(strncmp("GYRO_MUL", command, 8) == 0) {
      postResponse(uid, ins->getGyroMul().toString());
    }
    if(strncmp("MAG_SOFT_IRON", command, 13) == 0) {
      postResponse(uid, ins->getMagSoftIron());
    }
    if(strncmp("ACC_LPF", command,7) == 0) {
      postResponse(uid, ins->getAccLowpassFilter());
    }
    if(strncmp("GYRO_LPF", command,7) == 0) {
      postResponse(uid, ins->getGyroLowpassFilter());
    }
    if(strncmp("INS_ACC_INF", command, 11) == 0) {
      postResponse(uid, ins->getAccInfluence());
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
    if(strncmp("ACC_INFL", command, 8) == 0) {
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
    if(strncmp("INS_MAG_INF", command, 11) == 0) {
      postResponse(uid, ins->gatMagInfluence());
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
    if(strncmp("INS_ACC_MAX_G", command, 13) == 0) {
      postResponse(uid, ins->getMaxGError());
    }
    if(strncmp("USE_VCELL", command, 9) == 0) {
      postResponse(uid, useCellVoltage);
    }
    if(strncmp("ACC_ANGLE_OFFSET", command, 16) == 0) {
      postResponse(uid, ins->getAccAngleOffset().toString());
    }
    if(strncmp("ACC_OFFSET_MPU", command, 14) == 0) {
      postResponse(uid, sensors->getAccOffset().toString());
    }
    if(strncmp("ACC_SCALE_MPU", command, 13) == 0) {
      postResponse(uid, sensors->getAccScale().toString());
    }
    if(strncmp("MAG_OFFSET_MPU", command, 14) == 0) {
      postResponse(uid, sensors->getMagOffset().toString());
    }
    if(strncmp("MAG_SCALE_MPU", command, 13) == 0) {
      postResponse(uid, sensors->getMagScale().toString());
    }
  }

  // FC_DO
  if(bufferCount > 6 && strncmp("FC_DO_", buffer, 6) == 0) {
    command = buffer + 6;
    if(strncmp("ACC_CALIB", command, 9) == 0) {
      Serial.println("Calibrating Accelerometer");
      sensors->calibrateAcc();
      Serial.println("Done!");
      // ins->calibrateAcc();
    }
    if(strncmp("GYRO_CALIB", command, 10) == 0) {
      Serial.println("Calibrating Gyroscope");
      sensors->calibrateGyro();
      Serial.println("Done!");
      // ins->calibrateGyro();
    }
    if(strncmp("MAG_CALIB", command, 9) == 0) {
        Serial.println("Calibrating magnetometer");
        sensors->calibrateMag();
        Serial.println("Done!");
        // useMagTelem = true;
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
    if(strncmp("ACC_ANGLE_OFFSET", command, 16) == 0) {
      ins->setAccAngleOffset();
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
    if(strncmp("ACC_OFFSET", command, 10) == 0) {
      postResponse(uid, value);
      ins->setAccOffset(Vec3(value));
    }
    if(strncmp("GYRO_OFFSET", command, 11) == 0) {
      postResponse(uid, value);
      ins->setGyroOffset(Vec3(value));
    }
    if(strncmp("MAG_HARD_IRON", command, 13) == 0) {
      postResponse(uid, value);
      ins->setMagHardIron(Vec3(value));
    }
    if(strncmp("ACC_MUL", command, 7) == 0) {
      postResponse(uid, value);
      ins->setAccMul(Matrix3(value));
    }
    if(strncmp("GYRO_MUL", command, 8) == 0) {
      postResponse(uid, value);
      ins->setGyroMul(Vec3(value));
    }
    if(strncmp("MAG_SOFT_IRON", command, 13) == 0) {
      postResponse(uid, value);
      ins->setMagSoftIron(Matrix3(value));
    }
    if(strncmp("ACC_LPF", command, 7) == 0) {
      postResponse(uid, value);
      ins->setAccLowpassFilter(atof(value));
    }
    if(strncmp("GYRO_LPF", command, 8) == 0) {
      postResponse(uid, value);
      ins->setGyroLowpassFilter(atof(value));
    }
    if(strncmp("INS_ACC_INF", command, 11) == 0) {
      postResponse(uid, value);
      ins->setAccInfluence(atof(value));
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
      useLocTelem = value[0] == 't';
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
    if(strncmp("INS_MAG_INF", command, 11) == 0) {
      postResponse(uid, value);
      ins->setMagInfluence(atof(value));
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
    if(strncmp("INS_ACC_MAX_G", command, 13) == 0) {
      postResponse(uid, value);
      ins->setMaxGError(atof(value));
    }
    if(strncmp("VOLTAGE_CALIB", command, 13) == 0) {
      postResponse(uid, value);
      sensors->calibrateBat(atof(value));
    }
    if(strncmp("USE_VCELL", command, 9) == 0) {
      postResponse(uid, value);
      useCellVoltage = value[0] == 't';
    }
    if(strncmp("ACC_ANGLE_OFFSET", command, 16) == 0) {
      postResponse(uid, value);
      ins->setAccAngleOffset(Quaternion(value));
    }
    if(strncmp("ACC_OFFSET_MPU", command, 14) == 0) {
      postResponse(uid, value);
      sensors->setAccCal(Vec3(value), sensors->getAccScale());
    }
    if(strncmp("ACC_SCALE_MPU", command, 13) == 0) {
      postResponse(uid, value);
      sensors->setAccCal(sensors->getAccOffset(), Vec3(value));
    }
    if(strncmp("MAG_OFFSET_MPU", command, 14) == 0) {
      postResponse(uid, value);
      sensors->setMagCal(Vec3(value), sensors->getMagScale());
    }
    if(strncmp("MAG_SCALE_MPU", command, 13) == 0) {
      postResponse(uid, value);
      sensors->setMagCal(sensors->getMagOffset(), Vec3(value));
    }
  }
}

void Comunicator::postResponse(char* uid, String body) {
  Serial.print("FC_RES ");
  Serial.print(uid);
  Serial.print(" ");
  Serial.println(body);

  Serial2.print("FC_RES ");
  Serial2.print(uid);
  Serial2.print(" ");
  Serial2.println(body);
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
  Serial.println(pid.useAuxTuning ? "1" : "0");

  Serial2.print("FC_RES ");
  Serial2.print(uid);
  Serial2.print(" ");
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
  Serial2.println(pid.useAuxTuning ? "1" : "0");
}

void Comunicator::saveEEPROM() {
  // Storage::write(Matrix3Values::accMul, ins->getAccMul());
  // Storage::write(Vec3Values::gyroMul, ins->getGyroMul());

  // Storage::write(Vec3Values::accOffset, ins->getAccOffset());
  // Storage::write(Vec3Values::gyroOffset, ins->getGyroOffset());
  // Storage::write(Vec3Values::magHardIron, ins->getMagHardIron());

  Storage::write(Vec3Values::accOffset,  sensors->getAccOffset());
  Storage::write(Vec3Values::accScale,  sensors->getAccScale());
  Storage::write(Vec3Values::gyroOffset, sensors->getGyroOffset());
  Storage::write(Vec3Values::magOffset, sensors->getMagOffset());
  Storage::write(Vec3Values::magScale, sensors->getMagScale());

  Storage::write(QuaternionValues::accAngleOffset, ins->getAccAngleOffset());

  Storage::write(FloatValues::accInsInf, ins->getAccInfluence());
  Storage::write(FloatValues::magInsInf, ins->gatMagInfluence());
  Storage::write(FloatValues::accLPF, ins->getAccLowpassFilter());
  Storage::write(FloatValues::gyroLPF, ins->getGyroLowpassFilter());

  Storage::write(FloatValues::batLpf, sensors->batLpf);
  Storage::write(FloatValues::batMul, sensors->vBatMul);
  Storage::write(BoolValues::useVCell, useCellVoltage);

  Storage::write(BoolValues::propsIn, fc->propsIn);
  Storage::write(BoolValues::useLeds, useLeds);
  Storage::write(BoolValues::useAntiGravity, fc->useAntiGravity);

 //PIDs
  Storage::write(PidValues::ratePidR,   fc->rateRollPID);
  Storage::write(PidValues::ratePidP,   fc->ratePitchPID);
  Storage::write(PidValues::ratePidY,   fc->rateYawPID);

  Storage::write(PidValues::levelPidR,  fc->levelRollPID);
  Storage::write(PidValues::levelPidP,  fc->levelPitchPID);
  Storage::write(PidValues::levelPidY,  fc->levelYawPID);

  Storage::write(FloatValues::insAccMaxG, ins->getMaxGError());

  // Storage::write(Matrix3Values::magSoftIron, ins->getMagSoftIron());

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

  // ins->setAccMul(Storage::read(Matrix3Values::accMul));
  // ins->setGyroMul(Storage::read(Vec3Values::gyroMul));
  // ins->setMagSoftIron(Storage::read(Matrix3Values::magSoftIron));

  /**
   * Sensor Interface calibration
   */
  sensors->setAccCal (Storage::read(Vec3Values::accOffset), Storage::read(Vec3Values::accScale));
  sensors->setGyroCal(Storage::read(Vec3Values::gyroOffset));
  sensors->setMagCal (Storage::read(Vec3Values::magOffset), Storage::read(Vec3Values::magScale));


  // ins->setAccOffset(Storage::read(Vec3Values::accOffset));
  // ins->setGyroOffset(Storage::read(Vec3Values::gyroOffset));
  // ins->setMagHardIron(Storage::read(Vec3Values::magHardIron));

  ins->setAccAngleOffset(Storage::read(QuaternionValues::accAngleOffset));

  ins->setAccInfluence(Storage::read(FloatValues::accInsInf));
  ins->setMagInfluence(Storage::read(FloatValues::magInsInf));
  ins->setAccLowpassFilter(Storage::read(FloatValues::accLPF));
  ins->setGyroLowpassFilter(Storage::read(FloatValues::gyroLPF));

  sensors->batLpf = Storage::read(FloatValues::batLpf);
  sensors->vBatMul = Storage::read(FloatValues::batMul);

  fc->propsIn = Storage::read(BoolValues::propsIn);
  fc->useAntiGravity = Storage::read(BoolValues::useAntiGravity);
  useLeds = Storage::read(BoolValues::useLeds);
  useCellVoltage = Storage::read(BoolValues::useVCell);

  // PIDs
  fc->rateRollPID     = Storage::read(PidValues::ratePidR);
  fc->ratePitchPID    = Storage::read(PidValues::ratePidP);
  fc->rateYawPID      = Storage::read(PidValues::ratePidY);

  fc->levelRollPID    = Storage::read(PidValues::levelPidR);
  fc->levelPitchPID   = Storage::read(PidValues::levelPidP);
  fc->levelYawPID     = Storage::read(PidValues::levelPidY);

  ins->setMaxGError(Storage::read(FloatValues::insAccMaxG));

  loopFreqRate = Storage::read(FloatValues::loopFreqRate);
  loopFreqLevel = Storage::read(FloatValues::loopFreqLevel);
  fc->iRelaxMinRate = Storage::read(FloatValues::iRelaxMinRate);

  fc->antiGravityMul = Storage::read(FloatValues::antiGravityMul);
  fc->boostSpeed = Storage::read(FloatValues::boostSpeed);
  fc->boostLpf = Storage::read(FloatValues::boostLpf);
}

void Comunicator::handleCRSFTelem() {
  // vBat, current, mahDraw, remaining Percent

  crsf->updateTelemetryBattery(useCellVoltage ? sensors->bat.vCell : sensors->bat.vBat, fc->gForce, fc->maxGForce, max(0, ((sensors->bat.vCell - 3.3) / 0.9) * 100));
  crsf->updateTelemetryAttitude(-ins->getRoll(), -ins->getPitch(), ins->getYaw());
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
      crsf->updateTelemetryFlightMode("alt"); break;
    }
    case FlightMode::gpsHold: {
      crsf->updateTelemetryFlightMode("gps"); break;
    }
    case FlightMode::FlightModeSize: break;
  }
}