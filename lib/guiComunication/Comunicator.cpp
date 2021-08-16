#include <Arduino.h>
#include "Comunicator.h"

int strpos3(const char* haystack, const char needle, int start = 0) {
  for(int i = start; i < 100; i++) {
    if(haystack[i] == needle) return i;
  }
  return -1;
}

const char* FCCommand_ENUM_STR[] = {
	"FC_DO_START_TELEM",
	"FC_DO_RESET_INS",
	"FC_DO_STOP_TELEM",
	"FC_DO_ACC_CALIB",
	"FC_DO_GYRO_CALIB",
	"FC_DO_MAG_CALIB",
	"FC_DO_END_COM",
	"FC_GET_ACC_OFFSET",
	"FC_GET_GYRO_OFFSET",
	"FC_GET_MAG_OFFSET",
	"FC_GET_ACC_MUL",
	"FC_GET_GYRO_MUL",
	"FC_GET_MAG_MUL",
  "FC_GET_ACC_LPF",
	"FC_GET_GYRO_LPF",
	"FC_GET_INS_ACC_INFL",
	"FC_POST_SENSOR",
	"FC_SET_ACC_OFFSET",
	"FC_SET_GYRO_OFFSET",
	"FC_SET_MAG_OFFSET",
	"FC_SET_ACC_MUL",
	"FC_SET_GYRO_MUL",
	"FC_SET_MAG_MUL",
  "FC_SET_ACC_LPF",
	"FC_SET_GYRO_LPF",
	"FC_SET_INS_ACC_INFL",
};

void Comunicator::begin() {
    Serial.println("FC comunication initiated");
}

void Comunicator::handle() {
  while(Serial.available()) {
    char c = Serial.read();
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
  if(useTelemetry) {
    scheduleTelemetry();
  }
}

void Comunicator::scheduleTelemetry() {
  uint64_t now = micros();
  if(now - lastTelem > telemUs) {
    postTelemetry();
    lastTelem = now;
  }
}

void Comunicator::postTelemetry() {
  postSensorData("ATTI", "Pitch", ins->getPitch());
  postSensorData("ATTI", "Roll", ins->getRoll());
  postSensorData("ATTI", "Yaw", ins->getYaw());

  postSensorData("QUAT", "W", ins->getQuaternionRotation().w);
  postSensorData("QUAT", "X", ins->getQuaternionRotation().x);
  postSensorData("QUAT", "Y", ins->getQuaternionRotation().y);
  postSensorData("QUAT", "Z", ins->getQuaternionRotation().z);

  postSensorData("GYRO", "X", sensors->gyro.x);
  postSensorData("GYRO", "Y", sensors->gyro.y);
  postSensorData("GYRO", "Z", sensors->gyro.z);

  postSensorData("GYRO(f)", "X", ins->getLastFilteredGyro().x);
  postSensorData("GYRO(f)", "Y", ins->getLastFilteredGyro().y);
  postSensorData("GYRO(f)", "Z", ins->getLastFilteredGyro().z);

  postSensorData("ACC", "X", sensors->acc.x);
  postSensorData("ACC", "Y", sensors->acc.y);
  postSensorData("ACC", "Z", sensors->acc.z);

  postSensorData("ACC(f)", "X", ins->getLastFilteredAcc().x);
  postSensorData("ACC(f)", "Y", ins->getLastFilteredAcc().y);
  postSensorData("ACC(f)", "Z", ins->getLastFilteredAcc().z);

  if(sensors->mag.connected) {
    postSensorData("MAG", "X", sensors->mag.x);
    postSensorData("MAG", "Y", sensors->mag.y);
    postSensorData("MAG", "Z", sensors->mag.z);

    postSensorData("MAG(f)", "X", ins->getLastFilteredMag().x);
    postSensorData("MAG(f)", "Y", ins->getLastFilteredMag().y);
    postSensorData("MAG(f)", "Z", ins->getLastFilteredMag().z);
  }

  if(sensors->baro.connected) {
    postSensorData("BARO", "Alt", sensors->baro.altitude);
    postSensorData("BARO", "Temp", sensors->baro.temperature);
  }

  if(sensors->gps.connected) {
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
}

void Comunicator::end() {
  Serial.println("ending comunication");
  Serial.end();
}

void Comunicator::postSensorData(const char* sensorName, const char* subType, float value) {
  Serial.print("FC_POST_SENSOR");
  Serial.print(' ');
  Serial.print(sensorName);
  Serial.print(';');
  Serial.print(subType);
  Serial.print(';');
  Serial.println(value, 10);
}

void Comunicator::post(FCCommand command, String value) {
  Serial.print(FCCommand_ENUM_STR[command]);
  Serial.print(' ');
  Serial.println(value);
}

void Comunicator::post(FCCommand command, const char* value) {
  Serial.print(FCCommand_ENUM_STR[command]);
  Serial.print(' ');
  Serial.println(value);
}

void Comunicator::processSerialLine() {
  char* command;
  // FC_GET
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
      Serial.println("Invalid request. No id");
      return;
    }

    if(strncmp("ACC_OFFSET", command, 10) == 0) {
      postResponse(uid, ins->getAccOffset().toString());
    }
    if(strncmp("GYRO_OFFSET", command, 11) == 0) {
      postResponse(uid, ins->getGyroOffset().toString());
    }
    if(strncmp("MAG_OFFSET", command, 10) == 0) {
      postResponse(uid, ins->getMagOffset().toString());
    }
    if(strncmp("ACC_MUL", command, 7) == 0) {
      postResponse(uid, ins->getAccMul().toString());
    }
    if(strncmp("GYRO_MUL", command, 8) == 0) {
      postResponse(uid, ins->getGyroMul().toString());
    }
    if(strncmp("MAG_MUL", command, 7) == 0) {
      postResponse(uid, ins->getMagMul().toString());
    }
    if(strncmp("ACC_LPF", command,7) == 0) {
      postResponse(uid, ins->getAccLowpassFilter());
    }
    if(strncmp("GYRO_LPF", command,7) == 0) {
      postResponse(uid, ins->getGyroLowpassFilter());
    }
    if(strncmp("INS_ACC_INFL", command, 12) == 0) {
      postResponse(uid, ins->getAccInfluence());
    }
  }

  // FC_DO
  if(bufferCount > 6 && strncmp("FC_DO_", buffer, 6) == 0) {
    command = buffer + 6;
    if(strncmp("ACC_CALIB", command, 9) == 0) {
      ins->calibrateAcc();
    }
    if(strncmp("GYRO_CALIB", command, 10) == 0) {
      ins->calibrateGyro();
    }
    if(strncmp("MAG_CALIB", command, 9) == 0) {
      // ins.calibrateMag();//not implemented yet
    }
    if(strncmp("START_TELEM", command, 11) == 0) {
      Serial.println("starting telemetry");
      useTelemetry = true;
    }
    if(strncmp("STOP_TELEM", command, 10) == 0) {
      Serial.println("stopping telemetry");
      useTelemetry = false;
    }
    if(strncmp("RESET_INS", command, 9) == 0) {
      Serial.println("resetting INS");
      ins->reset();
    }
  }

  //FC_POST
  if(bufferCount > 8 && strncmp("FC_POST_", buffer, 8) == 0) {
    command = buffer + 8;
    int valueStart = strpos3(buffer, ' ', 8) + 1;
    if(valueStart == -1) return;
    char* value = buffer + valueStart;
    /**
     * POST HERE
     */
  }
  // FC_SET
  if(bufferCount > 7 && strncmp("FC_SET_", buffer, 7) == 0) {
    command = buffer + 7;
    int uidStart = strpos3(buffer, ' ', 7) + 1;
    if(uidStart == -1) {
      Serial.println("Invalid SET. No id");
      return;
    }
    char* uid = buffer + uidStart;
    int valueStart = strpos3(buffer, ' ', uidStart + 1) + 1;
    buffer[valueStart - 1] = 0;
    if(valueStart == -1) {
      Serial.println("Invalid SET. No value");
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
    if(strncmp("MAG_OFFSET", command, 10) == 0) {
      postResponse(uid, value);
      ins->setMagOffset(Vec3(value));
    }
    if(strncmp("ACC_MUL", command, 7) == 0) {
      postResponse(uid, value);
      ins->setAccMul(Vec3(value));
    }
    if(strncmp("GYRO_MUL", command, 8) == 0) {
      postResponse(uid, value);
      ins->setGyroMul(Vec3(value));
    }
    if(strncmp("MAG_MUL", command, 7) == 0) {
      postResponse(uid, value);
      ins->setMagMul(Vec3(value));
    }
    if(strncmp("ACC_LPF", command, 7) == 0) {
      postResponse(uid, value);
      // Serial.println(atof(value));
      ins->setAccLowpassFilter(atof(value));
    }
    if(strncmp("GYRO_LPF", command, 8) == 0) {
      postResponse(uid, value);
      ins->setGyroLowpassFilter(atof(value));
    }
    if(strncmp("INS_ACC_INFL", command, 12) == 0) {
      postResponse(uid, value);
      ins->setAccInfluence(atof(value));
    }
  }
}

void Comunicator::postResponse(char* uid, String body) {
  Serial.print("FC_RES ");
  Serial.print(uid);
  Serial.print(" ");
  Serial.println(body);
}

void Comunicator::postResponse(char* uid, float num) {
  Serial.print("FC_RES ");
  Serial.print(uid);
  Serial.print(" ");
  Serial.println(num, 5);
}