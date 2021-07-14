#include <Arduino.h>
#include "Comunicator.h"

void Comunicator::begin() {
    Serial.begin(115200); //usb serial
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
}

void Comunicator::end() {
    Serial.println("ending comunication");
    Serial.end();
}

void Comunicator::post(FCCommand command, const char* value) {
  const char* FCCommand_ENUM_STR[] = {"ACC", "GYRO", "MAG", "ROT", "VEL", "POS", "ACC"};
  Serial.print("FC_POST_");
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

    if(strncmp("ACC_CALIB", command, 9) == 0) {
      serialPost(uid, ins->getAccOffset().toString());
    }

    if(strncmp("GYRO_CALIB", command, 10) == 0) {
      serialPost(uid, ins->getGyroOffset().toString());
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
  }

  //FC_POST
  if(bufferCount > 8 && strncmp("FC_POST_", buffer, 8) == 0) {
    command = buffer + 8;
    char* value = nullptr;
    for(int i = 8; i < bufferCount; i++) {
      if(buffer[i] == ' ') {
        value = buffer + i + 1;
        break;
      }
    }
    if(strncmp("TEST1", command, 5) == 0) { // example
      Serial.println("echo test");
      Serial.println(value);
    }
  }
}

void Comunicator::serialPost(char* uid, String body) {
  Serial.print("FC_RES ");
  Serial.print(uid);
  Serial.print(" ");
  Serial.println(body);
}