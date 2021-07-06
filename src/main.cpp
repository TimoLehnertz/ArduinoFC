#include <Arduino.h>
// #include "crsf.h"
#include <Servo.h>
#include <crossfire.h>
// #include <SoftwareSerial.h>

// #include <Arduino_LSM9DS1.h>

// #include <maths.h>
// #include <ins.h>

// float x, y, z;

// INS ins;

// bool printIMU = false;

// char buffer[256];
// byte bufferCount = 0;

// void processSerialLine();
// void serialPost(char*, String);

// void setup() {
//   Serial.begin(115200);
//   // Serial.begin(9600);
//   while (!Serial);
//   if (!IMU.begin()) {
//     Serial.println("Failed to initialize IMU!");
//     while (1);
//   }
//   ins.requestCalibration();
//   Serial.println("FC initiated");
// }

// void loop() {
//   if (IMU.accelerationAvailable()) {
//     IMU.readAcceleration(x, y, z);
//     ins.updateAcc(x, y, z);
//     if(printIMU) {
//       Serial.print("Accelerometer data: ,");
//       Serial.print(x);
//       Serial.print(',');
//       Serial.print(y);
//       Serial.print(',');
//       Serial.println(z);
//       Serial.println();
//     }
//   }
//   if (IMU.gyroscopeAvailable()) {
//     IMU.readGyroscope(x, y, z);
//     // Serial.println(x);
//     // long time = micros();
//     ins.updateGyro(x, y, z);
//     if(printIMU) {
//       Serial.println(ins.getEulerRotationZYX().clone().toString());
//       Serial.print("INS quaternion: ,");
//       Serial.print(ins.getQuaternionRotation().w);
//       Serial.print(',');
//       Serial.print(ins.getQuaternionRotation().x);
//       Serial.print(',');
//       Serial.print(ins.getQuaternionRotation().y);
//       Serial.print(',');
//       Serial.println(ins.getQuaternionRotation().z);
//       Serial.print("Gyroscope data: ,");
//       Serial.print(x);
//       Serial.print(',');
//       Serial.print(y);
//       Serial.print(',');
//       Serial.println(z);
//     }
//   }
//   if (IMU.magneticFieldAvailable()) {
//     IMU.readMagneticField(x, y, z);
//     // ins.updateMag(x, y, z);
//     // Serial.print("Magnetometer data: ,");
//     // Serial.print(x);
//     // Serial.print(',');
//     // Serial.print(y);
//     // Serial.print(',');
//     // Serial.println(z);
//     // Serial.println();
//   }
//   while(Serial.available()) {
//     char c = Serial.read();
//     // Serial.print(int(c));
//     if(c == '\n') {
//       buffer[bufferCount] = 0;
//       processSerialLine();
//       bufferCount = 0;//ss
//       return;
//     } else {
//       buffer[bufferCount] = c;
//     }
//     bufferCount++;
//     if(bufferCount == 255)
//       bufferCount = 0;
//   }
// }


// enum FCCommand {
// 	//setter
// 	ACC,
// 	GYRO,
// 	MAG,
// 	ROT,
// 	VEL,
// 	POS,
// };

// const char* enum_str[] = {"ACC", "GYRO", "MAG", "ROT", "VEL", "POS", "ACC"};

// void post(FCCommand command, const char* value) {
//   Serial.print("FC_POST_");
//   Serial.print(enum_str[command]);
//   Serial.print(' ');
//   Serial.println(value);
// }

// void processSerialLine() {
//   char* command;
//   // FC_GET
//   if(bufferCount > 7 && strncmp("FC_GET_", buffer, 7) == 0) {
//     command = buffer + 7;
//     char* uid = nullptr;
//     for(int i = 8; i < bufferCount; i++) {
//       if(buffer[i] == ' ') {
//         uid = buffer + i + 1;
//         break;
//       }
//     }
//     if(uid == nullptr) {
//       Serial.println("Invalid request. No id");
//       return;
//     }

//     if(strncmp("ACC_CALIB", command, 9) == 0) {
//       serialPost(uid, ins.getAccOffset().toString());
//     }

//     if(strncmp("GYRO_CALIB", command, 10) == 0) {
//       serialPost(uid, ins.getGyroOffset().toString());
//     }
//   }

//   // FC_DO
//   if(bufferCount > 6 && strncmp("FC_DO_", buffer, 6) == 0) {
//     command = buffer + 6;
//     if(strncmp("ACC_CALIB", command, 9) == 0) {
//       ins.calibrateAcc();
//     }
//     if(strncmp("GYRO_CALIB", command, 10) == 0) {
//       ins.calibrateGyro();
//     }
//     if(strncmp("MAG_CALIB", command, 9) == 0) {
//       // ins.calibrateMag();//not implemented yet
//     }
//   }

//   //FC_POST
//   if(bufferCount > 8 && strncmp("FC_POST_", buffer, 8) == 0) {
//     command = buffer + 8;
//     char* value = nullptr;
//     for(int i = 8; i < bufferCount; i++) {
//       if(buffer[i] == ' ') {
//         value = buffer + i + 1;
//         break;
//       }
//     }
//     if(strncmp("TEST1", command, 5) == 0) { // example
//       Serial.println("echo test");
//       Serial.println(value);
//     }
//   }
// }

// void serialPost(char* uid, String body) {
//   Serial.print("FC_RES ");
//   Serial.print(uid);
//   Serial.print(" ");
//   Serial.println(body);
// }

Servo ESC1;
Servo ESC2;
Servo ESC3;
Servo ESC4;

Crossfire crsf(4,5);

// UART uart(2,2,0,0);

// SoftwareSerial s(4,5);

int throttleMin = 10;
int throttleMax = 180; //ppm values

void setup() {
  Serial.begin(115200);
  Serial.println("moin");
  crsf.begin();
  // ESC1.attach(2, 1000, 2000);
  ESC1.attach(14, 1000, 2000);
  ESC2.attach(12, 1000, 2000);
  ESC3.attach(13, 1000, 2000);
  ESC4.attach(15, 1000, 2000);
}

void loop() {
  // while(Serial1.available()) {
  //   // int v = Serial1.read();
  //   Serial.write(Serial1.read());
  //   // if(v == 41) {
  //   //   delay(100);
  //   //   while(Serial1.available()) {
  //   //     Serial1.read();
  //   //   }
  //   // }
  // }
  crsf.loop();

  CRSF_TxChanels_Converted chanels = crsf.getChanelsCoverted();

  double pitchRate = 0.2;
  double yawRate = 0.2;
  double rollRate = 0.2;

  double throttle = crsf.map(chanels.throttle, 0, 1, throttleMin, throttleMax);

  double fl = throttle;
  double fr = throttle;
  double bl = throttle;
  double br = throttle;

  //pitch
  fl -= pitchRate * chanels.pitch;
  fr -= pitchRate * chanels.pitch;
  bl += pitchRate * chanels.pitch;
  br += pitchRate * chanels.pitch;

  //roll
  fl += rollRate * chanels.roll;
  fr -= rollRate * chanels.roll;
  bl += rollRate * chanels.roll;
  br -= rollRate * chanels.roll;

  //yaw
  fl += yawRate * chanels.yaw;
  fr -= yawRate * chanels.yaw;
  bl -= yawRate * chanels.yaw;
  br += yawRate * chanels.yaw;

  fl = crsf.map(fl, 0, 1, 0, 180);
  fr = crsf.map(fr, 0, 1, 0, 180);
  bl = crsf.map(bl, 0, 1, 0, 180);
  br = crsf.map(br, 0, 1, 0, 180);

  if(!chanels.armed3) {
    fl = 0;
    fr = 0;
    bl = 0;
    br = 0;
  }

  // Serial.println("---------");
  // Serial.print(fl);
  // Serial.print("  ");
  // Serial.println(fr);
  // Serial.print(bl);
  // Serial.print("  ");
  // Serial.println(br);
  // Serial.println("---------");
  // delay(10);

  ESC1.write(fl);
  ESC2.write(fr);
  ESC3.write(bl);
  ESC4.write(br);
}