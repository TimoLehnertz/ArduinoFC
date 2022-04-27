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
  delay(2000);
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

// // #include <Wire.h>

// // void scanI2C(long frequency);

// // void setup(){
// //   Wire.begin();
// //   Serial.begin(115200);
// //   delay(3000);
// //   Serial.println("I2C Scanner ist bereit.");
// //   Serial.println();
// // }

// // void loop() {
// //   scanI2C(100000);
// //   scanI2C(400000);
// //  scanI2C(1000000); // nur aktivieren, wenn der Microcontroller diese Frequenz unterstützt
// //  scanI2C(3400000); // nur aktivieren, wenn der Microcontroller diese Frequenz unterstützt
// //  scanI2C(5000000); // nur aktivieren, wenn der Microcontroller diese Frequenz unterstützt
  
// //   Serial.println("****************************");
// //   Serial.println();
// //   delay(3000);
// // }
// // void scanI2C(long frequency) {
// //   String normal = "standard mode (100 kHz):";
// //   String fast = "fast mode (400 kHz):";
// //   String fastPlus = "fast mode plus (1 MHz):";
// //   String highSpeed = "high speed mode (3.4 MHz):";
// //   String ultraSpeed = "ultra fast mode (5.0 MHz):";
// //   String defaultStr = " !!!!! Unzulässige Frequenz !!!!!";
// //   bool error = true;
// //   bool addressFound = false;
// //   Serial.print("Scanne im ");
// //   switch(frequency){
// //     case 100000:
// //       Serial.println(normal);
// //       break;
// //     case 400000:
// //       Serial.println(fast);
// //       break;
// //     case 1000000:
// //       Serial.println(fastPlus);
// //       break;
// //     case 3400000:
// //       Serial.println(highSpeed);
// //       break;
// //     case 5000000:
// //       Serial.println(ultraSpeed);
// //       break;
// //     default:
// //       Serial.println(defaultStr);
// //       break;
// //   }
  
// //   Wire.setClock(frequency);
// //   for(int i=1; i<128; i++){
// //     Wire.beginTransmission(i);
// //     error = Wire.endTransmission();
// //     if(error == 0){
// //       addressFound = true;
// //       Serial.print("0x");
// //       Serial.println(i,HEX);
// //     }
// //   }
// //   if(!addressFound){
// //     Serial.println("Keine Adresse erkannt");
// //   }
// //   Serial.println();
// // }

// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>
// #include <Wire.h>

// Adafruit_MPU6050 mpu;

// void setup(void) {
//   Serial.begin(115200);
//   while (!Serial)
//     delay(10); // will pause Zero, Leonardo, etc until serial console opens

//   Serial.println("Adafruit MPU6050 test!");

//   // Try to initialize!
//   if (!mpu.begin()) {
//     Serial.println("Failed to find MPU6050 chip");
//     while (1) {
//       delay(10);
//     }
//   }
//   Serial.println("MPU6050 Found!");

//   //setupt motion detection
//   // mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
//   // mpu.setMotionDetectionThreshold(1);
//   // mpu.setMotionDetectionDuration(20);
//   // mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
//   // mpu.setInterruptPinPolarity(true);
//   // mpu.setMotionInterrupt(true);
//   // mpu.setCycleRate(4000);
//   Wire.setClock(1000000);
//   Serial.println("");
//   // Serial.println(mpu.getSampleRateDivisor());

//   delay(100);
// }

// float lastX, lastY, lastZ;

// int i = 0;

// uint32_t lastPrint = 0;
// uint32_t lastM = 0;

// void loop() {


//     // if(millis() % 100 == 0) {
//     //   Serial.println(1000000.0 / (micros() - lastM));
//     // }
//     // lastM = micros();

//     sensors_event_t a, g, temp;
//     mpu.getEvent(&a, &g, &temp);
//     if(lastX != a.acceleration.x || lastY != a.acceleration.y || lastZ != a.acceleration.z) {
//       i++;
//       lastX = a.acceleration.x;
//       lastY = a.acceleration.y;
//       lastZ = a.acceleration.z;
//     }
//     if(millis() > lastPrint + 1000) {
//       Serial.println(i);
//       i = 0;
//       lastPrint = millis();
//     }
//     delayMicroseconds(1000000 / 4000);

//     // /* Get new sensor events with the readings */
//     // mpu.getEvent(&a, &g, &temp);

//     // /* Print out the values */
//     // Serial.print("AccelX:");
//     // Serial.print(a.acceleration.x);
//     // Serial.print(",");
//     // Serial.print("AccelY:");
//     // Serial.print(a.acceleration.y);
//     // Serial.print(",");
//     // Serial.print("AccelZ:");
//     // Serial.print(a.acceleration.z);
//     // Serial.print(", ");
//     // Serial.print("GyroX:");
//     // Serial.print(g.gyro.x);
//     // Serial.print(",");
//     // Serial.print("GyroY:");
//     // Serial.print(g.gyro.y);
//     // Serial.print(",");
//     // Serial.print("GyroZ:");
//     // Serial.print(g.gyro.z);
//     // Serial.println("");
//   // }

//   // delay(10);
// }