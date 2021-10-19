#include "Storage.h"
#include <EEPROM.h>
#include <Arduino.h>
#include <FC.h>
#include <pid.h>

bool Storage::read(BoolValues floatVal) {
    bool flag;
    EEPROM.get(boolStart() + floatVal * 1, flag);
    return flag;
}

float Storage::read(FloatValues floatVal) {
    float val;
    EEPROM.get(floatStart() + floatVal * sizeof(float), val);
    return val;
}

Vec3 Storage::read(Vec3Values floatVal) {
    double x, y, z;
    int addr = vec3Start() + floatVal * sizeof(double) * 3;
    EEPROM.get(addr, x);
    EEPROM.get(addr + sizeof(double), y);
    EEPROM.get(addr + sizeof(double) * 2, z);
    return Vec3(x, y, z);
}

Quaternion Storage::read(QuaternionValues quatVal) {
    double x, y, z, w;
    int addr = quaternionStart() + quatVal * STORAGE_SIZE_QUATERNION;
    EEPROM.get(addr, x);
    EEPROM.get(addr + sizeof(double), y);
    EEPROM.get(addr + sizeof(double) * 2, z);
    EEPROM.get(addr + sizeof(double) * 3, w);
    return Quaternion(w, x, y, z);
}

Matrix3 Storage::read(Matrix3Values matVal) {
    Matrix3 mat;
    int addr = matrix3Start() + matVal * sizeof(double) * 9;
    for (size_t i = 0; i < 9; i++) {
        EEPROM.get(addr + i * sizeof(double), mat.m[i]);
    }
    return mat;
}

PID Storage::read(PidValues pidVal) {
    PID pid;
    int addr = pidStart() + pidVal * STORAGE_SIZE_PID;
    EEPROM.get(addr + 0 * sizeof(float), pid.p);
    EEPROM.get(addr + 1 * sizeof(float), pid.i);
    EEPROM.get(addr + 2 * sizeof(float), pid.d);
    EEPROM.get(addr + 3 * sizeof(float), pid.dlpf);
    EEPROM.get(addr + 4 * sizeof(float), pid.maxOut);
    float aux = 0;
    EEPROM.get(addr + 5 * sizeof(float), aux);
    pid.useAuxTuning = aux > 0;
    return pid;
}

void Storage::write(BoolValues boolVal, bool flag) {
    int addr = boolStart() +  boolVal * STORAGE_SIZE_BOOL;
    if(addr >= eepromSize - 10) return;//preserve last 10 bytes
    EEPROM.put(addr, flag);
}

void Storage::write(FloatValues floatVal, float val) {
    int addr = floatStart() + floatVal * STORAGE_SIZE_FLOAT;
    if(addr >= eepromSize - 10) return;//preserve last 10 bytes
    EEPROM.put(addr, val);
}

void Storage::write(Vec3Values vec3Val, Vec3 vec) {
    int addr = vec3Start() + vec3Val * STORAGE_SIZE_VEC3;
    if(addr >= eepromSize - 10) return;//preserve last 10 bytes
    EEPROM.put(addr, vec.x);
    EEPROM.put(addr + sizeof(double), vec.y);
    EEPROM.put(addr + sizeof(double) * 2, vec.z);
}

void Storage::write(QuaternionValues quatVal, Quaternion quat) {
    int addr = quaternionStart() + quatVal * STORAGE_SIZE_QUATERNION;
    if(addr >= eepromSize - 10) return;//preserve last 10 bytes
    EEPROM.put(addr, quat.x);
    EEPROM.put(addr + sizeof(double), quat.y);
    EEPROM.put(addr + sizeof(double) * 2, quat.z);
    EEPROM.put(addr + sizeof(double) * 3, quat.w);
}

void Storage::write(Matrix3Values mat3Val, Matrix3 mat) {
    int addr = matrix3Start() + mat3Val * STORAGE_SIZE_MATRIX3;
    if(addr >= eepromSize - 10) return;//preserve last 10 bytes
    for (size_t i = 0; i < 9; i++) {
        EEPROM.put(addr + i * sizeof(double), mat.m[i]);
    }
}

void Storage::write(PidValues pidVal, PID pid) {
    int addr = pidStart() + pidVal * STORAGE_SIZE_PID;
    if(addr >= eepromSize - 10) return;//preserve last 10 bytes
    EEPROM.put(addr + 0 * sizeof(float), pid.p);
    EEPROM.put(addr + 1 * sizeof(float), pid.i);
    EEPROM.put(addr + 2 * sizeof(float), pid.d);
    EEPROM.put(addr + 3 * sizeof(float), pid.dlpf);
    EEPROM.put(addr + 4 * sizeof(float), pid.maxOut);
    float aux = pid.useAuxTuning ? 1.0f : 0.0f;
    EEPROM.put(addr + 5 * sizeof(float), aux);
}

void Storage::begin() {
    int x;
    EEPROM.get(eepromSize - 5, x);
    if(x != STORAGE_VERSION) {
        erase();
        EEPROM.put(eepromSize - 5, STORAGE_VERSION);
    }
    Serial.print("EEPROM size: ");
    Serial.println(size());
}

void Storage::erase() {
    writeDefaults();
}

void Storage::writeDefaults() {
    Serial.println("Erasing EEPROM in 3s cut power to cancel!");
    Serial2.println("Erasing EEPROM in 2s cut power to cancel!");
    delay(1000);
    Serial.println("Erasing EEPROM in 2s cut power to cancel!");
    Serial2.println("Erasing EEPROM in 2s cut power to cancel!");
    delay(1000);
    Serial.println("Erasing EEPROM in 1s cut power to cancel!");
    Serial2.println("Erasing EEPROM in 1s cut power to cancel!");
    delay(1000);
    write(BoolValues::propsIn, true);
    write(BoolValues::useLeds, true);
    write(BoolValues::useAntiGravity, true);
    write(BoolValues::useVCell, true);

    PID rateR_PID    (RATE_PID_RP,   RATE_PID_RI,  RATE_PID_RD,  RATE_PID_RD_LPF,  RATE_PID_R_MAX);
    PID rateP_PID    (RATE_PID_PP,   RATE_PID_PI,  RATE_PID_PD,  RATE_PID_PD_LPF,  RATE_PID_P_MAX);
    PID rateY_PID    (RATE_PID_YP,   RATE_PID_YI,  RATE_PID_YD,  RATE_PID_YD_LPF,  RATE_PID_Y_MAX);
    PID levelR_PID   (LEVEL_PID_RP,  LEVEL_PID_RI, LEVEL_PID_RD, LEVEL_PID_RD_LPF, LEVEL_PID_R_MAX);
    PID levelP_PID   (LEVEL_PID_PP,  LEVEL_PID_PI, LEVEL_PID_PD, LEVEL_PID_PD_LPF, LEVEL_PID_P_MAX);
    PID levelY_PID   (LEVEL_PID_YP,  LEVEL_PID_YI, LEVEL_PID_YD, LEVEL_PID_YD_LPF, LEVEL_PID_Y_MAX);

    write(FloatValues::insAccMaxG, 0.2);
    write(FloatValues::accLPF, 0.8f);
    write(FloatValues::gyroLPF, 1.0f);
    write(FloatValues::accInsInf, 0.007f);
    write(FloatValues::magInsInf, 0.02f);
    write(FloatValues::antiGravityMul, 1.0f);
    write(FloatValues::boostLpf, 0.005f);
    write(FloatValues::boostSpeed, 40.0f);
    write(FloatValues::batLpf, 0.001f);
    write(FloatValues::batMul, 11.9f);

    write(FloatValues::insSensorFusion, 0.0f);

    /**
     * PIDs
     */
    write(PidValues::ratePidR,      rateR_PID);
    write(PidValues::ratePidP,      rateP_PID);
    write(PidValues::ratePidY,      rateY_PID);

    write(PidValues::levelPidR,    levelR_PID);
    write(PidValues::levelPidP,    levelP_PID);
    write(PidValues::levelPidY,    levelY_PID);

    write(FloatValues::loopFreqRate, 2000);
    write(FloatValues::loopFreqLevel, 2000);

    write(FloatValues::iRelaxMinRate, I_RELAX_MIN_RATE);

    write(Vec3Values::accOffset,    Vec3(-0.1754f, -0.2355f, 0.1227f));
    write(Vec3Values::accScale,     Vec3(0.9971, 0.9975, 0.9928));
    write(Vec3Values::gyroOffset,   Vec3(1.5007f, 0.2541f, 0.2211f));
    write(Vec3Values::magOffset,    Vec3(6.5348, 10.6397, -43.9933));
    write(Vec3Values::magScale,     Vec3(1.0142, 1.0106, 0.9761));


    write(QuaternionValues::accAngleOffset,  Quaternion());

    Serial.println("Wrote defaults into EEPROM");
    Serial2.println("Wrote defaults into EEPROM");
}

int Storage::boolStart() {
    return offset;
}

int Storage::floatStart() {
    return boolStart() + (BoolValues::BoolValuesCount) * STORAGE_SIZE_BOOL;
}

int Storage::vec3Start() {
    return floatStart() + (FloatValues::FloatValuesCount) * STORAGE_SIZE_FLOAT;
}

int Storage::quaternionStart() {
    return vec3Start() + (Vec3Values::Vec3ValuesCount) * STORAGE_SIZE_VEC3;
}

int Storage::matrix3Start() {
    return quaternionStart() + (QuaternionValues::QuaternionValuesCount) * STORAGE_SIZE_QUATERNION;
}

int Storage::pidStart() {
    return matrix3Start() + (Matrix3Values::Matrix3ValuesCount) * STORAGE_SIZE_MATRIX3;
}

int Storage::size() {
    return pidStart() + (PidValues::PidValuesCount) * STORAGE_SIZE_PID;
}