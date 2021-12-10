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

    write(FloatValues::m1Pin, 4);
    write(FloatValues::m2Pin, 3);
    write(FloatValues::m3Pin, 5);
    write(FloatValues::m4Pin, 2);

    PID rateR_PID    (RATE_PID_RP,   RATE_PID_RI,  RATE_PID_RD,  RATE_PID_RD_LPF,  RATE_PID_R_MAX);
    PID rateP_PID    (RATE_PID_PP,   RATE_PID_PI,  RATE_PID_PD,  RATE_PID_PD_LPF,  RATE_PID_P_MAX);
    PID rateY_PID    (RATE_PID_YP,   RATE_PID_YI,  RATE_PID_YD,  RATE_PID_YD_LPF,  RATE_PID_Y_MAX);
    PID levelR_PID   (LEVEL_PID_RP,  LEVEL_PID_RI, LEVEL_PID_RD, LEVEL_PID_RD_LPF, LEVEL_PID_R_MAX);
    PID levelP_PID   (LEVEL_PID_PP,  LEVEL_PID_PI, LEVEL_PID_PD, LEVEL_PID_PD_LPF, LEVEL_PID_P_MAX);
    PID levelY_PID   (LEVEL_PID_YP,  LEVEL_PID_YI, LEVEL_PID_YD, LEVEL_PID_YD_LPF, LEVEL_PID_Y_MAX);
    PID altitudePid  (ATITUDE_PID_P,     ATITUDE_PID_I,  ATITUDE_PID_D,  ATITUDE_PID_D_LPF,  ATITUDE_PID_MAX);
    PID velPid       (VEL_PID_P,         VEL_PID_I,      VEL_PID_D,      VEL_PID_D_LPF,      VEL_PID_P);

    write(FloatValues::insAccMaxG, 1.0);
    write(FloatValues::accLPF, 0.01f);
    write(FloatValues::gyroLPF, 1.0f);
    write(FloatValues::accInsInf, 0.0002f);
    write(FloatValues::magInsInf, 1.0f);
    write(FloatValues::antiGravityMul, 1.0f);
    write(FloatValues::boostLpf, 0.005f);
    write(FloatValues::boostSpeed, 40.0f);
    write(FloatValues::batLpf, 0.001f);
    write(FloatValues::batMul, 12.10f);

    write(FloatValues::insSensorFusion, 0.0f);

    write(FloatValues::insSensorFusion, 0.0f);
    write(FloatValues::magZOffset, 0.0f);
    write(FloatValues::throttleMul4S, 1.0f);
    write(FloatValues::throttleMul6S, 1.0f);

    /**
     * FC
     */
    write(FloatValues::angleModeMaxAngle, ANGLE_MODE_MAX_ANGLE);
    write(FloatValues::gpsMaxSpeedHorizontal, GPS_MAX_SPEED_HORIZONTAL);
    write(FloatValues::gpsMaxSpeedVertical, GPS_MAX_SPEED_VERTICAL);
    write(FloatValues::hoverThrottle, HOVER_THROTTLE);
    write(FloatValues::launchIBoostSeconds, LAUNCH_I_BOOST_SECONDS);
    write(FloatValues::launchIBoostLevel, LAUNCH_I_BOOST_LEVEL);
    write(FloatValues::launchIBoostAltitude, LAUNCH_I_BOOST_ALTITUDE);

    write(Vec3Values::rateR, Rates(RATE_RC, RATE_SUPER, RATE_RC_EXPO).toVec3());
    write(Vec3Values::rateP, Rates(RATE_RC, RATE_SUPER, RATE_RC_EXPO).toVec3());
    write(Vec3Values::rateY, Rates(RATE_RC, RATE_SUPER, RATE_RC_EXPO).toVec3());

    /**
     * PIDs
     */
    write(PidValues::ratePidR,      rateR_PID);
    write(PidValues::ratePidP,      rateP_PID);
    write(PidValues::ratePidY,      rateY_PID);

    write(PidValues::levelPidR,     levelR_PID);
    write(PidValues::levelPidP,     levelP_PID);
    write(PidValues::levelPidY,     levelY_PID);

    write(PidValues::altitudePid,   altitudePid);

    write(PidValues::velPid,        velPid);
    

    write(FloatValues::loopFreqRate, 4000);
    write(FloatValues::loopFreqLevel, 4000);

    write(FloatValues::iRelaxMinRate, I_RELAX_MIN_RATE);

    write(Vec3Values::accOffset,    Vec3(0, 0, 0));
    write(Vec3Values::accScale,     Vec3(1, 1, 1));
    write(Vec3Values::gyroOffset,   Vec3(0, 0, 0));
    write(Vec3Values::gyroScale,    Vec3(1.028, 1.028, 1.028));
    write(Vec3Values::magOffset,    Vec3(-5, 715, -212.5));
    write(Vec3Values::magScale,     Vec3(0.9743, 0.9943, 1.0331));


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