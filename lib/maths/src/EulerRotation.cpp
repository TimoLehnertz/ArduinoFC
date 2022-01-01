/**
 * @file EulerRotation.cpp
 * @author Timo Lehnertz
 * @brief 
 * @version 0.1
 * @date 2022-01-01
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "maths.h"

EulerRotation::EulerRotation() : Rotation(), mode(ZYX_EULER){}

EulerRotation::EulerRotation(const Vec3& v, EulerMode mode) : Rotation(v), mode(mode) {}

EulerRotation::EulerRotation(double x, double y, double z, EulerMode mode) : Rotation(x, y, z), mode(mode) {}

EulerRotation::operator Matrix3() const {
    return getMatrix();
}

EulerRotation EulerRotation::clone() const {
    return EulerRotation(x, y, z);
}

Matrix3 EulerRotation::getMatrix() const {
    Matrix3 xMat(
        1.0, 0.0, 0.0,
        0.0, cos(x), -sin(x),
        0.0, sin(x), cos(x));

    Matrix3 yMat(
        cos(y), 0, sin(y),
        0, 1, 0,
        -sin(y), 0, cos(y));

    Matrix3 zMat(
        cos(z), -sin(z), 0,
        sin(z), cos(z), 0,
        0, 0, 1);

    switch(mode) {
    case XYZ_EULER: return xMat * yMat * zMat;
    case ZYX_EULER: return zMat * yMat * xMat;
    default: return Matrix3();
    }
}

EulerRotation EulerRotation::toMode(EulerMode mdoe) {
    mode = mode;
    return *this;
}

void EulerRotation::rotate(Vec3 &v) const {
    v = getMatrix() * v;
}

void EulerRotation::rotateReverse(Vec3 &v) const {
    v = getMatrix().getTranspose() * v;
}

double EulerRotation::getPitch() {
    return y;
}

double EulerRotation::getRoll() {
    return x;
}

double EulerRotation::getYaw() {
    return z;
}