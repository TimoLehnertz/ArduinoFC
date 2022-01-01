/**
 * @file Matrix.cpp
 * @author Timo Lehnertz
 * @brief 
 * @version 0.1
 * @date 2022-01-01
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "maths.h"

Matrix3::Matrix3(double s0, double s1, double s2, double s3, double s4, double s5, double s6, double s7, double s8) {
    m[0] = s0;
    m[1] = s1;
    m[2] = s2;
    m[3] = s3;
    m[4] = s4;
    m[5] = s5;
    m[6] = s6;
    m[7] = s7;
    m[8] = s8;
}

Matrix3::Matrix3(const char* str) {
    int pos = 0;
    char numBuff[20];
    int numPos = 0;
    size_t len = strlen(str);
    for (size_t i = 0; i < len; i++) {
        char c = str[i];
        if(c == ',' || i == len - 1) {
            numBuff[numPos] = '\0';
            m[pos] = atof(numBuff);
            pos++;
            numPos = 0;
        } else if(numPos < 20){
            numBuff[numPos] = c;
            numPos++;
        }
    }
    
}

Matrix3::Matrix3() {

}

Matrix3::Matrix3(double arr[9]) {
    m[0] = arr[0];
    m[1] = arr[1];
    m[2] = arr[2];
    m[3] = arr[3];
    m[4] = arr[4];
    m[5] = arr[5];
    m[6] = arr[6];
    m[7] = arr[7];
    m[8] = arr[8];
}

Vec3 Matrix3::toVec3() const {
    return Vec3(m[0], m[3], m[6]);
}

Matrix3 Matrix3::operator * (const Matrix3& m2) const {
    double m3[9];
    for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 3; col++) {
            m3[rowCol(row, col)] = 0;
            for (int i = 0; i < 3; i++) {
                m3[rowCol(row, col)] += m[rowCol(row, i)] * m2.m[rowCol(i, col)];
            }
        }
    }
    return Matrix3(m3);
}

// Matrix3 Matrix3::operator * (const Vec3& v) const {
//     return *this * v.toMatrix3();
// }

Vec3 Matrix3::operator * (const Vec3& v) const {
    return (*this * v.toMatrix3()).toVec3();
}

Matrix3 Matrix3::getTranspose() const {
    Matrix3 transpose;
    for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 3; col++) {
            transpose.m[rowCol(row, col)] = m[rowCol(col, row)];
        }
    }
    return transpose;
}

int Matrix3::rowCol(int row, int col) {
    return row * 3 + col;
};