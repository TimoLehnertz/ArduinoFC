#pragma once
#include <Arduino.h>

class Vec3;
class Quaternion;


class Matrix3 {
public:
    double m[9];
    Matrix3();
    Matrix3(double, double, double, double, double, double, double, double, double);
    Matrix3(const char*);
    Matrix3(double*);

    Vec3 toVec3() const;

    Matrix3 getTranspose() const;

    Matrix3 operator * (const Matrix3&) const;
    Vec3 operator * (const Vec3&) const;
    String toString () const {
        return String("") + m[0] + String(",") + m[1] + String(",") + m[2] + String(",") + m[3] + String(",") + m[4] + String(",") + m[5] + String(",") + m[6] + String(",") + m[7] + String(",") + m[8] + String(",");
        // String s("");
        // for (size_t i = 0; i < 9; i++) {
        //     // Serial.println(m[i]);
        //     s += String(",") + m[i];
        // }
        // return s;
    }
private:
    static int rowCol(int, int);
};

struct Vec3 {
    double x, y, z;

    Vec3();
    Vec3(char*);
    Vec3(double);
    Vec3(double, double, double);
    Vec3(double[]);

    double getLength() const;

    double dot(const Vec3&) const;
    Vec3 crossProduct(const Vec3&) const;

    double getValue() const;

    Vec3 toRad();
    Vec3 toDeg();

    void setFrom(const Vec3&);

    void setFrom(const Matrix3&);

    void setFrom(const Quaternion&);

    void toUnitLength();

    bool equals(const Vec3& v) const;

    bool greaterThan(const Vec3& v) const;

    bool greaterThanEquals(const Vec3& v) const;

    double absSum() const;

    Vec3 clone() const;

    Matrix3 toMatrix3() const;

    // Vector operators
    Vec3 operator + (const Vec3&) const;
    Vec3 operator - (const Vec3&) const;
    Vec3 operator * (const Vec3&) const;
    Vec3 operator / (const Vec3&) const;
    Vec3 operator ^ (const Vec3&) const;

    Vec3 operator += (const Vec3&);
    Vec3 operator -= (const Vec3&);
    Vec3 operator *= (const Vec3&);
    Vec3 operator /= (const Vec3&);
    Vec3 operator ^= (const Vec3&);

    // scalar operators
    Vec3 operator + (double) const;
    Vec3 operator - (double) const;
    Vec3 operator * (double) const;
    Vec3 operator * (float) const;
    Vec3 operator / (double) const;
    // Vec3 operator / (long) const;
    Vec3 operator ^ (double) const;

    Vec3 operator += (double);
    Vec3 operator -= (double);
    Vec3 operator *= (double);
    Vec3 operator /= (double);
    Vec3 operator ^= (double);

    Vec3 operator = (const Vec3&);
    Vec3 operator = (const Matrix3&);

    bool operator == (const Vec3&) const;
    bool operator != (const Vec3&) const;

    bool operator > (const Vec3&) const;
    bool operator >= (const Vec3&) const;
    bool operator < (const Vec3&) const;
    bool operator <= (const Vec3&) const;

    operator double();
    operator Matrix3();

    String toString() const;
};


class Rotation {
public:
    double w, x, y, z;

    Rotation() : w(0), x(0), y(0), z(0) {}
    Rotation(const Vec3 &v) : w(0), x(v.x), y(v.y), z(v.z) {}
    Rotation(double x, double y, double z) : x(x), y(y), z(z) {}
    Rotation(double w , double x, double y, double z) : w(w), x(x), y(y), z(z) {}

    virtual void rotate(Vec3&) const = 0;
    virtual void rotateReverse(Vec3&) const = 0;

    void setV(const Vec3& v) {
        x = v.x;
        y = v.y;
        z = v.z;
    }

    Vec3 getV() const {
        return Vec3(x, y, z);
    }
};

enum EulerMode {
    XYZ_EULER,
    ZYX_EULER,
};

class EulerRotation : public Rotation {
public:
    // double x, y, z;
    EulerMode mode;

    EulerRotation();
    EulerRotation(const Vec3&, EulerMode = ZYX_EULER);
    EulerRotation(double, double, double, EulerMode = ZYX_EULER);

    void rotate(Vec3&) const;
    void rotateReverse(Vec3&) const;

    Matrix3 getMatrix() const;

    EulerRotation clone() const;

    EulerRotation toMode(EulerMode mode);

    operator Matrix3() const;

    double getPitch();
    double getRoll();
    double getYaw();

    String toString() const {
        return String("Euler (deg) (x=") + x * 57296 / 1000 + String(",y=") + y * 57296 / 1000 + String(",z=") + z * 57296 / 1000 + String(")");
    }
};

class Quaternion : public Rotation {
public:
    Quaternion();
    Quaternion(const Vec3&);
    Quaternion(double, double, double, double);
    Quaternion(const EulerRotation&);
    Quaternion(const Vec3&, double theta);
    Quaternion(char* str);
    
    void setFromAngle(const Vec3&, double);
    void setFromEuler(const EulerRotation&);

    EulerRotation toEulerZYX() const;

    void rotate(Vec3&) const;
    void rotateReverse(Vec3&) const;

    Quaternion normalize();
    Quaternion conjugate();
    Quaternion calibrate(double limit = 80); // compensate for potential gimbal lock from conversions
    double lengthSquared() const;

    double dot(const Quaternion&) const;

    static Quaternion getForward() {
        return Quaternion(EulerRotation(0, 0, 0));
    }

    Quaternion clone() const;

    Quaternion multiply(const Quaternion&);
    Quaternion multiply(const Vec3&);
    Quaternion multiply(double);

    Quaternion add(const Quaternion&);

    Quaternion operator * (const Quaternion&) const;
    Quaternion operator * (double) const;
    Quaternion operator *= (const Quaternion&);
    Quaternion operator * (const Vec3&) const;

    Quaternion operator + (const Quaternion&) const;

    static Quaternion lerp(const Quaternion &q1, const Quaternion q2, double t) {
        t = max(0.0, min(1.0, t));
        return (q1.clone() * (1 - t) + q2.clone() * t).normalize();
    }


    String toString() const {
        return String(w) + String(",") + x + String(",") + y + String(",") + z;
    }
};