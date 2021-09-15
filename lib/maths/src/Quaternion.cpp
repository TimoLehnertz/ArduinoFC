#include "maths.h"

Quaternion::Quaternion() : Rotation() {
    Quaternion f = Quaternion::getForward();
    w = f.w;
    x = f.x;
    y = f.y;
    z = f.z;
}

Quaternion::Quaternion(double w, double x, double y, double z) : Rotation(w, x, y, z) {}

Quaternion::Quaternion(const EulerRotation &euler) : Rotation() {
    setFromEuler(euler);
}

Quaternion::Quaternion(const Vec3 &v) : Rotation(v) {}

Quaternion::Quaternion(const Vec3 &v, double theta) : Rotation() {
    setFromAngle(v, theta);
    normalize();
}

void Quaternion::setFromAngle(const Vec3 &v, double theta) {
    w = cos(theta / 2);
    setV(v * sin(theta / 2));
}

void Quaternion::setFromEuler(const EulerRotation &euler) {
    switch(euler.mode) {
    case EulerMode::XYZ_EULER:
    {
        double roll = euler.x;
        double pitch = euler.y;
        double yaw = euler.z;
        
        x = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2);
        y = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2);
        z = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2);
        w = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2);
        break;
    }
    case EulerMode::ZYX_EULER:
    {
        double cy = cos(euler.z * 0.5);
        double sy = sin(euler.z * 0.5);
        double cp = cos(euler.y * 0.5);
        double sp = sin(euler.y * 0.5);
        double cr = cos(euler.x * 0.5);
        double sr = sin(euler.x * 0.5);

        w = cr * cp * cy + sr * sp * sy;
        x = sr * cp * cy - cr * sp * sy;
        y = cr * sp * cy + sr * cp * sy;
        z = cr * cp * sy - sr * sp * cy;
        break;
    }
    default:
        break;
    }
    normalize();
}

Quaternion Quaternion::normalize() {
    double n = sqrt(x*x + y*y + z*z + w*w);
    x /= n;
    y /= n;
    z /= n;
    w /= n;
    return *this;
}

Quaternion Quaternion::conjugate() {
    x *= -1;
    y *= -1;
    z *= -1;
    return *this;
}

Quaternion Quaternion::calibrate(double limit) {
    EulerRotation r = toEulerZYX();
    if(((r.y * 4068) / 71) > -limit && ((r.y * 4068) / 71) < limit) {
        setFromEuler(EulerRotation(r.x, -r.y, -r.z));
    }
    return *this;
}

double Quaternion::lengthSquared() const {
    return w * w + pow(getV(), 2);
}

EulerRotation Quaternion::toEulerZYX() const {
    double t0 = 2.0 * (w * x + y * z);
    double t1 = 1.0 - 2.0 * (x * x + y * y);
    double roll = atan2(t0, t1);//

    double t2 = 2.0 * (w * y - z * x);
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? 1.0 : t2;
    double pitch = asin(t2);

    double t3 = 2.0 * (w * z + x * y);
    double t4 = 1.0 - 2.0 * (y * y + z * z);
    double yaw = atan2(t3, t4);

    return EulerRotation(roll, -pitch, -yaw, ZYX_EULER);
}

double Quaternion::dot(const Quaternion &q) const {
    return q.getV() * getV() + q.w + w;
}

Quaternion Quaternion::multiply(const Quaternion &q) {
    double x = this->x;
    double y = this->y;
    double z = this->z;
    double w = this->w;
    this->x =  x * q.w + y * q.z - z * q.y + w * q.x;
    this->y = -x * q.z + y * q.w + z * q.x + w * q.y;
    this->z =  x * q.y - y * q.x + z * q.w + w * q.z;
    this->w = -x * q.x - y * q.y - z * q.z + w * q.w;
    return *this;
}

Quaternion Quaternion::multiply(const Vec3 &v) {
    double x = this->x;
    double y = this->y;
    double z = this->z;
    double w = this->w;
    this->w = -x * v.x - y * v.y - z * v.z;
    this->x = w  * v.x + y * v.z - z * v.y;
    this->y = w  * v.y + z * v.x - x * v.z;
    this->z = w  * v.z + x * v.y - y * v.x;
    return *this;
}

Quaternion Quaternion::multiply(double s) {
		w *= s;
		x *= s;
		y *= s;
		z *= s;
        return *this;
	}

Quaternion Quaternion::add(const Quaternion &q2) {
    x += q2.x;
    y += q2.y;
    z += q2.z;
    w += q2.w;
    return *this;
}

Quaternion Quaternion::operator * (const Quaternion &q) const {
    Quaternion qres(w, x, y, z);
    return qres.multiply(q);
}

Quaternion Quaternion::operator *= (const Quaternion &q) {
    multiply(q);
    return *this;
}

Quaternion Quaternion::operator * (const Vec3 &v) const {
    Quaternion qres(w, x, y, z);
    return qres.multiply(v);
}

Quaternion Quaternion::operator * (double s) const {
    Quaternion qres(w, x, y, z);
    return qres.multiply(s);
}

Quaternion Quaternion::operator + (const Quaternion &q) const {
    Quaternion qres(w, x, y, z);
    return qres.add(q);
}

// static Quaternion Quaternion::lerp(const Quaternion &q1, const Quaternion q2, double t) {
//     t = max(0, min(1, t));
//     return (q1.clone() * (1 - t) + q2.clone() * t).normalize();
// }

Quaternion Quaternion::clone() const {
    return Quaternion(w, x, y, z);
}

void Quaternion::rotate(Vec3 &v) const {
    Quaternion p(v);
    Quaternion q = clone();
    q.normalize();
    q *= p;
    q.multiply(clone().conjugate().normalize());
    v.setFrom(q);
}

void Quaternion::rotateReverse(Vec3 &v) const {
    Quaternion p(v);
    Quaternion q = clone().conjugate();
    q.normalize();
    q *= p;
    q.multiply(clone().normalize());
    v.setFrom(q);
}