#include "Algebra.h"

#include <Arduino.h>
#include <math.h>

float Vec3::operator[](unsigned i) const {
    switch (i) {
    case 0:
        return x;
    case 1:
        return y;
    case 2:
        return z;
    default:
        return 0;
    }
}
float & Vec3::operator[](unsigned i) {
    switch (i) {
    case 0:
        return x;
    case 1:
        return y;
    case 2:
        return z;
    default:
        return x;
    }
}

Vec3 & Vec3::operator+=(const Vec3 &v) {
    x += v.x;
    y += v.y;
    z += v.z;
    return *this;
}

Vec3 & Vec3::operator-=(const Vec3 &v) {
    x -= v.x;
    y -= v.y;
    z -= v.z;
    return *this;
}

Vec3 & Vec3::operator*=(float a) {
    x *= a;
    y *= a;
    z *= a;
    return *this;
}

Vec3 & Vec3::operator/=(float a) {
    x /= a;
    y /= a;
    z /= a;
    return *this;
}

float Vec3::dot(const Vec3 &v) const {
    return x * v.x + y * v.y + z * v.z;
}

float Vec3::norm() const {
    return sqrt(x * x + y * y + z * z);
}

Vec3 Vec3::normalized() const {
    float n = norm();
    return Vec3(x / n, y / n, z / n);
}


void Vec3::print(char prefix) const {
    char buf[64];
    if (prefix == '\0') sprintf(buf, "x:%f,y:%f,z:%f,", x, y, z);
    else sprintf(buf, "%cx:%f,%cy:%f,%cz:%f,",
                 prefix, x, prefix, y, prefix, z);
    Serial.print(buf);
}


const Vec3 Vec3::e_x(1, 0, 0);
const Vec3 Vec3::e_y(0, 1, 0);
const Vec3 Vec3::e_z(0, 0, 1);
const Vec3 Vec3::zero(0, 0, 0);

Quaternion Quaternion::fromEuler(float x, float y, float z) {
    float c1 = cos(y/2);
    float c2 = cos(z/2);
    float c3 = cos(x/2);

    float s1 = sin(y/2);
    float s2 = sin(z/2);
    float s3 = sin(x/2);
    Quaternion ret;
    ret.a = c1 * c2 * c3 + s1 * s2 * s3;
    ret.b = c1 * c2 * s3 - s1 * s2 * c3;
    ret.c = s1 * c2 * c3 + c1 * s2 * s3;
    ret.d = c1 * s2 * c3 - s1 * c2 * s3;
    return ret;
}

float Quaternion::operator[](unsigned i) const {
    switch (i) {
    case 0:
        return a;
    case 1:
        return b;
    case 2:
        return c;
    case 3:
        return d;
    default:
        return 0;
    }
}
float & Quaternion::operator[](unsigned i) {
    switch (i) {
    case 0:
        return a;
    case 1:
        return b;
    case 2:
        return c;
    case 3:
        return d;
    default:
        return a;
    }
}

Quaternion & Quaternion::operator*=(const Quaternion &q) {
    Quaternion ret;
    ret.a = a*q.a - b*q.b - c*q.c - d*q.d;
    ret.b = b*q.a + a*q.b + c*q.d - d*q.c;
    ret.c = a*q.c - b*q.d + c*q.a + d*q.b;
    ret.d = a*q.d + b*q.c - c*q.b + d*q.a;
    return (*this = ret);
}

Quaternion Quaternion::inverse() const {
    float norm2 = a * a + b * b + c * c + d * d;
    return Quaternion(a / norm2, -b / norm2, -c / norm2, -d / norm2);
}

Vec3 Quaternion::rotate(Vec3 &v) const {
    Quaternion q = (*this) * Quaternion(v) * inverse();
    return Vec3(q.b, q.c, q.d);
}

void Quaternion::print() {
    char buf[64];
    sprintf(buf, "a:%f,b:%f,c:%f,d:%f,", a, b, c, d);
    Serial.print(buf);
}


#define CIRCLES_OF_LONGITUDE 40008000.0
#define EQUATOR_RADIUS       6378137.0

float latitudeDistance(float lat) {
    return lat * CIRCLES_OF_LONGITUDE / 360.0;
}
float longitudeDistance(float lng, float lat) {
    return EQUATOR_RADIUS * cos(lat / 180.0 * M_PI) * lng / 180.0 * M_PI;
}

Vec3 ENU_between_LLH(Vec3 a_LLH, Vec3 b_LLH) {
    float lat_middle = (a_LLH.y + b_LLH.y) / 2;
    float east  = latitudeDistance(a_LLH.y - b_LLH.y);
    float north = longitudeDistance(a_LLH.x - b_LLH.x, lat_middle);
    float up    = a_LLH.z - b_LLH.z;
    return Vec3(east, north, up);
}


template <int size>
Vector<size>::Vector() {
    this->values = (float*)calloc(size, sizeof(float));
    this->allocated = true;
}

template <int size>
Vector<size>::Vector(float* values) {
    this->values = values;
    this->allocated = true;
}

template <int size>
Vector<size>::~Vector() { free(this->values); };


template <int rows, int cols>
Matrix<rows, cols>::Matrix() {
    values = (float *)calloc(rows * cols, sizeof(float));
}

template <int rows, int cols>
Matrix<rows, cols>::~Matrix() {
    free(this->values);
};

template <int rows, int cols>
template <int cols2>
Matrix<rows, cols2> Matrix<rows, cols>::operator*(Matrix<cols, cols2> &m) const {
    Matrix<rows, cols2> r;
    for (int k = 0; k < cols2; k++) {
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                r.values[i * cols2 + k] +=
                    this->values[i * cols + j] * m.values[j * cols + k];
            }
        }
    }
    return r;
}


template <int rows, int cols>
const Vector<cols> Matrix<rows, cols>::operator[](unsigned i) const {
    return Vector<cols>(this->values + i * cols);
}

template <int rows, int cols>
Vector<cols> Matrix<rows, cols>::operator[](unsigned i) {
    return Vector<cols>(this->values + i * cols);
}
