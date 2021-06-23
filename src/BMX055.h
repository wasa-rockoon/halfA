
#include <Arduino.h>
#include <Wire.h>

#ifndef BMX055_h
#define BMX055_h

#define Addr_Accl_Def 0x18
#define Addr_Gyro_Def 0x68
#define Addr_Mag_Def  0x10

struct Vec3 {
    float x;
    float y;
    float z;
};

enum AcclRange: uint8_t {
    PM2G = 2,
    PM4G = 4,
    PM8G = 8,
    PM16G = 16,
};

enum GyroRange: uint16_t {
    PM125DPS = 125,
    PM250DPS = 250,
    PM500DPS = 500,
    PM1000DPS = 1000,
    PM2000DPS = 2000,
};



class Function {
public:
    void writeRegister(uint8_t addr, uint8_t value);
    uint8_t begin(uint8_t addr);

protected:
    uint8_t Addr;
};

class Accelerometer: public Function {
public:
    Vec3 read();
    void setRange(AcclRange range);

private:
    float factor;
};

class Gyroscope: public Function {
public:
    Vec3 read();
    void setRange(GyroRange range);

private:
    float factor;
};


class Magnetometer: public Function {
public:
    Magnetometer();

    Vec3 center;
    float radius;

    Vec3 read();
    float calibrate(Vec3 mag, float gain = 0.0001);

private:
};



class BMX055 {
public:
    Accelerometer Accl;
    Gyroscope     Gyro;
    Magnetometer  Mag;

    BMX055();

    uint8_t begin();
    uint8_t begin(uint8_t accl, uint8_t gyro, uint8_t mag);
    void reset();

};

#endif
