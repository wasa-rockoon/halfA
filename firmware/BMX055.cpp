#include "BMX055.h"

BMX055::BMX055() {}

uint8_t BMX055::begin() {
    return begin(Addr_Accl_Def, Addr_Gyro_Def, Addr_Mag_Def);
}

uint8_t BMX055::begin(uint8_t Addr_Accl, uint8_t Addr_Gyro, uint8_t Addr_Mag) {
    Accl.begin(Addr_Accl);
    Gyro.begin(Addr_Gyro);
    Mag.begin(Addr_Mag);

    Accl.setRange(PM2G);
    delay(100);
    Accl.writeRegister(0x10, 0b1100);
    delay(100);
    Accl.writeRegister(0x11, 0x00);
    delay(100);

    Gyro.setRange(PM125DPS);
    delay(100);
    Gyro.writeRegister(0x10, 0x07);
    delay(100);
    Gyro.writeRegister(0x11, 0x00);
    delay(100);


    Mag.writeRegister(0x4B, 0x83);
    delay(100);
    Mag.writeRegister(0x4B, 0x01);
    delay(100);
    Mag.writeRegister(0x4C, 0x00);
    delay(100);
    Mag.writeRegister(0x4E, 0x84);
    delay(100);
    Mag.writeRegister(0x51, 0x04);
    delay(100);
    Mag.writeRegister(0x52, 0x0F);
    delay(100);

    return 1;
}

void BMX055::reset() {

}


// Function ====================================================================

uint8_t Function::begin(uint8_t addr) {
    Addr = addr;
    return 1;
}

void Function::writeRegister(uint8_t addr, uint8_t value) {
    Wire.beginTransmission(Addr);
    Wire.write(addr);
    Wire.write(value);
    Wire.endTransmission();
}



// Accelerometer ===============================================================

void Accelerometer::setRange(AcclRange range) {
    uint8_t value;
    switch (range) {
    case PM4G:
        value = 0b0101; break;
    case PM8G:
        value = 0b1000; break;
    case PM16G:
        value = 0b1100; break;
    default:
        value = 0x0011;
    }
    writeRegister(0x0F, value);

    factor = 0.00980665 / 2 * range;
}

Vec3 Accelerometer::read() {
    unsigned int data[6];
    for (int i = 0; i < 6; i++) {
        Wire.beginTransmission(Addr);
        Wire.write((2 + i)); // Select data register
        Wire.endTransmission();
        Wire.requestFrom(Addr, (uint8_t)1); // Request 1 byte of data
        // Read 6 bytes of data
        // x lsb, x msb, y lsb, y msb, z lsb, z msb
        if (Wire.available() == 1)
            data[i] = Wire.read();
    }
    // Convert the data to 12-bits
    int x = (data[1] << 4) + (data[0] >> 4);
    if (x > 2047) x -= 4096;
    int y = (data[3] << 4) + (data[2] >> 4);
    if (y > 2047) y -= 4096;
    int z = (data[5] << 4) + (data[4] >> 4);
    if (z > 2047) z -= 4096;

    return { x * factor, y * factor, z * factor };
}


// Gyroscope ===================================================================


Vec3 Gyroscope::read() {
    unsigned int data[6];
    for (int i = 0; i < 6; i++) {
        Wire.beginTransmission(Addr);
        Wire.write((2 + i)); // Select data register
        Wire.endTransmission();
        Wire.requestFrom(Addr, (uint8_t)1); // Request 1 byte of data
        // Read 6 bytes of data
        // x lsb, x msb, y lsb, y msb, z lsb, z msb
        if (Wire.available() == 1)
            data[i] = Wire.read();
    }
    // Convert the data
    int x = (data[1] << 8) + data[0];
    if (x > 32767) x -= 65536;
    int y = (data[3] << 8) + data[2];
    if (y > 32767) y -= 65536;
    int z = (data[5] << 8) + data[4];
    if (z > 32767) z -= 65536;

    return { x * factor, y * factor, z * factor };
}

void Gyroscope::setRange(GyroRange range) {
    uint8_t value;
    switch (range) {
    case PM2000DPS:
        value = 0b000; break;
    case PM1000DPS:
        value = 0b001; break;
    case PM500DPS:
        value = 0b010; break;
    case PM250DPS:
        value = 0b011; break;
    default:
        value = 0b100;
    }
    writeRegister(0x0F, value);

    factor = range / 32768.0;
}


// Magnetometer ================================================================

Magnetometer::Magnetometer() {
    radius = 10;
}

Vec3 Magnetometer::read() {
    uint8_t data[6];
    for (int i = 0; i < 6; i++) {
        Wire.beginTransmission(Addr);
        Wire.write((0x42 + i)); // Select data register
        Wire.endTransmission();
        Wire.requestFrom(Addr, (uint8_t)1); // Request 1 byte of data
        // Read 6 bytes of data
        // x lsb, x msb, y lsb, y msb, z lsb, z msb
        if (Wire.available() == 1)
            data[i] = Wire.read();
    }
    // Convert the data
    int x = (data[1] << 5) + (data[0] >> 3);
    if (x > 4095) x -= 8192;
    int y = (data[3] << 5) + (data[2] >> 3);
    if (y > 4095) y -= 8192;
    int z = (data[5] << 7) + (data[4] >> 1);
    if (z > 16383) z -= 32768;

    // xMag = ((data[1] <<5) | (data[0]>>3));
    // if (xMag > 4095)  xMag -= 8192;
    // yMag = ((data[3] <<5) | (data[2]>>3));
    // if (yMag > 4095)  yMag -= 8192;
    // zMag = ((data[5] <<7) | (data[4]>>1));
    // if (zMag > 16383)  zMag -= 32768;

    return { x / 16.0 - center.x, y / 16.0 - center.y, z / 16.0 - center.z };
}


float Magnetometer::calibrate(Vec3 mag, float gain) {
    float f = mag.x * mag.x + mag.y * mag.y + mag.z * mag.z - radius*radius;
    center.x += 4 * gain * f * mag.x;
    center.y += 4 * gain * f * mag.y;
    center.z += 4 * gain * f * mag.z;
    radius   += 4 * gain * f * radius;
    if (f > 1E30) resetCalibration();
    return f;
}
void Magnetometer::resetCalibration() {
  center.x = 0;
  center.y = 0;
  center.z = 0;
  radius = 10;
}
