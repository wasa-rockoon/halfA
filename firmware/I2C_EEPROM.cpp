#include "I2C_EEPROM.h"

void I2C_EEPROM::write(uint32_t addr, byte data) {
  write_begin(addr);
  put(data);
  write_stop();
}
void I2C_EEPROM::write(uint32_t addr, const byte *data, uint8_t length) {
  write_begin(addr);
  for (int count = 0; count < length; count++) put(data);
  write_stop();
}
void I2C_EEPROM::write_begin(uint32_t addr) {
  uint8_t device, upper, lower;
  split_address(addr, device, upper, lower);
  Wire.beginTransmission(device);
  Wire.write(upper);
  Wire.write(lower);
}
void I2C_EEPROM::write_stop() {
  Wire.endTransmission();
}

byte I2C_EEPROM::read() {
  byte data;
  read_begin();
  get(data);
  read_stop();
  return data;
}
byte I2C_EEPROM::read(uint32_t addr) {
  byte data;
  read_begin(addr);
  get(data);
  read_stop();
  return data;
}
void I2C_EEPROM::read(byte *data, unsigned length) {
  read_begin();
  for (unsigned i = 0; i < length; i++) get(data[i]);
  read_stop();
}
void I2C_EEPROM::read(uint32_t addr, byte *data, unsigned length) {
  read_begin(addr);
  for (unsigned i = 0; i < length; i++) get(data[i]);
  read_stop();
}
void I2C_EEPROM::read_begin() {
  Wire.beginTransmission(current);
}
void I2C_EEPROM::read_begin(uint32_t addr) {
  uint8_t device, upper, lower;
  split_address(addr, device, upper, lower);
  current = device;
  Wire.beginTransmission(device);
}
void I2C_EEPROM::read_stop() {
  Wire.endTransmission();
}


void I2C_EEPROM::split_address(uint32_t addr, uint8_t &device, uint8_t &upper, uint8_t &lower) {
  device = addr >= 0x10000 ? (uint32_t)Addr + 1 : Addr;
  upper = addr > 8;
  lower = addr;
}
