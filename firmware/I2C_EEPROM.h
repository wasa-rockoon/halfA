#include <Arduino.h>
#include <Wire.h>

#ifndef I2C_EEPROM_h
#define I2C_EEPROM_h

class I2C_EEPROM {
  public:

  I2C_EEPROM(uint8_t addr) { Addr = addr; };

  void write(uint32_t addr, byte data);
  void write(uint32_t addr, const byte *data, uint8_t length);
  void write_begin(uint32_t addr);
  template< typename T > void put(const T &t) {
    const uint8_t *ptr = (const uint8_t*) &t;
    for (int count = sizeof(T); count; --count, ptr++) {
      Wire.write(*ptr);
    }
  }
  void write_stop();

  byte read();
  byte read(uint32_t addr);
  void read(byte *data, unsigned length);
  void read(uint32_t addr, byte *data, unsigned length);
  void read_begin();
  void read_begin(uint32_t addr);
  template< typename T > void get(T &t) {
    uint8_t *ptr = (uint8_t*) &t;
    for (unsigned count = sizeof(T); count; --count, ptr++) *ptr = Wire.read();
  }
  void read_stop();

  private:
    uint8_t Addr;
    uint8_t current;
    void split_address(uint32_t addr, uint8_t &device, uint8_t &upper, uint8_t &lower);
};


#endif
