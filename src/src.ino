#include <Wire.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <Adafruit_BMP280.h>
#include "MadgwickAHRSQ.h"
#include "BMX055.h"

// Pin Assign
#define BAT_VOLTAGE_PIN A1
#define BAT_STATE_PIN   5
#define LED_GREEN_PIN   8
#define LED_RED_PIN     9
#define TWELITE_RX_PIN  2
#define TWELITE_TX_PIN  3

// I2C Addresses 
#define Addr_Accl 0x19
#define Addr_Gyro 0x69
#define Addr_Mag 0x13
#define Addr_BMP280 0x76
#define Addr_EEPROM_0 0x50
#define Addr_EEPROM_1 0x51


// Settings
#define SERIAL_SPEED     115200
#define TWE_SERIAL_SPEED 38400
#define SAMPLE_FREQ 10
//#define SUB_SAMPLE_DIVIDE 10
#define LED_BLINK 10
#define CALIB_GAIN 0.00001


enum MODE {
  NORMAL,
  CALIB,
  DEBUG,
  READ,
};

// Peripherals

SoftwareSerial twelite(TWELITE_RX_PIN, TWELITE_TX_PIN);
BMX055 bmx055;
Adafruit_BMP280 bmp280;
Madgwick MadgwickFilter;

// Variables

MODE mode = NORMAL;

unsigned long init_millis;
unsigned long start_millis;

int calibrate_timer = 0;
int sub_sample_counter = 0;



void setup() {
  
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(BAT_VOLTAGE_PIN, INPUT);
  pinMode(BAT_STATE_PIN,   INPUT);

  Serial.begin(SERIAL_SPEED);
  Serial.println("Boot");

  Wire.begin();

  twelite.begin(TWE_SERIAL_SPEED);

  bmx055.begin(Addr_Accl, Addr_Gyro, Addr_Mag);
  bmx055.Accl.setRange(PM16G);
  delay(100);
  bmx055.Gyro.setRange(PM2000DPS);
  delay(100);

  bmp280.begin(Addr_BMP280);
  /* Default settings from datasheet. */
  bmp280.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                     Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                     Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                     Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                     Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */

  MadgwickFilter.begin(SAMPLE_FREQ); 

  bmx055.Mag.center.x = EEPROM.read(0) - 128 + ((int)EEPROM.read(1) - 128) / 128.0;
  bmx055.Mag.center.y = EEPROM.read(2) - 128 + ((int)EEPROM.read(3) - 128)/ 128.0;
  bmx055.Mag.center.z = EEPROM.read(4) - 128 + ((int)EEPROM.read(5) - 128) / 128.0;
  Serial.print("Use calibrated mag center ");
  Serial.print(bmx055.Mag.center.x);
  Serial.print(" ");
  Serial.print(bmx055.Mag.center.y);
  Serial.print(" ");
  Serial.print(bmx055.Mag.center.z);
  Serial.println(" ");

  init_millis = millis();

  digitalWrite(LED_GREEN_PIN, LOW);
}

#define P0 1013.25

void loop() {
  // put your main code here, to run repeatedly:

  //  while (twelite.available()) {
//    Serial.print((char)twelite.read());
//  }
//  Serial.println("");

  if (Serial.available() > 0) {
    if (mode != DEBUG) Serial.println("Entering debug mode");
    mode = DEBUG;
    char command;
    command = Serial.read();
    switch (command) {
      case 'R':
        MadgwickFilter.reset();
        Serial.println("Resetting filter");
      case 'N':
        mode = NORMAL;
        Serial.println("Exit debug mode");
        break;
      case 'C':
        mode = CALIB;
        bmx055.Mag.center.x = 0;
        bmx055.Mag.center.y = 0;
        bmx055.Mag.center.z = 0;
        MadgwickFilter.reset();
        Serial.println("Starting calibration...");
        break;
    }
  }
  
  start_millis = millis();
  unsigned long T = start_millis - init_millis;

  // Read sensors

  Vec3 accl = bmx055.Accl.read();
  Vec3 gyro = bmx055.Gyro.read();
  Vec3 mag  = bmx055.Mag.read();

  float tmp = 0;
  float prs = 0;

  if (mode == DEBUG) {
    tmp = bmp280.readTemperature();
    prs = bmp280.readPressure();
  }
  
  float alt = bmp280.readAltitude(1013.25);

  float bat_voltage = analogRead(BAT_VOLTAGE_PIN) * 3.3 / 1023;

  MadgwickFilter.update(gyro.x, gyro.y, gyro.z, accl.x, accl.y, accl.z, mag.x, mag.y, mag.z);


  // Calibration 
  
  if (mode == CALIB) {
    float epsilon = bmx055.Mag.calibrate(mag, CALIB_GAIN);

    calibrate_timer++;

    if (calibrate_timer > SAMPLE_FREQ) {
      int x_int = round(bmx055.Mag.center.x);
      int y_int = round(bmx055.Mag.center.y);
      int z_int = round(bmx055.Mag.center.z);
      
      EEPROM.write(0, x_int + 128);
      EEPROM.write(1, (bmx055.Mag.center.x - x_int) * 128 + 128);
      EEPROM.write(2, y_int + 128); 
      EEPROM.write(3, (bmx055.Mag.center.y - y_int) * 128 + 128);
      EEPROM.write(4, z_int + 128);
      EEPROM.write(5, (bmx055.Mag.center.z - z_int) * 128 + 128);

      calibrate_timer = 0;

//      Serial.println("Wrote mag center");
    }

    Serial.print(",MagX:");
    Serial.print(mag.x);
    Serial.print(",MagY:");
    Serial.print(mag.y);
    Serial.print(",MagZ:");
    Serial.print(mag.z);
    
    Serial.print(",CenterX:");
    Serial.print(bmx055.Mag.center.x);
    Serial.print(",CenterY:");
    Serial.print(bmx055.Mag.center.y);
    Serial.print(",CenterZ:");
    
    Serial.print(bmx055.Mag.center.z);
    Serial.print(",Radius:");
    Serial.print(bmx055.Mag.radius);
    Serial.print(",Epsilon:");
    Serial.print(epsilon / 100.0);

    Serial.println("");
  }


  // Send telemetry

  twelite.print("T");
  twelite.println(T);

  twelite.print("X");
  twelite.println(accl.x, 6);
  twelite.print("Y");
  twelite.println(accl.y, 6);
  twelite.print("Z");
  twelite.println(accl.z, 6);

  float q[4];
  MadgwickFilter.getQuaternion(q);
  twelite.print("p");
  twelite.println(q[0], 7);
  twelite.print("q");
  twelite.println(q[1], 7);
  twelite.print("r");
  twelite.println(q[2], 7);
  twelite.print("s");
  twelite.println(q[3], 7);

  twelite.print("A");
  twelite.println(alt, 6);

  twelite.print("B");
  twelite.println(bat_voltage);


  // Logging
  
  if (mode == DEBUG) {

    Serial.print(",Time:");
    Serial.print(T / 100000);

    Serial.print(",Q0:");
    Serial.print(q[0]);
    Serial.print(",Q1:");
    Serial.print(q[1]);
    Serial.print(",Q2:");
    Serial.print(q[2]);
    Serial.print(",Q3:");
    Serial.print(q[2]);
  
    Serial.print(",Roll:");
    Serial.print(MadgwickFilter.getRollRadians());
    Serial.print(",Pitch:");
    Serial.print(MadgwickFilter.getPitchRadians());
    Serial.print(",Yaw:");
    Serial.print(MadgwickFilter.getYawRadians());
  
    Serial.print(",AcclX:");
    Serial.print(accl.x);
    Serial.print(",AcclY:");
    Serial.print(accl.y);
    Serial.print(",AcclZ:");
    Serial.print(accl.z);
  
    Serial.print(",GyroX:");
    Serial.print(gyro.x / 180.0 * 3.14);
    Serial.print(",GyroY:");
    Serial.print(gyro.y / 180.0 * 3.14);
    Serial.print(",GyroZ:");
    Serial.print(gyro.z / 180.0 * 3.14);

    Serial.print(",MagX:");
    Serial.print(mag.x);
    Serial.print(",MagY:");
    Serial.print(mag.y);
    Serial.print(",MagZ:");
    Serial.print(mag.z);

    Serial.print(",Battery:");
    Serial.print(bat_voltage);
    Serial.print(",Charging:");
    Serial.print(digitalRead(BAT_STATE_PIN));

    Serial.print(",Altitude:");
    Serial.print(alt / 10);
    Serial.print(",Temperature:");
    Serial.print(tmp / 10);
    Serial.print(",Pressure:");
    Serial.print(prs / 10000);
  }

  digitalWrite(LED_GREEN_PIN, HIGH);

  int wait = 1000 / SAMPLE_FREQ - (millis() - start_millis);
  if (wait > LED_BLINK) {
    delay(LED_BLINK);
    digitalWrite(LED_GREEN_PIN, LOW);
    wait -= LED_BLINK;
  }
  if (wait > 0) delay(wait);
//  else Serial.println("FRAME DROPPED");

  if (mode == DEBUG) {
    Serial.print(",Wait:");
    Serial.print(wait);
    Serial.println("");
  }


  digitalWrite(LED_GREEN_PIN, LOW);
}
