
#define DEV

#include <Wire.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include "BMP280.h"
#include "Algebra.h"
#include "MadgwickAHRSQ.h"
#include "BMX055.h"
// #include "I2C_EEPROM.h"
#include "Comm.h"

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
#define Addr_EEPROM 0x50

// EEPROM Addresses
#define Addr_Calib 0x00
// #define Addr_State 0x0F
#define Addr_Log_Start 0x10
#define Addr_Log_End 0x14

// Settings
#define SERIAL_SPEED     115200
#define TWE_SERIAL_SPEED 38400

#define SAMPLE_FREQ 20
#define FLIGHT_TLM_FREQ 10
#define FLIGHT_TLM_FREQ_LOW 1
#define STANDBY_TLM_FREQ 1
#define LOG_FREQ 10
#define CALIB_WRITE_FREQ 1
#define PRINT_FREQ 2

#define LED_BLINK_MS 10
#define CALIB_GAIN 0.00001
#define P0 1013.25

// Peripherals

#ifdef DEV
SerialComm comm('R', Serial);
#else
SoftwareSerial twelite(TWELITE_RX_PIN, TWELITE_TX_PIN);
SerialComm comm('R', twelite);
#endif


BMX055 bmx055;
BMP280 bmp280;
// I2C_EEPROM eeprom(Addr_EEPROM);
Madgwick MadgwickFilter;

// Data types

enum Mode {
  SLEEP = 0,
  STANDBY,
  FLIGHT,
  CALIB,
  DEBUG,
  REPLAY,
  MODE_MAX,
};

enum State {
  PREFLIGHT = 0,
  BURNING,
  ASCENT,
  DECENT,
  LANDED,
};

struct LogData {
  float T;
  float att[4];
  Vec3 accl;
  float alt;
}; // 9 byte

struct Data: LogData {
  Vec3 gyro;
  Vec3 mag;
  float bat;
};

// Variables

Mode mode = STANDBY;
State state = PREFLIGHT;

unsigned long init_millis;
unsigned long start_millis;

unsigned count = 0;

uint32_t log_start = 0;
uint32_t log_end = 0;
uint32_t log_cursor = 0;

float time;
Quaternion attitude;
Vec3 accel;
Vec3 velocity;
Vec3 position;
float battery_voltage;

void setup() {

  pinMode(LED_GREEN_PIN, OUTPUT);
//  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(BAT_VOLTAGE_PIN, INPUT);
  // pinMode(BAT_STATE_PIN,   INPUT);

#ifdef DEV
  Serial.begin(SERIAL_SPEED);
#else
  twelite.begin(TWE_SERIAL_SPEED);
#endif

  bmp280.begin();
  bmp280.setOversampling(1);

  bmx055.begin(Addr_Accl, Addr_Gyro, Addr_Mag);
  bmx055.Accl.setRange(PM16G);
  bmx055.Gyro.setRange(PM2000DPS);


  /* /\* Default settings from datasheet. *\/ */
  /* bmp280.setSampling */
  /*   (Adafruit_BMP280::MODE_NORMAL,   /\* Operating Mode. *\/ */
  /*    Adafruit_BMP280::SAMPLING_X2,   /\* Temp. oversampling *\/ */
  /*    Adafruit_BMP280::SAMPLING_X16,  /\* Pressure oversampling *\/ */
  /*    Adafruit_BMP280::FILTER_X16,    /\* Filtering. *\/ */
  /*    Adafruit_BMP280::STANDBY_MS_1); /\* Standby time. *\/ */

  MadgwickFilter.begin(SAMPLE_FREQ);

  read_calibration_data();

  /* EEPROM.get(Addr_Log_Start, log_start); */
  /* EEPROM.get(Addr_Log_End, log_end); */

  init_millis = millis();

  digitalWrite(LED_GREEN_PIN, LOW);

  comm.listen('M', receive_mode_transition);
}


void loop() {

  comm.update();

  // Receive command

  if (mode == SLEEP) {
    digitalWrite(LED_GREEN_PIN, HIGH);
    delay(100);
    digitalWrite(LED_GREEN_PIN, LOW);
    delay(900);
    return;
  }

  Data data;

  /* if (mode == REPLAY) { */
  /*   read_log(data); */
  /*   digitalWrite(LED_GREEN_PIN, HIGH); */
  /*   digitalWrite(LED_GREEN_PIN, LOW); */
  /*   if (log_cursor == log_start) mode_transition(SLEEP); */
  /*   delay(50); */
  /*   return; */
  /* } */

  start_millis = millis();

  float p_time = time;
  time = (start_millis - init_millis) / 1000.0;
  float dt = time - p_time;

  // Read sensors
  accel = bmx055.Accl.read();
  Vec3 gyro = bmx055.Gyro.read();
  Vec3 mag = bmx055.Mag.read();

  float altitude = 0;
  double T, P;
  char result = bmp280.startMeasurment();
  if(result!=0) {
    delay(result);
    result = bmp280.getTemperatureAndPressure(T,P);
    if(result!=0)
        altitude = bmp280.altitude(P,P0);
  }

  char buf[64];
  sprintf(buf, "%c %f %f %f", result, T, P,  altitude);
  Serial.println(buf);

  battery_voltage = analogRead(BAT_VOLTAGE_PIN) * 6.6 / 1023.0;

  MadgwickFilter.update(gyro.x, gyro.y, gyro.z,
                        accel.x, accel.y, accel.z,
                        mag.x, mag.y, mag.z);

  attitude = MadgwickFilter.getQuaternion();

  Vec3 accel_i = attitude.inverse().rotate(accel);
  if (mode == FLIGHT) {
    velocity.x += accel_i.x * dt;
    velocity.z += accel_i.z * dt;
    velocity.y = (altitude - position.y) / dt;

    position.x += velocity.x * dt;
    position.z += velocity.z * dt;
    position.y = altitude;
  }

  // Calibration
  if (mode == CALIB) {
    float epsilon = bmx055.Mag.calibrate(mag, CALIB_GAIN);
    if (count % (SAMPLE_FREQ / CALIB_WRITE_FREQ) == 0) write_calibration_data();
    print_calibration(mag, epsilon);
  }

  // Send comm
  if (mode == FLIGHT) {
    if (count % (SAMPLE_FREQ / FLIGHT_TLM_FREQ) == 0) {
      send_tlm_time();
      send_tlm_attitude();
    }
    if (count % (SAMPLE_FREQ / FLIGHT_TLM_FREQ_LOW) == 0) {
      send_tlm_mode();
      send_tlm_state();
    }
  }
  else if (mode == STANDBY) {
    if (count % (SAMPLE_FREQ / STANDBY_TLM_FREQ) == 0) {
      send_tlm_time();
      send_tlm_attitude();
      send_tlm_mode();
      send_tlm_state();
    }
  }

  // Logging
  if (mode == FLIGHT && count % (SAMPLE_FREQ / LOG_FREQ)) {
    /* write_log(data); */
    /* if (log_end == log_start) mode = SLEEP; */
  }

  digitalWrite(LED_GREEN_PIN, HIGH);

  int wait = 1000 / SAMPLE_FREQ - (millis() - start_millis);
  if (wait > LED_BLINK_MS) {
    delay(LED_BLINK_MS);
    digitalWrite(LED_GREEN_PIN, LOW);
    wait -= LED_BLINK_MS;
  }
  if (wait > 0) delay(wait);
//  else Serial.println("FRAME DROPPED");

  digitalWrite(LED_GREEN_PIN, LOW);

  count++;
  if (count >= SAMPLE_FREQ) count = 0;
}


// Telemetry and Command -------------------------------------------------------

void send_tlm_time() {
  comm.send('T', 0, (float)time);
}

void send_tlm_state() {
  comm.send('S', 0, (unsigned)state);
  comm.send('B', 0, (float)battery_voltage);
}

void send_tlm_mode() {
  comm.send('M', 0, (unsigned)mode);
}

void send_tlm_attitude() {
  comm.send('Q', 0, (float)attitude.a);
  comm.send('Q', 1, (float)attitude.b);
  comm.send('Q', 2, (float)attitude.c);
  comm.send('Q', 3, (float)attitude.d);

  comm.send('A', 0, (float)accel.x);
  comm.send('A', 1, (float)accel.y);
  comm.send('A', 2, (float)accel.z);

  comm.send('V', 0, (float)velocity.x);
  comm.send('V', 1, (float)velocity.y);
  comm.send('V', 2, (float)velocity.z);

  comm.send('P', 0, (float)position.x);
  comm.send('P', 1, (float)position.y);
  comm.send('P', 2, (float)position.z);
}


void receive_mode_transition(const Message<unsigned>& message) {
  if (message.payload < MODE_MAX)
    mode_transition(static_cast<Mode>(message.payload));
}

void mode_transition(Mode to) {
  switch (to) {
  case SLEEP:
    MadgwickFilter.reset();
    break;

  case FLIGHT:
    log_start = log_end;
    velocity = Vec3::zero;
    position = Vec3::zero;

    EEPROM.put(Addr_Log_Start, log_start);
    break;

  case CALIB:
    bmx055.Mag.resetCalibration();
    MadgwickFilter.reset();
    break;

  default:
    break;
  }
  mode = to;

  send_tlm_mode();
}

/* void read_command_from_serial() { */
/*   while (Serial.available() > 0) { */
/*     char c = Serial.read(); */
/*     switch (c) { */
/*     case 'D': */
/*       mode_transition(DEBUG); */
/*       break; */
/*     case 'R': */
/*       mode_transition(SLEEP); */
/*       log_start = 0; */
/*       log_end = 0; */
/*       EEPROM.put(Addr_Log_Start, log_start); */
/*       EEPROM.put(Addr_Log_End, log_end); */
/*       Serial.println("Resetting filter"); */
/*       break; */
/*     case 'S': */
/*       mode_transition(STANDBY); */
/*       break; */
/*     case 'F': */
/*       mode_transition(FLIGHT); */
/*       break; */
/*     case 'C': */
/*       mode_transition(CALIB); */
/*       break; */
/*     /\* case 'P': *\/ */
/*     /\*   mode_transition(REPLAY); *\/ */
/*     /\*   Serial.println(log_start); *\/ */
/*     /\*   Serial.println(log_end); *\/ */
/*     /\*   Serial.println("<Replay>"); *\/ */
/*     /\*   break; *\/ */
/*     } */
/*   } */
/* } */



// Read and Write EEPROM -------------------------------------------------------

void read_calibration_data() {
  EEPROM.get(Addr_Calib, bmx055.Mag.center);
#ifdef DEV
  Serial.print("Mag center: ");
  bmx055.Mag.center.print();
  Serial.println("");
#endif
}

void write_calibration_data() {
  EEPROM.put(Addr_Calib, bmx055.Mag.center);
}

// Logging ---------------------------------------------------------------------

/* void read_log(const LogData &d) { */
/*   if (log_cursor == log_start) eeprom.read_begin(log_start); */
/*   else eeprom.read_begin(); */
/*   eeprom.get(d); */
/*   eeprom.read_stop(); */
/*   log_cursor += sizeof(d); */
/*   if (log_cursor >= 0x10000) log_cursor = 0; */
/* } */

/* void write_log(const LogData &d) { */
/*   eeprom.write_begin(log_end); */
/*   eeprom.put(d); */
/*   eeprom.write_stop(); */
/*   log_end += sizeof(d); */
/*   if (log_end >= 0x20000) log_end = 0; */
/*   EEPROM.put(Addr_Log_End, log_end); */
/* } */

void print_calibration(const Vec3 &mag, const float epsilon) {
  mag.print('M');
  bmx055.Mag.center.print('C');
  char buf[16];
  sprintf(buf, "r:%f,e:%f", bmx055.Mag.radius, epsilon);
  Serial.println(buf);
}
