// =====================================================================
//  Measurement sketch for the BTP report (does NOT act as a mouse)
//  Flash this temporarily, record data over USB, then flash the mouse code back.
//
//  Streams one CSV line per loop at about 100 Hz:
//      t_us,gx,gy,gz,eye
//  t_us = time in microseconds, gx/gy/gz = raw gyro counts (131 LSB per deg/s),
//  eye  = TCRT5000 digital output (1 = eye closed).
//
//  Stuck-sensor check: if the gyro returns identical values for 100 readings
//  in a row, the MPU6050 is re-initialised and a "#" warning line is printed
//  (the analysis script skips it). Note: each restart pauses logging ~0.2 s,
//  so for blink-only tests keep the MPU6050 connected to avoid repeated restarts.
// =====================================================================

#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>

MPU6050 imu;
const int blinkPin = D1;
int16_t ax, ay, az, gx, gy, gz;
int16_t lastGx = 0, lastGy = 0, lastGz = 0;
int sameCount = 0;

void startIMU() {
  Wire.begin(D4, D5);
  Wire.setClock(100000);       // standard I2C speed, more tolerant of long wires
  delay(100);
  imu.initialize();            // same defaults as the mouse code (+/-250 deg/s)
  delay(100);
}

void setup() {
  Serial.begin(921600);
  delay(2000);
  startIMU();
  pinMode(blinkPin, INPUT);
  if (!imu.testConnection()) Serial.println("# WARNING: MPU6050 not responding");
  Serial.println("t_us,gx,gy,gz,eye");
}

void loop() {
  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  if (gx == lastGx && gy == lastGy && gz == lastGz) sameCount++; else sameCount = 0;
  lastGx = gx; lastGy = gy; lastGz = gz;

  if (sameCount >= 100) {
    Serial.println("# WARNING: gyro frozen, restarting MPU6050");
    startIMU();
    sameCount = 0;
    return;
  }

  int eye = digitalRead(blinkPin);
  Serial.printf("%lu,%d,%d,%d,%d\n", micros(), gx, gy, gz, eye);
  delay(10);
}
