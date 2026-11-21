// Logger for tests (not a mouse)
// prints t_us,gx,gy,gz,eye at ~100 Hz over USB

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
  Wire.setClock(100000);
  delay(100);
  imu.initialize();
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

  // MPU sometimes gets stuck (loose wire) - restart it if values stop changing
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
