#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <ESP32BLECombo.h>

MPU6050 headMotionSensor;
ESP32BLECombo ble;

int16_t rawAccelX, rawAccelY, rawAccelZ, rawGyroX, rawGyroY, rawGyroZ;
int cursorXMovement, cursorYMovement;

float gyroOffsetX = 0;
float gyroOffsetY = 0;

int horizontalSensitivityLevel = 250;
int verticalSensitivityLevel = 150;
int horizontalWobbleThreshold = 5;
int verticalWobbleThreshold = 5;

unsigned long lastHeartbeat = 0;
const unsigned long heartbeatInterval = 4000;

// --- Kalman filter (smooths MPU6050 noise) ---
// q = how fast the real head motion can change (higher = reacts faster)
// r = how noisy the sensor is (higher = smoother, but slightly slower)
struct KalmanFilter {
  float q;   // process noise
  float r;   // measurement noise
  float x;   // current estimate
  float p;   // estimate uncertainty

  KalmanFilter(float q_, float r_) : q(q_), r(r_), x(0), p(1) {}

  float update(float measurement) {
    p += q;                              // predict: uncertainty grows a little
    float k = p / (p + r);               // how much to trust the new reading
    x += k * (measurement - x);          // correct the estimate
    p *= (1 - k);                        // uncertainty shrinks after the update
    return x;
  }
};

KalmanFilter kalmanX(4.0, 16.0);
KalmanFilter kalmanY(4.0, 16.0);

// --- Blink sensor setup ---
int blinkPin = D1;
unsigned long blinkStart = 0;
bool isEyeClosed = false;

const int singleBlinkDuration = 300;
const int rightClickBlinkDuration = 700;
const int doubleClickBlinkDuration = 1300;
const int maxBlinkDuration = 1700;   // longer than this is ignored

void calibrateGyro() {
  const int samples = 500;
  long sumX = 0, sumY = 0;

  for (int i = 0; i < samples; i++) {
    headMotionSensor.getMotion6(&rawAccelX, &rawAccelY, &rawAccelZ,
                                 &rawGyroX, &rawGyroY, &rawGyroZ);
    sumX += rawGyroX;
    sumY += rawGyroY;
    delay(3);
  }

  gyroOffsetX = -((float)sumX / samples);
  gyroOffsetY = -((float)sumY / samples);
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  Wire.begin(D4, D5);
  delay(100);

  headMotionSensor.initialize();
  delay(100);

  calibrateGyro();

  pinMode(blinkPin, INPUT);

  ESP32BLEComboConfig cfg;
  cfg.mode = ESP32BLEComboMode::KEYBOARD_MOUSE;
  cfg.deviceName = "Ayush's Mouse";
  cfg.manufacturer = "Espressif";
  cfg.batteryLevel = 70;
  cfg.appearance = ESP32BLEComboAppearance::AUTO;
  ble.begin(cfg);

  Serial.println("Setup complete.");
}

void loop() {
  // --- Head motion -> cursor movement ---
  headMotionSensor.getMotion6(&rawAccelX, &rawAccelY, &rawAccelZ,
                               &rawGyroX, &rawGyroY, &rawGyroZ);

  // Same corrected values as before, now passed through the Kalman filter
  float correctedGyroX = kalmanX.update(rawGyroX + gyroOffsetX);
  float correctedGyroY = kalmanY.update(rawGyroY + gyroOffsetY);

  if (abs(correctedGyroY) > horizontalWobbleThreshold) {
    cursorXMovement = -correctedGyroY / horizontalSensitivityLevel;
  } else {
    cursorXMovement = 0;
  }

  if (abs(correctedGyroX) > verticalWobbleThreshold) {
    cursorYMovement = correctedGyroX / verticalSensitivityLevel;
  } else {
    cursorYMovement = 0;
  }

  if (ble.isConnected()) {
    if (cursorXMovement != 0 || cursorYMovement != 0) {
      ble.mouseMove(cursorXMovement, cursorYMovement);
      lastHeartbeat = millis();
    } else if (millis() - lastHeartbeat > heartbeatInterval) {
      ble.mouseMove(0, 0); // keeps the Bluetooth link alive
      lastHeartbeat = millis();
    }
  }

  // --- Blink detection ---
  int blinkState = digitalRead(blinkPin);

  if (blinkState == HIGH) {
    if (!isEyeClosed) {
      blinkStart = millis();
      isEyeClosed = true;
    }
  } else {
    if (isEyeClosed) {
      unsigned long blinkDuration = millis() - blinkStart;

      if (blinkDuration > maxBlinkDuration) {
        // too long, ignore
      } else if (ble.isConnected()) {
        if (blinkDuration > doubleClickBlinkDuration) {
          ble.mouseClick(ESP32BLECombo::MOUSE_LEFT);
          delay(50);
          ble.mouseClick(ESP32BLECombo::MOUSE_LEFT);
          Serial.println("Double click");
        } else if (blinkDuration > rightClickBlinkDuration) {
          ble.mouseClick(ESP32BLECombo::MOUSE_RIGHT);
          Serial.println("Right click");
        } else if (blinkDuration > singleBlinkDuration) {
          ble.mouseClick(ESP32BLECombo::MOUSE_LEFT);
          Serial.println("Single click");
        }
      }

      isEyeClosed = false;
    }
  }

  delay(10);
}
