// Head mouse v2 - same behaviour as the original, constants written in deg/s
// XIAO ESP32C3 + MPU6050 (SDA D4, SCL D5) + TCRT5000 (D1)

#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <ESP32BLECombo.h>

MPU6050 headMotionSensor;
ESP32BLECombo ble;

// MPU6050 at +/-250 deg/s -> 131 counts per deg/s
const float GYRO_LSB_PER_DPS = 131.0;

int16_t rawAccelX, rawAccelY, rawAccelZ, rawGyroX, rawGyroY, rawGyroZ;
int cursorXMovement, cursorYMovement;

float gyroOffsetX_dps = 0;
float gyroOffsetY_dps = 0;

// deg/s needed for one cursor step (250 and 150 counts in the old code)
const float HORIZ_DPS_PER_STEP = 250.0 / GYRO_LSB_PER_DPS;
const float VERT_DPS_PER_STEP  = 150.0 / GYRO_LSB_PER_DPS;

// old 5-count wobble threshold
const float HORIZ_WOBBLE_DPS = 5.0 / GYRO_LSB_PER_DPS;
const float VERT_WOBBLE_DPS  = 5.0 / GYRO_LSB_PER_DPS;

unsigned long lastHeartbeat = 0;
const unsigned long heartbeatInterval = 4000;

// simple 1D Kalman filter
struct KalmanFilter {
  float q;  // process noise
  float r;  // measurement noise
  float x;  // estimate
  float p;  // estimate variance

  KalmanFilter(float q_, float r_, float p0) : q(q_), r(r_), x(0), p(p0) {}

  float update(float measurement) {
    p += q;
    float k = p / (p + r);
    x += k * (measurement - x);
    p *= (1 - k);
    return x;
  }
};

// Q = 4, R = 16, P0 = 1 (in counts^2), converted to (deg/s)^2
const float LSB2_TO_DPS2 = 1.0 / (GYRO_LSB_PER_DPS * GYRO_LSB_PER_DPS);
KalmanFilter kalmanX(4.0 * LSB2_TO_DPS2, 16.0 * LSB2_TO_DPS2, 1.0 * LSB2_TO_DPS2);
KalmanFilter kalmanY(4.0 * LSB2_TO_DPS2, 16.0 * LSB2_TO_DPS2, 1.0 * LSB2_TO_DPS2);

// blink sensor
int blinkPin = D1;
unsigned long blinkStart = 0;
bool isEyeClosed = false;

const int singleBlinkDuration      = 300;
const int rightClickBlinkDuration  = 700;
const int doubleClickBlinkDuration = 1300;
const int maxBlinkDuration         = 1700;  // longer = resting eyes, ignore

// keep still for ~1.5 s at power-on
void calibrateGyro() {
  const int samples = 500;
  double sumX = 0, sumY = 0, sumSqX = 0, sumSqY = 0;

  for (int i = 0; i < samples; i++) {
    headMotionSensor.getMotion6(&rawAccelX, &rawAccelY, &rawAccelZ,
                                &rawGyroX, &rawGyroY, &rawGyroZ);
    float gx = rawGyroX / GYRO_LSB_PER_DPS;
    float gy = rawGyroY / GYRO_LSB_PER_DPS;
    sumX += gx;  sumSqX += (double)gx * gx;
    sumY += gy;  sumSqY += (double)gy * gy;
    delay(3);
  }

  double meanX = sumX / samples, meanY = sumY / samples;
  double sdX = sqrt(max(0.0, sumSqX / samples - meanX * meanX));
  double sdY = sqrt(max(0.0, sumSqY / samples - meanY * meanY));

  gyroOffsetX_dps = -meanX;
  gyroOffsetY_dps = -meanY;

  Serial.printf("Offset X = %.3f deg/s, Y = %.3f deg/s\n", meanX, meanY);
  Serial.printf("Resting noise X = %.4f deg/s (%.1f LSB), Y = %.4f deg/s (%.1f LSB)\n",
                sdX, sdX * GYRO_LSB_PER_DPS, sdY, sdY * GYRO_LSB_PER_DPS);
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
  // head motion -> cursor
  headMotionSensor.getMotion6(&rawAccelX, &rawAccelY, &rawAccelZ,
                              &rawGyroX, &rawGyroY, &rawGyroZ);

  float omegaX = kalmanX.update(rawGyroX / GYRO_LSB_PER_DPS + gyroOffsetX_dps);  // pitch
  float omegaY = kalmanY.update(rawGyroY / GYRO_LSB_PER_DPS + gyroOffsetY_dps);  // yaw

  if (fabs(omegaY) > HORIZ_WOBBLE_DPS) {
    cursorXMovement = -omegaY / HORIZ_DPS_PER_STEP;
  } else {
    cursorXMovement = 0;
  }

  if (fabs(omegaX) > VERT_WOBBLE_DPS) {
    cursorYMovement = omegaX / VERT_DPS_PER_STEP;
  } else {
    cursorYMovement = 0;
  }

  if (ble.isConnected()) {
    if (cursorXMovement != 0 || cursorYMovement != 0) {
      ble.mouseMove(cursorXMovement, cursorYMovement);
      lastHeartbeat = millis();
    } else if (millis() - lastHeartbeat > heartbeatInterval) {
      ble.mouseMove(0, 0);  // keep BLE link alive
      lastHeartbeat = millis();
    }
  }

  // blink detection
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
