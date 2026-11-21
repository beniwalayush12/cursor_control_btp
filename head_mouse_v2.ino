// =====================================================================
//  Head-motion BLE mouse  --  Version 2, "physical units" edition
//  Ayush Beniwal (23B1810), PH 587 B.Tech Project I, IIT Bombay
//
//  Hardware: Seeed XIAO ESP32C3 + MPU6050 (I2C: SDA=D4, SCL=D5)
//            + TCRT5000 IR reflective sensor (digital OUT -> D1)
//            + 3.7 V LiPo
//
//  This sketch behaves the same as the original v2 code. The only change
//  is that every quantity is written in physical units (deg/s, seconds,
//  Kelvin-free!) instead of raw sensor counts (LSB), so each constant
//  can be traced to a physical meaning in the report:
//
//     raw count (LSB)  =  131 LSB per (deg/s)  x  angular velocity (deg/s)
//
//  Old constant                 ->  New constant (same value, new units)
//  wobble threshold 5 LSB       ->  0.0382 deg/s
//  sensitivity 250 LSB / step   ->  1.908 deg/s per cursor step (horizontal)
//  sensitivity 150 LSB / step   ->  1.145 deg/s per cursor step (vertical)
//  Kalman Q = 4,  R = 16 LSB^2  ->  Q = 4/131^2,  R = 16/131^2 (deg/s)^2
//
//  Because the Kalman gain depends only on the ratio Q/R, and every
//  threshold is scaled by the same factor 131, the cursor output is the
//  same as before (apart from float rounding in the last decimal place).
//
//  Extra (does not change behaviour): calibration now also measures the
//  sensor's resting noise (standard deviation) and prints it over Serial.
//  That number is the physically correct value for R and goes in the report.
// =====================================================================

#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <ESP32BLECombo.h>

MPU6050 headMotionSensor;
ESP32BLECombo ble;

// ---------- Sensor physics ----------
// MPU6050 at its default full scale of +/-250 deg/s:
// a 16-bit signed number spans +/-32768 counts, so 32768 / 250 = 131.07.
const float GYRO_LSB_PER_DPS = 131.0;

int16_t rawAccelX, rawAccelY, rawAccelZ, rawGyroX, rawGyroY, rawGyroZ;
int cursorXMovement, cursorYMovement;

// Resting offset (zero-rate output) in deg/s, measured at every power-on.
// The datasheet allows up to +/-20 deg/s of offset, so this cannot be a fixed number.
float gyroOffsetX_dps = 0;
float gyroOffsetY_dps = 0;

// ---------- Cursor mapping (physical units) ----------
// Angular speed needed for ONE cursor step per loop. Because the result is
// truncated to an integer, this is also the dead zone: slower head motion
// gives zero movement.
const float HORIZ_DPS_PER_STEP = 250.0 / GYRO_LSB_PER_DPS;   // = 1.908 deg/s
const float VERT_DPS_PER_STEP  = 150.0 / GYRO_LSB_PER_DPS;   // = 1.145 deg/s

// Small threshold kept from v1 (5 LSB). It is much smaller than the
// dead zone above, so in practice the dead zone does the real filtering.
const float HORIZ_WOBBLE_DPS = 5.0 / GYRO_LSB_PER_DPS;       // = 0.038 deg/s
const float VERT_WOBBLE_DPS  = 5.0 / GYRO_LSB_PER_DPS;

unsigned long lastHeartbeat = 0;
const unsigned long heartbeatInterval = 4000;   // ms

// ---------- Kalman filter (random-walk model) ----------
// State model:        omega[n+1] = omega[n] + w,   w ~ noise of variance Q
// Measurement model:  z[n]       = omega[n] + v,   v ~ noise of variance R
// Steady state: K = P/(P+R) with P^2 - Q P - Q R = 0, which gives K = 0.39
// for Q/R = 1/4. That is a first-order low-pass filter with a -3 dB
// cutoff near 8 Hz and a time constant near 20 ms at a 10 ms loop period.
struct KalmanFilter {
  float q;   // process noise variance       (deg/s)^2
  float r;   // measurement noise variance   (deg/s)^2
  float x;   // current estimate of angular velocity (deg/s)
  float p;   // variance of that estimate    (deg/s)^2

  KalmanFilter(float q_, float r_, float p0) : q(q_), r(r_), x(0), p(p0) {}

  float update(float measurement) {
    p += q;                         // predict: uncertainty grows
    float k = p / (p + r);          // gain: how much to trust the new reading
    x += k * (measurement - x);     // correct the estimate
    p *= (1 - k);                   // uncertainty shrinks after the update
    return x;
  }
};

// Same numbers as before (Q = 4, R = 16, P0 = 1 in LSB^2), converted to (deg/s)^2.
const float LSB2_TO_DPS2 = 1.0 / (GYRO_LSB_PER_DPS * GYRO_LSB_PER_DPS);
KalmanFilter kalmanX(4.0 * LSB2_TO_DPS2, 16.0 * LSB2_TO_DPS2, 1.0 * LSB2_TO_DPS2);
KalmanFilter kalmanY(4.0 * LSB2_TO_DPS2, 16.0 * LSB2_TO_DPS2, 1.0 * LSB2_TO_DPS2);

// ---------- Blink sensor (TCRT5000) ----------
int blinkPin = D1;
unsigned long blinkStart = 0;
bool isEyeClosed = false;

const int singleBlinkDuration      = 300;   // ms; shorter = natural blink, ignored
const int rightClickBlinkDuration  = 700;   // ms
const int doubleClickBlinkDuration = 1300;  // ms
const int maxBlinkDuration         = 1700;  // ms; longer = resting eyes, ignored

// ---------- Boot-time calibration ----------
// Keep the head still. 500 readings, 3 ms apart (about 1.5 s).
// Mean  -> zero-rate offset (subtracted from every later reading).
// Std   -> resting noise of the sensor (printed for the report; the
//          physically motivated value of R would be its square).
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
  Serial.printf("Measured R (LSB^2): X = %.1f, Y = %.1f  (code uses R = 16)\n",
                sq(sdX * GYRO_LSB_PER_DPS), sq(sdY * GYRO_LSB_PER_DPS));
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
  cfg.batteryLevel = 70;   // fixed value shown to the computer, not measured
  cfg.appearance = ESP32BLEComboAppearance::AUTO;
  ble.begin(cfg);

  Serial.println("Setup complete.");
}

void loop() {
  // --- Head angular velocity -> cursor movement ---
  headMotionSensor.getMotion6(&rawAccelX, &rawAccelY, &rawAccelZ,
                              &rawGyroX, &rawGyroY, &rawGyroZ);

  // Convert to deg/s, remove the resting offset, then filter.
  float omegaX = kalmanX.update(rawGyroX / GYRO_LSB_PER_DPS + gyroOffsetX_dps);  // pitch -> vertical
  float omegaY = kalmanY.update(rawGyroY / GYRO_LSB_PER_DPS + gyroOffsetY_dps);  // yaw   -> horizontal

  // Cursor step per loop is proportional to angular velocity, so over many
  // loops the cursor displacement is proportional to the head ANGLE turned.
  if (fabs(omegaY) > HORIZ_WOBBLE_DPS) {
    cursorXMovement = -omegaY / HORIZ_DPS_PER_STEP;   // truncated toward zero
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
      ble.mouseMove(0, 0);   // keeps the Bluetooth link alive
      lastHeartbeat = millis();
    }
  }

  // --- Blink detection (eye-closed duration) ---
  int blinkState = digitalRead(blinkPin);

  if (blinkState == HIGH) {                 // eye closed
    if (!isEyeClosed) {
      blinkStart = millis();
      isEyeClosed = true;
    }
  } else {                                  // eye open
    if (isEyeClosed) {
      unsigned long blinkDuration = millis() - blinkStart;

      if (blinkDuration > maxBlinkDuration) {
        // too long: resting eyes, ignore
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
