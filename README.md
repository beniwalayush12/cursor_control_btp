# Inertial Motion Sensing for a Wireless Portable Cursor Control System

A head-worn Bluetooth mouse for people with limited hand movement. A gyroscope on a glasses frame moves the cursor with head rotation, and an infrared sensor next to the eye turns deliberate long blinks into clicks. The board appears to the computer as a standard Bluetooth LE mouse, so nothing needs to be installed on the computer.

B.Tech Project I (PH 587), Department of Physics, IIT Bombay — Ayush Beniwal (23B1810).

## Hardware

| Part | Notes |
|---|---|
| Seeed XIAO ESP32C3 | external 2.4 GHz antenna must be attached |
| MPU6050 (gyroscope + accelerometer) | SDA → D4, SCL → D5, VCC → 3V3, GND → GND |
| TCRT5000 IR reflective sensor | OUT → D1, VCC → 3V3, GND → GND; output HIGH when the eye is closed |
| 3.7 V LiPo battery | with a push button to start and shut down |

## How it works

1. **Calibration at power-on (about 1.5 s, hold still):** 500 gyro readings are averaged to find the resting offset, which is then subtracted from every reading.
2. **Head motion → cursor (about 100 Hz loop):** gyro Y (yaw) moves the cursor sideways and gyro X (pitch) moves it up and down. Each axis goes through a scalar Kalman filter (Q = 4, R = 16, steady-state gain 0.39, roughly an 8 Hz low-pass filter). The result is divided by 1.91 °/s (horizontal) or 1.15 °/s (vertical) per cursor step. Because that result is rounded down to a whole number, slower motion gives no movement (a dead zone).
3. **Blinks → clicks (eye-closed duration):**

   | Duration | Action |
   |---|---|
   | < 300 ms | ignored (natural blink) |
   | 300–700 ms | left click |
   | 700–1300 ms | right click |
   | 1300–1700 ms | double click |
   | > 1700 ms | ignored (resting eyes) |

4. **Heartbeat:** an empty mouse movement every 4 s keeps the Bluetooth link alive.

## Results (BTP-1)

| Quantity | Measured | Model |
|---|---|---|
| Loop period | 11.0 ± 0.4 ms (91 Hz) | 10 ms assumed |
| Gyro noise at rest (X, Y, Z) | 0.13, 0.20, 0.13 °/s r.m.s., white | thermal (white) noise |
| Angle random walk (X, Y) | 0.76, 1.0 °/√h | — |
| Kalman noise reduction (X, Y) | 0.50, 0.47 | 0.49 |
| Cursor gain vs head angle | 46.0 counts/°, r = 0.99 | 47.5 counts/° |
| Blink clicks recognised (left, right, double) | 6/9, 9/11, 0/9 | — |

Double clicks failed because the user's intended 1.5 s closures actually lasted 1.0–1.2 s (below the 1.3 s window), so the click windows should be calibrated per user. Figures are in `results/`.

## Repository contents

```
data/                       raw logs used in the report (t_us,gx,gy,gz,eye)
  still.csv                 10 min at rest; undisturbed stretch used: 434.5–544.7 s
  headturns.csv             1 min of head turns; clean part used: 0–50.5 s
  blinks_left/right/double.csv   ~10 attempted closures of each click type
results/                    figures in the report
firmware/
  head_mouse_v2/            final firmware, written in physical units (°/s); behaves the same as the original
  head_mouse_v2_original/   the original v2 firmware, written in raw sensor counts
  gyro_blink_logger/        measurement sketch: streams raw gyro and blink data over USB (not a mouse)
analysis/
  analyze_logs.py           Allan deviation, raw vs filtered head motion, blink-duration histogram
```

## Building the firmware

Arduino IDE with the ESP32 board package, board **XIAO_ESP32C3**, with **USB CDC On Boot** enabled.
Libraries: `I2Cdev`, `MPU6050`, `ESP32BLECombo`, `Wire`.
Use a USB cable that carries data. An I²C scanner should find the MPU6050 at address `0x68`.

## Measurements

Flash `gyro_blink_logger`, record the serial output, then run:

```bash
pip install numpy matplotlib tqdm pyserial
python analysis/analyze_logs.py record --port <serial-port> --out still.csv
python analysis/analyze_logs.py allan still.csv        # device lying still, 30–60 min
python analysis/analyze_logs.py motion headturns.csv   # wearing it, slow and fast head turns
python analysis/analyze_logs.py blink blinks.csv       # 20 blinks of each type
```

To reproduce the report's numbers from the included data:

```bash
python analysis/analyze_logs.py allan  data/still.csv --tmin 434.5 --tmax 544.7
python analysis/analyze_logs.py motion data/headturns.csv --tmax 50.5
python analysis/analyze_logs.py blink  data/blinks_left.csv
```

Flash `head_mouse_v2` back afterwards.
