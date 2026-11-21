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
2. **Head motion → cursor (about 90 Hz loop):** gyro Y (yaw) moves the cursor sideways and gyro X (pitch) moves it up and down. Each axis goes through a scalar Kalman filter (Q = 4, R = 16, steady-state gain 0.39, roughly an 8 Hz low-pass filter). The result is divided by 1.91 °/s (horizontal) or 1.15 °/s (vertical) per cursor step. Because that result is rounded down to a whole number, slower motion gives no movement (a dead zone).
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

Double clicks failed because the user's intended 1.5 s closures actually lasted 1.0–1.2 s (below the 1.3 s window), so the click windows should be calibrated per user. The figures are the `fig_*.pdf` files.

## Files

| File | What it is |
|---|---|
| `head_mouse_v2.ino` | final firmware (constants written in °/s; behaves the same as the original) |
| `head_mouse_v2_original.ino` | original v2 firmware (raw sensor counts) |
| `gyro_blink_logger.ino` | test sketch: streams raw gyro and blink data over USB (not a mouse) |
| `analyze_logs.py` | analysis: Allan deviation, head motion vs cursor, blink durations |
| `still.csv` | 10 min at rest (undisturbed stretch used: 434.5–544.7 s) |
| `headturns.csv` | 1 min of head turns (clean part used: 0–50.5 s) |
| `blinks_left.csv`, `blinks_right.csv`, `blinks_double.csv` | ~10 attempted closures of each click type |
| `fig_allan.pdf`, `fig_motion.pdf`, `fig_blinks.pdf` | figures used in the report |

Data columns: `t_us,gx,gy,gz,eye` (time in µs, raw gyro counts at 131 per °/s, eye = 1 when closed).

## Building the firmware

Arduino IDE with the ESP32 board package, board **XIAO_ESP32C3**, with **USB CDC On Boot** enabled.
Libraries: `I2Cdev`, `MPU6050`, `ESP32BLECombo`, `Wire`.
The Arduino IDE needs each `.ino` file inside a folder with the same name; it offers to create this when you open the file. Use a USB cable that carries data. An I²C scanner should find the MPU6050 at address `0x68`.

## Measurements

Flash `gyro_blink_logger.ino`, record the serial output, then analyse it:

```bash
pip install numpy matplotlib tqdm pyserial
python analyze_logs.py record --port <serial-port> --out still.csv
python analyze_logs.py allan still.csv
python analyze_logs.py motion headturns.csv
python analyze_logs.py blink blinks_left.csv
```

To reproduce the report's numbers from the included data:

```bash
python analyze_logs.py allan  still.csv --tmin 434.5 --tmax 544.7
python analyze_logs.py motion headturns.csv --tmax 50.5
python analyze_logs.py blink  blinks_left.csv
```

Flash `head_mouse_v2.ino` back afterwards.

## Use of AI tools

An AI assistant (Claude, Anthropic) wrote the test and analysis code in this repository (`gyro_blink_logger`, `analyze_logs.py`, and the physical-units firmware `head_mouse_v2`). The original firmware (`head_mouse_v2_original`), hardware and recorded data are the author's own.
