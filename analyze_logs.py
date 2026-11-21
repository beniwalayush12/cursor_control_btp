"""
BTP data analysis: head-motion BLE mouse (Ayush Beniwal, 23B1810, PH 587).

Reads CSV logs from gyro_blink_logger.ino (columns: t_us,gx,gy,gz,eye) and
makes the figures and numbers for the report.

Install:  pip install numpy matplotlib tqdm pyserial

Usage:
  # 0) Record a log straight from the board (Ctrl+C to stop)
  python analyze_logs.py record --port /dev/cu.usbmodem1101 --out still.csv

  # A) Allan deviation of the gyro (device still for 30-60 min)
  python analyze_logs.py allan still.csv

  # B) Raw vs Kalman-filtered head motion, plus cursor steps
  python analyze_logs.py motion headturns.csv

  # C) Blink durations and how the firmware would classify them
  python analyze_logs.py blink blinks.csv

  # Any analysis mode can be limited to a time window, e.g. an undisturbed stretch:
  python analyze_logs.py allan still.csv --tmin 434.5 --tmax 544.7

Every mode also prints the real loop period (needed for filter lag and
counts-per-degree in the report). Figures are saved as PDF next to the CSV.
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from tqdm import tqdm

LSB_PER_DPS = 131.0                       # MPU6050 at +/-250 deg/s
Q_LSB2, R_LSB2, P0_LSB2 = 4.0, 16.0, 1.0  # same as firmware
H_STEP_LSB, V_STEP_LSB = 250.0, 150.0     # sensitivity divisors
WOBBLE_LSB = 5.0
BLINK_EDGES_MS = [300, 700, 1300, 1700]   # firmware click windows
BLINK_LABELS = ["ignored (natural)", "left click", "right click", "double click", "ignored (resting)"]

plt.rcParams.update({"font.size": 10, "font.family": "serif", "figure.dpi": 150})


# ----------------------------------------------------------------- I/O
TMIN, TMAX = 0.0, float("inf")   # analysis window in seconds, set by --tmin/--tmax


def load_log(path):
    """Load CSV, drop malformed and '#' warning lines, keep only TMIN..TMAX seconds."""
    rows = []
    with open(path, "r", errors="ignore") as f:
        lines = f.readlines()
    for line in tqdm(lines, desc="Reading log", unit="line"):
        parts = line.strip().split(",")
        if len(parts) != 5:
            continue
        try:
            rows.append([float(p) for p in parts])
        except ValueError:
            continue  # header or garbage
    if len(rows) < 10:
        sys.exit(f"Not enough valid rows in {path}")
    a = np.array(rows)
    t = a[:, 0] * 1e-6
    t = np.unwrap(t, period=2**32 * 1e-6)  # micros() overflows every ~71 min
    t = t - t[0]
    keep = (t >= TMIN) & (t <= TMAX)
    if keep.sum() < 10:
        sys.exit(f"Not enough rows between {TMIN} and {TMAX} s")
    a, t = a[keep], t[keep]
    return {"t": t - t[0], "gx": a[:, 1], "gy": a[:, 2], "gz": a[:, 3], "eye": a[:, 4].astype(int)}


def report_loop_period(t):
    dt = np.diff(t)
    print(f"Samples: {len(t)}, duration: {t[-1]:.1f} s")
    print(f"Loop period: mean {dt.mean()*1e3:.2f} ms, median {np.median(dt)*1e3:.2f} ms, "
          f"std {dt.std()*1e3:.2f} ms  ->  rate {1/dt.mean():.1f} Hz")
    return dt.mean()


def record(port, out):
    import serial  # pyserial
    with serial.Serial(port, 921600, timeout=1) as ser, open(out, "w") as f:
        print(f"Recording from {port} to {out}. Press Ctrl+C to stop.")
        bar = tqdm(desc="Recording", unit="line")
        try:
            while True:
                line = ser.readline().decode(errors="ignore")
                if line:
                    f.write(line)
                    bar.update(1)
        except KeyboardInterrupt:
            pass
        bar.close()
    print("Saved", out)


# ----------------------------------------------------------------- A: Allan
def overlapping_allan_dev(rate, T, n_taus=60):
    """Overlapping Allan deviation of a rate signal sampled every T seconds."""
    N = len(rate)
    theta = np.concatenate(([0.0], np.cumsum(rate) * T))  # integrated angle
    ms = np.unique(np.logspace(0, np.log10((N - 1) // 2), n_taus).astype(int))
    taus, adev = [], []
    for m in tqdm(ms, desc="Allan deviation", unit="tau"):
        d = theta[2 * m:] - 2 * theta[m:-m] + theta[:-2 * m]
        avar = np.sum(d**2) / (2 * (m * T) ** 2 * (N - 2 * m))
        taus.append(m * T)
        adev.append(np.sqrt(avar))
    return np.array(taus), np.array(adev)


def run_allan(path):
    d = load_log(path)
    T = report_loop_period(d["t"])
    fig, ax = plt.subplots(figsize=(4.2, 3.2))
    for axis, color in [("gx", "C0"), ("gy", "C1")]:
        rate = d[axis] / LSB_PER_DPS
        print(f"\n[{axis}] mean offset {rate.mean():.3f} deg/s, raw std {rate.std():.4f} deg/s "
              f"({rate.std()*LSB_PER_DPS:.1f} LSB, variance {np.var(d[axis]):.1f} LSB^2)")
        taus, adev = overlapping_allan_dev(rate - rate.mean(), T)
        # Angle random walk: read the -1/2 slope line at tau = 1 s
        sel = taus <= 1.0
        if sel.sum() >= 2:
            coef = np.polyfit(np.log10(taus[sel]), np.log10(adev[sel]), 1)
            slope = coef[0]
            arw = 10 ** np.polyval(coef, 0.0)  # deg/s at tau = 1 s == deg/sqrt(s)
            print(f"  short-tau slope {slope:.2f} (white noise gives -0.5)")
            print(f"  angle random walk ~ {arw:.4f} deg/sqrt(s) = {arw*60:.2f} deg/sqrt(h)")
        i = np.argmin(adev)
        print(f"  bias instability ~ {adev[i]/0.664:.4f} deg/s = {adev[i]/0.664*3600:.1f} deg/h "
              f"(Allan minimum {adev[i]:.4f} deg/s at tau = {taus[i]:.1f} s)")
        ax.loglog(taus, adev, color=color, label=f"gyro {axis[1].upper()}")
    t_ref = np.array([T, 1.0])
    ax.loglog(t_ref, 0.005 / np.sqrt(t_ref), "k--", lw=0.8,
              label=r"datasheet $0.005^\circ$/s/$\sqrt{\rm Hz}$")
    ax.set_xlabel(r"averaging time $\tau$ (s)")
    ax.set_ylabel(r"Allan deviation $\sigma(\tau)$ ($^\circ$/s)")
    ax.grid(True, which="both", lw=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    out = Path(path).with_name("fig_allan.pdf")
    fig.savefig(out)
    print("\nSaved", out)


# ----------------------------------------------------------------- B: motion
def kalman(z, q, r, p0):
    x, p, out = 0.0, p0, np.empty_like(z)
    for i in tqdm(range(len(z)), desc="Kalman filter", unit="sample", leave=False):
        p += q
        k = p / (p + r)
        x += k * (z[i] - x)
        p *= 1 - k
        out[i] = x
    return out


def steps(omega_lsb, divisor):
    s = np.trunc(omega_lsb / divisor)
    s[np.abs(omega_lsb) <= WOBBLE_LSB] = 0
    return s


def run_motion(path, n_cal=500):
    d = load_log(path)
    T = report_loop_period(d["t"])
    t = d["t"]
    gy = d["gy"] - d["gy"][:n_cal].mean()   # first ~5 s should be still (like boot calibration)
    gx = d["gx"] - d["gx"][:n_cal].mean()
    fy = kalman(gy, Q_LSB2, R_LSB2, P0_LSB2)
    fx = kalman(gx, Q_LSB2, R_LSB2, P0_LSB2)
    sx, sy = -steps(fy, H_STEP_LSB), steps(fx, V_STEP_LSB)

    still = slice(0, n_cal)
    print(f"Noise while still: raw {gy[still].std()/LSB_PER_DPS:.4f} deg/s, "
          f"filtered {fy[still].std()/LSB_PER_DPS:.4f} deg/s, "
          f"ratio {fy[still].std()/gy[still].std():.2f} (theory 0.49)")
    angle = np.cumsum(-gy) * T / LSB_PER_DPS
    print(f"Total horizontal steps {sx.sum():.0f}; head yaw from integration {angle[-1]:.1f} deg")

    fig, (a1, a2) = plt.subplots(2, 1, figsize=(4.6, 4.2), sharex=True)
    a1.plot(t, gy / LSB_PER_DPS, lw=0.5, color="0.6", label="raw")
    a1.plot(t, fy / LSB_PER_DPS, lw=1.0, color="C0", label="Kalman")
    a1.axhspan(-H_STEP_LSB / LSB_PER_DPS, H_STEP_LSB / LSB_PER_DPS, color="C3", alpha=0.12,
               label="dead zone")
    a1.set_ylabel(r"$\omega_y$ ($^\circ$/s)")
    a1.legend(fontsize=7, loc="upper right")
    a2.plot(t, np.cumsum(sx), color="C2", label="cursor x (steps)")
    a2b = a2.twinx()
    a2b.plot(t, angle, color="k", lw=0.8, ls="--", label="head yaw")
    a2.set_ylabel("cursor steps")
    a2b.set_ylabel(r"head angle ($^\circ$)")
    a2.set_xlabel("time (s)")
    fig.tight_layout()
    out = Path(path).with_name("fig_motion.pdf")
    fig.savefig(out)
    print("Saved", out)


# ----------------------------------------------------------------- C: blink
def run_blink(path):
    d = load_log(path)
    report_loop_period(d["t"])
    t, eye = d["t"], d["eye"]
    durations, start = [], None
    for i in tqdm(range(len(eye)), desc="Finding blinks", unit="sample"):
        if eye[i] == 1 and start is None:
            start = t[i]
        elif eye[i] == 0 and start is not None:
            durations.append((t[i] - start) * 1e3)
            start = None
    dur = np.array(durations)
    if dur.size == 0:
        sys.exit("No eye closures found (eye column never went to 1).")
    cls = np.digitize(dur, BLINK_EDGES_MS, right=True)  # matches firmware '>' comparisons
    print(f"\n{dur.size} closures found. Durations (ms): min {dur.min():.0f}, "
          f"median {np.median(dur):.0f}, max {dur.max():.0f}")
    for k, lab in enumerate(BLINK_LABELS):
        print(f"  {lab:18s}: {np.sum(cls == k)}")
    print("Compare these counts with how many of each blink you intended to make.")
    np.savetxt(Path(path).with_name("blink_durations_ms.txt"), dur, fmt="%.1f")

    fig, ax = plt.subplots(figsize=(4.2, 2.8))
    ax.hist(dur, bins=np.arange(0, max(2000, dur.max() + 50), 50), color="C0")
    for e in BLINK_EDGES_MS:
        ax.axvline(e, color="C3", lw=0.8, ls="--")
    ax.set_xlabel("eye-closed duration (ms)")
    ax.set_ylabel("count")
    fig.tight_layout()
    out = Path(path).with_name("fig_blinks.pdf")
    fig.savefig(out)
    print("Saved", out)


# ----------------------------------------------------------------- main
if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="mode", required=True)
    r = sub.add_parser("record"); r.add_argument("--port", required=True); r.add_argument("--out", required=True)
    for m in ("allan", "motion", "blink"):
        sp = sub.add_parser(m)
        sp.add_argument("csv")
        sp.add_argument("--tmin", type=float, default=0.0, help="start of analysis window (s)")
        sp.add_argument("--tmax", type=float, default=float("inf"), help="end of analysis window (s)")
    args = ap.parse_args()
    if args.mode != "record":
        TMIN, TMAX = args.tmin, args.tmax
    if args.mode == "record":
        record(args.port, args.out)
    elif args.mode == "allan":
        run_allan(args.csv)
    elif args.mode == "motion":
        run_motion(args.csv)
    else:
        run_blink(args.csv)
