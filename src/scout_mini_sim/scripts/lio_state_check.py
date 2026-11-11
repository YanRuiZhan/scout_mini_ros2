#!/usr/bin/env python3
"""Check a FAST-LIO run on the flat warehouse floor from its state log.

FAST-LIO writes one line per LiDAR frame to ``fast_lio_ros2/Log/mat_pre.txt``
(overwritten at every start). On a flat floor with the simulated ideal IMU the
truth in ``camera_init`` is known without ground-truth poses:

    roll = pitch = 0, z = 0, gyro bias = accel bias = 0, gravity = (0, 0, -9.81)

so every deviation is estimator error. The report shows which state drifts,
when it drifts (stationary, driving straight or turning) and whether the run
passes the acceptance limits.

    python3 lio_state_check.py <ws>/src/third_party/fast_lio_ros2/Log/mat_pre.txt
    python3 lio_state_check.py mat_pre.txt --every 5      # table every 5 s
"""
import argparse
import math
import sys

import numpy as np

# mat_pre.txt columns (laserMapping.cpp): t, euler[deg](3), pos(3), ext_euler(3),
# ext_T(3), vel(3), bg(3), ba(3), grav(3)
COLUMNS = 25

# Acceptance limits for the same route with the fixes enabled.
MAX_TILT_DEG = 0.3
MAX_GYRO_BIAS_MDEG_S = 10.0
MAX_ACC_BIAS = 0.02
MAX_SLOPE_DEG = 0.1

MOVING_SPEED = 0.05        # m/s
TURNING_RATE_DEG = 5.0     # deg/s


def load(path):
    rows = []
    with open(path) as log:
        for line in log:
            fields = line.split()
            if len(fields) < COLUMNS:
                continue
            try:
                rows.append([float(x) for x in fields[:COLUMNS]])
            except ValueError:
                continue
    if len(rows) < 2:
        raise SystemExit(f"{path}: fewer than two state lines")
    a = np.array(rows)
    return dict(t=a[:, 0], rpy=a[:, 1:4], pos=a[:, 4:7], vel=a[:, 13:16],
                bg=a[:, 16:19], ba=a[:, 19:22], grav=a[:, 22:25])


def rotation(roll, pitch, yaw):
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def derive(s):
    """Add path length, motion and tilt quantities to the loaded state."""
    t, rpy, pos, vel, grav = s["t"], s["rpy"], s["pos"], s["vel"], s["grav"]
    s["dist"] = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(pos[:, :2], axis=0), axis=1))]
    s["speed"] = np.linalg.norm(vel[:, :2], axis=1)
    yaw = np.unwrap(np.radians(rpy[:, 2]))
    s["yaw_rate"] = np.degrees(np.gradient(yaw, t))
    up = np.array([rotation(*r)[:, 2] for r in np.radians(rpy)])
    s["tilt"] = np.degrees(np.arccos(np.clip(up[:, 2], -1, 1)))
    unit_g = grav / np.linalg.norm(grav, axis=1, keepdims=True)
    s["grav_tilt"] = np.degrees(np.arccos(np.clip(-unit_g[:, 2], -1, 1)))
    # Body tilt relative to the ESTIMATED gravity: what the accelerometer agrees with.
    s["body_vs_grav"] = np.degrees(np.arccos(np.clip((up * -unit_g).sum(1), -1, 1)))
    s["bg_mdeg"] = np.degrees(s["bg"]) * 1e3
    moving = s["speed"] > MOVING_SPEED
    turning = np.abs(s["yaw_rate"]) > TURNING_RATE_DEG
    s["motion"] = np.where(turning, "turning", np.where(moving, "straight", "stationary"))
    s["slope_deg"] = (math.degrees(math.atan(np.polyfit(s["dist"], pos[:, 2], 1)[0]))
                      if s["dist"][-1] > 1.0 else 0.0)
    return s


def summary(s):
    lines = [f"frames={len(s['t'])}  duration={s['t'][-1] - s['t'][0]:.1f} s  "
             f"path length={s['dist'][-1]:.1f} m",
             f"{'':14}{'start':>10}{'end':>10}{'max|.|':>10}"]
    rows = (("roll deg", s["rpy"][:, 0]), ("pitch deg", s["rpy"][:, 1]), ("z m", s["pos"][:, 2]),
            ("vz m/s", s["vel"][:, 2]),
            ("bg_x mdeg/s", s["bg_mdeg"][:, 0]), ("bg_y mdeg/s", s["bg_mdeg"][:, 1]),
            ("bg_z mdeg/s", s["bg_mdeg"][:, 2]),
            ("ba_x m/s2", s["ba"][:, 0]), ("ba_y m/s2", s["ba"][:, 1]), ("ba_z m/s2", s["ba"][:, 2]),
            ("grav tilt", s["grav_tilt"]), ("body-vs-g", s["body_vs_grav"]))
    for name, v in rows:
        lines.append(f"{name:14}{v[0]:10.4f}{v[-1]:10.4f}{np.abs(v).max():10.4f}")
    lines.append(f"z vs path length slope = {s['slope_deg']:+.3f} deg")
    return lines


def growth_by_motion(s):
    """Tilt gained in each motion class: {class: (seconds, metres, tilt change in deg)}."""
    out = {}
    dt = np.diff(s["t"])
    dd = np.diff(s["dist"])
    dtilt = np.diff(s["tilt"])
    for name in ("stationary", "straight", "turning"):
        mask = s["motion"][1:] == name
        out[name] = (float(dt[mask].sum()), float(dd[mask].sum()), float(dtilt[mask].sum()))
    return out


def timeseries(s, every):
    lines = [f"{'t s':>7}{'dist m':>8}{'motion':>12}{'roll':>8}{'pitch':>8}{'z m':>8}"
             f"{'|bg| mdeg/s':>12}{'|ba_xy|':>9}{'grav tilt':>10}"]
    next_t = s["t"][0]
    for i, t in enumerate(s["t"]):
        if t < next_t:
            continue
        next_t = t + every
        lines.append(f"{t:7.1f}{s['dist'][i]:8.1f}{s['motion'][i]:>12}{s['rpy'][i, 0]:8.3f}"
                     f"{s['rpy'][i, 1]:8.3f}{s['pos'][i, 2]:8.3f}"
                     f"{np.linalg.norm(s['bg_mdeg'][i]):12.1f}{np.linalg.norm(s['ba'][i, :2]):9.4f}"
                     f"{s['grav_tilt'][i]:10.3f}")
    return lines


def checks(s):
    """Acceptance limits as (name, value, limit, passed)."""
    values = (
        ("max |roll|, |pitch| deg", float(np.abs(s["rpy"][:, :2]).max()), MAX_TILT_DEG),
        ("max |gyro bias| mdeg/s", float(np.abs(s["bg_mdeg"]).max()), MAX_GYRO_BIAS_MDEG_S),
        ("max |accel bias| m/s2", float(np.abs(s["ba"]).max()), MAX_ACC_BIAS),
        ("|z slope| deg", abs(s["slope_deg"]), MAX_SLOPE_DEG),
    )
    return [(name, value, limit, value < limit) for name, value, limit in values]


def report(s, every):
    lines = summary(s)
    lines.append("")
    lines.append("Tilt gained by motion class (flat floor: all should be ~0):")
    for name, (seconds, metres, gain) in growth_by_motion(s).items():
        lines.append(f"  {name:11} {seconds:7.1f} s {metres:7.1f} m  tilt change {gain:+.3f} deg")
    if every > 0:
        lines.append("")
        lines.extend(timeseries(s, every))
    lines.append("")
    results = checks(s)
    for name, value, limit, passed in results:
        lines.append(f"  [{'ok' if passed else 'FAIL'}] {name:26} {value:9.4f}  (limit {limit})")
    return lines, all(passed for *_, passed in results)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("log", help="path to fast_lio_ros2/Log/mat_pre.txt")
    parser.add_argument("--every", type=float, default=10.0,
                        help="seconds between time-series rows; 0 hides the table")
    args = parser.parse_args()
    lines, passed = report(derive(load(args.log)), args.every)
    print("\n".join(lines))
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
