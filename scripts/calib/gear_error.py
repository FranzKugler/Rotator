"""What the AS5600 cannot see.

The sensor sits on the motor shaft, upstream of the 10:1 reduction, so it
measures the motor and only infers the output. Anything the gearing
contributes between the two - eccentricity, tooth-to-tooth transmission
error, a bearing running out - is invisible to it at any resolution, and no
amount of sensor calibration will ever recover it. The camera looks at the
output shaft directly, so the difference between the two is exactly that
missing term.

    theta_sensor = step revolution + corrected AS5600 within it
    theta_camera = measured by the chessboard on the output shaft
    gear error   = theta_camera - theta_sensor

This matters for the goal of calibrating a fresh unit without a camera: the
sensor-side errors are calibratable that way and this one is not, so
whatever this measures is the accuracy floor of a camera-free calibration.

The harmonic breakdown is against the OUTPUT revolution, where the terms
separate by origin: 1 cycle per output revolution is the output stage
running out, 10 cycles per output revolution is the motor pinion (once per
motor revolution), and anything much higher is tooth mesh.
"""

import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from measure_frames import (load_manifest, detect_all, angles_from_corners,
                            resolve_flips)

DEG_PER_MICROSTEP = 360.0 / 400.0 / 10.0 / 256.0
OUTPUT_DEG_PER_COUNT = 36.0 / 4096.0
SENSOR_PERIOD_DEG = 36.0


def correct(raw, c0, a, b):
    theta = 2.0 * np.pi * np.asarray(raw) / 4096.0
    error = np.full_like(np.asarray(raw, float), c0)
    for k in range(1, len(a)):
        error += a[k] * np.cos(k * theta) + b[k] * np.sin(k * theta)
    return np.asarray(raw, float) - error


def wrap(value, period):
    return (value + period / 2.0) % period - period / 2.0


def sensor_angles(raw, step_position, c0, a, b):
    """Absolute output angle from the step counter plus the corrected sensor.

    The counter supplies which 36-degree period we are in - its absolute
    error is far below the 18 degrees that would be needed to pick the wrong
    one - and the corrected sensor supplies the position inside it."""
    coarse = np.asarray(step_position, float) * DEG_PER_MICROSTEP
    fine = correct(raw, c0, a, b) * OUTPUT_DEG_PER_COUNT
    offset = np.mean(wrap(coarse - fine, SENSOR_PERIOD_DEG))
    return coarse + wrap(fine + offset - coarse, SENSOR_PERIOD_DEG)


def harmonics(angle_deg, value, orders):
    """Least-squares harmonic content against the output revolution."""
    phase = np.radians(angle_deg)
    columns = [np.ones_like(phase)]
    for k in orders:
        columns.append(np.cos(k * phase))
        columns.append(np.sin(k * phase))
    M = np.column_stack(columns)
    coefficients, *_ = np.linalg.lstsq(M, value, rcond=None)
    residual = value - M @ coefficients
    amplitude = {k: math.hypot(coefficients[2 * i + 1], coefficients[2 * i + 2])
                 for i, k in enumerate(orders)}
    return amplitude, residual, coefficients


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dir", required=True)
    ap.add_argument("--coefficients", required=True)
    ap.add_argument("--orders", default="1,2,3,10,20",
                    help="harmonic orders per OUTPUT revolution to report")
    ap.add_argument("--export")
    args = ap.parse_args()

    with open(args.coefficients) as handle:
        fit = json.load(handle)
    header, frames = load_manifest(args.dir)
    corners, ok = detect_all(args.dir, frames, quiet=True)
    keep = [i for i, good in enumerate(ok) if good]
    corners = [corners[i] for i in keep]
    usable = [frames[i] for i in keep]
    print(f"{len(usable)}/{len(frames)} frames usable")

    camera, anisotropy, residual_px = angles_from_corners(corners)
    raw = np.array([f["rawSensor"] for f in usable])
    step = np.array([f["stepPosition"] for f in usable])
    sensor = sensor_angles(raw, step, fit["C0"], fit["A"], fit["B"])

    # The camera's sense of rotation and its zero are its own; neither is an
    # error, so both are removed before anything is called a gear error.
    # The board's 180-degree labelling is settled against the sensor-derived
    # angle, which is good to a few hundredths of a degree - far inside the
    # 90 degrees that decision needs (see board.py).
    best = None
    for sign in (1.0, -1.0):
        resolved, flips = resolve_flips(sign * camera, sensor)
        difference = np.radians(resolved - sensor)
        offset = np.arctan2(np.sin(difference).mean(), np.cos(difference).mean())
        error = np.degrees(np.angle(np.exp(1j * (difference - offset))))
        score = float(np.sqrt(np.mean(error ** 2)))
        if best is None or score < best[0]:
            best = (score, sign, error, flips)
    _, sign, gear, flips = best
    camera = resolve_flips(sign * camera, sensor)[0]
    print(f"camera sense {'as sensor' if sign > 0 else 'reversed'}, "
          f"{flips} frame(s) had the board's 180-degree labelling resolved")

    # Fitted against the ABSOLUTE mechanical angle, not against this run's
    # own mean. The harmonics are only portable between runs - and usable as
    # a correction - if their phase is referenced to the machine's mechanical
    # zero, which is what the step counter already counts from.
    gear = gear - gear.mean()
    orders = [int(x) for x in args.orders.split(",")]
    amplitude, harmonic_residual, coefficients = harmonics(sensor, gear, orders)
    output_angle = sensor - sensor.mean()

    print(f"\ngear error (camera minus sensor-derived), over "
          f"{output_angle.max()-output_angle.min():.1f} deg of output:")
    print(f"  rms  {np.std(gear)*1000:8.2f} mdeg")
    print(f"  p-p  {(gear.max()-gear.min())*1000:8.2f} mdeg")
    print(f"  camera repeatability for scale: ~2.3 mdeg\n")
    print(f"  {'cycles/output rev':>18}  {'amplitude':>12}   origin")
    labels = {1: "output stage run-out", 2: "output stage, 2nd",
              10: "motor pinion (once per motor revolution)",
              20: "pinion, 2nd harmonic"}
    for k in orders:
        print(f"  {k:18d}  {amplitude[k]*1000:9.2f} mdeg   "
              f"{labels.get(k, '')}")
    print(f"\n  residual after those harmonics: {np.std(harmonic_residual)*1000:.2f} mdeg rms")
    print(f"  homography residual {residual_px.mean():.4f} px, "
          f"anisotropy {np.abs(anisotropy).max():.2e}")

    if args.export:
        with open(args.export, "w") as handle:
            json.dump({"dir": args.dir, "outputAngleDeg": output_angle.tolist(),
                       "cameraDeg": camera.tolist(), "sensorDeg": sensor.tolist(),
                       "gearErrorDeg": gear.tolist(),
                       "rawSensor": raw.tolist(), "stepPosition": step.tolist(),
                       "amplitudeMdeg": {str(k): amplitude[k] * 1000 for k in orders},
                       "model": {"orders": orders,
                                 "constant": coefficients[0],
                                 "cos": [coefficients[2 * i + 1] for i in range(len(orders))],
                                 "sin": [coefficients[2 * i + 2] for i in range(len(orders))],
                                 "reference": "absolute mechanical angle in degrees, "
                                              "as counted from the machine's mechanical zero"},
                       "residualMdeg": float(np.std(harmonic_residual) * 1000),
                       "coefficients": args.coefficients}, handle, indent=2)
        print(f"wrote {args.export}")


if __name__ == "__main__":
    main()
