"""Answers Franz's question: instead of storing the 400-entry raw
full-step table verbatim, can a smooth harmonic fit at some higher order
capture the same systematic (repeatable) error while averaging out the
rest as noise?

Held-out comparison using scripts/camera_fullstep_sweep.py's two
independently-measured revolutions: rev 0 fits every candidate (a range of
harmonic orders, plus the raw single-revolution table itself), rev 1
scores it - a real test of whether each candidate generalizes to a fresh
measurement of the same repeatable error, not just how well it fits the
data it was built from (the same overfitting trap
scripts/segment_calibration_experiment.py's own held-out design guards
against).

Usage:
    /tmp/camvenv/bin/python scripts/fullstep_order_sweep_experiment.py \\
        --sweep-dir /tmp/fullstep_sweep1
"""

import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from camera_angle_analyze import fit_fourier_against_reference, track_rotation, wrap4096  # noqa: E402
from fullstep_table import despike  # noqa: E402


def correct(raw, model):
    C0, A, B = model
    theta = 2.0 * math.pi * raw / 4096.0
    err = C0
    for k in range(1, len(A)):
        err += A[k] * math.cos(k * theta) + B[k] * math.sin(k * theta)
    return raw - err


def rms_peak(errs):
    rms = (sum(e * e for e in errs) / len(errs)) ** 0.5
    return rms, max(abs(e) for e in errs)


def build_rows(sweep_dir, pattern_size):
    with open(os.path.join(sweep_dir, "manifest.json")) as f:
        manifest = json.load(f)
    frames = manifest["frames"]
    tracked = {t["index"]: t for t in track_rotation(sweep_dir, frames, pattern_size)}
    rows = []
    for fr in frames:
        t = tracked.get(fr["index"])
        if t is None:
            continue
        rows.append({**fr, "cameraOutputDeg": t["cameraOutputDeg"]})
    offset = frames[0]["idealOutputDeg"] - rows[0]["cameraOutputDeg"]
    for r in rows:
        r["cameraOutputDeg"] += offset
        r["cameraMotorDeg"] = r["cameraOutputDeg"] * 10.0  # REDUCTION
    return rows, manifest["fullstepsPerRotation"]


def run(sweep_dir, pattern_size, orders):
    rows, n = build_rows(sweep_dir, pattern_size)
    train = [r for r in rows if r["rev"] == 0]
    test = [r for r in rows if r["rev"] == 1]
    print(f"[order-sweep] {len(train)} train (rev 0) / {len(test)} test (rev 1) points, n={n}")

    print(f"\n{'order':>6} {'params':>7} {'train RMS':>10} {'test RMS':>9} {'test peak':>10}  (motor-shaft deg)")
    for k in orders:
        model = fit_fourier_against_reference(
            [r["idealMotorDeg"] for r in train],
            [r["rawSensor"] for r in train],
            [r["cameraMotorDeg"] for r in train],
            k,
        )
        train_errs = [wrap4096(correct(r["rawSensor"], model) - 4096.0 * (r["idealMotorDeg"] % 360.0) / 360.0)
                      for r in train]
        test_errs = [wrap4096(correct(r["rawSensor"], model) - 4096.0 * (r["idealMotorDeg"] % 360.0) / 360.0)
                     for r in test]
        train_rms, _ = rms_peak(train_errs)
        test_rms, test_peak = rms_peak(test_errs)
        deg_per_count = 360.0 / 4096.0
        print(f"{k:6d} {2 * k + 1:7d} {train_rms * deg_per_count:10.4f} {test_rms * deg_per_count:9.4f} "
              f"{test_peak * deg_per_count:10.4f}")

    # --- raw table, built from rev 0 ONLY (one measurement per bin - no
    # averaging across revolutions, for a fair comparison at the same
    # "how much data did this see" footing as the fits above), scored on
    # rev 1 ---
    bins = [None] * n
    for r in train:
        bins[r["stepIndex"]] = wrap4096(r["rawSensor"] - 4096.0 * (r["cameraMotorDeg"] % 360.0) / 360.0)
    missing = [i for i, v in enumerate(bins) if v is None]
    if missing:
        print(f"\n  !! table has {len(missing)} empty bin(s), filled with 0: {missing}")
    bins = [v if v is not None else 0.0 for v in bins]
    bins, spikes = despike(bins)
    mean_e = sum(bins) / n
    table = [v - mean_e for v in bins]

    test_errs = []
    for r in test:
        idx = r["stepIndex"]
        smooth_c0_only = r["rawSensor"] - mean_e  # C0-equivalent only, matching the table's own convention
        corrected = smooth_c0_only - table[idx]
        ideal_counts = 4096.0 * (r["idealMotorDeg"] % 360.0) / 360.0
        test_errs.append(wrap4096(corrected - ideal_counts))
    table_rms, table_peak = rms_peak(test_errs)
    deg_per_count = 360.0 / 4096.0
    print(f"\nraw table (rev 0 only, {n} params), test RMS={table_rms * deg_per_count:.4f} deg  "
          f"peak={table_peak * deg_per_count:.4f} deg (despiked {len(spikes)} point(s))")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sweep-dir", required=True, help="directory produced by camera_fullstep_sweep.py")
    ap.add_argument("--cols", type=int, default=9)
    ap.add_argument("--rows", type=int, default=8)
    ap.add_argument("--orders", default="1,2,4,6,10,15,20,30,40,60,80,100,150,199",
                     help="comma-separated harmonic orders to try")
    args = ap.parse_args()

    orders = [int(x) for x in args.orders.split(",")]
    run(args.sweep_dir, (args.cols - 1, args.rows - 1), orders)


if __name__ == "__main__":
    main()
