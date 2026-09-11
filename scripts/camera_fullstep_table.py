"""Builds a 400-entry per-full-step AS5600 correction table (a residual on
top of whatever C0/A/B smooth model is currently deployed), from
scripts/camera_fullstep_sweep.py's capture - the camera-referenced
counterpart to scripts/fullstep_table.py (which builds the same shape of
table from calibration_lab.py's self-referential fullstep-accuracy
measurement instead).

Fetches the currently-deployed C0/A/B live from the device
(GET /api/calibration/coefficients) so the table is built as a residual on
top of whatever is actually running, not a value that might be stale in a
local file. For each of the N_STEPS full-step indices, averages the
(smooth-corrected raw - camera reference) residual across every measured
revolution, then despikes and mean-subtracts it (mean-subtracted for the
same reason as fullstep_table.py's: that's C0's job, not this table's) -
the result is what RotatorHW::setFullStepTable() expects.

Needs opencv/numpy - run in the throwaway venv, same as
camera_angle_analyze.py.

Usage:
    /tmp/camvenv/bin/python scripts/camera_fullstep_table.py \\
        --sweep-dir /tmp/fullstep_sweep1 --host 172.22.102.30 --out table.json
"""

import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from camera_angle_analyze import track_rotation, wrap4096  # noqa: E402
from fullstep_table import despike  # noqa: E402
from upload_angle_calibration import fetch_coefficients  # noqa: E402

REDUCTION = 10  # matches main/RotatorHW.cpp


def correct_smooth(raw, C0, A, B):
    """Same formula as RotatorHW::correctSensorReading()'s smooth (C0/A/B)
    stage only - what the table below is a residual on top of."""
    theta = 2.0 * math.pi * raw / 4096.0
    err = C0
    for k in range(1, len(A)):
        err += A[k] * math.cos(k * theta) + B[k] * math.sin(k * theta)
    return raw - err


def build(sweep_dir, pattern_size, host):
    with open(os.path.join(sweep_dir, "manifest.json")) as f:
        manifest = json.load(f)
    frames = manifest["frames"]
    n = manifest["fullstepsPerRotation"]
    revolutions = manifest["revolutions"]
    print(f"[camera-fullstep-table] {len(frames)} frames, {revolutions} revolution(s) x {n} full steps")

    tracked = {t["index"]: t for t in track_rotation(sweep_dir, frames, pattern_size)}
    rows = []
    for fr in frames:
        t = tracked.get(fr["index"])
        if t is None:
            continue
        rows.append({**fr, "cameraOutputDeg": t["cameraOutputDeg"]})
    print(f"  {len(rows)}/{len(frames)} frames usable")

    offset = frames[0]["idealOutputDeg"] - rows[0]["cameraOutputDeg"]
    for r in rows:
        r["cameraOutputDeg"] += offset
        r["cameraMotorDeg"] = r["cameraOutputDeg"] * REDUCTION

    coeffs = fetch_coefficients(host)
    C0, A, B = coeffs["C0"], coeffs["A"], coeffs["B"]
    print(f"  currently-deployed smooth model: C0={C0:.3f}  "
          f"A={['%.3f' % v for v in A[1:]]}  B={['%.3f' % v for v in B[1:]]}")

    bins = [[] for _ in range(n)]
    for r in rows:
        smooth = correct_smooth(r["rawSensor"], C0, A, B)
        cam_counts = 4096.0 * (r["cameraMotorDeg"] % 360.0) / 360.0
        bins[r["stepIndex"]].append(wrap4096(smooth - cam_counts))

    missing = [i for i, vals in enumerate(bins) if not vals]
    if missing:
        print(f"  !! {len(missing)} bin(s) with no data (kept at 0): {missing}")
    avg = [sum(vals) / len(vals) if vals else 0.0 for vals in bins]

    # Rev-to-rev spread per bin, before despiking/mean-subtracting - the
    # noise floor this table is limited by (same idea as
    # calibration_lab.py's fullstep-accuracy rev0-vs-rev1 check, but camera-
    # referenced and per-bin rather than a single aggregate number).
    if revolutions >= 2:
        spreads = [max(vals) - min(vals) for vals in bins if len(vals) >= 2]
        if spreads:
            mean_spread = sum(spreads) / len(spreads)
            print(f"  rev-to-rev spread per bin: mean={mean_spread * 360 / 4096:.4f} deg  "
                  f"max={max(spreads) * 360 / 4096:.4f} deg (motor-shaft)")

    avg, spikes = despike(avg)
    if spikes:
        print(f"  despiked {len(spikes)} point(s) at index {spikes} - see fullstep_table.py's despike() docstring")
    mean_error = sum(avg) / len(avg)
    table = [v - mean_error for v in avg]

    rms = math.sqrt(sum(t * t for t in table) / len(table)) * 360.0 / 4096.0
    peak = max(abs(t) for t in table) * 360.0 / 4096.0
    print(f"\n  camera-referenced full-step table: shape RMS={rms:.4f} deg  peak={peak:.4f} deg (motor-shaft)")
    print(f"  mean offset {mean_error:.3f} counts folded out (C0 unchanged, same convention as fullstep_table.py)")

    return {
        "fullstepsPerRotation": n,
        "table": table,
        "rmsDeg": rms,
        "peakDeg": peak,
        "meanErrorCounts": mean_error,
        "deployedModel": {"C0": C0, "A": A, "B": B},
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sweep-dir", required=True, help="directory produced by camera_fullstep_sweep.py")
    ap.add_argument("--host", required=True, help="rotator IP - fetches the currently-deployed C0/A/B from it")
    ap.add_argument("--cols", type=int, default=9, help="printed squares across (generate_pattern.py default: 9)")
    ap.add_argument("--rows", type=int, default=8, help="printed squares down (generate_pattern.py default: 8)")
    ap.add_argument("--out", required=True, help="where to write the table JSON")
    args = ap.parse_args()

    result = build(args.sweep_dir, (args.cols - 1, args.rows - 1), args.host)
    with open(args.out, "w") as f:
        json.dump(result, f, indent=2)
    print(f"  wrote {args.out}")


if __name__ == "__main__":
    main()
