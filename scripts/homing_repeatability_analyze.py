"""Analyzes homing_repeatability.py's capture set: tracks each photo's true
output-shaft angle (checkerboard corner tracking, same method as
camera_angle_analyze.py) and reports how much the physical "mechanical
zero" position actually scattered trial to trial - independent of the
AS5600 the homing routine's own edge search is judged against.

Needs numpy/opencv - run in the same venv as camera_angle_analyze.py.

Usage:
    python3 scripts/homing_repeatability_analyze.py --dir /tmp/homing
"""

import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from camera_angle_analyze import track_rotation  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dir", required=True)
    ap.add_argument("--cols", type=int, default=9)
    ap.add_argument("--rows", type=int, default=8)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    with open(os.path.join(args.dir, "manifest.json")) as f:
        manifest = json.load(f)
    records = manifest["records"]
    pattern_size = (args.cols - 1, args.rows - 1)

    tracked = track_rotation(args.dir, records, pattern_size)
    print(f"  {len(tracked)}/{len(records)} frames usable")
    if len(tracked) < len(records) * 0.8:
        print("  !! more than 20% of frames failed - treat results as provisional")

    angles = [t["cameraOutputDeg"] for t in tracked]
    mean_a = sum(angles) / len(angles)
    centered = [a - mean_a for a in angles]
    rms = math.sqrt(sum(x * x for x in centered) / len(centered))
    peak = max(abs(x) for x in centered)
    spread = max(angles) - min(angles)

    sensor_by_index = {r["index"]: r for r in records}
    print(f"  camera-measured true zero position, {len(angles)} trials (deviation from their own mean):")
    for t, a in zip(tracked, centered):
        rec = sensor_by_index[t["index"]]
        print(f"    trial {t['index']:2d}: {a:+.4f} deg  (wander={rec['wanderDeg']:+6.2f} deg, "
              f"correctedSensor={rec['correctedSensor']:7.3f})")
    print(f"  mean={mean_a:.4f} deg  RMS scatter={rms:.4f} deg  peak deviation={peak:.4f} deg  "
          f"full spread={spread:.4f} deg")

    if args.out:
        with open(args.out, "w") as f:
            json.dump({"angles": angles, "meanDeg": mean_a, "rmsDeg": rms, "peakDeg": peak,
                       "spreadDeg": spread}, f, indent=2)
        print(f"  wrote {args.out}")


if __name__ == "__main__":
    main()
