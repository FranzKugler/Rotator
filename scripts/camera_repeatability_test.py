"""Isolates the camera's OWN measurement noise/bias from real rotator
motion: captures N photos in a row at a completely FIXED, unmoving rotator
position, then runs the same checkerboard-corner-correspondence analysis
(camera_angle_analyze.py's track_rotation()) every other camera-based
measurement in this project relies on.

Since the rotator does not move at all between frames, any "rotation" this
reports is pure measurement noise/bias in the camera + corner-detection
pipeline (JPEG compression, sub-pixel corner jitter, lens distortion
interacting with vibration, etc.) - not the rotator. A direct, decisive way
to find out whether the ~0.1-0.15 deg residuals this project's camera-based
verifications keep landing on are a real rotator/calibration accuracy limit,
or an artifact of the reference measurement itself.

Needs opencv/numpy - run in the throwaway venv, same as
camera_angle_analyze.py:
    python3 -m venv /tmp/camvenv && /tmp/camvenv/bin/pip install -r scripts/requirements-camera-analysis.txt

Usage:
    /tmp/camvenv/bin/python scripts/camera_repeatability_test.py \\
        --camera-host 172.22.102.226 --out-dir /tmp/cam_repeat --n 30
"""

import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from camera_angle_analyze import track_rotation  # noqa: E402
from camera_client import capture as camera_capture  # noqa: E402


def run(camera_host, out_dir, n, capture_timeout, interval_s, pattern_size):
    os.makedirs(out_dir, exist_ok=True)
    print(f"[camera-repeatability] {n} frames at a fixed, unmoving position, "
          f"interval={interval_s}s, pattern={pattern_size}")

    frames = []
    t0 = time.time()
    for i in range(n):
        img_name = f"frame_{i:04d}.jpg"
        jpeg = camera_capture(camera_host, timeout=capture_timeout)
        with open(os.path.join(out_dir, img_name), "wb") as f:
            f.write(jpeg)
        # idealOutputDeg is always 0 - nothing ever moves in this test, so
        # track_rotation()'s "large per-frame step" warning (calibrated for
        # a real sweep's expected step size) does not apply and is
        # deliberately left silent here by never differing from 0.
        frames.append({"index": i, "image": img_name, "idealOutputDeg": 0.0})
        if interval_s > 0:
            time.sleep(interval_s)
    print(f"  captured {n} frames in {time.time() - t0:.1f}s")

    manifest = {"kind": "camera_repeatability", "n": n, "cameraHost": camera_host, "frames": frames}
    with open(os.path.join(out_dir, "manifest.json"), "w") as f:
        json.dump(manifest, f, indent=2)

    tracked = track_rotation(out_dir, frames, pattern_size)
    if len(tracked) < n:
        print(f"  !! only {len(tracked)}/{n} frames had a detectable checkerboard")
    angles = [t["cameraOutputDeg"] for t in tracked]
    residuals_px = [t["residualPx"] for t in tracked]

    mean = sum(angles) / len(angles)
    rms = (sum((a - mean) ** 2 for a in angles) / len(angles)) ** 0.5
    peak_to_peak = max(angles) - min(angles)
    print(f"\n  apparent rotation with the rotator NOT moving at all: "
          f"RMS={rms:.4f} deg  peak-to-peak={peak_to_peak:.4f} deg  (n={len(angles)})")
    print(f"  corner reprojection residual (px): mean={sum(residuals_px) / len(residuals_px):.2f}  "
          f"max={max(residuals_px):.2f}")
    print("\n  this is the camera+corner-detection pipeline's own noise floor - any project figure at or "
          "below this is indistinguishable from camera noise, not a real rotator/calibration result.")

    result = {"n": len(angles), "rmsDeg": rms, "peakToPeakDeg": peak_to_peak, "angles": angles}
    with open(os.path.join(out_dir, "result.json"), "w") as f:
        json.dump(result, f, indent=2)
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--camera-host", default="172.22.102.226")
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--n", type=int, default=30)
    ap.add_argument("--interval", type=float, default=1.0, help="seconds between captures")
    ap.add_argument("--capture-timeout", type=float, default=15.0)
    ap.add_argument("--cols", type=int, default=9, help="printed squares across (generate_pattern.py default: 9)")
    ap.add_argument("--rows", type=int, default=8, help="printed squares down (generate_pattern.py default: 8)")
    args = ap.parse_args()

    run(args.camera_host, args.out_dir, args.n, args.capture_timeout, args.interval, (args.cols - 1, args.rows - 1))


if __name__ == "__main__":
    main()
