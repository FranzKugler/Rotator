"""Turns a scripts/camera_angle_sweep.py capture set into an independent,
output-shaft-referenced angle curve, and uses it to do two things the AS5600
alone structurally cannot:

1. Separate the two error sources the sweep was designed to distinguish
   (see camera_angle_sweep.py's docstring for why a full OUTPUT revolution
   makes this possible): target-mounting eccentricity, one cycle per sweep,
   from AS5600 sensor error, REDUCTION (10) cycles per sweep - via an FFT
   over the camera-vs-ideal residual.
2. Refit the AS5600's own harmonic error model with theta taken from the
   camera's independent measurement instead of the sensor's own raw reading
   - calibration_lab.py's fit_fourier() has to use the raw reading as its
   phase basis (correctSensorReading() never has anything else available at
   runtime), which is a known, deliberately-accepted self-referential bias.
   This script can finally check what that fit looks like against a real
   ground truth instead.

Needs numpy + opencv (opencv-python-headless) - NOT part of the ESP-IDF
dev container's pinned Python env, see scripts/requirements-camera-analysis.txt.
Run this in its own venv:
    python3 -m venv /tmp/camvenv && /tmp/camvenv/bin/pip install -r scripts/requirements-camera-analysis.txt
    /tmp/camvenv/bin/python scripts/camera_angle_analyze.py --sweep-dir /tmp/sweep1

Usage:
    python3 scripts/camera_angle_analyze.py --sweep-dir /tmp/sweep1 --out /tmp/sweep1/analysis.json
    python3 scripts/camera_angle_analyze.py --sweep-dir /tmp/sweep1 --check-frame frame_0000.jpg  # detector sanity check on one frame
"""

import argparse
import json
import math
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from calibration_lab import DEG_PER_COUNT, wrap4096  # noqa: E402

REDUCTION = 10


def find_corners(gray, pattern_size):
    """pattern_size = (inner corners per row, inner corners per column), i.e.
    (cols-1, rows-1) of the printed squares. Tries the modern, more robust
    detector first (OpenCV >=4.x), falls back to the classic one + manual
    sub-pixel refinement. Returns an (N,2) float array in OpenCV's own scan
    order, or None if the board wasn't found.
    """
    if hasattr(cv2, "findChessboardCornersSB"):
        ok, corners = cv2.findChessboardCornersSB(gray, pattern_size, flags=cv2.CALIB_CB_EXHAUSTIVE)
        if ok:
            return corners.reshape(-1, 2)

    ok, corners = cv2.findChessboardCorners(
        gray, pattern_size,
        flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not ok:
        return None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    return corners.reshape(-1, 2)


def rotation_step(prev_centered, cand_centered):
    """2D Procrustes, rotation-only: the angle (radians) that best rotates
    cand onto prev, plus the residual RMS distance after applying it - a
    closed-form single-angle fit (no scale/translation terms - both point
    sets are already centered on their own centroid and the physical board
    doesn't change size), via the complex-number identity
    sum(conj(prev_k) * cand_k) = |.| * exp(i*theta).
    """
    prev_c = prev_centered[:, 0] + 1j * prev_centered[:, 1]
    cand_c = cand_centered[:, 0] + 1j * cand_centered[:, 1]
    s = np.sum(np.conj(prev_c) * cand_c)
    theta = float(np.angle(s))
    rotated = cand_centered @ np.array([[math.cos(theta), -math.sin(theta)],
                                         [math.sin(theta), math.cos(theta)]]).T
    residual = float(np.sqrt(np.mean(np.sum((rotated - prev_centered) ** 2, axis=1))))
    return theta, residual


def track_rotation(sweep_dir, frames, pattern_size, verbose=True):
    """Walks the frames in capture order, resolving each one's corner
    ordering against the previous frame (see module docstring point 1's
    sibling problem: a plain checkerboard's corner order is only
    determined up to a start-end swap - i.e. reading the same physical
    grid backwards, corners[::-1] - not fully pinned down by an asymmetric
    row/col count on its own). Small per-step commanded rotations (the sweep
    is finely sampled) make the correct choice unambiguous: try both
    readings against the previous frame's already-resolved corners, keep
    whichever gives the smaller rotation.

    Returns a list of dicts (one per successfully detected frame): index,
    cameraOutputDeg (unwrapped, frame 0 = 0), residualPx.
    """
    results = []
    prev_centered = None
    cum_theta = 0.0  # radians, unwrapped
    n_failed = 0

    for i, fr in enumerate(frames):
        if fr["image"] is None:
            n_failed += 1
            continue
        path = os.path.join(sweep_dir, fr["image"])
        img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            print(f"  !! could not read {path}")
            n_failed += 1
            continue
        corners = find_corners(img, pattern_size)
        if corners is None:
            print(f"  !! no checkerboard found in {fr['image']} (index {i})")
            n_failed += 1
            continue

        centroid = corners.mean(axis=0)
        centered = corners - centroid

        if prev_centered is None:
            # Frame 0 defines both the canonical corner ordering and angle=0.
            prev_centered = centered
            results.append({"index": i, "cameraOutputDeg": 0.0, "residualPx": 0.0})
            continue

        theta_fwd, res_fwd = rotation_step(prev_centered, centered)
        theta_rev, res_rev = rotation_step(prev_centered, centered[::-1])
        if abs(theta_rev) < abs(theta_fwd):
            theta_step, residual, chosen = theta_rev, res_rev, centered[::-1]
        else:
            theta_step, residual, chosen = theta_fwd, res_fwd, centered

        cum_theta += theta_step
        prev_centered = chosen
        results.append({
            "index": i,
            "cameraOutputDeg": math.degrees(cum_theta),
            "residualPx": residual,
        })
        if verbose and abs(math.degrees(theta_step)) > 10:
            msg = f"  ! large per-frame step at index {i}: {math.degrees(theta_step):.2f} deg"
            if "idealOutputDeg" in fr and "idealOutputDeg" in frames[i - 1]:
                expected = fr["idealOutputDeg"] - frames[i - 1]["idealOutputDeg"]
                msg += f" (expected roughly {expected:.2f} deg) - check for a dropped/blurred frame"
            else:
                msg += " - check for a dropped/blurred frame (no idealOutputDeg on this manifest to compare against)"
            print(msg)

    if n_failed:
        print(f"  {n_failed}/{len(frames)} frames failed detection")
    return results


def dft_order_spectrum(residual_deg, max_order=25):
    """Magnitude of each integer cycles-per-sweep component in a residual
    sampled at (assumed) uniform angle steps over one full 0-360 deg sweep,
    via a plain real-input DFT - no assumption of a power-of-two length, and
    the residual list already excludes any frames the tracker dropped, which
    a resampling step below fixes so this DFT still sees a uniform grid.
    """
    n = len(residual_deg)
    x = np.asarray(residual_deg)
    orders = list(range(0, max_order + 1))
    mags = []
    for k in orders:
        theta = 2 * math.pi * k * np.arange(n) / n
        c = np.sum(x * np.cos(theta))
        s = np.sum(x * np.sin(theta))
        amp = 2.0 * math.hypot(c, s) / n if k > 0 else abs(c) / n
        mags.append(amp)
    return orders, mags


def fit_fourier_against_reference(ideal_motor_deg, sensor_counts, reference_motor_deg, kmax=6):
    """Same model as calibration_lab.py's fit_fourier() - C0 + sum(A_k cos + B_k
    sin) fit to (sensor - ideal) in AS5600 counts - but with theta taken from
    `reference_motor_deg` (the camera's independent measurement, converted to
    motor-domain degrees and wrapped to one motor revolution) instead of the
    sensor's own raw reading. This is the one change this whole rig exists to
    make possible; everything else in this function is deliberately identical
    to the self-referential version so the two residuals are comparable
    apples-to-apples.
    """
    n = len(ideal_motor_deg)
    ideal_counts = [4096.0 * (d % 360.0) / 360.0 for d in ideal_motor_deg]
    ref_counts = [4096.0 * (d % 360.0) / 360.0 for d in reference_motor_deg]
    sum0 = 0.0
    sumC = [0.0] * (kmax + 1)
    sumS = [0.0] * (kmax + 1)
    for ideal, raw, ref in zip(ideal_counts, sensor_counts, ref_counts):
        e = wrap4096(raw - ideal)
        sum0 += e
        theta = 2.0 * math.pi * ref / 4096.0
        for k in range(1, kmax + 1):
            sumC[k] += e * math.cos(k * theta)
            sumS[k] += e * math.sin(k * theta)
    C0 = sum0 / n
    A = [0.0] + [2.0 * sumC[k] / n for k in range(1, kmax + 1)]
    B = [0.0] + [2.0 * sumS[k] / n for k in range(1, kmax + 1)]
    return C0, A, B


def correct_with_ref(raw, ref_motor_deg, model):
    C0, A, B = model
    theta = 2.0 * math.pi * (ref_motor_deg % 360.0) / 360.0
    err = C0
    for k in range(1, len(A)):
        err += A[k] * math.cos(k * theta) + B[k] * math.sin(k * theta)
    return raw - err


def analyze(sweep_dir, pattern_size, kmax=6):
    with open(os.path.join(sweep_dir, "manifest.json")) as f:
        manifest = json.load(f)
    frames = manifest["frames"]

    print(f"[analyze] {len(frames)} frames, pattern inner-corner grid={pattern_size}")
    tracked = track_rotation(sweep_dir, frames, pattern_size)
    tracked_by_index = {t["index"]: t for t in tracked}

    # Only keep frames the tracker actually resolved, but carry the sweep's
    # own idealOutputDeg/correctedSensor along so every series stays aligned.
    rows = []
    for fr in frames:
        t = tracked_by_index.get(fr["index"])
        if t is None:
            continue
        rows.append({
            "index": fr["index"],
            "idealOutputDeg": fr["idealOutputDeg"],
            "idealMotorDeg": fr["idealMotorDeg"],
            "cameraOutputDeg": t["cameraOutputDeg"],
            "residualPx": t["residualPx"],
            "rawSensor": fr["rawSensor"],
            "correctedSensor": fr["correctedSensor"],
        })
    print(f"  {len(rows)}/{len(frames)} frames usable")
    if len(rows) < len(frames) * 0.9:
        print("  !! more than 10% of frames failed - treat results as provisional "
              "until framing/focus is fixed and the sweep is redone")

    # Anchor the camera curve's offset (not its scale - that's fixed by
    # construction, direct output-shaft measurement) so it starts at the same
    # value as the ideal ramp; only the shape of camera_residual matters below.
    offset = rows[0]["idealOutputDeg"] - rows[0]["cameraOutputDeg"]
    for r in rows:
        r["cameraOutputDeg"] += offset
        r["cameraResidualDeg"] = r["cameraOutputDeg"] - r["idealOutputDeg"]
        r["as5600OutputDeg"] = r["correctedSensor"] * DEG_PER_COUNT / REDUCTION
        r["as5600ResidualDeg"] = wrap4096(r["correctedSensor"] - 4096.0 * (r["idealMotorDeg"] % 360.0) / 360.0) \
            * DEG_PER_COUNT / REDUCTION

    # The on-device correction was calibrated against gotoMechanicalZero()'s
    # own homing reference, not against wherever THIS sweep's stepPosition
    # happened to start - so as5600ResidualDeg above carries a large but
    # constant (not periodic) offset between those two unrelated zero points,
    # the same way fit_fourier()'s own C0 absorbs an equivalent arbitrary
    # offset for THIS sweep's start. De-mean it so what's left is the
    # leftover ripple - the only part comparable to cameraResidualDeg's shape.
    as5600_mean = sum(r["as5600ResidualDeg"] for r in rows) / len(rows)
    for r in rows:
        r["as5600ResidualDeg"] -= as5600_mean

    camera_residuals = [r["cameraResidualDeg"] for r in rows]
    rms = math.sqrt(sum(e * e for e in camera_residuals) / len(camera_residuals))
    print(f"  camera residual (vs. commanded ideal): RMS={rms:.4f} deg  "
          f"peak={max(abs(e) for e in camera_residuals):.4f} deg")

    print("  order spectrum of the camera residual (cycles per sweep -> amplitude):")
    orders, mags = dft_order_spectrum(camera_residuals, max_order=min(25, len(rows) // 2))
    top = sorted(zip(orders, mags), key=lambda om: -om[1])[:8]
    for order, amp in top:
        tag = " <- eccentricity band" if order in (1, 2) else (
            " <- AS5600-coupled band" if order % REDUCTION == 0 and order > 0 else "")
        print(f"    order {order:2d}: {amp:.4f} deg{tag}")

    # --- refit the AS5600 model against the camera's independent reference ---
    ideal_motor = [r["idealMotorDeg"] for r in rows]
    raw_counts = [r["rawSensor"] for r in rows]
    camera_motor_ref = [r["cameraOutputDeg"] * REDUCTION for r in rows]  # output deg -> equivalent motor-domain deg

    model_camera_ref = fit_fourier_against_reference(ideal_motor, raw_counts, camera_motor_ref, kmax)
    errs = []
    for r, ref in zip(rows, camera_motor_ref):
        corrected = correct_with_ref(r["rawSensor"], ref, model_camera_ref)
        ideal_counts = 4096.0 * (r["idealMotorDeg"] % 360.0) / 360.0
        errs.append(wrap4096(corrected - ideal_counts) * DEG_PER_COUNT)
    rms_camera_ref = math.sqrt(sum(e * e for e in errs) / len(errs))
    peak_camera_ref = max(abs(e) for e in errs)
    print(f"  AS5600 fit refit against CAMERA reference (order {kmax}): "
          f"RMS={rms_camera_ref:.4f} deg  peak={peak_camera_ref:.4f} deg (motor-shaft degrees)")
    print(f"    C0={model_camera_ref[0]:.3f} counts")
    for k in range(1, kmax + 1):
        A, B = model_camera_ref[1][k], model_camera_ref[2][k]
        print(f"    A{k}={A:8.3f}  B{k}={B:8.3f}  amplitude={math.hypot(A, B):7.3f} counts")

    return {
        "sweepDir": sweep_dir,
        "patternSize": list(pattern_size),
        "rows": rows,
        "cameraResidualRmsDeg": rms,
        "orderSpectrum": {"orders": orders, "amplitudesDeg": mags},
        "as5600FitAgainstCamera": {
            "C0": model_camera_ref[0], "A": model_camera_ref[1], "B": model_camera_ref[2],
            "rmsDeg": rms_camera_ref, "peakDeg": peak_camera_ref, "kmax": kmax,
        },
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sweep-dir", required=True, help="directory produced by camera_angle_sweep.py")
    ap.add_argument("--cols", type=int, default=9, help="printed squares across (generate_pattern.py default: 9)")
    ap.add_argument("--rows", type=int, default=8, help="printed squares down (generate_pattern.py default: 8)")
    ap.add_argument("--out", default=None, help="write full per-frame results as JSON here")
    ap.add_argument("--check-frame", default=None,
                     help="just test corner detection on one frame (filename inside --sweep-dir) and exit")
    args = ap.parse_args()

    pattern_size = (args.cols - 1, args.rows - 1)

    if args.check_frame:
        path = os.path.join(args.sweep_dir, args.check_frame)
        img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            sys.exit(f"could not read {path}")
        print(f"image size: {img.shape[1]}x{img.shape[0]}")
        corners = find_corners(img, pattern_size)
        if corners is None:
            sys.exit(f"NO checkerboard found (pattern_size={pattern_size})")
        xs, ys = corners[:, 0], corners[:, 1]
        print(f"found {len(corners)} corners (expected {pattern_size[0] * pattern_size[1]})")
        print(f"  x range: {xs.min():.1f} .. {xs.max():.1f}  (image width {img.shape[1]})")
        print(f"  y range: {ys.min():.1f} .. {ys.max():.1f}  (image height {img.shape[0]})")
        margin = min(xs.min(), img.shape[1] - xs.max(), ys.min(), img.shape[0] - ys.max())
        print(f"  closest corner to frame edge: {margin:.1f}px")
        return

    result = analyze(args.sweep_dir, pattern_size)
    if args.out:
        with open(args.out, "w") as f:
            json.dump(result, f, indent=2)
        print(f"  wrote {args.out}")


if __name__ == "__main__":
    main()
