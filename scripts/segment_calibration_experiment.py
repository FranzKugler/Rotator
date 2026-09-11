"""Tests Franz's hypothesis: the belt pulley driving the OUTPUT stage might
itself be eccentric or unevenly deformed, so a given motor-shaft angle might
not behave identically depending on which of the 10 motor revolutions (one
per REDUCTION segment across a full output revolution) it falls in - in
which case ten segment-specific AS5600 Fourier fits should out-perform the
single global fit camera_angle_analyze.py computes.

Reuses the full-revolution sweep scripts/camera_angle_sweep.py already
captured (main/RotatorHW.cpp's KMAX=4 harmonics, same as the firmware
supports) - no new hardware run needed, this is pure re-analysis.

The obvious trap: 10 segments x 9 free parameters (C0 + 4 harmonics) is 90
parameters fit against the same ~360 points a single 9-parameter global fit
already uses. An in-sample residual improves with 10x the parameters
whether or not Franz's hypothesis is true - that would just be measuring
overfitting, not the pulley. So this does a HELD-OUT comparison instead:
every other frame (even index) trains each model, the other half (odd
index) scores it, and only that held-out number is reported as the answer.
With ~18 held-in points per segment, 9 parameters is already a thin fit -
if segmenting overfits, the held-out residual will show it directly, not
just look suspiciously good.

Usage:
    /tmp/camvenv/bin/python scripts/segment_calibration_experiment.py \\
        --sweep-dir /tmp/camera_cal_sweep1 --kmax 4
"""

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from camera_angle_analyze import correct_with_ref, fit_fourier_against_reference, track_rotation, wrap4096  # noqa: E402

REDUCTION = 10  # matches main/RotatorHW.cpp


def build_rows(sweep_dir, pattern_size):
    """Same camera-anchoring idiom as camera_angle_analyze.py's analyze():
    track_rotation() gives a relative curve, anchored here to the sweep's
    own idealOutputDeg at frame 0 so only its shape (not its offset) is
    compared against."""
    with open(os.path.join(sweep_dir, "manifest.json")) as f:
        manifest = json.load(f)
    frames = manifest["frames"]

    # camera_angle_sweep.py's last frame closes the loop back to frame 0's
    # physical position (n_samples+1 frames span one full revolution
    # inclusive of both ends) - a real, deliberate round-trip check for that
    # tool, but a near-duplicate sample here that made segment 9 uniquely
    # contain both its own start AND this closing frame at nearly the same
    # phase. With only ~19 points feeding a 9-parameter fit, that duplicate
    # was enough to make the Fourier extraction blow up (live-caught:
    # A1..4 ~70-77 counts vs ~2-6 everywhere else) - drop it before
    # segmenting, same as any periodic-data analysis would.
    if len(frames) > 1 and abs(frames[-1]["idealOutputDeg"] - frames[0]["idealOutputDeg"] - 360.0) < 1e-6:
        frames = frames[:-1]

    tracked = {t["index"]: t for t in track_rotation(sweep_dir, frames, pattern_size)}
    rows = []
    for fr in frames:
        t = tracked.get(fr["index"])
        if t is None:
            continue
        rows.append({
            "index": fr["index"],
            "idealMotorDeg": fr["idealMotorDeg"],
            "rawSensor": fr["rawSensor"],
            "cameraOutputDeg": t["cameraOutputDeg"],
        })
    offset = frames[0]["idealOutputDeg"] - rows[0]["cameraOutputDeg"]
    for r in rows:
        r["cameraOutputDeg"] += offset
        r["cameraMotorDeg"] = r["cameraOutputDeg"] * REDUCTION
        r["segment"] = min(int(r["idealMotorDeg"] // 360.0), REDUCTION - 1)
    return rows


def residual_deg(row, model):
    """Motor-shaft degrees of AS5600 error against the camera reference,
    for one row under one (C0, A, B) model - same maths as
    camera_angle_analyze.py's analyze(), just factored out for reuse here
    on arbitrary row subsets."""
    corrected = correct_with_ref(row["rawSensor"], row["cameraMotorDeg"], model)
    ideal_counts = 4096.0 * (row["idealMotorDeg"] % 360.0) / 360.0
    return wrap4096(corrected - ideal_counts) * (360.0 / 4096.0)


def fit_and_score(train_rows, test_rows, kmax):
    if len(train_rows) <= kmax * 2 + 1:
        return None, []  # not even enough points to constrain the fit
    model = fit_fourier_against_reference(
        [r["idealMotorDeg"] for r in train_rows],
        [r["rawSensor"] for r in train_rows],
        [r["cameraMotorDeg"] for r in train_rows],
        kmax,
    )
    errs = [residual_deg(r, model) for r in test_rows]
    return model, errs


def rms_peak(errs):
    if not errs:
        return float("nan"), float("nan")
    rms = (sum(e * e for e in errs) / len(errs)) ** 0.5
    return rms, max(abs(e) for e in errs)


def run(sweep_dir, pattern_size, kmax):
    rows = build_rows(sweep_dir, pattern_size)
    print(f"[segment-experiment] {len(rows)} usable frames, kmax={kmax}, {REDUCTION} segments")

    train = [r for r in rows if r["index"] % 2 == 0]
    test = [r for r in rows if r["index"] % 2 == 1]
    print(f"  {len(train)} train (even index) / {len(test)} test (odd index) frames overall")

    # --- baseline: one global fit, scored on the held-out half ---
    _, global_errs = fit_and_score(train, test, kmax)
    global_rms, global_peak = rms_peak(global_errs)
    print(f"\n  GLOBAL fit (1 x {2 * kmax + 1} params), held-out residual: "
          f"RMS={global_rms:.4f} deg  peak={global_peak:.4f} deg  (motor-shaft degrees, n={len(global_errs)})")

    # --- experiment: one fit per segment, each scored on its own held-out half ---
    print(f"\n  SEGMENTED fit ({REDUCTION} x {2 * kmax + 1} params), held-out residual per segment:")
    all_segment_errs = []
    for seg in range(REDUCTION):
        seg_train = [r for r in train if r["segment"] == seg]
        seg_test = [r for r in test if r["segment"] == seg]
        model, errs = fit_and_score(seg_train, seg_test, kmax)
        if model is None:
            print(f"    segment {seg}: only {len(seg_train)} train points - skipped (too few for kmax={kmax})")
            continue
        rms, peak = rms_peak(errs)
        all_segment_errs.extend(errs)
        print(f"    segment {seg}: RMS={rms:.4f} deg  peak={peak:.4f} deg  "
              f"(n_train={len(seg_train)}, n_test={len(seg_test)})")

    seg_rms, seg_peak = rms_peak(all_segment_errs)
    print(f"\n  SEGMENTED fit, combined held-out residual: RMS={seg_rms:.4f} deg  peak={seg_peak:.4f} deg  "
          f"(n={len(all_segment_errs)})")

    print(f"\n  verdict: segmented is {'BETTER' if seg_rms < global_rms else 'WORSE (or no better)'} than global "
          f"on held-out data ({seg_rms:.4f} vs {global_rms:.4f} deg RMS)")
    if seg_rms < global_rms:
        print("  (this is the held-out number, not an in-sample one - a real improvement, not just more parameters "
              "fitting noise)")

    return {
        "global": {"rmsDeg": global_rms, "peakDeg": global_peak, "n": len(global_errs)},
        "segmented": {"rmsDeg": seg_rms, "peakDeg": seg_peak, "n": len(all_segment_errs)},
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sweep-dir", required=True, help="directory produced by camera_angle_sweep.py")
    ap.add_argument("--cols", type=int, default=9, help="printed squares across (generate_pattern.py default: 9)")
    ap.add_argument("--rows", type=int, default=8, help="printed squares down (generate_pattern.py default: 8)")
    ap.add_argument("--kmax", type=int, default=4, help="harmonics per fit (default 4, matches main/RotatorHW.h's KMAX)")
    args = ap.parse_args()

    run(args.sweep_dir, (args.cols - 1, args.rows - 1), args.kmax)


if __name__ == "__main__":
    main()
