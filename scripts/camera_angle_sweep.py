"""Drives one full OUTPUT-shaft revolution while photographing the printed
checkerboard target through RotatorCam, to build an independent,
output-shaft-referenced ground truth for the rotator's angle behaviour.

Why a full output revolution specifically: the AS5600 sits on the MOTOR
shaft, upstream of the 10:1 reduction (see main/RotatorHW.cpp). One full
output revolution is REDUCTION motor revolutions - with REDUCTION=10, the
AS5600's own periodic error (period = 1 motor revolution) repeats 10 times
across the sweep, while any target-mounting eccentricity (period = 1 output
revolution, since it's a fixed function of the true output angle) appears
exactly once. That frequency separation - order ~10 and its harmonics vs.
order ~1 - is what scripts/camera_angle_analyze.py exploits to pull the two
error sources apart instead of only ever seeing them tangled together, and,
for the first time in this project, to fit the AS5600's own error model
against a real independent reference instead of the sensor's own reading
(see fit_fourier()'s self-referential-phase-basis comment in
calibration_lab.py - that structural bias is exactly what this rig exists to
finally get around).

Pure standard library - reuses jog() from calibration_lab.py and capture()
from camera_client.py rather than duplicating either transport.

Usage:
    python3 scripts/camera_angle_sweep.py --out-dir /tmp/sweep1
    python3 scripts/camera_angle_sweep.py --out-dir /tmp/dry --samples 8   # quick framing/timing check first

Returns the motor to its starting position (net zero motion) before exiting
normally; interrupting midway leaves the motor wherever it happens to be, same
convention as calibration_lab.py.
"""

import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from calibration_lab import jog, align_fullstep, FULLSTEPS_PER_ROTATION, MICROSTEPS, DEG_PER_COUNT  # noqa: E402
from camera_client import capture as camera_capture  # noqa: E402

REDUCTION = 10  # matches main/RotatorHW.cpp
MICROSTEPS_PER_OUTPUT_REV = FULLSTEPS_PER_ROTATION * MICROSTEPS * REDUCTION  # 1,024,000


JOG_LIMIT = 2 * FULLSTEPS_PER_ROTATION * MICROSTEPS  # matches JOG_LIMIT in main/WebServer.cpp's debug_jog_handler()


def jog_chunked(host, microsteps, samples):
    """Like jog(), but splits a move larger than the firmware's JOG_LIMIT into
    several calls - needed for the single full-revolution return-to-start
    move (1,024,000 microsteps), which the per-sweep-point moves never hit
    since those are already sized well under the limit. Returns the final
    stepPosition.
    """
    remaining = microsteps
    step_pos = None
    while remaining != 0:
        chunk = max(-JOG_LIMIT, min(JOG_LIMIT, remaining))
        step_pos = jog(host, chunk, samples)["stepPosition"]
        remaining -= chunk
    return step_pos


def cumulative_targets(total, n):
    """n+1 cumulative microstep targets spanning [0, total], as evenly spaced
    as integer microsteps allow. Round-then-diff (not floor-division) so the
    per-step deltas sum to exactly `total` with no drift, even though
    total/n is essentially never a whole number - matches the exactness
    concern in cmd_motor_sweep()'s full-step-offset comment.
    """
    return [round(i * total / n) for i in range(n + 1)]


def run_sweep(rotator_host, camera_host, out_dir, n_samples, sensor_samples, settle_s, capture_timeout,
              align_first=False):
    os.makedirs(out_dir, exist_ok=True)
    targets = cumulative_targets(MICROSTEPS_PER_OUTPUT_REV, n_samples)

    print(f"[camera-sweep] {n_samples} points over one full output revolution "
          f"({MICROSTEPS_PER_OUTPUT_REV} microsteps = {REDUCTION} motor revolutions)")
    print(f"  rotator={rotator_host}  camera={camera_host}  out_dir={out_dir}")

    if align_first:
        # Only a true full-step position is a genuine mechanical equilibrium
        # (see calibration_lab.py's align_fullstep() comment) - without this,
        # "start" is whatever microstep position the motor happened to be
        # holding, which is fine for the general sweep (it just sets an
        # arbitrary phase origin) but matters a lot for
        # camera_eccentricity_calibrate.py's probe points, which assume every
        # 400-full-step multiple lands on an identically-behaved equilibrium.
        start_pos = align_fullstep(rotator_host)["stepPosition"]
        print(f"  aligned to true full-step position {start_pos}")
    else:
        start = jog(rotator_host, 0, 1)
        start_pos = start["stepPosition"]
        print(f"  start stepPosition={start_pos}  hall={start['hall']}")

    frames = []
    t0 = time.time()
    for i in range(n_samples + 1):
        delta = targets[i] - (targets[i - 1] if i > 0 else 0)
        state = jog(rotator_host, delta, sensor_samples)
        if settle_s > 0:
            time.sleep(settle_s)

        img_name = f"frame_{i:04d}.jpg"
        img_path = os.path.join(out_dir, img_name)
        try:
            jpeg = camera_capture(camera_host, timeout=capture_timeout)
            with open(img_path, "wb") as f:
                f.write(jpeg)
            capture_ok = True
        except Exception as e:  # noqa: BLE001 - report and keep the sweep moving
            print(f"  !! capture failed at point {i}: {e}")
            capture_ok = False

        ideal_motor_deg = 360.0 * targets[i] / (FULLSTEPS_PER_ROTATION * MICROSTEPS)
        frames.append({
            "index": i,
            "cumulativeMicrosteps": targets[i],
            "idealMotorDeg": ideal_motor_deg,
            "idealOutputDeg": ideal_motor_deg / REDUCTION,
            "rawSensor": state["rawSensor"],
            "correctedSensor": state["correctedSensor"],
            "stepPosition": state["stepPosition"],
            "hall": state["hall"],
            "image": img_name if capture_ok else None,
        })

        if i % max(1, (n_samples + 1) // 20) == 0 or i == n_samples:
            elapsed = time.time() - t0
            rate = (i + 1) / elapsed if elapsed > 0 else 0
            eta = (n_samples - i) / rate if rate > 0 else float("nan")
            print(f"  point {i:4d}/{n_samples}  outputDeg={frames[-1]['idealOutputDeg']:7.3f}"
                  f"  corrSensor={state['correctedSensor']:8.2f}  hall={state['hall']}"
                  f"  ({elapsed:6.1f}s elapsed, ~{eta:5.1f}s left)")

    end_pos = jog_chunked(rotator_host, -MICROSTEPS_PER_OUTPUT_REV, 1)
    if end_pos != start_pos:
        print(f"  !! position mismatch after sweep: start={start_pos} end={end_pos} delta={end_pos - start_pos}")
    else:
        print(f"  position round-trip OK: {start_pos}")
    print(f"  swept in {time.time() - t0:.1f}s, returned to start")

    manifest = {
        "rotatorHost": rotator_host,
        "cameraHost": camera_host,
        "reduction": REDUCTION,
        "fullstepsPerRotation": FULLSTEPS_PER_ROTATION,
        "microsteps": MICROSTEPS,
        "microstepsPerOutputRev": MICROSTEPS_PER_OUTPUT_REV,
        "nSamples": n_samples,
        "sensorSamples": sensor_samples,
        "degPerCount": DEG_PER_COUNT,
        "startStepPosition": start_pos,
        "endStepPosition": end_pos,
        "frames": frames,
    }
    manifest_path = os.path.join(out_dir, "manifest.json")
    with open(manifest_path, "w") as f:
        json.dump(manifest, f, indent=2)
    print(f"  wrote {manifest_path}")
    failed = sum(1 for fr in frames if fr["image"] is None)
    if failed:
        print(f"  !! {failed}/{len(frames)} frames have no image (capture failures)")
    return manifest


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rotator-host", default="172.22.102.30")
    ap.add_argument("--camera-host", default="172.22.102.226")
    ap.add_argument("--out-dir", required=True, help="directory for frame_*.jpg + manifest.json")
    ap.add_argument("--samples", type=int, default=360,
                     help="grid points across the full output revolution (default: 360, "
                          "~1 output degree/point - Nyquist margin up to order ~150 in output-domain, "
                          "comfortably above the order-~40..60 range AS5600 harmonics land in once "
                          "referenced through the 10:1 reduction)")
    ap.add_argument("--sensor-samples", type=int, default=8, help="AS5600 averaging depth per point")
    ap.add_argument("--settle", type=float, default=0.2, help="seconds to wait after the move before photographing")
    ap.add_argument("--capture-timeout", type=float, default=15.0)
    ap.add_argument("--align-fullstep", action="store_true",
                     help="align to a true full-step position before starting (recommended for --samples 10, "
                          "camera_eccentricity_calibrate.py's dedicated probe - every point then lands on a "
                          "genuine mechanical equilibrium, not just an arbitrary microstep position)")
    args = ap.parse_args()

    run_sweep(args.rotator_host, args.camera_host, args.out_dir,
              args.samples, args.sensor_samples, args.settle, args.capture_timeout,
              align_first=args.align_fullstep)


if __name__ == "__main__":
    main()
