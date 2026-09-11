"""Camera-referenced version of calibration_lab.py's fullstep-accuracy: one
photo + AS5600 reading per full step (400/motor revolution = 36 output
degrees, since REDUCTION=10), across `--revolutions` motor revolutions.

Why this exists alongside camera_angle_sweep.py: that sweep samples ~1
output degree/point (~11 full steps apart) - fine for the smooth, low-order
(kmax<=6) Fourier fit, but far too coarse to see the full-step-scale error
calibration_lab.py's fullstep-accuracy command found (order-6 fit residual
RMS=1.30 deg, peak=3.56 deg, motor-shaft, live-measured 2026-09-11) - that
error is repeatable rev-to-rev (shape correlation +1.000) but varies too
fast (period = 1 full step) for any reasonably-low harmonic order to
capture. scripts/fullstep_table.py already builds a raw per-full-step
lookup table from exactly that self-referential measurement; this script
captures the same shape of data but against the camera's independent
reference instead, so scripts/camera_fullstep_table.py can build that table
without the self-referential-phase-basis bias (see
CALIBRATION_FINDINGS.md and camera_angle_analyze.py's module docstring).

Pure standard library - reuses jog()/align_fullstep() from
calibration_lab.py and capture() from camera_client.py.

Usage:
    python3 scripts/camera_fullstep_sweep.py --out-dir /tmp/fullstep_sweep1 --revolutions 2
"""

import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from calibration_lab import DEG_PER_COUNT, FULLSTEPS_PER_ROTATION, MICROSTEPS, align_fullstep, jog  # noqa: E402
from camera_client import capture as camera_capture  # noqa: E402

REDUCTION = 10  # matches main/RotatorHW.cpp
JOG_LIMIT = 2 * FULLSTEPS_PER_ROTATION * MICROSTEPS  # matches WebServer.cpp's debug_jog_handler()


def jog_chunked(host, microsteps, samples):
    """Same reason as camera_angle_sweep.py's own jog_chunked(): the single
    return-to-start move can exceed JOG_LIMIT even though every per-point
    move here (one full step = MICROSTEPS) never does. Returns the final
    stepPosition (not the whole jog() response dict)."""
    remaining = microsteps
    step_pos = None
    while remaining != 0:
        chunk = max(-JOG_LIMIT, min(JOG_LIMIT, remaining))
        step_pos = jog(host, chunk, samples)["stepPosition"]
        remaining -= chunk
    return step_pos


def run_sweep(rotator_host, camera_host, out_dir, revolutions, sensor_samples, settle_s, capture_timeout):
    os.makedirs(out_dir, exist_ok=True)
    n = FULLSTEPS_PER_ROTATION
    print(f"[camera-fullstep-sweep] {revolutions} revolution(s) x {n} full steps "
          f"({n * revolutions} points total)")
    print(f"  rotator={rotator_host}  camera={camera_host}  out_dir={out_dir}")

    start_pos = align_fullstep(rotator_host)["stepPosition"]
    print(f"  aligned to true full-step position {start_pos}")

    frames = []
    t0 = time.time()
    index = 0
    for rev in range(revolutions):
        for i in range(n):
            state = jog(rotator_host, MICROSTEPS, sensor_samples)
            if settle_s > 0:
                time.sleep(settle_s)

            img_name = f"frame_{index:04d}.jpg"
            img_path = os.path.join(out_dir, img_name)
            capture_ok = True
            try:
                jpeg = camera_capture(camera_host, timeout=capture_timeout)
                with open(img_path, "wb") as f:
                    f.write(jpeg)
            except Exception as e:  # noqa: BLE001 - report and keep the sweep moving
                print(f"  !! capture failed at rev {rev} step {i}: {e}")
                capture_ok = False

            ideal_motor_deg = 360.0 * i / n
            frames.append({
                "index": index,
                "rev": rev,
                "stepIndex": i,
                "idealMotorDeg": ideal_motor_deg,
                "idealOutputDeg": ideal_motor_deg / REDUCTION,
                "rawSensor": state["rawSensor"],
                "stepPosition": state["stepPosition"],
                "hall": state["hall"],
                "image": img_name if capture_ok else None,
            })

            if i % 100 == 0 or (rev == revolutions - 1 and i == n - 1):
                elapsed = time.time() - t0
                print(f"  rev {rev} step {i:4d}/{n}  raw={state['rawSensor']:8.2f}  ({elapsed:6.1f}s elapsed)")
            index += 1

    print(f"  captured in {time.time() - t0:.1f}s")

    def write_manifest(end_pos):
        manifest = {
            "rotatorHost": rotator_host,
            "cameraHost": camera_host,
            "reduction": REDUCTION,
            "fullstepsPerRotation": n,
            "microsteps": MICROSTEPS,
            "revolutions": revolutions,
            "sensorSamples": sensor_samples,
            "degPerCount": DEG_PER_COUNT,
            "startStepPosition": start_pos,
            "endStepPosition": end_pos,
            "frames": frames,
        }
        manifest_path = os.path.join(out_dir, "manifest.json")
        with open(manifest_path, "w") as f:
            json.dump(manifest, f, indent=2)
        return manifest, manifest_path

    # Written BEFORE the return-to-start move, not after - a failure in that
    # move (or in this function's own bookkeeping) must not cost the frames
    # and readings already safely captured. Live-caught losing exactly this
    # (2026-09-11): a bug in this function's own end-of-sweep print crashed
    # after 800/800 points were captured but before the manifest existed,
    # discarding an otherwise-complete ~12 minute capture.
    manifest, manifest_path = write_manifest(end_pos=None)
    print(f"  wrote {manifest_path} (before returning to start, see comment above)")
    failed = sum(1 for fr in frames if fr["image"] is None)
    if failed:
        print(f"  !! {failed}/{len(frames)} frames have no image (capture failures)")

    end_pos = jog_chunked(rotator_host, -revolutions * n * MICROSTEPS, 1)
    if end_pos != start_pos:
        print(f"  !! position mismatch after sweep: start={start_pos} end={end_pos} delta={end_pos - start_pos}")
    else:
        print(f"  position round-trip OK: {start_pos}")
    manifest, manifest_path = write_manifest(end_pos)
    print(f"  updated {manifest_path} with the return-to-start result")
    return manifest


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rotator-host", default="172.22.102.30")
    ap.add_argument("--camera-host", default="172.22.102.226")
    ap.add_argument("--out-dir", required=True, help="directory for frame_*.jpg + manifest.json")
    ap.add_argument("--revolutions", type=int, default=2,
                     help="motor revolutions to repeat (>=2 recommended for averaging/noise estimate, "
                          "matching calibration_lab.py's fullstep-accuracy)")
    ap.add_argument("--sensor-samples", type=int, default=16, help="AS5600 averaging depth per point")
    ap.add_argument("--settle", type=float, default=0.2, help="seconds to wait after the move before photographing")
    ap.add_argument("--capture-timeout", type=float, default=15.0)
    args = ap.parse_args()

    run_sweep(args.rotator_host, args.camera_host, args.out_dir,
              args.revolutions, args.sensor_samples, args.settle, args.capture_timeout)


if __name__ == "__main__":
    main()
