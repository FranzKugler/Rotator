"""Two experiments the camera rig + PI controller together finally make
possible:

1. `backlash`: visit N fixed output-shaft positions, each once approached
   clockwise (cw) and once counterclockwise (ccw), photographing after each.
   The AS5600 sits on the motor shaft, upstream of the 10:1 reduction - it
   cannot see backlash in the output stage (belt/gears) at all, only motor/
   driver-side hysteresis (already characterized as small, see
   calibration_lab.py's `backlash` command). The camera can, because it
   watches the actual output shaft: whatever difference remains between the
   cw- and ccw-approached camera angle at the SAME nominal target is real
   output-stage backlash.

2. `random`: visit N random relative targets, direction (cw/ccw) chosen at
   random each time, photographing after each - a broad, honest accuracy
   check of the closed-loop controller against independent ground truth
   across many positions and both directions at once, rather than the
   carefully-chosen points every earlier measurement in this project used.

Both approach a target by first moving PAST it by `--overshoot` degrees
(a fast, open-loop jog - precision doesn't matter here, it only needs to
land on the correct side) and then PI-converging back onto it from that
side (scripts/pi_position_control.py's goto_relative()) - so the final,
precision-relevant move always happens in the intended direction,
consistently engaging (or not engaging) whatever backlash exists.

Position tracking is a pure open-loop odometer (sum of every commanded
microstep, converted to output degrees) - exact by construction, not
dependent on the AS5600 at all. That's the "ideal" the camera's independent
measurement (via camera_angle_analyze.py's track_rotation(), run separately -
this script only captures) gets compared against.

Usage:
    python3 scripts/backlash_and_random_validation.py --rotator-host 172.22.102.30 \\
        --camera-host 172.22.102.226 --out-dir /tmp/backlash backlash --positions 6
    python3 scripts/backlash_and_random_validation.py --rotator-host 172.22.102.30 \\
        --camera-host 172.22.102.226 --out-dir /tmp/random100 random --n 100
"""

import argparse
import json
import os
import random
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from calibration_lab import jog, FULLSTEPS_PER_ROTATION, MICROSTEPS  # noqa: E402
from camera_client import capture as camera_capture  # noqa: E402
from pi_position_control import goto_relative, REDUCTION  # noqa: E402

MICROSTEPS_PER_OUTPUT_DEG = FULLSTEPS_PER_ROTATION * MICROSTEPS * REDUCTION / 360.0


def visit(rotator_host, camera_host, out_dir, index, cumulative_ideal_deg,
          delta_from_current, approach, overshoot_deg, pi_kwargs, capture_timeout):
    """Moves by `delta_from_current` output degrees relative to wherever the
    rotator currently is, arriving via `approach` ('cw' or 'ccw'), then
    photographs. Returns (new cumulative_ideal_deg, record dict).
    """
    sign = 1.0 if approach == "cw" else -1.0
    rough_deg = delta_from_current - sign * overshoot_deg
    rough_microsteps = round(rough_deg * MICROSTEPS_PER_OUTPUT_DEG)
    jog(rotator_host, rough_microsteps, 1)  # fast, open-loop - only needs to land on the right side

    final_result = goto_relative(rotator_host, sign * overshoot_deg, **pi_kwargs)

    net_deg = rough_microsteps / MICROSTEPS_PER_OUTPUT_DEG + final_result["netMicrosteps"] / MICROSTEPS_PER_OUTPUT_DEG
    cumulative_ideal_deg += net_deg

    img_name = f"visit_{index:04d}.jpg"
    img_path = os.path.join(out_dir, img_name)
    capture_ok = True
    try:
        jpeg = camera_capture(camera_host, timeout=capture_timeout)
        with open(img_path, "wb") as f:
            f.write(jpeg)
    except Exception as e:  # noqa: BLE001
        print(f"  !! capture failed at visit {index}: {e}")
        capture_ok = False

    record = {
        "index": index, "approach": approach, "idealOutputDeg": cumulative_ideal_deg,
        "as5600ConfirmedErrorDeg": final_result["confirmedErrorDeg"],
        "iterations": len(final_result["history"]) - 1,
        "image": img_name if capture_ok else None,
    }
    return cumulative_ideal_deg, record


def run_backlash(rotator_host, camera_host, out_dir, positions, span_deg, overshoot_deg, pi_kwargs, capture_timeout):
    os.makedirs(out_dir, exist_ok=True)
    targets = [i * span_deg / (positions - 1) for i in range(positions)] if positions > 1 else [0.0]
    print(f"[backlash] {positions} position(s) across {span_deg} deg, cw+ccw each ({positions * 2} visits)")

    cum = 0.0
    records = []
    t0 = time.time()
    for i, target in enumerate(targets):
        for approach in ("cw", "ccw"):
            delta = target - cum
            cum, rec = visit(rotator_host, camera_host, out_dir, len(records), cum,
                              delta, approach, overshoot_deg, pi_kwargs, capture_timeout)
            records.append(rec)
            print(f"  visit {rec['index']:3d}  target~{target:+.3f} deg  approach={approach:3s}  "
                  f"iterations={rec['iterations']}  as5600_err={rec['as5600ConfirmedErrorDeg']:+.4f} deg  "
                  f"({time.time() - t0:.1f}s elapsed)")

    # net zero motion at the end
    return_microsteps = round(-cum * MICROSTEPS_PER_OUTPUT_DEG)
    jog(rotator_host, return_microsteps, 1)
    print(f"  returned {return_microsteps} microsteps to starting position ({time.time() - t0:.1f}s total)")

    manifest = {"kind": "backlash", "positions": positions, "spanDeg": span_deg,
                "overshootDeg": overshoot_deg, "records": records}
    with open(os.path.join(out_dir, "manifest.json"), "w") as f:
        json.dump(manifest, f, indent=2)
    print(f"  wrote {out_dir}/manifest.json")
    return manifest


def run_random(rotator_host, camera_host, out_dir, n, max_step_deg, overshoot_deg, seed, pi_kwargs, capture_timeout):
    os.makedirs(out_dir, exist_ok=True)
    rng = random.Random(seed)
    print(f"[random] {n} random visit(s), step<=+/-{max_step_deg} deg, random cw/ccw, seed={seed}")

    cum = 0.0
    records = []
    t0 = time.time()
    for i in range(n):
        delta = rng.uniform(-max_step_deg, max_step_deg)
        approach = rng.choice(("cw", "ccw"))
        cum, rec = visit(rotator_host, camera_host, out_dir, i, cum, delta, approach,
                          overshoot_deg, pi_kwargs, capture_timeout)
        records.append(rec)
        if i % 10 == 0 or i == n - 1:
            print(f"  visit {i:3d}/{n}  cumulative~{cum:+.3f} deg  approach={approach:3s}  "
                  f"iterations={rec['iterations']}  as5600_err={rec['as5600ConfirmedErrorDeg']:+.4f} deg  "
                  f"({time.time() - t0:.1f}s elapsed)")

    return_microsteps = round(-cum * MICROSTEPS_PER_OUTPUT_DEG)
    jog(rotator_host, return_microsteps, 1)
    print(f"  returned {return_microsteps} microsteps to starting position ({time.time() - t0:.1f}s total)")

    manifest = {"kind": "random", "n": n, "maxStepDeg": max_step_deg, "seed": seed,
                "overshootDeg": overshoot_deg, "records": records}
    with open(os.path.join(out_dir, "manifest.json"), "w") as f:
        json.dump(manifest, f, indent=2)
    print(f"  wrote {out_dir}/manifest.json")
    return manifest


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rotator-host", default="172.22.102.30")
    ap.add_argument("--camera-host", default="172.22.102.226")
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--overshoot", type=float, default=0.5, help="degrees to overshoot before the final approach")
    ap.add_argument("--kp", type=float, default=0.9)
    ap.add_argument("--ki", type=float, default=0.15)
    ap.add_argument("--tolerance", type=float, default=0.005)
    ap.add_argument("--samples", type=int, default=8)
    ap.add_argument("--settle", type=float, default=0.15)
    ap.add_argument("--capture-timeout", type=float, default=15.0)
    sub = ap.add_subparsers(dest="command", required=True)

    b = sub.add_parser("backlash")
    b.add_argument("--positions", type=int, default=6)
    b.add_argument("--span", type=float, default=20.0, help="output degrees the positions spread across")

    r = sub.add_parser("random")
    r.add_argument("--n", type=int, default=100)
    r.add_argument("--max-step", type=float, default=3.0, help="max output degrees per random move")
    r.add_argument("--seed", type=int, default=42)

    args = ap.parse_args()
    pi_kwargs = dict(kp=args.kp, ki=args.ki, tolerance_deg=args.tolerance, sensor_samples=args.samples,
                      settle_s=args.settle)

    if args.command == "backlash":
        run_backlash(args.rotator_host, args.camera_host, args.out_dir, args.positions, args.span,
                     args.overshoot, pi_kwargs, args.capture_timeout)
    elif args.command == "random":
        run_random(args.rotator_host, args.camera_host, args.out_dir, args.n, args.max_step,
                   args.overshoot, args.seed, pi_kwargs, args.capture_timeout)


if __name__ == "__main__":
    main()
