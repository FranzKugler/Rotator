"""Measures RotatorHW::gotoMechanicalZero()'s repeatability with an
independent, output-shaft-referenced ground truth: move to a random
position, home, photograph, repeat. An AS5600-only repeatability check
(reading correctedSensor after each home) can't tell real output-shaft
scatter apart from the routine's own systematic bias, since its edge search
is judged against the very sensor reading it's trying to reproduce - the
same reason the camera rig exists at all in this project (see
CALIBRATION_FINDINGS.md).

Needs main/WebServer.cpp's /api/debug/goto-mechanical-zero (added
alongside this script) - gotoMechanicalZero() previously only ran once, at
boot.

Usage:
    python3 scripts/homing_repeatability.py --out-dir /tmp/homing --trials 12
"""

import argparse
import json
import os
import random
import sys
import time
from urllib.request import Request, urlopen

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from calibration_lab import jog, FULLSTEPS_PER_ROTATION, MICROSTEPS  # noqa: E402
from camera_client import capture as camera_capture  # noqa: E402

REDUCTION = 10
MICROSTEPS_PER_OUTPUT_DEG = FULLSTEPS_PER_ROTATION * MICROSTEPS * REDUCTION / 360.0
JOG_LIMIT = 2 * FULLSTEPS_PER_ROTATION * MICROSTEPS  # matches main/WebServer.cpp's debug_jog_handler()


def goto_mechanical_zero(host, timeout=40):
    """POST /api/debug/goto-mechanical-zero - see main/WebServer.cpp. Blocks
    for the full homing routine (~20s: a ~10s hardcoded settle delay plus
    the Hall sweep and microstep edge search), hence the generous timeout.
    """
    req = Request(f"http://{host}/api/debug/goto-mechanical-zero", data=b"", method="POST")
    with urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read())


def wander(host, delta_deg):
    """Fast, open-loop move away from wherever we currently are - precision
    doesn't matter here, it only needs to put the routine's Hall/edge search
    in a meaningfully different starting state each trial. Chunked under
    JOG_LIMIT like camera_angle_sweep.py's return move.
    """
    remaining = round(delta_deg * MICROSTEPS_PER_OUTPUT_DEG)
    while remaining != 0:
        chunk = max(-JOG_LIMIT, min(JOG_LIMIT, remaining))
        jog(host, chunk, 1)
        remaining -= chunk


def run(rotator_host, camera_host, out_dir, trials, max_wander_deg, seed, capture_timeout, zero_timeout):
    os.makedirs(out_dir, exist_ok=True)
    rng = random.Random(seed)
    records = []
    t0 = time.time()

    for i in range(trials):
        wander_deg = rng.uniform(-max_wander_deg, max_wander_deg)
        if abs(wander_deg) < 2.0:  # a near-zero wander wouldn't meaningfully vary the approach
            wander_deg = 2.0 if wander_deg >= 0 else -2.0
        wander(rotator_host, wander_deg)

        state = goto_mechanical_zero(rotator_host, timeout=zero_timeout)

        img_name = f"zero_{i:03d}.jpg"
        capture_ok = True
        try:
            jpeg = camera_capture(camera_host, timeout=capture_timeout)
            with open(os.path.join(out_dir, img_name), "wb") as f:
                f.write(jpeg)
        except Exception as e:  # noqa: BLE001
            print(f"  !! capture failed at trial {i}: {e}")
            capture_ok = False

        rec = {
            "index": i, "wanderDeg": wander_deg, "stepPosition": state["stepPosition"],
            "correctedSensor": state["correctedSensor"], "hall": state["hall"],
            "image": img_name if capture_ok else None,
        }
        records.append(rec)
        print(f"  trial {i:2d}/{trials}: wander={wander_deg:+6.2f} deg  stepPosition={state['stepPosition']:6d}  "
              f"correctedSensor={state['correctedSensor']:7.3f}  hall={state['hall']}  "
              f"({time.time() - t0:.1f}s elapsed)")

    manifest = {"kind": "homing-repeatability", "trials": trials, "maxWanderDeg": max_wander_deg,
                "seed": seed, "records": records}
    with open(os.path.join(out_dir, "manifest.json"), "w") as f:
        json.dump(manifest, f, indent=2)
    print(f"  wrote {out_dir}/manifest.json ({time.time() - t0:.1f}s total)")
    return manifest


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rotator-host", default="172.22.102.30")
    ap.add_argument("--camera-host", default="172.22.102.226")
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--trials", type=int, default=12)
    ap.add_argument("--max-wander", type=float, default=25.0, help="max output degrees to move before each home")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--capture-timeout", type=float, default=15.0)
    ap.add_argument("--zero-timeout", type=float, default=40.0)
    args = ap.parse_args()

    run(args.rotator_host, args.camera_host, args.out_dir, args.trials, args.max_wander,
        args.seed, args.capture_timeout, args.zero_timeout)


if __name__ == "__main__":
    main()
