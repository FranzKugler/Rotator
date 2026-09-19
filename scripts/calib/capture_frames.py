"""Capture camera frames at commanded rotator positions.

Two modes, because the campaign needs two different kinds of truth:

  --mode steps   drives raw microsteps through /api/debug/jog. The commanded
                 increments are then mechanically exact by construction (a
                 full step is a full step), which is what the camera
                 calibration and the scale check want.

  --mode alpaca  drives ASCOM MoveAbsolute through scripts/alpaca_client.py -
                 the end-user path, closed loop, including whatever the
                 firmware's own correction and settling do. That is the only
                 honest way to run the final validation.
"""

import argparse
import json
import os
import random
import sys
import time
from datetime import datetime, timezone
from urllib.request import urlopen

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))
from rotator_io import Rotator, MICROSTEPS_PER_FULLSTEP, check_reachable


def grab(camera_host, path, timeout=20, retries=3):
    for attempt in range(retries):
        try:
            with urlopen(f"http://{camera_host}/capture", timeout=timeout) as resp:
                data = resp.read()
            if len(data) > 5000:
                with open(path, "wb") as handle:
                    handle.write(data)
                return len(data)
        except Exception as error:  # noqa: BLE001 - retry any transport hiccup
            last = error
            time.sleep(1.0)
    raise RuntimeError(f"camera capture failed after {retries} attempts: {last}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="172.22.102.30")
    ap.add_argument("--camera", default="172.22.102.226")
    ap.add_argument("--out", required=True, help="output directory")
    ap.add_argument("--mode", choices=["steps", "alpaca"], default="steps")
    ap.add_argument("--count", type=int, default=24, help="number of positions")
    ap.add_argument("--step-fullsteps", type=int, default=100,
                    help="steps mode: full steps between positions (100 = 9 deg out)")
    ap.add_argument("--range", type=float, default=180.0,
                    help="alpaca mode: positions are drawn from +/- this many degrees")
    ap.add_argument("--seed", type=int, default=20260918)
    ap.add_argument("--approach-deg", type=float, default=0.0,
                    help="alpaca mode: reach every target from the same side by "
                         "first moving this far past it in the negative direction. "
                         "Removes gear backlash, which is downstream of the sensor "
                         "and so invisible to the firmware's closed loop - measured "
                         "live at 64 mdeg peak-to-peak on random targets")
    ap.add_argument("--precompensate", nargs="*", default=None,
                    help="alpaca mode: gear_error.py --export JSON(s). Targets are "
                         "shifted by the measured output-shaft error before being "
                         "commanded, so the shaft lands where the target says "
                         "rather than where the sensor says")
    ap.add_argument("--settle", type=float, default=1.0,
                    help="seconds between the move finishing and the capture")
    ap.add_argument("--samples", type=int, default=8)
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    rotator = Rotator(args.host)
    check_reachable(rotator)

    manifest = open(os.path.join(args.out, "manifest.jsonl"), "w")
    manifest.write(json.dumps({
        "record": "__header__", "mode": args.mode, "host": args.host,
        "camera": args.camera, "utc": datetime.now(timezone.utc).isoformat(),
        "count": args.count, "stepFullsteps": args.step_fullsteps,
        "range": args.range, "seed": args.seed, "settle": args.settle,
        "precompensate": args.precompensate, "approachDeg": args.approach_deg,
    }) + "\n")
    manifest.flush()

    if args.mode == "steps":
        reference = rotator.align_fullstep()
        print(f"aligned to full step at {reference}")
        rotator.jog(8 * MICROSTEPS_PER_FULLSTEP, samples=1)  # run-up, forward
        targets = [i * args.step_fullsteps for i in range(args.count)]
    else:
        from alpaca_client import AlpacaClient
        from gear_model import GearModel
        client = AlpacaClient(args.host)
        client.put_connected(True)
        random.seed(args.seed)
        targets = [random.uniform(-args.range, args.range) for _ in range(args.count)]
        gear = (GearModel.average(args.precompensate) if args.precompensate else None)
        if gear:
            print(f"pre-compensating with {len(args.precompensate)} gear measurement(s)")

    previous = 0
    for index, target in enumerate(targets):
        if args.mode == "steps":
            delta = (target - previous) * MICROSTEPS_PER_FULLSTEP
            if delta:
                rotator.jog(delta, samples=1)
            previous = target
            commanded = {"fullsteps": target,
                         "outputDeg": target * 36.0 / 400.0}
        else:
            wanted = target % 360.0
            commanded_deg = gear.precompensate(wanted) % 360.0 if gear else wanted
            if args.approach_deg:
                client.move_absolute((commanded_deg - args.approach_deg) % 360.0)
                client.wait_until_stopped()
            client.move_absolute(commanded_deg)
            client.wait_until_stopped()
            commanded = {"alpacaTargetDeg": wanted,
                         "alpacaCommandedDeg": commanded_deg,
                         "precompensationDeg": commanded_deg - wanted if gear else 0.0,
                         "alpacaPositionDeg": client.get_position(),
                         "alpacaMechanicalDeg": client.get_mechanicalposition()}

        time.sleep(args.settle)
        reading = rotator.measure(samples=args.samples)
        name = f"frame_{index:04d}.jpg"
        size = grab(args.camera, os.path.join(args.out, name))
        record = {"record": "frame", "index": index, "file": name, "bytes": size,
                  "commanded": commanded, "rawSensor": reading["rawSensor"],
                  "correctedSensor": reading["correctedSensor"],
                  "stepPosition": reading["stepPosition"], "hall": reading["hall"],
                  "t": time.time()}
        manifest.write(json.dumps(record) + "\n")
        manifest.flush()
        print(f"  {index:3d}/{len(targets)}  {commanded}  raw={reading['rawSensor']:.3f}  "
              f"{size} B", flush=True)

    manifest.close()
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
