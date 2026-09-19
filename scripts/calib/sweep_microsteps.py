"""Step 4: measure what the driver's microsteps actually do, using the
now-characterised sensor.

The difficulty is resolution. One microstep is 4096/(400*256) = 0.04 sensor
counts, so the AS5600 cannot see a single microstep at all - at rest it
reports an exact integer. The way through is dither: sample the same
microstep *phase* (0..255 within a full step) at many different full steps.
Each full step sits at a different, essentially arbitrary sub-count offset,
so the quantisation error decorrelates across them and averaging by phase
recovers the systematic part far below one count.

With `--phase-divisions P` over `--fullsteps N`, each phase bin gets N
samples, and the quantisation floor falls to about 1/sqrt(12*N) counts.
"""

import argparse
import json
import sys
import time
from datetime import datetime, timezone

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from rotator_io import Rotator, MICROSTEPS_PER_FULLSTEP, check_reachable


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="172.22.102.30")
    ap.add_argument("--out", required=True)
    ap.add_argument("--fullsteps", type=int, default=60,
                    help="how many full steps the sweep covers")
    ap.add_argument("--phase-divisions", type=int, default=16,
                    help="samples per full step (must divide 256)")
    ap.add_argument("--samples", type=int, default=4)
    ap.add_argument("--settle", type=float, default=0.2)
    ap.add_argument("--runup", type=int, default=8)
    ap.add_argument("--echo", type=int, default=50)
    args = ap.parse_args()

    if MICROSTEPS_PER_FULLSTEP % args.phase_divisions:
        sys.exit(f"--phase-divisions must divide {MICROSTEPS_PER_FULLSTEP}")
    increment = MICROSTEPS_PER_FULLSTEP // args.phase_divisions

    rotator = Rotator(args.host)
    check_reachable(rotator)
    reference = rotator.align_fullstep()
    rotator.jog(args.runup * MICROSTEPS_PER_FULLSTEP, samples=1)
    origin = rotator.measure(samples=1)["stepPosition"]
    print(f"aligned at {reference}, run-up done, origin stepPosition={origin}")

    total = args.fullsteps * args.phase_divisions
    with open(args.out, "w") as out:
        out.write(json.dumps({
            "record": "__header__", "utc": datetime.now(timezone.utc).isoformat(),
            "host": args.host, "fullsteps": args.fullsteps,
            "phaseDivisions": args.phase_divisions, "increment": increment,
            "samples": args.samples, "settle": args.settle, "origin": origin,
            "diagnostics": rotator.sensor_diagnostics(),
        }) + "\n")
        started = time.time()
        reading = rotator.measure(samples=args.samples, settle=args.settle)
        for index in range(total + 1):
            offset = reading["stepPosition"] - origin
            out.write(json.dumps({
                "record": "point", "index": index,
                "microstepOffset": offset,
                "phase": offset % MICROSTEPS_PER_FULLSTEP,
                "fullstep": offset // MICROSTEPS_PER_FULLSTEP,
                "stepPosition": reading["stepPosition"],
                "rawSensor": reading["rawSensor"],
                "t": round(time.time() - started, 3),
            }) + "\n")
            out.flush()
            if args.echo and index % args.echo == 0:
                print(f"  {index:5d}/{total}  offset={offset:7d}  "
                      f"raw={reading['rawSensor']:8.3f}", flush=True)
            if index == total:
                break
            rotator.jog(increment, samples=1)
            reading = rotator.measure(samples=args.samples, settle=args.settle)
    print(f"wrote {args.out} ({time.time()-started:.0f}s)")


if __name__ == "__main__":
    main()
