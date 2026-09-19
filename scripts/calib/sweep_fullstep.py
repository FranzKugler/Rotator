"""Step 1 of the calibration campaign: measure the AS5600's characteristic
curve over one full motor revolution, sampled at the motor's own 400 full
steps.

Why full steps: the AS5600 sits on the motor shaft, so 400 full steps is
exactly one sensor revolution. A stepper's full-step positions are its true
magnetic equilibria - the most repeatable mechanical references the machine
has without a second encoder - and 400 of them give one support point every
10.24 sensor counts.

The run records raw, uncorrected sensor readings only, so its result is
independent of whatever calibration currently sits on the device.

Output: newline-delimited JSON (one record per support point) plus a header
record, written to --out.

    python3 scripts/calib/sweep_fullstep.py --host 172.22.102.30 \
        --out run.jsonl --revs 1 --passes fwd,rev --samples 16
"""

import argparse
import json
import sys
import time
from datetime import datetime, timezone

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from rotator_io import (Rotator, MICROSTEPS_PER_FULLSTEP, FULLSTEPS_PER_MOTOR_REV,
                        check_reachable)


def run_pass(rotator, out, direction, revs, samples, settle, runup, tag, echo):
    """One uninterrupted sweep in a single direction.

    A run-up of `runup` full steps in the sweep direction precedes every pass
    and is not recorded: it puts the motor's magnetic history, and any slop
    between rotor and load, into the state the recorded points will be
    measured in. Reversing direction without it would fold the reversal
    transient into the first support points.
    """
    step = direction * MICROSTEPS_PER_FULLSTEP
    if runup:
        rotator.jog(step * runup, samples=1)
        time.sleep(settle)

    total = revs * FULLSTEPS_PER_MOTOR_REV
    started = time.time()
    # Measure before the first jog so index 0 is a real support point and
    # index `total` closes the revolution back onto it.
    reading = rotator.measure(samples=samples, settle=settle)
    for index in range(total + 1):
        record = {
            "pass": tag,
            "direction": direction,
            "index": index,
            "stepPosition": reading["stepPosition"],
            "rawSensor": reading["rawSensor"],
            "hall": reading["hall"],
            "t": round(time.time() - started, 3),
        }
        out.write(json.dumps(record) + "\n")
        out.flush()
        if echo and index % echo == 0:
            print(f"  [{tag}] {index:4d}/{total}  raw={reading['rawSensor']:8.3f}  "
                  f"step={reading['stepPosition']}", flush=True)
        if index == total:
            break
        rotator.jog(step, samples=1)
        reading = rotator.measure(samples=samples, settle=settle)
    return time.time() - started


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="172.22.102.30")
    ap.add_argument("--out", required=True)
    ap.add_argument("--revs", type=int, default=1, help="motor revolutions per pass")
    ap.add_argument("--passes", default="fwd",
                    help="comma-separated: fwd, rev (in run order)")
    ap.add_argument("--samples", type=int, default=16,
                    help="AS5600 samples averaged per support point (10 ms apart)")
    ap.add_argument("--settle", type=float, default=0.15,
                    help="seconds between the jog and the measurement")
    ap.add_argument("--runup", type=int, default=8,
                    help="unrecorded full steps in the sweep direction before each pass")
    ap.add_argument("--echo", type=int, default=25, help="progress every N points (0=off)")
    args = ap.parse_args()

    rotator = Rotator(args.host)
    check_reachable(rotator)
    diagnostics = rotator.sensor_diagnostics()
    print(f"AS5600 diagnostics: {diagnostics}", flush=True)

    reference = rotator.align_fullstep()
    print(f"aligned to full step, stepPosition={reference}", flush=True)

    with open(args.out, "w") as out:
        out.write(json.dumps({
            "pass": "__header__",
            "host": args.host,
            "utc": datetime.now(timezone.utc).isoformat(),
            "revs": args.revs,
            "samples": args.samples,
            "settle": args.settle,
            "runup": args.runup,
            "alignedStepPosition": reference,
            "diagnostics": diagnostics,
            "microstepsPerFullstep": MICROSTEPS_PER_FULLSTEP,
            "fullstepsPerMotorRev": FULLSTEPS_PER_MOTOR_REV,
        }) + "\n")
        for tag in [p.strip() for p in args.passes.split(",") if p.strip()]:
            direction = 1 if tag.startswith("f") else -1
            print(f"pass '{tag}' (direction {direction:+d}) ...", flush=True)
            elapsed = run_pass(rotator, out, direction, args.revs, args.samples,
                               args.settle, args.runup, tag, args.echo)
            print(f"pass '{tag}' done in {elapsed:.1f}s", flush=True)

    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
