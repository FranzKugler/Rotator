"""Step 3: closed-loop positioning on the corrected sensor, with the control
law as an experiment rather than an assumption.

Everything runs in Python over /api/debug/jog, so the control law, the sensor
correction and the gains can all be changed between runs without reflashing.
The firmware's own refineToTarget() is deliberately not involved.

Angle estimate
--------------
The AS5600 is on the motor shaft, so it only knows the output angle modulo
36 degrees. The step counter knows the whole angle but drifts by exactly the
errors being corrected. So: the step counter supplies the revolution, the
corrected sensor supplies the position within it.

    coarse   = stepPosition * DEG_PER_MICROSTEP
    fine     = corrected_counts * 36/4096 + offset
    estimate = coarse + wrap(fine - coarse, 36)

`offset` is fixed once at the start of a run from the step counter, which is
sound because the counter's absolute error is far below the 18-degree
half-period that would be needed to pick the wrong revolution.

Control laws
------------
P, PI, PD and PID over the same loop, so they can be compared on the same
targets in the same session. The integral term uses conditional integration
(it only accumulates once the error is already small); without that, the
large error of a long move winds the integral up and it then dominates for
many iterations after the error is small.
"""

import argparse
import json
import math
import random
import sys
import time
from datetime import datetime, timezone

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from rotator_io import (Rotator, MICROSTEPS_PER_FULLSTEP, OUTPUT_DEG_PER_COUNT,
                        check_reachable)

DEG_PER_MICROSTEP = 360.0 / 400.0 / 10.0 / 256.0
SENSOR_PERIOD_DEG = 36.0
# main/WebServer.cpp clamps a single jog to two motor revolutions.
JOG_LIMIT = 2 * 400 * MICROSTEPS_PER_FULLSTEP


class Correction:
    """The Fourier sensor model, evaluated exactly as the firmware does:
    corrected = raw - (C0 + sum_k A_k cos(k theta) + B_k sin(k theta))."""

    def __init__(self, c0=0.0, a=(), b=()):
        self.c0, self.a, self.b = c0, list(a), list(b)

    @classmethod
    def from_json(cls, path):
        with open(path) as handle:
            data = json.load(handle)
        return cls(data["C0"], data["A"], data["B"])

    def __call__(self, raw):
        theta = 2.0 * math.pi * raw / 4096.0
        error = self.c0
        for k in range(1, len(self.a)):
            error += self.a[k] * math.cos(k * theta) + self.b[k] * math.sin(k * theta)
        return raw - error


def wrap(value, period):
    return (value + period / 2.0) % period - period / 2.0


class Loop:
    def __init__(self, rotator, correction, law="pi", kp=0.9, ki=0.15, kd=0.0,
                 tolerance=0.004, integral_band=0.05, max_iterations=25,
                 samples=16, settle=0.12):
        self.rotator, self.correction = rotator, correction
        self.law, self.kp, self.ki, self.kd = law, kp, ki, kd
        self.tolerance, self.integral_band = tolerance, integral_band
        self.max_iterations, self.samples, self.settle = max_iterations, samples, settle

    def read(self, offset):
        reading = self.rotator.measure(samples=self.samples, settle=self.settle)
        coarse = reading["stepPosition"] * DEG_PER_MICROSTEP
        fine = self.correction(reading["rawSensor"]) * OUTPUT_DEG_PER_COUNT + offset
        return coarse + wrap(fine - coarse, SENSOR_PERIOD_DEG), reading

    def run(self, target_deg, trace=None):
        # Tie the sensor's 36-degree scale to the absolute angle once.
        first = self.rotator.measure(samples=self.samples)
        coarse = first["stepPosition"] * DEG_PER_MICROSTEP
        offset = coarse - self.correction(first["rawSensor"]) * OUTPUT_DEG_PER_COUNT

        # Open-loop approach first, so the closed loop only ever has to clean
        # up a small residual. /api/debug/jog refuses anything beyond two
        # motor revolutions per call (main/WebServer.cpp's JOG_LIMIT), which
        # a long move easily exceeds, so it goes out in chunks.
        estimate, _ = self.read(offset)
        gross = round((target_deg - estimate) / DEG_PER_MICROSTEP)
        while gross:
            chunk = max(-JOG_LIMIT, min(JOG_LIMIT, gross))
            self.rotator.jog(chunk, samples=1)
            gross -= chunk

        integral, previous_error = 0.0, None
        for iteration in range(self.max_iterations):
            estimate, reading = self.read(offset)
            error = target_deg - estimate
            if trace is not None:
                trace.append({"iteration": iteration, "estimate": estimate,
                              "error": error, "rawSensor": reading["rawSensor"],
                              "stepPosition": reading["stepPosition"]})
            if abs(error) <= self.tolerance:
                return estimate, iteration, True

            command = 0.0
            if self.law in ("p", "pi", "pd", "pid"):
                command += self.kp * error
            if self.law in ("pi", "pid"):
                if abs(error) < self.integral_band:
                    integral += error
                else:
                    integral = 0.0
                command += self.ki * integral
            if self.law in ("pd", "pid") and previous_error is not None:
                command += self.kd * (error - previous_error)
            previous_error = error

            steps = round(command / DEG_PER_MICROSTEP)
            if steps == 0:
                steps = 1 if error > 0 else -1
            self.rotator.jog(steps, samples=1)

        estimate, _ = self.read(offset)
        return estimate, self.max_iterations, False


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="172.22.102.30")
    ap.add_argument("--coefficients", help="fit_sensor.py --export JSON (default: none)")
    ap.add_argument("--laws", default="p,pi,pd,pid")
    ap.add_argument("--kp", type=float, default=0.9)
    ap.add_argument("--ki", type=float, default=0.15)
    ap.add_argument("--kd", type=float, default=0.25)
    ap.add_argument("--targets", type=int, default=8)
    ap.add_argument("--range", type=float, default=60.0)
    ap.add_argument("--tolerance", type=float, default=0.004)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    rotator = Rotator(args.host)
    check_reachable(rotator)
    correction = (Correction.from_json(args.coefficients) if args.coefficients
                  else Correction())

    random.seed(args.seed)
    targets = [random.uniform(-args.range, args.range) for _ in range(args.targets)]

    results = {}
    with open(args.out, "w") as out:
        out.write(json.dumps({"record": "__header__",
                              "utc": datetime.now(timezone.utc).isoformat(),
                              "kp": args.kp, "ki": args.ki, "kd": args.kd,
                              "tolerance": args.tolerance, "targets": targets,
                              "coefficients": args.coefficients}) + "\n")
        for law in [x.strip() for x in args.laws.split(",") if x.strip()]:
            loop = Loop(rotator, correction, law=law, kp=args.kp, ki=args.ki,
                        kd=args.kd, tolerance=args.tolerance)
            errors, iterations, settled = [], [], 0
            for target in targets:
                trace = []
                started = time.time()
                final, count, ok = loop.run(target, trace)
                errors.append(final - target)
                iterations.append(count)
                settled += int(ok)
                out.write(json.dumps({"record": "run", "law": law, "target": target,
                                      "final": final, "error": final - target,
                                      "iterations": count, "settled": ok,
                                      "seconds": round(time.time() - started, 2),
                                      "trace": trace}) + "\n")
                out.flush()
            rms = math.sqrt(sum(e * e for e in errors) / len(errors))
            results[law] = (rms, max(abs(e) for e in errors),
                            sum(iterations) / len(iterations), settled)
            print(f"{law.upper():4s}  rms {rms*1000:7.2f} mdeg   "
                  f"max {max(abs(e) for e in errors)*1000:7.2f} mdeg   "
                  f"mean {sum(iterations)/len(iterations):4.1f} iterations   "
                  f"settled {settled}/{len(targets)}", flush=True)

    print(f"\nwrote {args.out}")


if __name__ == "__main__":
    main()
