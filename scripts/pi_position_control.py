"""Closed-loop position control: reach a target OUTPUT-shaft angle by
repeatedly measuring and correcting, instead of predicting the right
microstep count from a motor/gearbox model up front.

Why this instead of the fullstep-table/motor-sweep direction: those measured
real, but small and only partially explained, position-dependent effects
(cogging-scale AS5600 ripple, a full-step slope that varies +/-25% around
the revolution, weak wobble-shape correlation between full steps) - each
new finding needed another model refinement. A PI loop sidesteps predicting
any of that: it only needs an accurate MEASUREMENT after each move, and lets
the integral term absorb whatever the true, imperfect open-loop gain turns
out to be at that particular spot on the revolution. Simpler, and robust to
exactly the kind of position-dependent behaviour cmd_motor_sweep found.

Measurement comes from the same AS5600 correctedSensor jog() already
returns, refined by angle_filter_lab.py's AngleKalman1D - predict() on every
commanded move, update() on every new reading, unwrapped near the filter's
own running estimate rather than trusting the sensor's raw 0-36 deg (one
AS5600 period = one motor revolution = 360/REDUCTION output degrees) wrap on
its own.

Usage:
    python3 scripts/pi_position_control.py --host 172.22.102.30 goto --delta 1.0
    python3 scripts/pi_position_control.py --host 172.22.102.30 goto --delta -0.2 --kp 0.9 --ki 0.15 --tolerance 0.003
"""

import argparse
import json
import math
import sys
import time

sys.path.insert(0, __import__("os").path.dirname(__import__("os").path.abspath(__file__)))
from calibration_lab import jog, FULLSTEPS_PER_ROTATION, MICROSTEPS, DEG_PER_COUNT  # noqa: E402
from angle_filter_lab import AngleKalman1D, REDUCTION, AS5600_NOISE_FLOOR_OUTPUT_DEG  # noqa: E402

MICROSTEPS_PER_OUTPUT_DEG = FULLSTEPS_PER_ROTATION * MICROSTEPS * REDUCTION / 360.0
AS5600_PERIOD_OUTPUT_DEG = 360.0 / REDUCTION  # one AS5600 revolution = 36 output degrees


def nearest(value, near, period):
    """value, shifted by whole multiples of period, to land as close as
    possible to `near` - the same shortest-distance idiom as wrap4096(),
    generalized to the 36 output-degree AS5600 period instead of 4096 counts.
    """
    return near + (value - near + period / 2.0) % period - period / 2.0


def goto_relative(host, delta_deg, kp, ki, tolerance_deg=0.005, max_iterations=20,
                   sensor_samples=8, settle_s=0.15, integral_band_deg=0.05, kf_q=1e-7, kf_r=None):
    """integral_band_deg: only accumulate the integral term while the error
    is already smaller than this - otherwise a single large first-move error
    (e.g. the whole commanded delta, before anything has moved yet) windows
    up the integral and then dominates the proportional term for many
    iterations after the error is already small, causing exactly the
    overshoot-then-slow-decay seen in the first live test without this
    guard. Standard conditional-integration anti-windup, not a hardware
    issue - Kp alone is expected to do most of the initial "get close" work,
    Ki only trims the small residual left once errors are already inside the
    band.
    """
    kf_r = kf_r if kf_r is not None else (AS5600_NOISE_FLOOR_OUTPUT_DEG ** 2) / sensor_samples

    state0 = jog(host, 0, sensor_samples)
    ref_raw = state0["correctedSensor"] * DEG_PER_COUNT / REDUCTION
    kf = AngleKalman1D(x0=0.0, p0=kf_r, q=kf_q, r=kf_r)

    target = delta_deg
    integral = 0.0
    history = [{"iter": 0, "commandedMicrosteps": 0, "error": target - kf.x, "estimate": kf.x}]

    print(f"[pi-goto] target={target:+.4f} deg (output-shaft)  Kp={kp} Ki={ki}  tolerance={tolerance_deg} deg")

    for it in range(1, max_iterations + 1):
        error = target - kf.x
        if abs(error) < tolerance_deg:
            print(f"  converged after {it - 1} move(s): error={error:+.4f} deg")
            break

        if abs(error) < integral_band_deg:
            integral += error
        control_deg = kp * error + ki * integral
        control_microsteps = round(control_deg * MICROSTEPS_PER_OUTPUT_DEG)
        if control_microsteps == 0:
            control_microsteps = 1 if control_deg > 0 else -1  # never stall on rounding alone

        state = jog(host, control_microsteps, sensor_samples)
        if settle_s > 0:
            time.sleep(settle_s)

        commanded_deg = control_microsteps / MICROSTEPS_PER_OUTPUT_DEG
        kf.predict(commanded_deg)

        raw_output = state["correctedSensor"] * DEG_PER_COUNT / REDUCTION
        measured = nearest(raw_output - ref_raw, kf.x, AS5600_PERIOD_OUTPUT_DEG)
        kf.update(measured)

        print(f"  iter {it:2d}: commanded={control_microsteps:+6d} us ({commanded_deg:+.4f} deg)  "
              f"measured={measured:+.4f}  estimate={kf.x:+.4f}  error={target - kf.x:+.4f} deg")
        history.append({
            "iter": it, "commandedMicrosteps": control_microsteps, "commandedDeg": commanded_deg,
            "measuredDeg": measured, "estimate": kf.x, "error": target - kf.x,
        })
    else:
        print(f"  !! did not converge within {max_iterations} moves - final error={target - kf.x:+.4f} deg")

    # Independent confirmation read, heavily averaged, not just the filter's
    # own running estimate - the same "verify with a fresh measurement"
    # discipline as everywhere else in this project.
    confirm = jog(host, 0, sensor_samples * 4)
    confirm_raw = confirm["correctedSensor"] * DEG_PER_COUNT / REDUCTION
    confirm_measured = nearest(confirm_raw - ref_raw, kf.x, AS5600_PERIOD_OUTPUT_DEG)
    confirm_error = target - confirm_measured
    print(f"  confirmed (samples={sensor_samples * 4}): position={confirm_measured:+.4f} deg  "
          f"final error={confirm_error:+.4f} deg")

    net_microsteps = sum(h["commandedMicrosteps"] for h in history)
    return {
        "targetDeg": target, "kp": kp, "ki": ki, "toleranceDeg": tolerance_deg,
        "history": history, "netMicrosteps": net_microsteps,
        "confirmedMeasuredDeg": confirm_measured, "confirmedErrorDeg": confirm_error,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", required=True)
    sub = ap.add_subparsers(dest="command", required=True)

    g = sub.add_parser("goto", help="move by a relative output-degree amount, closed-loop")
    g.add_argument("--delta", type=float, required=True, help="target relative move, output-shaft degrees")
    g.add_argument("--kp", type=float, default=0.9)
    g.add_argument("--ki", type=float, default=0.15)
    g.add_argument("--tolerance", type=float, default=0.005, help="output-shaft degrees")
    g.add_argument("--integral-band", type=float, default=0.05,
                    help="only integrate while |error| is below this (anti-windup), output-shaft degrees")
    g.add_argument("--max-iterations", type=int, default=20)
    g.add_argument("--samples", type=int, default=8, help="AS5600 averaging depth per measurement")
    g.add_argument("--settle", type=float, default=0.15)
    g.add_argument("--return-after", action="store_true", help="move back to the starting position afterward")
    g.add_argument("--out", default=None)

    args = ap.parse_args()

    if args.command == "goto":
        result = goto_relative(
            args.host, args.delta, args.kp, args.ki,
            tolerance_deg=args.tolerance, max_iterations=args.max_iterations,
            sensor_samples=args.samples, settle_s=args.settle, integral_band_deg=args.integral_band,
        )
        if args.return_after and result["netMicrosteps"] != 0:
            print(f"  returning {-result['netMicrosteps']} microsteps to starting position")
            jog(args.host, -result["netMicrosteps"], 1)
        if args.out:
            with open(args.out, "w") as f:
                json.dump(result, f, indent=2)
            print(f"  wrote {args.out}")


if __name__ == "__main__":
    main()
