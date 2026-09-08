"""Kalman-filter design lab for a smoothed live rotator angle.

Fresh start: the earlier attempts (an unfinished, never-called EKF sketch in
main/RotatorHW.cpp, and a SensorMotorEKF prototype that used to live in
calibration_lab.py) are gone. This is not a resurrection of either - it is a
smaller, more honestly-scoped filter, split from calibration_lab.py because it
answers a different question (how to smooth a *live* angle stream) rather than
calibration_lab's own question (what are this sensor/motor/driver's static
error characteristics).

Units, and why they matter here: the AS5600 sits on the MOTOR shaft, upstream
of the 10:1 reduction to the rotator's OUTPUT shaft. calibration_lab.py's
DEG_PER_COUNT (360/4096) is therefore a MOTOR-shaft degree - every "wobble"
figure quoted in that script's output and in the motor-sweep lab reports (all
in the 0.02-02.09 degree range) is a motor-shaft degree, a factor of 10 larger
than the corresponding OUTPUT-shaft degree Alpaca's Position property reports
and that actually matters for field derotation. This module works in OUTPUT
degrees throughout (see OUTPUT_DEG_PER_COUNT below) to keep that straight.

Design, deliberately simple for a first cut:
  - State: a single scalar, the unwrapped absolute OUTPUT-shaft angle. No
    velocity term yet - the next planned step is a PI position controller,
    which needs an angle estimate and its running integral, not a velocity.
  - Process model: x_k = x_{k-1} + commanded_delta, where commanded_delta
    comes from an exact, known control input (microsteps actually issued to
    the driver, converted to output degrees) - not re-deriving the within-
    full-step motor/driver wobble model. That wobble is real physical motion,
    not sensor noise, and folding a wobble-compensation term into the process
    model (as the old SensorMotorEKF did) conflates "smooth out sensor noise"
    with "predict and cancel a real mechanical nonlinearity" - two different
    problems. Keeping the process model dumb here means any residual wobble
    shows up as apparent measurement noise and gets averaged down by the
    filter, which is exactly what "a smoothed live angle" asks for.
  - Measurement model: z_k = x_k, i.e. h(x) = x. This is a plain linear
    Kalman filter, not an EKF: the old design's Jacobian existed only because
    it folded calibration_lab.py's static sensor eccentricity correction
    (correct(), fit_fourier()) *inside* the measurement function. That
    correction is already solved, deterministic, and cheap to apply before
    the filter ever sees a reading - there is no reason to linearize it on
    every update. correct_to_output_deg() below does that conversion once,
    up front.

Two ways to exercise the filter, both offline:
  - `simulate`: synthetic ground truth (initially a constant-velocity ramp,
    the derotator/continuous-tracking case) plus injected noise built from
    this project's own characterized noise floor, so Q/R tuning and the
    filter's actual tracking-lag-vs-smoothing trade-off can be explored
    cheaply and repeatably with no hardware involved.
  - `replay`: replays a motor-sweep JSON produced by calibration_lab.py's
    motor-sweep command against real, already sensor-corrected readings at a
    known, constant commanded step spacing. There is no independent
    high-precision ground truth in that data (this project's old proxy for
    one, the ekf-walk command's heavily-averaged "ref" reading, was removed
    along with the rest of that attempt) - so this mode reports point-to-point
    noise reduction, not absolute tracking error.

Usage:
    python3 scripts/angle_filter_lab.py simulate
    python3 scripts/angle_filter_lab.py simulate --q 1e-6 --r 3e-5
    python3 scripts/angle_filter_lab.py sweep
    python3 scripts/angle_filter_lab.py replay --file motor_sweep_280ma.json --group 3
    python3 scripts/angle_filter_lab.py revolution --file full_revolution.json --out rev_plot.json
"""

import argparse
import json
import math
import random
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from calibration_lab import DEG_PER_COUNT, MICROSTEPS, FULLSTEPS_PER_ROTATION, wrap4096, correct  # noqa: E402

REDUCTION = 10  # matches main/RotatorHW.cpp's fixed 10:1 gear reduction
OUTPUT_DEG_PER_COUNT = DEG_PER_COUNT / REDUCTION  # motor-shaft sensor degree -> output-shaft degree
IDEAL_OUTPUT_DEG_PER_MICROSTEP = 360.0 / FULLSTEPS_PER_ROTATION / REDUCTION / MICROSTEPS

# AS5600's own datasheet noise floor at its default (and, per RotatorHW.cpp's
# as5600.begin(), never-overridden) slow-filter setting SF=00/16x: 0.015 deg
# RMS on the sensor's own (motor-shaft) output. That is a lower bound on
# measurement noise no amount of filtering downstream can beat.
AS5600_NOISE_FLOOR_SENSOR_DEG = 0.015
AS5600_NOISE_FLOOR_OUTPUT_DEG = AS5600_NOISE_FLOOR_SENSOR_DEG / REDUCTION


class AngleKalman1D:
    """Scalar linear Kalman filter over an unwrapped OUTPUT-shaft angle."""

    def __init__(self, x0, p0, q, r):
        self.x = x0
        self.P = p0
        self.Q = q  # process noise variance, (output deg)^2 added per predict()
        self.R = r  # measurement noise variance, (output deg)^2

    def predict(self, commanded_delta_output_deg):
        self.x += commanded_delta_output_deg
        self.P += self.Q

    def update(self, measured_output_deg):
        y = measured_output_deg - self.x
        S = self.P + self.R
        K = self.P / S
        self.x += K * y
        self.P *= 1.0 - K
        return y  # innovation, handy for diagnostics


def rms(values):
    return math.sqrt(sum(v * v for v in values) / len(values)) if values else float("nan")


# ------------------------------------------------------------------ simulate

def make_ground_truth(n, velocity_deg_per_step=0.02, seed=0):
    """A constant-velocity ramp: the continuous-tracking/derotator case, the
    use pattern an ASCOM Rotator is actually built for. `velocity_deg_per_step`
    is deliberately in output degrees per *filter tick*, not per second - this
    is a step-indexed simulation, not a real-time one.
    """
    return [i * velocity_deg_per_step for i in range(n)]


def inject_measurement_noise(truth, quantization_deg, noise_sigma_deg, seed=0):
    """AS5600 measurements: quantized to its 12-bit count grid (translated to
    output degrees) plus Gaussian sensor noise on top - the two error sources
    the datasheet documents separately (RES=12 bit; ON_SLOW=0.015 deg RMS).
    """
    rng = random.Random(seed)
    out = []
    for t in truth:
        quantized = round(t / quantization_deg) * quantization_deg
        out.append(quantized + rng.gauss(0.0, noise_sigma_deg))
    return out


def run_filter(truth, measurements, commanded_deltas, q, r, x0=None, p0=None):
    x0 = measurements[0] if x0 is None else x0
    p0 = r if p0 is None else p0
    kf = AngleKalman1D(x0, p0, q, r)
    estimates = [x0]
    for i in range(1, len(truth)):
        kf.predict(commanded_deltas[i])
        kf.update(measurements[i])
        estimates.append(kf.x)
    return estimates


def cmd_simulate(args):
    n = args.n
    velocity = args.velocity
    truth = make_ground_truth(n, velocity_deg_per_step=velocity, seed=args.seed)
    quant = OUTPUT_DEG_PER_COUNT  # one AS5600 LSB, in output degrees
    noise_sigma = args.noise_sigma if args.noise_sigma is not None else AS5600_NOISE_FLOOR_OUTPUT_DEG
    measurements = inject_measurement_noise(truth, quant, noise_sigma, seed=args.seed)
    commanded_deltas = [0.0] + [velocity] * (n - 1)  # exact - this is the simulated control input

    q = args.q
    r = args.r if args.r is not None else noise_sigma ** 2
    estimates = run_filter(truth, measurements, commanded_deltas, q, r)

    raw_err = [m - t for m, t in zip(measurements, truth)]
    filt_err = [e - t for e, t in zip(estimates, truth)]
    print(f"[simulate] n={n} points, velocity={velocity:.5f} output-deg/step, "
          f"quantization={quant:.6f} deg, noise sigma={noise_sigma:.6f} deg")
    print(f"  Q={q:.3e}  R={r:.3e}")
    print(f"  raw sensor RMS error      : {rms(raw_err):.6f} deg")
    print(f"  filtered RMS error        : {rms(filt_err):.6f} deg")
    print(f"  raw sensor peak error     : {max(abs(e) for e in raw_err):.6f} deg")
    print(f"  filtered peak error       : {max(abs(e) for e in filt_err):.6f} deg")
    if args.out:
        with open(args.out, "w") as f:
            json.dump({
                "truth": truth, "measurements": measurements, "estimates": estimates,
                "q": q, "r": r,
            }, f, indent=2)
        print(f"  wrote {args.out}")


def cmd_sweep(args):
    """Grid-search Q (trust in the process model) against R (trust in the
    sensor) to see the smoothing/lag trade-off directly, instead of guessing.
    """
    n = args.n
    velocity = args.velocity
    truth = make_ground_truth(n, velocity_deg_per_step=velocity, seed=args.seed)
    quant = OUTPUT_DEG_PER_COUNT
    noise_sigma = args.noise_sigma if args.noise_sigma is not None else AS5600_NOISE_FLOOR_OUTPUT_DEG
    measurements = inject_measurement_noise(truth, quant, noise_sigma, seed=args.seed)
    commanded_deltas = [0.0] + [velocity] * (n - 1)
    r = noise_sigma ** 2

    print(f"[sweep] n={n}, velocity={velocity:.5f} output-deg/step, noise sigma={noise_sigma:.6f} deg, R={r:.3e}")
    print(f"  raw sensor RMS error (unfiltered baseline): {rms([m - t for m, t in zip(measurements, truth)]):.6f} deg")
    print(f"  {'Q':>12}  {'RMS err (deg)':>14}  {'peak err (deg)':>15}")
    for q_exp in range(-9, -1):
        q = 10.0 ** q_exp
        estimates = run_filter(truth, measurements, commanded_deltas, q, r)
        filt_err = [e - t for e, t in zip(estimates, truth)]
        print(f"  {q:12.3e}  {rms(filt_err):14.6f}  {max(abs(e) for e in filt_err):15.6f}")


# --------------------------------------------------------------------- replay

def cmd_replay(args):
    """Replay a stored motor-sweep group: constant, known commanded spacing,
    real (already sensor-corrected) readings. No independent ground truth is
    available in this data, so this reports noise reduction (detrended
    point-to-point scatter), not absolute tracking error.
    """
    with open(args.file) as f:
        data = json.load(f)
    group = data["motor_sweep"][str(args.group)]
    angles_motor_shaft_counts = group["angles"]  # calibration_lab's unwrapped, corrected sensor counts
    measurements = [a * OUTPUT_DEG_PER_COUNT for a in angles_motor_shaft_counts]
    step_span = args.step_span  # must match what motor-sweep was run with
    commanded_delta = step_span * IDEAL_OUTPUT_DEG_PER_MICROSTEP
    commanded_deltas = [0.0] + [commanded_delta] * (len(measurements) - 1)

    r = args.r if args.r is not None else AS5600_NOISE_FLOOR_OUTPUT_DEG ** 2
    q = args.q
    estimates = run_filter(measurements, measurements, commanded_deltas, q, r)

    def detrended_rms(series):
        xs = list(range(len(series)))
        n = len(xs)
        mx = sum(xs) / n
        my = sum(series) / n
        num = sum((x - mx) * (y - my) for x, y in zip(xs, series))
        den = sum((x - mx) ** 2 for x in xs) or 1e-12
        slope = num / den
        intercept = my - slope * mx
        residuals = [y - (slope * x + intercept) for x, y in zip(xs, series)]
        return rms(residuals)

    print(f"[replay] {args.file} group {args.group}, {len(measurements)} points, "
          f"step_span={step_span} microsteps/point, Q={q:.3e}, R={r:.3e}")
    print(f"  raw (already sensor-corrected) detrended RMS : {detrended_rms(measurements):.6f} output-deg")
    print(f"  filtered detrended RMS                       : {detrended_rms(estimates):.6f} output-deg")


def cmd_revolution(args):
    """One full motor revolution (calibration_lab.py's sensor-sweep, 400
    points, one per full step): what the sensor's own mounting eccentricity
    looks like uncorrected, corrected by the static Fourier fit, and further
    smoothed by the Kalman filter on top of that correction. Unlike replay's
    motor-sweep data, this has a real, known-good ground truth per point -
    the ideal, commanded full-step position - so real (not just relative)
    tracking error can be reported here.
    """
    with open(args.file) as f:
        data = json.load(f)
    sweep = data["sensor_sweep"]
    ideal_counts = sweep["ideal"]
    measured_counts = sweep["measured"]  # raw AS5600 reading, unwrapped
    model = (sweep["C0"], sweep["A"], sweep["B"])
    n = len(ideal_counts)

    ideal_deg = [c * OUTPUT_DEG_PER_COUNT for c in ideal_counts]
    raw_deg = [c * OUTPUT_DEG_PER_COUNT for c in measured_counts]
    corrected_deg = [correct(c, model) * OUTPUT_DEG_PER_COUNT for c in measured_counts]
    commanded_delta = ideal_deg[1] - ideal_deg[0]  # one full step (MICROSTEPS microsteps), constant
    commanded_deltas = [0.0] + [commanded_delta] * (n - 1)

    r = args.r if args.r is not None else AS5600_NOISE_FLOOR_OUTPUT_DEG ** 2
    q = args.q
    filtered_deg = run_filter(ideal_deg, corrected_deg, commanded_deltas, q, r,
                               x0=corrected_deg[0], p0=r)

    raw_err = [a - b for a, b in zip(raw_deg, ideal_deg)]
    corrected_err = [a - b for a, b in zip(corrected_deg, ideal_deg)]
    filtered_err = [a - b for a, b in zip(filtered_deg, ideal_deg)]
    # raw_err's mean is dominated by a large constant: the AS5600's own zero
    # position vs. the sweep's arbitrary ideal=0 reference (~C0, a few degrees
    # of pure bias with no bearing on precision). correct() removes that bias
    # along with the periodic terms, which is exactly why corrected_err looks
    # so much smaller - not an apples-to-oranges comparison, but debiasing
    # raw_err here too, for the plot, isolates the *shape* worth looking at.
    raw_bias = statistics.mean(raw_err)
    raw_err_debiased = [e - raw_bias for e in raw_err]

    print(f"[revolution] {args.file}, {n} full steps (1 motor revolution = {ideal_deg[-1] + commanded_delta:.3f} output-deg)")
    print(f"  Q={q:.3e}  R={r:.3e}")
    print(f"  raw error, incl. zero-offset bias of {raw_bias:+.3f} deg : "
          f"RMS={rms(raw_err):.5f}  peak={max(abs(e) for e in raw_err):.5f}  (output-deg)")
    print(f"  raw error, bias removed (shape only)          : "
          f"RMS={rms(raw_err_debiased):.5f}  peak={max(abs(e) for e in raw_err_debiased):.5f}  (output-deg)")
    print(f"  corrected (static fit) error                  : "
          f"RMS={rms(corrected_err):.5f}  peak={max(abs(e) for e in corrected_err):.5f}  (output-deg)")
    print(f"  filtered (KF on top) error                    : "
          f"RMS={rms(filtered_err):.5f}  peak={max(abs(e) for e in filtered_err):.5f}  (output-deg)")

    if args.out:
        with open(args.out, "w") as f:
            json.dump({
                "fullstep_index": list(range(n)),
                "ideal_output_deg": ideal_deg,
                "raw_output_deg": raw_deg,
                "corrected_output_deg": corrected_deg,
                "filtered_output_deg": filtered_deg,
                "raw_err_deg": raw_err,
                "raw_err_deg_debiased": raw_err_debiased,
                "corrected_err_deg": corrected_err,
                "filtered_err_deg": filtered_err,
                "q": q, "r": r,
            }, f, indent=2)
        print(f"  wrote {args.out}")


COMMANDS = {
    "simulate": cmd_simulate,
    "sweep": cmd_sweep,
    "replay": cmd_replay,
    "revolution": cmd_revolution,
}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("command", choices=list(COMMANDS))
    parser.add_argument("--n", type=int, default=300, help="simulate/sweep: number of filter ticks")
    parser.add_argument("--velocity", type=float, default=0.02,
                         help="simulate/sweep: ground-truth output-deg advance per tick (constant-velocity ramp)")
    parser.add_argument("--noise-sigma", type=float, default=None,
                         help="simulate/sweep: measurement noise sigma in output-deg "
                              "(default: the AS5600 datasheet noise floor)")
    parser.add_argument("--q", type=float, default=1e-7, help="process noise variance, (output-deg)^2")
    parser.add_argument("--r", type=float, default=None, help="measurement noise variance, (output-deg)^2")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--out", help="simulate/revolution: write plot-ready data as JSON")
    parser.add_argument("--file", help="replay/revolution: a calibration_lab.py --out JSON (motor_sweep/sensor_sweep)")
    parser.add_argument("--group", type=int, default=1, help="replay: which motor-sweep group to replay")
    parser.add_argument("--step-span", type=int, default=1, help="replay: microsteps/point the sweep was run with")
    args = parser.parse_args()

    COMMANDS[args.command](args)


if __name__ == "__main__":
    main()
