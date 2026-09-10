"""Offline calibration/model-fitting lab for the rotator's AS5600 + stepper.

Drives the rotator via the firmware's POST /api/debug/jog debug endpoint
(main/WebServer.cpp) instead of the embedded calibrateAngleSensor() routine,
so sweep patterns, averaging depth and analysis can be iterated on without a
rebuild+reflash cycle per experiment. Standard library only - no numpy/pandas
- so it runs anywhere Python 3 does, including the ESP-IDF python_env in the
dev container.

Requires expert mode already unlocked on the target (the endpoint is
expert-gated, see main/ExpertLock.c).

Usage:
    python3 scripts/calibration_lab.py --host 172.22.102.30 noise
    python3 scripts/calibration_lab.py --host 172.22.102.30 sensor-sweep
    python3 scripts/calibration_lab.py --host 172.22.102.30 motor-sweep
    python3 scripts/calibration_lab.py --host 172.22.102.30 backlash
    python3 scripts/calibration_lab.py --host 172.22.102.30 all --out lab_results.json

Each command returns the motor to its starting position (net zero motion)
before exiting normally; interrupting a command midway will leave the motor
wherever it happens to be.
"""

import argparse
import json
import math
import statistics
import sys
import time
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

FULLSTEPS_PER_ROTATION = 400  # matches main/RotatorHW.cpp
MICROSTEPS = 256              # matches main/RotatorHW.cpp
DEG_PER_COUNT = 360.0 / 4096.0
KMAX = 6  # deliberately more than the firmware's fixed 4, to see where harmonics actually drop off


# ---------------------------------------------------------------- transport

def jog(host, microsteps=0, samples=1, timeout=10):
    """POST /api/debug/jog - see main/WebServer.cpp's debug_jog_handler()."""
    body = json.dumps({"microsteps": microsteps, "samples": samples}).encode()
    req = Request(
        f"http://{host}/api/debug/jog",
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read())


def align_fullstep(host, timeout=15):
    """POST /api/debug/align-fullstep - see RotatorHW::alignToFullStep(). A
    stepper only has FULLSTEPS_PER_ROTATION true mechanical equilibrium
    positions, not FULLSTEPS_PER_ROTATION*MICROSTEPS - "stepPosition 137" on
    its own is only ever a position *within* whichever full step the motor
    happened to be inside. This briefly switches the driver to full-step
    mode, takes one real full step, switches back, and returns the resulting
    stepPosition as a "phase 0" reference the caller can trust, instead of
    assuming whatever stepPosition already happened to be was full-step-
    aligned - see cmd_motor_sweep(), which needs exactly that assumption.
    """
    req = Request(f"http://{host}/api/debug/align-fullstep", data=b"", method="POST")
    with urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read())


# ------------------------------------------------------------------- maths
# Deliberately mirrors RotatorHW.cpp's calibrateAngleSensorStep()/
# calibrateAngleSensorFinalize()/correctSensorReading()/computeResidual() so
# results here are directly comparable to what the firmware itself measures.

def wrap4096(x):
    """Shortest signed distance on the 4096-count circle, i.e. wrap to (-2048, 2048]."""
    return x - 4096.0 * round(x / 4096.0)


def fit_fourier(ideal_angles, measured, kmax=KMAX):
    """Fit C0 + sum(A_k*cos(k*theta) + B_k*sin(k*theta)) against measured - ideal,
    correlated against theta derived from the *measured* reading (not the ideal
    ramp) - this is the phase-basis fix from RotatorHW.cpp's calibrateAngleSensorStep().
    """
    n = len(ideal_angles)
    sum0 = 0.0
    sumC = [0.0] * (kmax + 1)
    sumS = [0.0] * (kmax + 1)
    for ideal, raw in zip(ideal_angles, measured):
        e = wrap4096(raw - ideal)
        sum0 += e
        theta = 2.0 * math.pi * raw / 4096.0
        for k in range(1, kmax + 1):
            sumC[k] += e * math.cos(k * theta)
            sumS[k] += e * math.sin(k * theta)
    C0 = sum0 / n
    A = [0.0] + [2.0 * sumC[k] / n for k in range(1, kmax + 1)]
    B = [0.0] + [2.0 * sumS[k] / n for k in range(1, kmax + 1)]
    return C0, A, B


def correct(raw, model):
    C0, A, B = model
    theta = 2.0 * math.pi * raw / 4096.0
    err = C0
    for k in range(1, len(A)):
        err += A[k] * math.cos(k * theta) + B[k] * math.sin(k * theta)
    return raw - err


def residual_stats(ideal_angles, measured, model):
    errs = [wrap4096(correct(raw, model) - ideal) for ideal, raw in zip(ideal_angles, measured)]
    rms = math.sqrt(sum(e * e for e in errs) / len(errs))
    peak = max(abs(e) for e in errs)
    return rms * DEG_PER_COUNT, peak * DEG_PER_COUNT


def check_position_roundtrip(label, start_pos, end_pos):
    """Every sweep below returns the motor to its starting position, so
    stepPosition should exactly match before and after - a free, continuous
    check for the FastAccelStepper position-tracking reliability issue found
    while testing the firmware's own calibration routine (see git history for
    RotatorHW.cpp's calibrateAngleSensor()). Many short, independently
    HTTP-triggered moves is a different usage pattern than that routine's one
    long uninterrupted sequence, so this is useful signal either way.
    """
    if start_pos != end_pos:
        print(f"  !! position mismatch after {label}: start={start_pos} end={end_pos} delta={end_pos - start_pos}")
    else:
        print(f"  position round-trip OK ({label}): {start_pos}")


def linear_fit(xs, ys):
    """Ordinary least squares, plain Python - slope, intercept."""
    n = len(xs)
    mx = sum(xs) / n
    my = sum(ys) / n
    num = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    den = sum((x - mx) ** 2 for x in xs) or 1e-9
    slope = num / den
    return slope, my - slope * mx


# -------------------------------------------------------------- experiments

def cmd_noise(host, n=30, **_):
    """How repeatable is a single AS5600 reading, held still, at two averaging depths?"""
    print(f"[noise] {n} reads at samples=1 and samples=20, motor held still")
    raw1 = [jog(host, 0, 1)["rawSensor"] for _ in range(n)]
    raw20 = [jog(host, 0, 20)["rawSensor"] for _ in range(n)]

    def report(label, raws):
        mean = statistics.mean(raws)
        sd = statistics.pstdev(raws)
        print(f"  {label}: mean={mean:8.3f} counts  stdev={sd:.4f} counts ({sd * DEG_PER_COUNT:.5f} deg)")

    report("samples= 1", raw1)
    report("samples=20", raw20)
    return {"samples1": raw1, "samples20": raw20}


def cmd_sensor_sweep(host, revolutions=1, samples=8, kmax=KMAX, **_):
    """Full-step sweep over `revolutions` motor turns, fitting the AS5600 error model."""
    n = FULLSTEPS_PER_ROTATION * revolutions
    print(f"[sensor-sweep] {n} full steps ({revolutions} revolution(s)), samples={samples}/point")
    start_pos = jog(host, 0, 1)["stepPosition"]
    t0 = time.time()
    ideal_angles, measured = [], []
    for i in range(n):
        raw = jog(host, MICROSTEPS, samples)["rawSensor"]
        ideal = 4096.0 * (i % FULLSTEPS_PER_ROTATION) / FULLSTEPS_PER_ROTATION
        measured.append(ideal + wrap4096(raw - ideal))  # unwrap near the ideal ramp
        ideal_angles.append(ideal)
        if i % 100 == 0:
            print(f"  step {i:4d}/{n}  raw={raw:8.2f}   ({time.time() - t0:5.1f}s elapsed)")
    end_pos = jog(host, -n * MICROSTEPS, 1)["stepPosition"]  # back to the starting position
    check_position_roundtrip("sensor-sweep", start_pos, end_pos)
    print(f"  swept in {time.time() - t0:.1f}s, returned to start")

    model = fit_fourier(ideal_angles, measured, kmax)
    C0, A, B = model
    print(f"  C0 = {C0:.3f} counts")
    for k in range(1, kmax + 1):
        amp = math.hypot(A[k], B[k])
        print(f"  A{k}={A[k]:8.3f}  B{k}={B[k]:8.3f}  amplitude={amp:7.3f} counts ({amp * DEG_PER_COUNT:.4f} deg)")
    for order in (4, kmax):
        rms, peak = residual_stats(ideal_angles, measured, (C0, A[: order + 1], B[: order + 1]))
        print(f"  residual at order {order}: RMS={rms:.4f} deg  peak={peak:.4f} deg")

    return {"ideal": ideal_angles, "measured": measured, "C0": C0, "A": A, "B": B}


def cmd_motor_sweep(host, sensor_model, groups=4, step_span=4, samples=15, run_in_microsteps=256, **_):
    """Fine microstep sweep across one full step, repeated at `groups` different
    absolute positions, to see whether the stepper's own within-step
    nonlinearity is a fixed, position-independent characteristic (evidence it
    is a genuine motor/driver property, correctable once) or varies with
    absolute position (evidence of something else, e.g. mechanical).
    """
    C0, A, B = sensor_model
    model = (C0, A, B)
    n_points = MICROSTEPS // step_span
    print(f"[motor-sweep] {groups} groups x {n_points} points ({step_span} microsteps/point, samples={samples})")

    # Establish a true mechanical full-step reference *before* measuring -
    # without this, "start here" is only ever an assumption, not a fact, and
    # the group-offset bug below would have gone undetected without first
    # asking whether motor-sweep measurements even started from a genuine
    # full-step reference in the first place.
    start_pos = align_fullstep(host)["stepPosition"]
    print(f"  aligned to true full-step position {start_pos}")

    if run_in_microsteps:
        # align_fullstep() just switched the TMC2209 out of full-step mode -
        # without this, group 0 would be the *only* group ever measured
        # immediately after that switch, while every later group already has
        # hundreds of ordinary 256-microstep moves behind it by the time it's
        # reached. A live A/B test (near vs. 180 degrees from the Hall
        # magnet) found group 0 anomalous in both positions, though less so
        # far from the magnet - this run-in (net zero motion) isolates how
        # much of that is a driver-settling artifact of the mode switch
        # itself versus genuine magnet proximity.
        jog(host, run_in_microsteps, 1)
        jog(host, -run_in_microsteps, 1)
        print(f"  ran in {run_in_microsteps} microsteps forward/back before measuring group 0")

    results = {}
    for g in range(groups):
        # A whole number of FULL STEPS apart, not a raw division of
        # FULLSTEPS_PER_ROTATION*MICROSTEPS by `groups` - the latter isn't
        # always a multiple of MICROSTEPS (e.g. groups=6:
        # 400*256//6=17066, 17066/256=66.664...), which silently
        # phase-misaligns each group's "microstep 0" against the full-step
        # reference above and against every other group. Confirmed real bug
        # in an earlier run.
        fullstep_offset = g * (FULLSTEPS_PER_ROTATION // groups)
        offset = fullstep_offset * MICROSTEPS
        jog(host, offset, 1)
        angles = []
        for _ in range(n_points):
            raw = jog(host, step_span, samples)["rawSensor"]
            angles.append(correct(raw, model))
        jog(host, -(offset + n_points * step_span), 1)  # back to start
        base = angles[0]
        unwrapped = [base + wrap4096(a - base) for a in angles]
        xs = list(range(n_points))
        slope, intercept = linear_fit(xs, unwrapped)
        residuals = [y - (slope * x + intercept) for x, y in zip(xs, unwrapped)]
        rms = math.sqrt(sum(r * r for r in residuals) / len(residuals)) * DEG_PER_COUNT
        peak = max(abs(r) for r in residuals) * DEG_PER_COUNT
        ideal_slope = 4096.0 / FULLSTEPS_PER_ROTATION / n_points
        print(
            f"  group {g}: slope={slope:.4f} counts/point (ideal={ideal_slope:.4f}), "
            f"detrended wobble RMS={rms:.4f} deg  peak={peak:.4f} deg"
        )
        results[g] = {"angles": unwrapped, "slope": slope, "residual_rms_deg": rms, "residual_peak_deg": peak}

    # Cross-correlate group 0's detrended wobble against the others - a high
    # correlation means the same microstep-level error shape shows up
    # regardless of absolute position (a motor/driver property); low
    # correlation would point elsewhere.
    ref = results[0]["angles"]
    ref_slope, ref_intercept = linear_fit(range(len(ref)), ref)
    ref_wobble = [y - (ref_slope * x + ref_intercept) for x, y in enumerate(ref)]
    for g in range(1, groups):
        other = results[g]["angles"]
        slope, intercept = linear_fit(range(len(other)), other)
        wobble = [y - (slope * x + intercept) for x, y in enumerate(other)]
        if len(wobble) == len(ref_wobble) and statistics.pstdev(wobble) > 0 and statistics.pstdev(ref_wobble) > 0:
            mean_r, mean_o = statistics.mean(ref_wobble), statistics.mean(wobble)
            cov = sum((r - mean_r) * (o - mean_o) for r, o in zip(ref_wobble, wobble)) / len(wobble)
            corr = cov / (statistics.pstdev(ref_wobble) * statistics.pstdev(wobble))
            print(f"  wobble shape correlation group0 vs group{g}: {corr:+.3f}")

    end_pos = jog(host, 0, 1)["stepPosition"]
    check_position_roundtrip("motor-sweep", start_pos, end_pos)
    return results


def cmd_fullstep_accuracy(host, revolutions=2, samples=16, fit_order=6, **_):
    """One AS5600 reading per full step (400/motor revolution), repeated
    `revolutions` times, to look for real motor full-step positioning error
    (cogging - period = 1 full step, so very high spatial frequency, order
    ~hundreds per revolution) underneath the low-order (1-6) sensor error
    fit_fourier() already characterizes.

    This can't on its own tell real motor cogging apart from a repeatable
    AS5600 sensor defect at the same spatial frequency - the AS5600 rides the
    motor shaft with no backlash between them, so both would look identical
    here. That distinction doesn't matter for encoder-readback compensation
    (nudge the commanded microstep target per full step until the AS5600
    reads the intended value - repeatable is all that takes); it does matter
    if the goal is the true physical full-step spacing, which needs an
    independent check (see scripts/camera_angle_sweep.py) against a subset of
    these points.
    """
    n = FULLSTEPS_PER_ROTATION
    print(f"[fullstep-accuracy] {revolutions} revolution(s) x {n} full steps, samples={samples}/point")
    start_pos = align_fullstep(host)["stepPosition"]
    print(f"  aligned to true full-step position {start_pos}")

    ideal = [4096.0 * i / n for i in range(n)]
    reps_raw = []
    t0 = time.time()
    for rev in range(revolutions):
        readings = []
        for i in range(n):
            raw = jog(host, MICROSTEPS, samples)["rawSensor"]
            readings.append(raw)
            if i % 100 == 0:
                print(f"  rev {rev} step {i:4d}/{n}  raw={raw:8.2f}  ({time.time() - t0:5.1f}s elapsed)")
        reps_raw.append(readings)

    # Chunked, not a single call - JOG_LIMIT in main/WebServer.cpp's
    # debug_jog_handler() is 2*FULLSTEPS_PER_ROTATION*MICROSTEPS, so this
    # would silently 400 past revolutions=2.
    remaining = -revolutions * n * MICROSTEPS
    jog_limit = 2 * FULLSTEPS_PER_ROTATION * MICROSTEPS
    end_pos = start_pos
    while remaining != 0:
        chunk = max(-jog_limit, min(jog_limit, remaining))
        end_pos = jog(host, chunk, 1)["stepPosition"]
        remaining -= chunk
    check_position_roundtrip("fullstep-accuracy", start_pos, end_pos)
    print(f"  measured in {time.time() - t0:.1f}s, returned to start")

    # Fit the usual low-order model against rev 0 only - matching what a
    # single ordinary sensor-sweep would see - so what's left below is
    # specifically the part that fit doesn't explain, not the low-order
    # error this project has already characterized many times over.
    model = fit_fourier(ideal, reps_raw[0], kmax=fit_order)
    smooth_rms, smooth_peak = residual_stats(ideal, reps_raw[0], model)
    print(f"  rev 0: order-{fit_order} fit residual RMS={smooth_rms:.4f} deg  peak={smooth_peak:.4f} deg (motor-shaft)")

    residuals = []
    for rep in reps_raw:
        res = [wrap4096(correct(raw, model) - ideal[i]) * DEG_PER_COUNT for i, raw in enumerate(rep)]
        residuals.append(res)
        rms = math.sqrt(sum(x * x for x in res) / n)
        print(f"  rev {len(residuals) - 1} full-step residual (post-fit): RMS={rms:.4f} deg  peak={max(abs(x) for x in res):.4f} deg")

    if revolutions >= 2:
        ref = residuals[0]
        sd_r = statistics.pstdev(ref)
        for r in range(1, revolutions):
            other = residuals[r]
            diff = [o - f for o, f in zip(other, ref)]
            diff_rms = math.sqrt(sum(d * d for d in diff) / n)
            sd_o = statistics.pstdev(other)
            mean_r, mean_o = statistics.mean(ref), statistics.mean(other)
            cov = sum((f - mean_r) * (o - mean_o) for f, o in zip(ref, other)) / n
            corr = cov / (sd_r * sd_o) if sd_r > 0 and sd_o > 0 else float("nan")
            verdict = "repeatable - likely real & compensable" if corr > 0.7 else (
                "weak/no repeatability - looks like noise, not a fixed per-step property")
            print(f"  rev0 vs rev{r}: shape correlation={corr:+.3f}  diff RMS={diff_rms:.4f} deg  -> {verdict}")

    return {
        "ideal": ideal, "reps_raw": reps_raw,
        "model": {"C0": model[0], "A": model[1], "B": model[2]},
        "residuals_deg": residuals,
    }


def cmd_backlash(host, distance=2000, samples=15, repeats=3, **_):
    """Approach the same nominal position from + and - direction; the
    difference is mechanical backlash/hysteresis the sensor+motor models
    cannot see or correct, since the AS5600 sits on the motor shaft, upstream
    of the 10:1 reduction to the output axis.
    """
    print(f"[backlash] {repeats} repeats, +/-{distance} microstep approach")
    diffs = []
    for r in range(repeats):
        jog(host, distance, 1)
        from_plus = jog(host, -distance, samples)["rawSensor"]
        jog(host, -distance, 1)
        from_minus = jog(host, distance, samples)["rawSensor"]
        diff = wrap4096(from_minus - from_plus)
        diffs.append(diff)
        print(
            f"  repeat {r}: from(+)={from_plus:8.2f}  from(-)={from_minus:8.2f}  "
            f"diff={diff:+7.2f} counts ({diff * DEG_PER_COUNT:+.4f} deg)"
        )
    print(f"  mean diff = {statistics.mean(diffs):+.2f} counts ({statistics.mean(diffs) * DEG_PER_COUNT:+.4f} deg)")
    return {"diffs_counts": diffs}



COMMANDS = {
    "noise": cmd_noise,
    "sensor-sweep": cmd_sensor_sweep,
    "motor-sweep": cmd_motor_sweep,
    "fullstep-accuracy": cmd_fullstep_accuracy,
    "backlash": cmd_backlash,
}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", required=True, help="rotator IP or hostname, e.g. 172.22.102.30")
    parser.add_argument("command", choices=list(COMMANDS) + ["all"])
    parser.add_argument("--revolutions", type=int, default=1, help="sensor-sweep: motor revolutions to cover")
    parser.add_argument("--samples", type=int, default=8, help="AS5600 averaging depth per measurement")
    parser.add_argument("--groups", type=int, default=4, help="motor-sweep: positions around the revolution to sample")
    parser.add_argument("--step-span", type=int, default=4, help="motor-sweep: microsteps between sample points")
    parser.add_argument("--run-in", type=int, default=256, dest="run_in_microsteps",
                         help="motor-sweep: microsteps to move forward/back after align_fullstep(), "
                              "before measuring group 0, to separate driver-settling from position effects "
                              "(0 to disable)")
    parser.add_argument("--fullstep-revolutions", type=int, default=2,
                         help="fullstep-accuracy: motor revolutions to repeat (>=2 needed for the repeatability check)")
    parser.add_argument("--fit-order", type=int, default=6, help="fullstep-accuracy: order of the low-order fit to subtract first")
    parser.add_argument("--distance", type=int, default=2000, help="backlash: approach distance in microsteps")
    parser.add_argument("--repeats", type=int, default=3, help="backlash: number of approach pairs")
    parser.add_argument("--load", help="reuse a prior --out JSON's sensor_sweep instead of re-sweeping")
    parser.add_argument("--out", help="write all results as JSON to this path")
    args = parser.parse_args()

    try:
        jog(args.host, 0, 1)  # fail fast with a clear error if the target/expert-gate isn't reachable
    except HTTPError as e:
        sys.exit(f"debug endpoint refused the request ({e.code}) - is expert mode unlocked on the target?")
    except URLError as e:
        sys.exit(f"could not reach {args.host}: {e.reason}")

    out = {}
    if args.load:
        with open(args.load) as f:
            loaded = json.load(f)
        if "sensor_sweep" in loaded:
            out["sensor_sweep"] = loaded["sensor_sweep"]
            print(f"[load] reusing sensor_sweep from {args.load} (C0={out['sensor_sweep']['C0']:.3f})")
        if "motor_sweep" in loaded:
            out["motor_sweep"] = loaded["motor_sweep"]
            print(f"[load] reusing motor_sweep from {args.load}")

    if args.command in ("noise", "all"):
        out["noise"] = cmd_noise(args.host, n=30)
    if args.command in ("sensor-sweep", "all") and "sensor_sweep" not in out:
        out["sensor_sweep"] = cmd_sensor_sweep(args.host, revolutions=args.revolutions, samples=args.samples)
    if args.command in ("motor-sweep", "all") and "sensor_sweep" not in out:
        print("[info] no sensor model available - sweeping one revolution first")
        out["sensor_sweep"] = cmd_sensor_sweep(args.host, revolutions=1, samples=args.samples)
    if args.command in ("motor-sweep", "all") and "motor_sweep" not in out:
        model = (out["sensor_sweep"]["C0"], out["sensor_sweep"]["A"], out["sensor_sweep"]["B"])
        out["motor_sweep"] = cmd_motor_sweep(
            args.host, model, groups=args.groups, step_span=args.step_span, samples=args.samples,
            run_in_microsteps=args.run_in_microsteps,
        )
    if args.command in ("fullstep-accuracy", "all"):
        out["fullstep_accuracy"] = cmd_fullstep_accuracy(
            args.host, revolutions=args.fullstep_revolutions, samples=args.samples, fit_order=args.fit_order,
        )
    if args.command in ("backlash", "all"):
        out["backlash"] = cmd_backlash(args.host, distance=args.distance, samples=args.samples, repeats=args.repeats)

    if args.out:
        with open(args.out, "w") as f:
            json.dump(out, f, indent=2)
        print(f"\nWrote results to {args.out}")


if __name__ == "__main__":
    main()
