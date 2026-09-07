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


def motor_scale_from_sensor_sweep(sensor_sweep):
    """The revolution-averaged counts/microstep scale, from the sensor-sweep's
    own data (each point commanded exactly MICROSTEPS microsteps). This is a
    global correction to the *ideal* 4096/(FULLSTEPS_PER_ROTATION*MICROSTEPS)
    scale - a real stepper's mechanical step angle is rarely exactly nominal -
    separate from the finer within-full-step wobble motor-sweep() measures.
    """
    measured = sensor_sweep["measured"]
    n = len(measured)
    # measured[i] is anchored within +/-2048 counts of ideal[i], which itself
    # resets every FULLSTEPS_PER_ROTATION points - undo that wrap to get a
    # monotonic unwrapped angle across the whole multi-revolution sweep.
    unwrapped = [m + 4096.0 * (i // FULLSTEPS_PER_ROTATION) for i, m in enumerate(measured)]
    xs = [i * MICROSTEPS for i in range(n)]
    slope, _ = linear_fit(xs, unwrapped)
    ideal_slope = 4096.0 / FULLSTEPS_PER_ROTATION / MICROSTEPS
    print(f"  motor scale from sensor sweep: {slope:.6f} counts/microstep (ideal={ideal_slope:.6f}, "
          f"{100 * (slope / ideal_slope - 1):+.3f}%)")
    return slope


def fit_motor_wobble(motor_sweep_groups, step_span=2, exclude=("0",), kmax=3):
    """Fourier model (period MICROSTEPS) of the within-full-step motor/driver
    wobble motor-sweep() measures, pooled across groups (default: all but
    group 0, the Hall-magnet-adjacent outlier - see the published lab
    report). Each group's own angles are detrended (its own best-fit line
    removed) before pooling, exactly like the per-group wobble analysis in
    cmd_motor_sweep(), so this is independent of any group's absolute
    position or slope estimate.
    """
    phases, wobbles = [], []
    for g, data in motor_sweep_groups.items():
        if g in exclude:
            continue
        angles = data["angles"] if isinstance(data, dict) else data
        n = len(angles)
        slope, intercept = linear_fit(range(n), angles)
        for i, a in enumerate(angles):
            phases.append((i * step_span) % MICROSTEPS)
            wobbles.append(a - (slope * i + intercept))

    n = len(phases)
    sum0 = sum(wobbles)
    sumC = [0.0] * (kmax + 1)
    sumS = [0.0] * (kmax + 1)
    for phase, w in zip(phases, wobbles):
        theta = 2.0 * math.pi * phase / MICROSTEPS
        for k in range(1, kmax + 1):
            sumC[k] += w * math.cos(k * theta)
            sumS[k] += w * math.sin(k * theta)
    C0 = sum0 / n
    A = [0.0] + [2.0 * sumC[k] / n for k in range(1, kmax + 1)]
    B = [0.0] + [2.0 * sumS[k] / n for k in range(1, kmax + 1)]
    print(f"  motor wobble model (pooled {len(motor_sweep_groups) - len(exclude)} groups, {n} points): "
          f"C0={C0:.4f}  " + "  ".join(f"A{k}={A[k]:.4f}/B{k}={B[k]:.4f}" for k in range(1, kmax + 1)))
    return (C0, A, B)


def motor_wobble(phase_microsteps, model):
    """Evaluate the fitted within-full-step wobble at a given (known, commanded)
    phase - counts of angle deviation from the ideal linear microstep ramp."""
    C0, A, B = model
    theta = 2.0 * math.pi * phase_microsteps / MICROSTEPS
    val = C0
    for k in range(1, len(A)):
        val += A[k] * math.cos(k * theta) + B[k] * math.sin(k * theta)
    return val


class SensorMotorEKF:
    """The simplest useful version of the EKF sketched (but left unfinished,
    dead code) in RotatorHW.cpp's h_meas()/H_jacobian()/ekf_predict()/
    ekf_update(): state x is the estimated absolute motor-shaft angle, in
    AS5600 count units, unwrapped (not wrapped to 0..4096). Predict advances
    it by a commanded microstep delta through the measured (not ideal) motor
    scale; update pulls it toward the raw sensor reading through the fitted
    eccentricity model, exactly like correctSensorReading() but expressed as
    an EKF measurement function so its Jacobian can weight the correction by
    how sensitive the sensor curve is at the current angle.

    The optional `motor_wobble_model` is a second Fourier model (period
    MICROSTEPS, see fit_motor_wobble()) for the within-full-step motor/driver
    nonlinearity found by motor-sweep. Unlike the sensor model, this one is
    evaluated at the *commanded* microstep count - a known, exact quantity,
    not the uncertain state - so applying it needs no Jacobian/linearization
    at all: it is just a better deterministic prediction, folded straight
    into the process model. Tracking `u` (cumulative commanded microsteps)
    separately from `x` (the fused, uncertain estimate) is what makes that
    possible.
    """

    def __init__(self, x0, sensor_model, motor_scale, q, r, motor_wobble_model=None, u0=0):
        self.x = x0
        self.P = 1.0
        self.model = sensor_model
        self.motor_scale = motor_scale
        self.Q = q
        self.R = r
        self.motor_wobble_model = motor_wobble_model
        self.u = u0

    def predict(self, delta_microsteps):
        if self.motor_wobble_model is not None:
            u_new = self.u + delta_microsteps
            g_old = motor_wobble(self.u % MICROSTEPS, self.motor_wobble_model)
            g_new = motor_wobble(u_new % MICROSTEPS, self.motor_wobble_model)
            self.x += self.motor_scale * delta_microsteps + (g_new - g_old)
            self.u = u_new
        else:
            self.x += self.motor_scale * delta_microsteps
        self.P += self.Q

    def _h_and_H(self):
        C0, A, B = self.model
        theta = 2.0 * math.pi * self.x / 4096.0
        dtheta_dx = 2.0 * math.pi / 4096.0
        err = C0
        Hj = 1.0
        for k in range(1, len(A)):
            err += A[k] * math.cos(k * theta) + B[k] * math.sin(k * theta)
            Hj += dtheta_dx * k * (-A[k] * math.sin(k * theta) + B[k] * math.cos(k * theta))
        return self.x + err, Hj

    def update(self, raw_measurement):
        pred, Hj = self._h_and_H()
        y = wrap4096(raw_measurement - pred)
        S = Hj * self.P * Hj + self.R
        K = self.P * Hj / S
        self.x += K * y
        self.P *= 1.0 - K * Hj


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


def cmd_motor_sweep(host, sensor_model, groups=4, step_span=4, samples=15, **_):
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
    start_pos = jog(host, 0, 1)["stepPosition"]
    results = {}
    for g in range(groups):
        offset = g * (FULLSTEPS_PER_ROTATION * MICROSTEPS // groups)
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


def cmd_ekf_walk(host, sensor_model, motor_scale, n=120, max_step=400, ref_samples=40, op_samples=5,
                  fault_at=60, fault_microsteps=60, seed=42, **_):
    """Collect a random walk of small moves for offline EKF validation.

    At each point records: the microstep delta actually commanded (ground
    truth motion), a heavily-averaged "reference" reading (the best proxy for
    true position this rig has), and a lightly-averaged "operational" reading
    (what a live tracker would realistically read on every step). Partway
    through, one step's *reported* delta is deliberately wrong by
    `fault_microsteps` while the real motion (and therefore the sensor
    readings) reflect what actually happened - simulating a missed/miscounted
    step without needing to actually cause one. This is later replayed
    offline through three estimators with no further hardware access needed.
    """
    import random

    rng = random.Random(seed)
    print(f"[ekf-walk] {n} random steps, fault of {fault_microsteps} microsteps injected at step {fault_at}")
    start_pos = jog(host, 0, 1)["stepPosition"]
    records = []
    for i in range(n):
        true_delta = rng.randint(20, max_step) * rng.choice([1, -1])
        jog(host, true_delta, 1)
        ref = jog(host, 0, ref_samples)["rawSensor"]
        op = jog(host, 0, op_samples)["rawSensor"]
        reported_delta = true_delta - fault_microsteps if i == fault_at else true_delta
        records.append({"true_delta": true_delta, "reported_delta": reported_delta, "ref": ref, "op": op})
        if i % 20 == 0:
            print(f"  step {i:3d}/{n}")
    total_true = sum(r["true_delta"] for r in records)
    end_pos = jog(host, -total_true, 1)["stepPosition"]
    check_position_roundtrip("ekf-walk", start_pos, end_pos)
    return {"records": records, "sensor_model": sensor_model, "motor_scale": motor_scale, "start_pos": start_pos}


def analyze_ekf_walk(walk, motor_wobble_model=None):
    """Offline replay: naive open-loop step counting vs sensor-only vs the
    fused EKF (with and without the within-full-step motor wobble model, if
    one is supplied), all judged against the heavily-averaged reference
    reading (corrected through the same sensor model) at each step. No
    further hardware access - this is the point of collecting `records` up
    front.
    """
    records = walk["records"]
    model = tuple(walk["sensor_model"]) if not isinstance(walk["sensor_model"], tuple) else walk["sensor_model"]
    C0, A, B = model
    model = (C0, A, B)
    ideal_scale = 4096.0 / FULLSTEPS_PER_ROTATION / MICROSTEPS
    motor_scale = walk["motor_scale"]

    # records[0]'s ref/op readings were already taken *after* records[0]'s
    # move (see cmd_ekf_walk: jog() happens before the readings), so
    # initializing state from records[0]["op"] already reflects that first
    # move - applying records[0]["reported_delta"] again in the loop would
    # double-count it. Each subsequent record's delta is the move that
    # happened *between* the previous reading and this one, so it is applied
    # before comparing against that record's truth.
    x0 = correct(records[0]["op"], model)
    naive_x = x0
    r_op = (0.4714 ** 2) / 5  # R for op_samples=5, from the noise probe
    ekf = SensorMotorEKF(x0, model, motor_scale, q=0.3 ** 2, r=r_op)
    ekf_wobble = None
    if motor_wobble_model is not None:
        # u tracks *reported* (not true) cumulative microsteps - the wobble
        # model, like everything else here, only ever sees what the tracker
        # was told, exactly mirroring a real onboard estimator.
        u0 = walk["start_pos"] + records[0]["reported_delta"]
        ekf_wobble = SensorMotorEKF(x0, model, motor_scale, q=0.15 ** 2, r=r_op,
                                     motor_wobble_model=motor_wobble_model, u0=u0)

    rows = [{
        "naive_err_deg": 0.0,
        "sensor_only_err_deg": 0.0,
        "ekf_err_deg": 0.0,
        "ekf_wobble_err_deg": 0.0,
    }]
    for rec in records[1:]:
        ref_truth = correct(rec["ref"], model)

        naive_x += ideal_scale * rec["reported_delta"]
        sensor_only = correct(rec["op"], model)
        ekf.predict(rec["reported_delta"])
        if ekf_wobble is not None:
            ekf_wobble.predict(rec["reported_delta"])
            ekf_wobble.update(rec["op"])
        ekf.update(rec["op"])

        row = {
            "naive_err_deg": wrap4096(naive_x - ref_truth) * DEG_PER_COUNT,
            "sensor_only_err_deg": wrap4096(sensor_only - ref_truth) * DEG_PER_COUNT,
            "ekf_err_deg": wrap4096(ekf.x - ref_truth) * DEG_PER_COUNT,
        }
        if ekf_wobble is not None:
            row["ekf_wobble_err_deg"] = wrap4096(ekf_wobble.x - ref_truth) * DEG_PER_COUNT
        rows.append(row)

    def rms(key, skip_before=0):
        vals = [r[key] for r in rows[skip_before:] if key in r]
        return math.sqrt(sum(v * v for v in vals) / len(vals)) if vals else float("nan")

    fault_at = None
    for i, rec in enumerate(records):
        if rec["reported_delta"] != rec["true_delta"]:
            fault_at = i
    keys = ["naive_err_deg", "sensor_only_err_deg", "ekf_err_deg"] + (["ekf_wobble_err_deg"] if ekf_wobble else [])
    print("[ekf-walk analysis]")
    print("  RMS error, whole walk     : " + "  ".join(f"{k.replace('_err_deg', '')}={rms(k):.4f}" for k in keys) + " (deg)")
    if fault_at is not None:
        print(f"  RMS error, after the fault: " + "  ".join(f"{k.replace('_err_deg', '')}={rms(k, fault_at):.4f}" for k in keys) + " (deg)")
    return rows


COMMANDS = {
    "noise": cmd_noise,
    "sensor-sweep": cmd_sensor_sweep,
    "motor-sweep": cmd_motor_sweep,
    "backlash": cmd_backlash,
    "ekf-walk": cmd_ekf_walk,
}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", required=True, help="rotator IP or hostname, e.g. 172.22.102.30")
    parser.add_argument("command", choices=list(COMMANDS) + ["all"])
    parser.add_argument("--revolutions", type=int, default=1, help="sensor-sweep: motor revolutions to cover")
    parser.add_argument("--samples", type=int, default=8, help="AS5600 averaging depth per measurement")
    parser.add_argument("--groups", type=int, default=4, help="motor-sweep: positions around the revolution to sample")
    parser.add_argument("--step-span", type=int, default=4, help="motor-sweep: microsteps between sample points")
    parser.add_argument("--distance", type=int, default=2000, help="backlash: approach distance in microsteps")
    parser.add_argument("--repeats", type=int, default=3, help="backlash: number of approach pairs")
    parser.add_argument("--walk-n", type=int, default=120, help="ekf-walk: number of random steps")
    parser.add_argument("--fault-at", type=int, default=60, help="ekf-walk: step index to misreport")
    parser.add_argument("--fault-microsteps", type=int, default=60, help="ekf-walk: size of the injected miscount")
    parser.add_argument("--ref-samples", type=int, default=40, help="ekf-walk: averaging depth for the 'truth' reading")
    parser.add_argument("--op-samples", type=int, default=5, help="ekf-walk: averaging depth for the per-step reading")
    parser.add_argument("--seed", type=int, default=42, help="ekf-walk: RNG seed, for reproducible walks")
    parser.add_argument("--load", help="reuse a prior --out JSON's sensor_sweep instead of re-sweeping")
    parser.add_argument("--load-motor", help="reuse a prior --out JSON's motor_sweep for the wobble model (ekf-walk)")
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
    if args.load_motor:
        with open(args.load_motor) as f:
            loaded = json.load(f)
        if "motor_sweep" in loaded:
            out["motor_sweep"] = loaded["motor_sweep"]
            print(f"[load] reusing motor_sweep from {args.load_motor}")

    if args.command in ("noise", "all"):
        out["noise"] = cmd_noise(args.host, n=30)
    if args.command in ("sensor-sweep", "all") and "sensor_sweep" not in out:
        out["sensor_sweep"] = cmd_sensor_sweep(args.host, revolutions=args.revolutions, samples=args.samples)
    if args.command in ("motor-sweep", "all", "ekf-walk"):
        if "sensor_sweep" not in out:
            print("[info] no sensor model available - sweeping one revolution first")
            out["sensor_sweep"] = cmd_sensor_sweep(args.host, revolutions=1, samples=args.samples)
    if args.command in ("motor-sweep", "all") and "motor_sweep" not in out:
        model = (out["sensor_sweep"]["C0"], out["sensor_sweep"]["A"], out["sensor_sweep"]["B"])
        out["motor_sweep"] = cmd_motor_sweep(
            args.host, model, groups=args.groups, step_span=args.step_span, samples=args.samples
        )
    if args.command in ("backlash", "all"):
        out["backlash"] = cmd_backlash(args.host, distance=args.distance, samples=args.samples, repeats=args.repeats)
    if args.command in ("ekf-walk", "all"):
        model = (out["sensor_sweep"]["C0"], out["sensor_sweep"]["A"], out["sensor_sweep"]["B"])
        scale = motor_scale_from_sensor_sweep(out["sensor_sweep"])
        wobble_model = fit_motor_wobble(out["motor_sweep"], step_span=args.step_span) if "motor_sweep" in out else None
        if wobble_model is None:
            print("[ekf-walk] no motor_sweep available (pass --load-motor or run motor-sweep first) - "
                  "skipping the wobble-aware estimator")
        walk = cmd_ekf_walk(
            args.host, model, scale, n=args.walk_n, fault_at=args.fault_at,
            fault_microsteps=args.fault_microsteps, ref_samples=args.ref_samples,
            op_samples=args.op_samples, seed=args.seed,
        )
        walk["rows"] = analyze_ekf_walk(walk, motor_wobble_model=wobble_model)
        out["ekf_walk"] = walk

    if args.out:
        with open(args.out, "w") as f:
            json.dump(out, f, indent=2)
        print(f"\nWrote results to {args.out}")


if __name__ == "__main__":
    main()
