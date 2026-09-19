"""Steps 1+2: turn a full-step sweep into a sensor characteristic and a
"measured angle -> real angle" model.

Model
-----
With the motor assumed linear, the true mechanical angle at support point i
is m_i = m_0 + i * 4096/400 counts, so the sensor's error is

    d_i = raw_unwrapped_i - m_i

and the correction the firmware needs is that same d, expressed as a
function of the *raw reading* (which is all the firmware has at correction
time):

    err(theta) = C0 + sum_k [ A_k cos(k theta) + B_k sin(k theta) ],
    theta = 2 pi raw / 4096,     corrected = raw - err(raw)

This is exactly main/RotatorHW.cpp:correctSensorReading()'s convention, so a
fit produced here can be uploaded unchanged.

Fitting err against theta (the measurement) rather than against the step
index is what makes the result usable: the same fit anchored to step 0
instead would be phase-rotated by (start angle * k) for every harmonic as
soon as a run did not begin at raw == 0.

Least squares over all support points of all forward passes; the harmonic
basis is orthogonal enough over a full revolution that no regularisation is
needed.
"""

import argparse
import json
import sys

import numpy as np

COUNTS = 4096.0
FULLSTEPS = 400
COUNTS_PER_FULLSTEP = COUNTS / FULLSTEPS
OUTPUT_DEG_PER_COUNT = 36.0 / COUNTS


def load(path):
    header, passes = None, {}
    with open(path) as handle:
        for line in handle:
            record = json.loads(line)
            if record["pass"] == "__header__":
                header = record
                continue
            passes.setdefault(record["pass"], []).append(record)
    # A run with several same-direction passes gets them keyed apart here,
    # so 'fwd' appearing twice stays two separate passes.
    return header, passes


def split_repeats(records):
    """A pass tag used twice in one run arrives as one concatenated list;
    split it wherever the index counter restarts."""
    out, current = [], []
    previous = None
    for record in records:
        if previous is not None and record["index"] <= previous:
            out.append(current)
            current = []
        current.append(record)
        previous = record["index"]
    out.append(current)
    return out


def unwrap(values, period=COUNTS):
    values = np.asarray(values, float)
    return values[0] + np.cumsum(np.concatenate(
        [[0.0], (np.diff(values) + period / 2) % period - period / 2]))


MICROSTEPS_PER_MOTOR_REV = 400 * 256
COUNTS_PER_MICROSTEP = COUNTS / MICROSTEPS_PER_MOTOR_REV  # exactly 1/25


def sweep_error(records):
    """(theta, d, raw, step, closure) for one pass, in sensor counts.

    The error is taken against the *absolute* commanded position, not
    against the pass's own first sample:

        d = raw_unwrapped - stepPosition / 25

    (25 microsteps per sensor count, exactly). Two reasons this matters.
    A run-up offsets each pass's starting point, so two passes' index i are
    not the same place on the machine and an index-wise comparison of them
    silently compares positions eight full steps apart. And a constant
    offset between a forward and a reverse pass is exactly the hysteresis
    the run is trying to measure, so it must not be defined away by
    referencing each pass to its own start.

    The unwrapped reading still carries an arbitrary whole-revolution
    branch, which is removed by snapping the pass's mean error onto the
    branch nearest zero - a whole number of revolutions, so it cannot
    absorb anything physical.
    """
    step = np.array([r["stepPosition"] for r in records], float)
    raw = unwrap([r["rawSensor"] for r in records])
    d = raw - step * COUNTS_PER_MICROSTEP
    d = d - COUNTS * np.round(d.mean() / COUNTS)

    order = np.argsort(step)
    step, raw, d = step[order], raw[order], d[order]
    closure = d[-1] - d[0]
    theta = 2.0 * np.pi * (raw % COUNTS) / COUNTS
    return theta, d, raw, step, closure


def seam_mask(raw, counts=14.0):
    """True for support points clear of the AS5600 wrap seam.

    The AS5600's ANGLE register carries a documented ~10 LSB hysteresis at
    the limit of its 360-degree range, and this rotator's firmware reads
    that register. Live-measured on 2026-09-18 with a 1-count fine sweep:
    travelling upwards the output sticks at exactly 4095 for 275 microsteps
    (11 counts) and then jumps straight to 11, so roughly 12 counts once per
    motor revolution carry no position information at all. Downwards the
    same crossing is smooth, which is what makes it hysteresis rather than a
    dead sensor.

    Those points are not measurements of anything and must not be allowed
    into a least-squares fit. The real fix is to read the RAW ANGLE register
    (0x0C) instead, which has no such hysteresis."""
    distance = np.minimum(raw % COUNTS, COUNTS - (raw % COUNTS))
    return distance > counts


def design(theta, order):
    columns = [np.ones_like(theta)]
    for k in range(1, order + 1):
        columns.append(np.cos(k * theta))
        columns.append(np.sin(k * theta))
    return np.column_stack(columns)


def fit(theta, d, order):
    M = design(theta, order)
    coefficients, *_ = np.linalg.lstsq(M, d, rcond=None)
    residual = d - M @ coefficients
    C0 = coefficients[0]
    A = np.zeros(order + 1)
    B = np.zeros(order + 1)
    for k in range(1, order + 1):
        A[k] = coefficients[2 * k - 1]
        B[k] = coefficients[2 * k]
    return C0, A, B, residual


def evaluate(theta, C0, A, B):
    value = np.full_like(theta, C0)
    for k in range(1, len(A)):
        value = value + A[k] * np.cos(k * theta) + B[k] * np.sin(k * theta)
    return value


def rms(x):
    return float(np.sqrt(np.mean(np.square(x))))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", required=True, help="sweep_fullstep.py .jsonl")
    ap.add_argument("--max-order", type=int, default=14)
    ap.add_argument("--order", type=int, default=7, help="order to report/export")
    ap.add_argument("--export", help="write the chosen fit as JSON here")
    ap.add_argument("--exclude-seam", action="store_true",
                    help="drop points in the AS5600 ANGLE-register wrap-seam "
                         "dead band - only needed for runs recorded before the "
                         "firmware switched to the RAW ANGLE register")
    args = ap.parse_args()

    header, passes = load(args.run)
    print(f"run: {header['utc']}  samples={header['samples']}  settle={header['settle']}")
    print(f"AS5600: {header['diagnostics']}\n")

    segments = []
    for tag, records in passes.items():
        for number, chunk in enumerate(split_repeats(records)):
            theta, d, raw, step, closure = sweep_error(chunk)
            name = f"{tag}#{number + 1}"
            segments.append({"name": name, "direction": chunk[0]["direction"],
                             "theta": theta, "d": d, "raw": raw, "step": step,
                             "closure": closure})
            print(f"{name:8s} dir={chunk[0]['direction']:+d}  n={len(chunk)}  "
                  f"span={raw[-1]-raw[0]:9.3f} counts  "
                  f"closure={closure:+7.3f} counts ({closure*OUTPUT_DEG_PER_COUNT*1000:+7.2f} mdeg)  "
                  f"p-p error={d.max()-d.min():7.3f} counts "
                  f"({(d.max()-d.min())*OUTPUT_DEG_PER_COUNT:6.3f} deg out)")

    forward = [s for s in segments if s["direction"] > 0]
    reverse = [s for s in segments if s["direction"] < 0]

    def compare(first, second, label):
        """Difference between two passes at the positions they share.

        Matching on stepPosition rather than on the index is what makes this
        meaningful - the run-up means equal indices are not equal places."""
        lookup = dict(zip(second["step"], second["d"]))
        shared = [(a, lookup[position])
                  for position, a in zip(first["step"], first["d"])
                  if position in lookup]
        if len(shared) < 10:
            print(f"{label}: only {len(shared)} shared positions - skipped")
            return
        delta = np.array([a - b for a, b in shared])
        spread = rms(delta - delta.mean())
        print(f"{label}: n={len(shared)}  mean={delta.mean():+.3f} counts "
              f"({delta.mean()*OUTPUT_DEG_PER_COUNT*1000:+.2f} mdeg out)  "
              f"rms about mean={spread:.3f} counts "
              f"({spread*OUTPUT_DEG_PER_COUNT*1000:.2f} mdeg out)  "
              f"p-p={delta.max()-delta.min():.3f} counts")

    print()
    if len(forward) >= 2:
        compare(forward[0], forward[1], "forward repeatability ")
    if forward and reverse:
        compare(forward[0], reverse[0], "fwd-vs-rev hysteresis ")

    # --- order sweep on the forward data ----------------------------------
    theta = np.concatenate([s["theta"] for s in forward])
    d = np.concatenate([s["d"] for s in forward])
    raw = np.concatenate([s["raw"] for s in forward])
    if args.exclude_seam:
        keep = seam_mask(raw)
        print(f"\nexcluding {(~keep).sum()} support points inside the AS5600 "
              f"wrap-seam dead band")
        theta, d = theta[keep], d[keep]
    print(f"fit on {len(theta)} forward support points")
    print(f"{'order':>5} {'resid rms':>10} {'resid p-p':>10} {'rms mdeg out':>13} "
          f"{'A_k':>9} {'B_k':>9} {'|c_k|':>9}")
    previous = None
    for order in range(1, args.max_order + 1):
        C0, A, B, residual = fit(theta, d, order)
        magnitude = np.hypot(A[order], B[order])
        gain = "" if previous is None else f"  ({100*(previous-rms(residual))/previous:+5.1f}%)"
        print(f"{order:5d} {rms(residual):10.4f} {residual.max()-residual.min():10.4f} "
              f"{rms(residual)*OUTPUT_DEG_PER_COUNT*1000:13.3f} "
              f"{A[order]:9.4f} {B[order]:9.4f} {magnitude:9.4f}{gain}")
        previous = rms(residual)

    # --- the chosen order --------------------------------------------------
    C0, A, B, residual = fit(theta, d, args.order)
    print(f"\n--- order {args.order} ---")
    print(f"C0 = {C0:.6f}")
    for k in range(1, args.order + 1):
        print(f"  k={k}: A={A[k]:+10.6f}  B={B[k]:+10.6f}  "
              f"amplitude={np.hypot(A[k],B[k]):8.4f} counts "
              f"({np.hypot(A[k],B[k])*OUTPUT_DEG_PER_COUNT*1000:7.2f} mdeg out)")
    print(f"residual rms = {rms(residual):.4f} counts "
          f"({rms(residual)*OUTPUT_DEG_PER_COUNT*1000:.2f} mdeg out)")

    if args.export:
        with open(args.export, "w") as handle:
            json.dump({"order": args.order, "C0": C0,
                       "A": A.tolist(), "B": B.tolist(),
                       "residualRmsCounts": rms(residual),
                       "source": args.run}, handle, indent=2)
        print(f"wrote {args.export}")


if __name__ == "__main__":
    main()
