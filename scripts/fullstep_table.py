"""Builds a 400-entry per-full-step AS5600 correction table from
calibration_lab.py's fullstep-accuracy output, as a shape-only (mean
subtracted) lookup replacement for the fixed 4th-order Fourier model
(A[]/B[] in config.json) - see cmd_fullstep_accuracy()'s docstring for why a
direct lookup table sidesteps the self-referential-phase-basis bias a curve
fit suffers from at this resolution (measured, not fitted: order-6 self-fit
turned a 0.64 deg RMS residual into 2.3-6.3 deg; the raw table reproduces the
400 measured points essentially exactly, limited only by the ~0.04 deg
rev-to-rev repeatability noise floor).

Deliberately mean-subtracted (zero-mean shape only, like A[]/B[] already are)
so this table is portable across any C0 - C0 stays config.json's job, still
calibrated separately against gotoMechanicalZero()'s own reference. Averages
however many revolutions were measured (>=2 recommended) to further reduce
noise beyond what a single sweep gives.

Usage:
    python3 scripts/fullstep_table.py --in fullstep_accuracy.json --out table.json
"""

import argparse
import json
import math


def wrap4096(x):
    return x - 4096.0 * round(x / 4096.0)


def despike(values, threshold=5.0):
    """Replaces isolated points that deviate from their neighbors' average by
    more than `threshold` counts, one at a time (worst first, each fix using
    its now-corrected neighbors before the next comparison) - found to be
    necessary by a real, exactly-reproduced artifact at the AS5600 raw
    register's 4095->0 wraparound (both measured revolutions read exactly
    raw=4095.000 at the same full step, where the smooth trend implies the
    register should already have wrapped to roughly 6) - looks like a torn
    read straddling the rollover, not a physical position error, and baking
    it into a firmware correction table would be wrong.

    One-at-a-time matters: a single bad point drags its immediate neighbors'
    *own* neighbor-average test off too (comparing against one good + one
    spiky value), which a single all-at-once pass would misflag as two more
    spikes. threshold=5 counts is well above the ~0.5-1 count typical
    point-to-point step but below the ~7 count size of the one artifact
    found, so it won't need to trigger more than once here.
    """
    n = len(values)
    fixed = list(values)
    spikes = []
    while True:
        worst_i, worst_dev = None, threshold
        for i in range(n):
            prev, cur, nxt = fixed[(i - 1) % n], fixed[i], fixed[(i + 1) % n]
            dev = abs(cur - (prev + nxt) / 2)
            if dev > worst_dev:
                worst_i, worst_dev = i, dev
        if worst_i is None:
            break
        prev, nxt = fixed[(worst_i - 1) % n], fixed[(worst_i + 1) % n]
        fixed[worst_i] = (prev + nxt) / 2
        spikes.append(worst_i)
    return fixed, spikes


def build_table(fa):
    ideal = fa["ideal"]
    reps = fa["reps_raw"]
    n = len(ideal)

    # Average the raw reading at each full-step index across all measured
    # revolutions first (reduces noise), THEN compute the correction - not
    # the other way around, so a single outlier reading in one rev can't
    # skew the shape more than its share.
    avg_raw = [sum(rep[i] for rep in reps) / len(reps) for i in range(n)]
    raw_error = [wrap4096(avg_raw[i] - ideal[i]) for i in range(n)]
    raw_error, spikes = despike(raw_error)
    if spikes:
        print(f"  despiked {len(spikes)} point(s) at index {spikes} - see despike()'s docstring")
    mean_error = sum(raw_error) / n
    table = [e - mean_error for e in raw_error]  # zero-mean shape only

    rms = math.sqrt(sum(t * t for t in table) / n) * 360.0 / 4096.0
    peak = max(abs(t) for t in table) * 360.0 / 4096.0
    return table, mean_error, rms, peak


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--in", dest="infile", required=True, help="fullstep_accuracy.json from calibration_lab.py")
    ap.add_argument("--out", required=True, help="where to write the table JSON")
    args = ap.parse_args()

    with open(args.infile) as f:
        data = json.load(f)
    fa = data.get("fullstep_accuracy", data)  # accept either the wrapped --out or a bare cmd result

    table, mean_error, rms, peak = build_table(fa)
    n = len(table)
    print(f"built {n}-entry table from {len(fa['reps_raw'])} revolution(s)")
    print(f"  shape RMS={rms:.4f} deg  peak={peak:.4f} deg (motor-shaft) - "
          f"mean offset {mean_error:.2f} counts discarded (that's C0's job, not this table's)")

    with open(args.out, "w") as f:
        json.dump({
            "fullstepsPerRotation": n,
            "table": table,
            "note": "zero-mean shape only (counts); add to whatever C0 is currently in use, "
                    "index by round(raw / (4096/n)) mod n with linear interpolation to the next entry",
        }, f, indent=2)
    print(f"  wrote {args.out}")

    # Proposed firmware array, for review - not written into the repo here.
    print("\n--- proposed C array (RotatorHW.cpp), for review ---")
    print(f"static const float FULLSTEP_TABLE[{n}] = {{")
    for i in range(0, n, 10):
        row = ", ".join(f"{v:.3f}f" for v in table[i:i + 10])
        print(f"    {row},")
    print("};")


if __name__ == "__main__":
    main()
