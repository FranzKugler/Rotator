"""Franz's idea: use the 10 exact-full-motor-revolution points already inside
one camera_angle_sweep.py sweep (400 full steps apart = exactly 1/10 of an
output revolution, landing on a true mechanical full-step equilibrium each
time - see calibration_lab.py's align_fullstep() comment on why only
full-step positions are genuine equilibria) to measure the target's mounting
eccentricity in isolation.

Why this isolates eccentricity specifically: at those 10 points the AS5600 -
and, under the assumption this script's name states plainly, the real output
stage too - is in the SAME state every time (identical motor phase, exactly
1/10/.../9/10 whole revolutions apart). Whatever the camera measures
differently between them is then a function of the true OUTPUT angle alone,
which is exactly what target eccentricity is and a real motor/gearbox error
is not. This fits a low-order (up to 2nd harmonic - a plain offset target
mainly produces order 1) model to just those 10 points, then subtracts it
from the whole sweep to see what's left.

Operates on the JSON scripts/camera_angle_analyze.py already wrote (no
images, no opencv needed - cameraResidualDeg is already computed per point) -
standard library only.

Usage:
    python3 scripts/camera_eccentricity_calibrate.py --analysis /tmp/sweep1/analysis.json
"""

import argparse
import json
import math


def solve_linear(A, b):
    """Plain Gaussian elimination with partial pivoting - A is a small square
    matrix (5x5 here), avoids pulling in numpy for a script that otherwise
    needs nothing but the standard library.
    """
    n = len(A)
    M = [row[:] + [b[i]] for i, row in enumerate(A)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(M[r][col]))
        M[col], M[pivot] = M[pivot], M[col]
        if abs(M[col][col]) < 1e-12:
            raise ValueError("singular matrix - not enough distinct angles in the probe points")
        for r in range(n):
            if r == col:
                continue
            factor = M[r][col] / M[col][col]
            for c in range(col, n + 1):
                M[r][c] -= factor * M[col][c]
    return [M[i][n] / M[i][i] for i in range(n)]


def fit_low_order(thetas_deg, ys, order=2):
    """Least-squares fit of y = C0 + sum_{k=1..order} A_k cos(k*theta) + B_k sin(k*theta),
    via the normal equations - fine for the handful of points this is meant for.
    """
    basis = []
    for th in thetas_deg:
        t = math.radians(th)
        row = [1.0]
        for k in range(1, order + 1):
            row += [math.cos(k * t), math.sin(k * t)]
        basis.append(row)
    n_params = 1 + 2 * order
    AtA = [[sum(basis[i][r] * basis[i][c] for i in range(len(ys))) for c in range(n_params)] for r in range(n_params)]
    Atb = [sum(basis[i][r] * ys[i] for i in range(len(ys))) for r in range(n_params)]
    coeffs = solve_linear(AtA, Atb)
    C0 = coeffs[0]
    A = [0.0] + [coeffs[1 + 2 * (k - 1)] for k in range(1, order + 1)]
    B = [0.0] + [coeffs[2 + 2 * (k - 1)] for k in range(1, order + 1)]
    return C0, A, B


def eval_model(theta_deg, model):
    C0, A, B = model
    t = math.radians(theta_deg % 360.0)
    v = C0
    for k in range(1, len(A)):
        v += A[k] * math.cos(k * t) + B[k] * math.sin(k * t)
    return v


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--analysis", required=True, help="analysis.json written by camera_angle_analyze.py")
    ap.add_argument("--order", type=int, default=2, help="highest harmonic order to fit against the probe points")
    ap.add_argument("--out", default=None, help="write the eccentricity-corrected per-frame residuals here")
    args = ap.parse_args()

    with open(args.analysis) as f:
        data = json.load(f)
    rows = {r["index"]: r for r in data["rows"]}
    max_index = max(rows)
    if max_index % 10 != 0:
        raise SystemExit(f"expected a sweep sampled in exact tenths (index range a multiple of 10), got max index {max_index}")
    step = max_index // 10

    probe_indices = [i * step for i in range(11)]  # 0, 36, 72, ..., 360 for the default 360-sample sweep
    missing = [i for i in probe_indices if i not in rows]
    if missing:
        raise SystemExit(f"sweep is missing frames at the exact-tenth indices {missing} (failed detection?)")

    print(f"[eccentricity-calibrate] probing at indices {probe_indices} "
          f"(every {step} points = 400 full steps = exactly 1/10 output revolution)")

    # The closure check: index 0 and the last probe index are the SAME
    # physical output angle (0 deg == 360 deg), reached after the motor did
    # exactly 10 whole revolutions (verified separately by the sweep's own
    # position round-trip check) - so under Franz's assumption they should
    # read identically. Any difference is either genuine slip in the
    # reduction stage, or accumulated frame-to-frame tracking bias in
    # track_rotation()'s Procrustes chain (a systematic few-thousandths-of-a-
    # degree-per-step bias adds up like a random walk over 360 steps) -
    # this script can't tell those apart on its own, but it's the single
    # most important number to look at before trusting the fit below.
    closure_gap = rows[probe_indices[-1]]["cameraResidualDeg"] - rows[probe_indices[0]]["cameraResidualDeg"]
    print(f"  closure check (index {probe_indices[0]} vs {probe_indices[-1]}, same physical angle): "
          f"{closure_gap:+.4f} deg - {'small, tracking looks self-consistent' if abs(closure_gap) < 0.05 else 'NOT small - see the docstring: either real slip or accumulated tracking drift, do not fully trust the fit below without checking this first'}")

    thetas = [rows[i]["idealOutputDeg"] % 360.0 for i in probe_indices]
    ys = [rows[i]["cameraResidualDeg"] for i in probe_indices]
    model = fit_low_order(thetas, ys, order=args.order)
    C0, A, B = model
    print(f"  fitted eccentricity model (order {args.order}): C0={C0:.4f} deg")
    for k in range(1, args.order + 1):
        amp = math.hypot(A[k], B[k])
        phase = math.degrees(math.atan2(B[k], A[k]))
        print(f"    order {k}: amplitude={amp:.4f} deg  phase={phase:7.2f} deg")

    probe_residual_after = [y - eval_model(th, model) for th, y in zip(thetas, ys)]
    rms_before = math.sqrt(sum(y * y for y in ys) / len(ys))
    rms_after = math.sqrt(sum(e * e for e in probe_residual_after) / len(probe_residual_after))
    print(f"  at the 11 probe points: RMS {rms_before:.4f} deg -> {rms_after:.4f} deg after removing the fit")

    # Apply the fitted eccentricity model to every point in the sweep, not
    # just the 11 probe points - this is the actual payoff: a camera curve
    # with the target's mounting error calibrated out, leaving (ideally)
    # just the real output-stage behaviour plus whatever tracking noise the
    # closure check above found.
    all_rows = sorted(rows.values(), key=lambda r: r["index"])
    corrected = []
    for r in all_rows:
        ecc = eval_model(r["idealOutputDeg"], model)
        corrected.append({
            "index": r["index"],
            "idealOutputDeg": r["idealOutputDeg"],
            "cameraResidualDeg": r["cameraResidualDeg"],
            "eccentricityModelDeg": ecc,
            "eccentricityCorrectedResidualDeg": r["cameraResidualDeg"] - ecc,
        })
    all_resid = [c["cameraResidualDeg"] for c in corrected]
    all_corr = [c["eccentricityCorrectedResidualDeg"] for c in corrected]
    rms_full_before = math.sqrt(sum(y * y for y in all_resid) / len(all_resid))
    rms_full_after = math.sqrt(sum(e * e for e in all_corr) / len(all_corr))
    peak_full_after = max(abs(e) for e in all_corr)
    print(f"  across the full {len(all_rows)}-point sweep: camera residual RMS "
          f"{rms_full_before:.4f} deg -> {rms_full_after:.4f} deg after eccentricity removal "
          f"(peak {peak_full_after:.4f} deg)")

    if args.out:
        with open(args.out, "w") as f:
            json.dump({
                "probeIndices": probe_indices,
                "model": {"order": args.order, "C0": C0, "A": A, "B": B},
                "closureGapDeg": closure_gap,
                "rows": corrected,
            }, f, indent=2)
        print(f"  wrote {args.out}")


if __name__ == "__main__":
    main()
