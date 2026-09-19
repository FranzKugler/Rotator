"""Decompose an Alpaca test run into the terms that can be corrected and the
part that cannot, and say honestly how much of each.

Three quantities, deliberately kept apart:

  loop error     the device's own sensor-derived angle against its Alpaca
                 target - how well the firmware's closed loop does its job,
                 which is only about the motor shaft;
  output error   the camera against the Alpaca target - what a user gets;
  sensor-blind   the camera against the sensor-derived angle - everything
                 downstream of the reduction.

The correctable part is fitted as harmonics of the output angle plus one
direction term for backlash, and **scored by k-fold cross-validation**. A
harmonic model with sixteen parameters will always look better in sample;
the only figure worth quoting is what it achieves on targets it was not
fitted to, and that is what a real correction would face.
"""

import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from gear_error import sensor_angles
from measure_frames import (load_manifest, detect_all, angles_from_corners,
                            resolve_flips)


def centred(degrees):
    """Mean-removed, in mdeg, done on the circle so a wrap cannot bias it."""
    radians = np.radians(degrees)
    offset = np.arctan2(np.sin(radians).mean(), np.cos(radians).mean())
    return np.degrees(np.angle(np.exp(1j * (radians - offset)))) * 1000.0


def design(angle_deg, direction, orders, use_direction):
    phase = np.radians(angle_deg)
    columns = [np.ones_like(phase)]
    for k in orders:
        columns += [np.cos(k * phase), np.sin(k * phase)]
    if use_direction:
        columns.append(direction)
    return np.column_stack(columns)


def cross_validated(M, y, folds=5, seed=0):
    """Residual rms on held-out targets, averaged over the folds."""
    rng = np.random.default_rng(seed)
    index = rng.permutation(len(y))
    held = []
    for fold in range(folds):
        test = index[fold::folds]
        train = np.setdiff1d(index, test)
        if len(train) <= M.shape[1]:
            return float("nan")
        coefficients, *_ = np.linalg.lstsq(M[train], y[train], rcond=None)
        held.append(y[test] - M[test] @ coefficients)
    return float(np.sqrt(np.mean(np.concatenate(held) ** 2)))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dir", required=True)
    ap.add_argument("--coefficients", required=True)
    ap.add_argument("--folds", type=int, default=5)
    ap.add_argument("--export")
    ap.add_argument("--export-model",
                    help="write a correction in gear_model.GearModel form, fitted "
                         "only on the targets that were approached in the direction "
                         "given by --model-direction")
    ap.add_argument("--model-direction", type=int, default=1, choices=(1, -1))
    ap.add_argument("--model-orders", default="1,2,3,4,8,10,20")
    args = ap.parse_args()

    with open(args.coefficients) as handle:
        fit = json.load(handle)
    header, frames = load_manifest(args.dir)
    corners, ok = detect_all(args.dir, frames, quiet=True)
    keep = [i for i, good in enumerate(ok) if good]
    corners = [corners[i] for i in keep]
    usable = [frames[i] for i in keep]
    print(f"{len(usable)}/{len(frames)} frames usable, mode={header['mode']}, "
          f"approach={header.get('approachDeg', 0)} deg, "
          f"precompensated={bool(header.get('precompensate'))}")

    raw = np.array([f["rawSensor"] for f in usable])
    step = np.array([f["stepPosition"] for f in usable], float)
    sensor = sensor_angles(raw, step, fit["C0"], fit["A"], fit["B"])
    target = np.array([f["commanded"]["alpacaTargetDeg"] for f in usable])
    # On a pre-compensated run the loop is asked for a deliberately different
    # angle than the target, so judging it against the target would report
    # the correction as loop error. It is judged against what it was told.
    commanded = np.array([f["commanded"].get("alpacaCommandedDeg",
                                             f["commanded"]["alpacaTargetDeg"])
                          for f in usable])
    direction = np.sign(np.diff(step, prepend=step[0]))
    direction[direction == 0] = 1.0

    camera, anisotropy, residual_px = angles_from_corners(corners)
    best = None
    for sign in (1.0, -1.0):
        resolved, flips = resolve_flips(sign * camera, sensor)
        value = centred(resolved - sensor)
        if best is None or value.std() < best[0]:
            best = (value.std(), sign, resolved, flips)
    _, sign, camera, flips = best

    # The Alpaca scale and the step counter's mechanical scale differ by a
    # fixed offset, which any correction expressed against one and applied
    # to the other has to carry with it.
    alpaca_offset = np.degrees(np.angle(np.exp(1j * np.radians(sensor - target)).mean()))
    loop = centred(sensor - commanded)
    output = centred(camera - target)
    blind = centred(camera - sensor)

    print(f"\n  Alpaca scale sits {alpaca_offset:+.4f} deg from the mechanical angle")
    print(f"  {'firmware loop (sensor vs what it was commanded)':46s} "
          f"rms {loop.std():6.2f}  max {np.abs(loop).max():7.2f} mdeg")
    print(f"  {'what the user gets (camera vs target)':46s} "
          f"rms {output.std():6.2f}  max {np.abs(output).max():7.2f} mdeg")
    print(f"  {'downstream of the sensor (camera vs sensor)':46s} "
          f"rms {blind.std():6.2f}  max {np.abs(blind).max():7.2f} mdeg")
    forward, reverse = output[direction > 0], output[direction < 0]
    print(f"\n  backlash: forward targets {forward.mean():+.2f} mdeg (n={len(forward)}), "
          f"reverse {reverse.mean():+.2f} (n={len(reverse)}), "
          f"split {forward.mean()-reverse.mean():+.2f} mdeg")
    print(f"  camera health: homography {residual_px.mean():.4f} px, "
          f"anisotropy {np.abs(anisotropy).max():.2e}, {flips} labelling flips resolved")

    print(f"\n  correctable part of the user-facing error, "
          f"{args.folds}-fold cross-validated:")
    print(f"  {'model':40s} {'params':>7} {'in-sample':>10} {'held-out':>10}")
    table = []
    for orders, use_direction in [((), True), ((1, 2, 3), False), ((1, 2, 3), True),
                                  ((1, 2, 3, 10), True), ((1, 2, 3, 4, 8, 10), True),
                                  ((1, 2, 3, 4, 8, 10, 20), True),
                                  (tuple(range(1, 11)), True),
                                  (tuple(range(1, 21)), True)]:
        M = design(sensor, direction, orders, use_direction)
        coefficients, *_ = np.linalg.lstsq(M, output, rcond=None)
        in_sample = float(np.sqrt(np.mean((output - M @ coefficients) ** 2)))
        held = cross_validated(M, output, args.folds)
        label = ("direction only" if not orders
                 else f"k={list(orders)}" + (" + direction" if use_direction else ""))
        print(f"  {label:40.40s} {M.shape[1]:7d} {in_sample:10.2f} {held:10.2f}")
        table.append({"orders": list(orders), "direction": use_direction,
                      "params": int(M.shape[1]), "inSampleMdeg": in_sample,
                      "heldOutMdeg": held})

    print(f"\n  uncorrected: {output.std():.2f} mdeg rms, "
          f"{100*np.mean(np.abs(output)<=10):.0f} % within 10 mdeg")

    if args.export_model:
        # Fitted on one approach direction only. A run that reaches every
        # target from the same side has no backlash term left to fit, and
        # mixing both directions into one harmonic model would bury half the
        # backlash inside the harmonics, where it does not belong.
        orders = [int(x) for x in args.model_orders.split(",")]
        subset = direction == args.model_direction
        if subset.sum() <= 2 * len(orders) + 2:
            sys.exit(f"only {subset.sum()} targets approached in direction "
                     f"{args.model_direction:+d} - not enough to fit {len(orders)} orders")
        M = design(sensor[subset], direction[subset], orders, False)
        coefficients, *_ = np.linalg.lstsq(M, output[subset] / 1000.0, rcond=None)
        held = cross_validated(M, output[subset] / 1000.0, args.folds) * 1000.0
        with open(args.export_model, "w") as handle:
            json.dump({"model": {"orders": orders,
                                 "constant": coefficients[0],
                                 "cos": [coefficients[2 * i + 1] for i in range(len(orders))],
                                 "sin": [coefficients[2 * i + 2] for i in range(len(orders))],
                                 "reference": "absolute mechanical angle in degrees, "
                                              "as counted from the machine's mechanical zero",
                                 "alpacaToMechanicalDeg": float(alpaca_offset)},
                       "fittedOn": int(subset.sum()),
                       "approachDirection": args.model_direction,
                       "heldOutMdeg": held,
                       "source": args.dir}, handle, indent=2)
        print(f"\n  wrote {args.export_model}: {len(orders)} orders fitted on "
              f"{subset.sum()} targets approached {args.model_direction:+d}, "
              f"held-out {held:.2f} mdeg")

    if args.export:
        with open(args.export, "w") as handle:
            json.dump({"dir": args.dir, "n": len(usable),
                       "loopMdeg": loop.tolist(), "outputMdeg": output.tolist(),
                       "blindMdeg": blind.tolist(),
                       "sensorDeg": sensor.tolist(), "targetDeg": target.tolist(),
                       "direction": direction.tolist(), "models": table},
                      handle, indent=2)
        print(f"  wrote {args.export}")


if __name__ == "__main__":
    main()
