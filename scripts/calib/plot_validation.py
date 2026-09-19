"""Graphs for the end-to-end test run: random targets commanded over ASCOM
Alpaca, measured against the chessboard on the output shaft.

This is the only measurement in the campaign that exercises what a user
actually does - Alpaca MoveAbsolute, the firmware's own correction and
settling, no debug endpoints - and the only one that judges the output shaft
rather than the sensor. Everything else feeds into it."""

import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import vizstyle
from vizstyle import SERIES, INK_SOFT, plt

CAMERA_REPEATABILITY_MDEG = 2.3


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--run", required=True, help="measure_frames.py --export JSON")
    ap.add_argument("--gear", help="gear_error.py --export JSON, overlaid if given")
    ap.add_argument("--gear-offset", type=float, default=0.0,
                    help="degrees to add to the gear run's angles to put them on "
                         "the Alpaca scale. The gear measurement is referenced to "
                         "the step counter's mechanical angle and the test run to "
                         "Alpaca Position; on this machine those differ by 20.2539 "
                         "deg, so overlaying them without this is meaningless")
    ap.add_argument("--outdir", required=True)
    ap.add_argument("--title", default="Alpaca test run")
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    vizstyle.apply()
    with open(args.run) as handle:
        data = json.load(handle)

    target = np.array(data["commandedDeg"])
    error = np.array(data["errorDeg"]) * 1000.0
    # Plot against the target folded to +/-180, which is how a user thinks
    # about it, not the unwrapped sequence the run happened to visit in.
    folded = (target + 180.0) % 360.0 - 180.0
    order = np.argsort(folded)

    figure = plt.figure(figsize=(10.5, 7.6))
    grid = figure.add_gridspec(2, 2, height_ratios=[3, 2], hspace=0.38, wspace=0.26)

    ax = figure.add_subplot(grid[0, :])
    ax.plot(folded, error, "o", markersize=4.2, color=SERIES[0],
            markeredgecolor=vizstyle.SURFACE, markeredgewidth=0.7,
            label=f"{len(error)} random targets")
    if args.gear:
        with open(args.gear) as handle:
            gear = json.load(handle)
        angle = np.array(gear["outputAngleDeg"])
        value = np.array(gear["gearErrorDeg"]) * 1000.0
        angle_folded = (angle + args.gear_offset + 180.0) % 360.0 - 180.0
        sort = np.argsort(angle_folded)
        ax.plot(angle_folded[sort], value[sort] - value.mean(), "-",
                color=SERIES[1], linewidth=1.8, alpha=0.85,
                label="gear error measured separately")
    ax.axhspan(-CAMERA_REPEATABILITY_MDEG, CAMERA_REPEATABILITY_MDEG,
               color=SERIES[2], alpha=0.16, zorder=0)
    ax.axhline(10.0, color=INK_SOFT, linewidth=0.9, linestyle=(0, (4, 3)), alpha=0.6)
    ax.axhline(-10.0, color=INK_SOFT, linewidth=0.9, linestyle=(0, (4, 3)), alpha=0.6)
    ax.text(ax.get_xlim()[1], 10.5, "0.01 deg goal  ", fontsize=7.5,
            color=INK_SOFT, ha="right", va="bottom")
    vizstyle.zeroline(ax)
    vizstyle.tidy(ax, f"{args.title}", "commanded position [deg]", "error [mdeg]")
    ax.legend(loc="lower right", framealpha=0.0, ncol=2)

    ax = figure.add_subplot(grid[1, 0])
    limit = max(30.0, np.abs(error).max() * 1.05)
    ax.hist(error, bins=28, range=(-limit, limit), color=SERIES[0],
            edgecolor=vizstyle.SURFACE, linewidth=0.8)
    ax.axvline(0.0, color=INK_SOFT, linewidth=0.8, alpha=0.5)
    vizstyle.tidy(ax, f"Distribution  (rms {error.std():.1f}, "
                      f"95% within {np.percentile(np.abs(error), 95):.1f} mdeg)",
                  "error [mdeg]", "targets")

    ax = figure.add_subplot(grid[1, 1])
    step = np.abs(np.diff(target))
    step = np.minimum(step, 360.0 - step)
    ax.plot(step, np.abs(error[1:]), "o", markersize=4.0, color=SERIES[3],
            markeredgecolor=vizstyle.SURFACE, markeredgewidth=0.7)
    vizstyle.tidy(ax, "Error against how far the move was",
                  "distance from the previous target [deg]", "|error| [mdeg]")

    figure.savefig(os.path.join(args.outdir, "validation_run.png"), bbox_inches="tight")
    plt.close(figure)
    print(f"wrote {args.outdir}/validation_run.png")

    print(f"\n{len(error)} targets")
    print(f"  rms        {error.std():8.2f} mdeg")
    print(f"  mean |err| {np.abs(error).mean():8.2f} mdeg")
    print(f"  95th pct   {np.percentile(np.abs(error), 95):8.2f} mdeg")
    print(f"  max        {np.abs(error).max():8.2f} mdeg")
    print(f"  within 10 mdeg: {100*np.mean(np.abs(error) <= 10):.1f} %")
    print(f"  within 20 mdeg: {100*np.mean(np.abs(error) <= 20):.1f} %")


if __name__ == "__main__":
    main()
