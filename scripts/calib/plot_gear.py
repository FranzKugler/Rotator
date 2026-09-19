"""Graphs for the part of the error the AS5600 cannot see."""

import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import vizstyle
from vizstyle import SERIES, INK_SOFT, plt


def harmonic_fit(angle_deg, value, orders):
    phase = np.radians(angle_deg)
    columns = [np.ones_like(phase)]
    for k in orders:
        columns += [np.cos(k * phase), np.sin(k * phase)]
    M = np.column_stack(columns)
    coefficients, *_ = np.linalg.lstsq(M, value, rcond=None)
    amplitude = np.array([np.hypot(coefficients[2 * i + 1], coefficients[2 * i + 2])
                          for i in range(len(orders))])
    return amplitude, M @ coefficients, value - M @ coefficients


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--runs", nargs="+", required=True, help="gear_error.py --export JSONs")
    ap.add_argument("--labels", nargs="*", default=None)
    ap.add_argument("--max-order", type=int, default=25,
                    help="keep this well below each run's Nyquist limit "
                         "(points / 2 per revolution) - a harmonic fitted at "
                         "Nyquist is ill-conditioned and reports a spike that "
                         "is not in the data")
    ap.add_argument("--outdir", required=True)
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    vizstyle.apply()
    labels = args.labels or [os.path.basename(p) for p in args.runs]
    runs = []
    for path, label in zip(args.runs, labels):
        with open(path) as handle:
            data = json.load(handle)
        angle = np.array(data["outputAngleDeg"])
        gear = np.array(data["gearErrorDeg"]) * 1000.0
        order = np.argsort(angle)
        runs.append((label, angle[order], gear[order]))

    orders = list(range(1, args.max_order + 1))

    figure, axes = plt.subplots(2, 1, figsize=(10.0, 7.0),
                                gridspec_kw={"height_ratios": [3, 2], "hspace": 0.35})

    ax = axes[0]
    for index, (label, angle, gear) in enumerate(runs):
        ax.plot(angle, gear, "-o", color=SERIES[index], markersize=3.2,
                markeredgecolor=vizstyle.SURFACE, markeredgewidth=0.8,
                linewidth=1.3, label=f"{label}  (rms {gear.std():.1f} mdeg)")
    ax.axhspan(-2.3, 2.3, color=SERIES[2], alpha=0.16, zorder=0)
    ax.text(ax.get_xlim()[0] + 4, 2.6, "camera repeatability", fontsize=7.5,
            color=INK_SOFT, va="bottom")
    vizstyle.zeroline(ax)
    vizstyle.tidy(ax, "Output-shaft error the motor-shaft sensor cannot see",
                  "output angle [deg]", "camera minus sensor-derived [mdeg]")
    ax.legend(loc="lower left", ncol=len(runs), framealpha=0.0)

    ax = axes[1]
    width = 0.8 / len(runs)
    for index, (label, angle, gear) in enumerate(runs):
        amplitude, _, _ = harmonic_fit(angle, gear, orders)
        ax.bar(np.array(orders) + (index - (len(runs) - 1) / 2) * width, amplitude,
               width=width, color=SERIES[index], edgecolor=vizstyle.SURFACE,
               linewidth=0.6, label=label)
    for k in (10, 20, 30):
        ax.axvline(k, color=INK_SOFT, linewidth=0.8, alpha=0.35, linestyle=(0, (3, 3)),
                   zorder=0)
    ax.text(10.6, ax.get_ylim()[1] * 0.98, "once per motor revolution, and harmonics",
            fontsize=7.5, color=INK_SOFT, va="top")
    ax.set_xticks([1, 2, 5, 10, 15, 20, 25])
    vizstyle.tidy(ax, "Harmonic content against the output revolution",
                  "cycles per output revolution", "amplitude [mdeg]")
    ax.legend(loc="upper right", ncol=1, framealpha=0.0)

    figure.savefig(os.path.join(args.outdir, "gear_error.png"), bbox_inches="tight")
    plt.close(figure)
    print(f"wrote {args.outdir}/gear_error.png")


if __name__ == "__main__":
    main()
