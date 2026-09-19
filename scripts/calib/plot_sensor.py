"""Calibration graphs for the full-step sweep: what the sensor's error looks
like, how much of it the Fourier model removes, and what is left over."""

import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import vizstyle
from vizstyle import SERIES, INK_SOFT, plt
from fit_sensor import (load, split_repeats, sweep_error, fit, evaluate, rms,
                        seam_mask, COUNTS, OUTPUT_DEG_PER_COUNT)


def collect(path):
    header, passes = load(path)
    segments = []
    for tag, records in passes.items():
        for number, chunk in enumerate(split_repeats(records)):
            theta, d, raw, step, closure = sweep_error(chunk)
            segments.append({"name": f"{tag}#{number + 1}",
                             "direction": chunk[0]["direction"],
                             "theta": theta, "d": d, "raw": raw,
                             "step": step, "closure": closure})
    return header, segments


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--run", required=True)
    ap.add_argument("--order", type=int, default=5)
    ap.add_argument("--max-order", type=int, default=14)
    ap.add_argument("--outdir", required=True)
    ap.add_argument("--exclude-seam", action="store_true",
                    help="drop points in the AS5600 ANGLE-register wrap-seam dead "
                         "band - only needed for runs recorded before the firmware "
                         "switched to the RAW ANGLE register")
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    vizstyle.apply()
    header, segments = collect(args.run)
    forward = [s for s in segments if s["direction"] > 0]
    reverse = [s for s in segments if s["direction"] < 0]

    theta = np.concatenate([s["theta"] for s in forward])
    d = np.concatenate([s["d"] for s in forward])
    raw = np.concatenate([s["raw"] for s in forward])
    keep = seam_mask(raw) if args.exclude_seam else np.ones(len(raw), bool)
    C0, A, B, residual = fit(theta[keep], d[keep], args.order)

    grid = np.linspace(0, 2 * np.pi, 1441)
    model = evaluate(grid, C0, A, B)

    # The constant term is an arbitrary zero offset - it is 40x the size of
    # everything else on this plot, so showing it would flatten the entire
    # characteristic into a line. Everything below is plotted about C0.
    def about(values):
        return values - C0

    # --- Figure 1: the characteristic and what the model leaves behind -----
    figure, axes = plt.subplots(2, 1, figsize=(9.5, 7.0), sharex=True,
                                gridspec_kw={"height_ratios": [2, 1], "hspace": 0.28})

    ax = axes[0]
    seen = set()
    for segment in segments:
        forward_pass = segment["direction"] > 0
        colour = SERIES[0] if forward_pass else SERIES[1]
        label = "forward passes" if forward_pass else "reverse pass"
        ax.plot(np.degrees(segment["theta"]), about(segment["d"]), ".",
                markersize=2.6, color=colour, alpha=0.7,
                label=None if label in seen else label)
        seen.add(label)
    ax.plot(np.degrees(grid), about(model), "-", color=SERIES[6], linewidth=2.0,
            label=f"Fourier fit, order {args.order}")
    if args.exclude_seam:
        for edge in (0.0, 360.0):
            ax.axvspan(edge - 1.3, edge + 1.3, color=SERIES[3], alpha=0.18, zorder=0)
        ax.text(3.0, ax.get_ylim()[0], " AS5600 wrap-seam dead band", fontsize=7,
                color=INK_SOFT, va="bottom")
    vizstyle.zeroline(ax)
    vizstyle.tidy(ax, "AS5600 error against a linear motor, one full revolution",
                  None, "sensor error about its mean [counts]")
    vizstyle.counts_axis(ax, "output error [deg]")
    ax.legend(loc="lower right", ncol=3, framealpha=0.0)

    ax = axes[1]
    for segment in forward:
        good = (seam_mask(segment["raw"]) if args.exclude_seam
                else np.ones(len(segment["raw"]), bool))
        ax.plot(np.degrees(segment["theta"][good]),
                segment["d"][good] - evaluate(segment["theta"][good], C0, A, B),
                ".", markersize=2.6, color=SERIES[2], alpha=0.7)
    if args.exclude_seam:
        for edge in (0.0, 360.0):
            ax.axvspan(edge - 1.3, edge + 1.3, color=SERIES[3], alpha=0.18, zorder=0)
    vizstyle.zeroline(ax)
    vizstyle.tidy(ax, f"Residual after the order-{args.order} fit  "
                      f"(rms {rms(residual):.3f} counts = "
                      f"{rms(residual)*OUTPUT_DEG_PER_COUNT*1000:.1f} mdeg output)",
                  "sensor reading [deg of one motor revolution]", "residual [counts]")
    vizstyle.counts_axis(ax, "output error [deg]")
    figure.savefig(os.path.join(args.outdir, "sensor_characteristic.png"),
                   bbox_inches="tight")
    plt.close(figure)

    # --- Figure 2: how much each harmonic and each order buys --------------
    figure, axes = plt.subplots(1, 2, figsize=(11.5, 3.8))
    figure.subplots_adjust(wspace=0.42)

    _, Afull, Bfull, _ = fit(theta[keep], d[keep], args.max_order)
    orders = np.arange(1, args.max_order + 1)
    amplitude = np.hypot(Afull[1:], Bfull[1:])
    ax = axes[0]
    bars = ax.bar(orders, amplitude, width=0.62, color=SERIES[0],
                  edgecolor=vizstyle.SURFACE, linewidth=1.2)
    for index in range(args.order, args.max_order):
        bars[index].set_color(SERIES[3])
    ax.set_xticks(orders)
    vizstyle.tidy(ax, "Harmonic content of the sensor error",
                  "harmonic order k", "amplitude [counts]")
    vizstyle.counts_axis(ax, "[deg output]")
    ax.text(0.98, 0.94, f"blue: kept at order {args.order}\ngold: dropped",
            transform=ax.transAxes, ha="right", va="top", fontsize=8, color=INK_SOFT)

    residuals = [rms(fit(theta[keep], d[keep], k)[3]) for k in orders]
    ax = axes[1]
    ax.plot(orders, residuals, "-o", color=SERIES[0], markersize=5,
            markeredgecolor=vizstyle.SURFACE, markeredgewidth=1.0)
    ax.plot([args.order], [residuals[args.order - 1]], "o", markersize=9,
            color=SERIES[1], markeredgecolor=vizstyle.SURFACE, markeredgewidth=1.4)
    ax.annotate(f"order {args.order}\n{residuals[args.order-1]:.3f} counts",
                (args.order, residuals[args.order - 1]),
                textcoords="offset points", xytext=(10, 12), fontsize=8, color=INK_SOFT)
    ax.set_xticks(orders)
    vizstyle.tidy(ax, "Residual against model order",
                  "highest harmonic in the fit", "residual rms [counts]")
    vizstyle.counts_axis(ax, "[deg output]")
    figure.savefig(os.path.join(args.outdir, "sensor_orders.png"), bbox_inches="tight")
    plt.close(figure)

    # --- Figure 3: direction dependence ------------------------------------
    if forward and reverse:
        # Matched on stepPosition, not on the index: the run-up puts the two
        # passes eight full steps apart, so equal indices are not equal places.
        lookup = dict(zip(reverse[0]["step"], zip(reverse[0]["d"], reverse[0]["theta"])))
        shared = [(theta_f, d_f - lookup[position][0])
                  for position, d_f, theta_f in zip(forward[0]["step"],
                                                    forward[0]["d"],
                                                    forward[0]["theta"])
                  if position in lookup]
        if len(shared) >= 10:
            shared_theta = np.array([t for t, _ in shared])
            delta = np.array([v for _, v in shared])
            figure, ax = plt.subplots(figsize=(9.5, 3.6))
            ax.plot(np.degrees(shared_theta), delta, ".", markersize=2.8,
                    color=SERIES[1], label="forward - reverse")
            ax.axhline(delta.mean(), color=SERIES[6], linewidth=1.6,
                       label=f"mean {delta.mean():+.3f} counts "
                             f"({delta.mean()*OUTPUT_DEG_PER_COUNT*1000:+.1f} mdeg out)")
            vizstyle.zeroline(ax)
            vizstyle.tidy(ax, f"Direction dependence at the {len(shared)} shared positions",
                          "sensor reading [deg of one motor revolution]",
                          "difference [counts]")
            vizstyle.counts_axis(ax, "[deg output]")
            ax.legend(loc="upper right", ncol=2)
            figure.savefig(os.path.join(args.outdir, "sensor_hysteresis.png"),
                           bbox_inches="tight")
            plt.close(figure)

    print(f"wrote graphs to {args.outdir}")


if __name__ == "__main__":
    main()
