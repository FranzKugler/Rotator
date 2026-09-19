"""Shared matplotlib styling for the calibration graphs.

One palette, one set of rules, applied to every figure the campaign
produces, so the graphs read as one report rather than as a pile of
throwaway plots.
"""

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_SOFT = "#52514e"
GRID = "#dedcd6"

# Categorical slots, fixed order - never cycled, never reassigned per chart.
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100",
          "#e87ba4", "#008300", "#4a3aa7", "#e34948"]

OUTPUT_DEG_PER_COUNT = 36.0 / 4096.0


def apply():
    plt.rcParams.update({
        "figure.facecolor": SURFACE,
        "axes.facecolor": SURFACE,
        "savefig.facecolor": SURFACE,
        "axes.edgecolor": GRID,
        "axes.labelcolor": INK_SOFT,
        "axes.titlecolor": INK,
        "axes.titlesize": 11,
        "axes.titleweight": "semibold",
        "axes.labelsize": 9,
        "axes.grid": True,
        "axes.axisbelow": True,
        "grid.color": GRID,
        "grid.linewidth": 0.6,
        "xtick.color": INK_SOFT,
        "ytick.color": INK_SOFT,
        "xtick.labelsize": 8,
        "ytick.labelsize": 8,
        "legend.frameon": False,
        "legend.fontsize": 8,
        "lines.linewidth": 1.6,
        "font.size": 9,
        "figure.dpi": 130,
    })


def tidy(ax, title=None, xlabel=None, ylabel=None):
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    if title:
        ax.set_title(title, loc="left", pad=8)
    if xlabel:
        ax.set_xlabel(xlabel)
    if ylabel:
        ax.set_ylabel(ylabel)
    return ax


def counts_axis(ax, label="output degrees"):
    """Mirror a counts axis in output degrees - the same measure in the other
    unit, which is why it is allowed a second scale where a second *measure*
    would not be."""
    secondary = ax.secondary_yaxis(
        "right",
        functions=(lambda c: c * OUTPUT_DEG_PER_COUNT,
                   lambda d: d / OUTPUT_DEG_PER_COUNT))
    secondary.set_ylabel(label, color=INK_SOFT, fontsize=9)
    secondary.tick_params(labelsize=8, colors=INK_SOFT)
    secondary.spines["right"].set_color(GRID)
    return secondary


def zeroline(ax):
    ax.axhline(0.0, color=INK_SOFT, linewidth=0.8, alpha=0.5, zorder=1)
