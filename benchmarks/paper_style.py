"""Shared academic-figure style for all benchmark plots.

Design goals (per paper requirements):
  - Pure WHITE background (figure + axes), never grey.
  - NO gridlines.
  - Serif font (DejaVu Serif) with full glyph coverage for the symbols
    used in labels (mean +/-, micro, times, Delta, sigma, arrows).
  - Generous spacing / auto-layout so no text overlaps another element.
  - No "testXX" tokens anywhere (handled per-figure in the rebuild scripts).

Import this module from each folder's rebuild script:
    import sys, pathlib
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
    from paper_style import apply_style, COLORS, save, finish
"""

import os
import glob
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


# ----------------------------------------------------------------------
# Consistent, colour-blind-friendly palette reused across all 8 benchmarks.
# ----------------------------------------------------------------------
COLORS = {
    "blue":   "#3b6ea5",   # primary / Gen-3 / mean / process
    "blue2":  "#7ba6cf",   # lighter blue
    "tan":    "#d68a3c",   # secondary / Gen-2 / FG / baseline accent
    "tan2":   "#e3b486",
    "green":  "#5a9367",   # PASS / voltage / process-thread
    "purple": "#8a6fae",   # cross-group
    "red":    "#c0392b",   # P99 markers, thresholds
    "grey":   "#8c8c8c",   # baseline / annotations
    "grey_l": "#cfcfcf",
    # 5-step sequential blue ramp for "per round" series (R1..R5)
    "ramp5":  ["#1f4e79", "#2e6da4", "#5a93c4", "#8fb9dd", "#c3d9ee"],
    "header_blue": "#2e5b8a",   # table header fill
    "row_blue":    "#e8f0f8",   # table zebra row fill
    "pass_green":  "#2e7d4f",
}


def apply_style():
    """Apply the global rcParams used by every figure."""
    plt.rcParams.update({
        "font.family": "serif",
        "font.serif": ["DejaVu Serif", "STIXGeneral", "Times New Roman"],
        "mathtext.fontset": "dejavuserif",
        "font.size": 11,
        "axes.titlesize": 13,
        "axes.titleweight": "bold",
        "axes.labelsize": 11.5,
        "xtick.labelsize": 10,
        "ytick.labelsize": 10,
        "legend.fontsize": 9.5,
        "figure.titlesize": 14,
        "figure.titleweight": "bold",
        # White everywhere.
        "figure.facecolor": "white",
        "axes.facecolor": "white",
        "savefig.facecolor": "white",
        "legend.facecolor": "white",
        "legend.edgecolor": "#bbbbbb",
        "legend.framealpha": 1.0,
        # No grid.
        "axes.grid": False,
        # Clean spines.
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.edgecolor": "#333333",
        "axes.linewidth": 0.9,
        "xtick.color": "#333333",
        "ytick.color": "#333333",
        "xtick.direction": "out",
        "ytick.direction": "out",
        # High-res raster; vector-friendly text.
        "figure.dpi": 200,
        "savefig.dpi": 300,
        "savefig.bbox": "tight",
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
        "lines.linewidth": 1.8,
        "patch.linewidth": 0.6,
    })


def save(fig, path):
    """Save with tight bbox + small pad so nothing is clipped, then close."""
    fig.savefig(path, dpi=300, bbox_inches="tight", pad_inches=0.12,
                facecolor="white")
    plt.close(fig)
    print(f"  wrote {os.path.basename(path)}")


def finish(ax):
    """Common per-axes cleanup: white bg, no grid."""
    ax.set_facecolor("white")
    ax.grid(False)


def run_cli(namespace):
    """Command-line dispatcher so each figure can be built individually.

    Pass the caller's ``globals()``. Every top-level callable whose name
    starts with ``fig`` (e.g. fig00, fig00a, fig01 ...) is registered as a
    buildable figure.

    Usage from the shell:
        python rebuild_figures.py            # build every figure
        python rebuild_figures.py fig03      # build only fig03
        python rebuild_figures.py fig00 fig06 # build a subset
        python rebuild_figures.py --list     # list available figures

    Selection is by exact name or by prefix, so ``fig00`` also matches
    ``fig00a`` / ``fig00b`` when a folder splits that figure.
    """
    import sys

    figs = {name: obj for name, obj in namespace.items()
            if name.startswith("fig") and callable(obj)}
    args = sys.argv[1:]

    if args and args[0] in ("-h", "--help", "--list", "list"):
        print("Available figures:")
        for name in sorted(figs):
            print(f"  {name}")
        print("\nUsage: python rebuild_figures.py [figNN ...]   "
              "(no args = build all)")
        return

    if not args:
        selected = sorted(figs)
    else:
        selected = []
        for raw in args:
            key = raw.replace(".png", "")
            matches = [n for n in figs if n == key or n.startswith(key)]
            if not matches:
                print(f"[warn] no figure matches '{raw}' "
                      f"(available: {', '.join(sorted(figs))})")
                continue
            for m in sorted(matches):
                if m not in selected:
                    selected.append(m)

    if not selected:
        print("Nothing to build.")
        return

    print(f"Building {len(selected)} figure(s): {', '.join(selected)}")
    for name in selected:
        print(f"  -> {name}()")
        figs[name]()
    print("Done.")
