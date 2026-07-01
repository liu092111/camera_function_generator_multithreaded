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
# Greyscale palette reused across all 8 benchmarks (paper / print-safe).
#
# The dictionary KEYS keep their original semantic names ("blue", "tan",
# "green", ...) so none of the per-figure rebuild scripts need to change
# which key they request; only the underlying greys differ. Distinct
# categories map to well-separated grey levels (near-black -> light grey)
# so they remain distinguishable in a black-and-white print.
# ----------------------------------------------------------------------
#
# Palette policy (revised per thesis-review feedback 2026-07):
#   Reviewer wants COLOUR used ONLY where it is needed for distinguishability
#   (the multi-line CDFs, plus the few comment-flagged figures). Everything
#   else — bar charts, architecture/flow diagrams, tables, mean±std/P99 bar
#   figures — returns to the ORIGINAL print-safe GREYSCALE below. Categories
#   there stay separable via well-spaced grey levels + dark bar edges + (for
#   line plots) dash styles.
#
#   The colour used by the CDFs is NOT stored here; it lives in the separate
#   CDF_COLORS palette and is applied only inside cdf_lines(). That keeps the
#   grey/colour split automatic: any figure pulling from COLORS stays grey,
#   only CDFs (and the handful of figures that opt in explicitly) get colour.
#
# Dictionary KEYS keep their original semantic names so none of the per-figure
# rebuild scripts need to change which key they request.
# ----------------------------------------------------------------------
COLORS = {
    # Fills sit in a LIGHT grey band (~#9e..#d4) so that black (mean +/- std)
    # error bars and the dark bar outlines remain clearly visible on top of
    # every bar; series in line plots are additionally separated by dash
    # style. Bars get a dark edge (see BAR_EDGE) for a crisp border.
    "blue":   "#c2c2c2",   # primary / Gen-3 / mean / process  -> light grey
    "blue2":  "#d4d4d4",   # lighter companion
    "tan":    "#9e9e9e",   # secondary / Gen-2 / FG accent     -> mid-light grey
    "tan2":   "#c9c9c9",
    "green":  "#b0b0b0",   # PASS / voltage / process-thread
    "purple": "#bdbdbd",   # cross-group
    "red":    "#000000",   # P99 / threshold emphasis markers   -> black
    "grey":   "#aeaeae",   # baseline / annotations
    "grey_l": "#e0e0e0",
    # 5-step sequential grey ramp for "per round" bar series (R1..R5).
    "ramp5":  ["#3d3d3d", "#5e5e5e", "#7d7d7d", "#969696", "#aeaeae"],
    "header_blue": "#404040",   # table header fill (dark grey)
    "row_blue":    "#ededed",   # table zebra row fill (light grey)
    "pass_green":  "#7d7d7d",
}

# ----------------------------------------------------------------------
# CDF-ONLY colour palette (Okabe-Ito colour-blind-safe qualitative set).
# Applied exclusively inside cdf_lines(); the rest of the figures stay grey.
# Simple, saturated, well separated — one hue per curve, no gradients.
# ----------------------------------------------------------------------
CDF_COLORS = [
    "#0072B2",   # blue
    "#E69F00",   # orange
    "#009E73",   # bluish green
    "#CC79A7",   # reddish purple
    "#D55E00",   # vermillion
    "#56B4E9",   # sky blue
]

# Shared dark edge colour + width for bar/patch outlines, so light-grey
# fills always have a crisp border that frames the black error bars.
BAR_EDGE = "#1a1a1a"
BAR_EDGE_LW = 0.9

# Single-series line/CDF plots use a solid BLACK stroke for maximum slide
# legibility; multi-series plots keep colour + dash-style differentiation.
LINE_BLACK = "#000000"
# Recommended stroke widths for projected slides.
LW_MAIN = 3.2   # primary single-series line
LW_MULTI = 2.6  # each line in a multi-series CDF/overlay

# ----------------------------------------------------------------------
# Line-style cycle for figures whose series are distinguished ONLY by
# colour (CDFs, time-series overlays, per-round line plots). In greyscale
# the line dash pattern carries the distinction that colour used to.
# Index into LINE_STYLES with the series number; it wraps via modulo.
# ----------------------------------------------------------------------
LINE_STYLES = [
    "-",                    # solid
    (0, (5, 2)),            # dashed
    (0, (1, 1.4)),          # dotted
    (0, (6, 2, 1, 2)),      # dash-dot
    (0, (3, 1.5, 1, 1.5, 1, 1.5)),  # dash-dot-dot
    (0, (8, 3)),            # long dash
]


def line_style(i):
    """Return a print-distinguishable line style for series index ``i``."""
    return LINE_STYLES[i % len(LINE_STYLES)]


def _lighten(hex_color, amount=0.55):
    """Blend ``hex_color`` toward white by ``amount`` (0=same, 1=white)."""
    h = hex_color.lstrip("#")
    r, g, b = (int(h[i:i + 2], 16) for i in (0, 2, 4))
    r = int(round(r + (255 - r) * amount))
    g = int(round(g + (255 - g) * amount))
    b = int(round(b + (255 - b) * amount))
    return f"#{r:02x}{g:02x}{b:02x}"


def cdf_lines(ax, series, where=None, base_lw=None):
    """Plot several CDF/line series as plain SOLID coloured lines.

    Per the latest review preference, curves are distinguished PURELY by
    colour: each series is a single solid stroke from the dedicated
    CDF_COLORS palette (no dashes, no pale two-layer halo). Colour stays
    confined to CDFs — every other figure remains greyscale.

    ``series`` is a list of dicts, each with keys:
        x, y        : the data (already sorted / cumulative)
        color       : IGNORED — the curve colour always comes from CDF_COLORS
                      by series index. (Kept in the dicts only so callers need
                      not change.) Pass ``force_color`` to override.
        force_color : optional explicit colour that overrides CDF_COLORS[i]
        label       : legend label
        style       : IGNORED — all CDF lines are solid now.
        step        : optional "post"/"pre"/"mid" to use ax.step instead of plot

    Returns the list of Line2D handles (for legend control).
    """
    lw = base_lw if base_lw is not None else LW_MULTI
    handles = []
    for i, s in enumerate(series):
        color = s.get("force_color", CDF_COLORS[i % len(CDF_COLORS)])
        step = s.get("step", where)
        plot = (lambda *a, **k: ax.step(*a, where=step, **k)) if step \
            else ax.plot
        h = plot(s["x"], s["y"], color=color, linewidth=lw,
                 linestyle="-", solid_capstyle="round",
                 zorder=3 + i * 0.01, label=s["label"])
        handles.append(h[0] if isinstance(h, list) else h)
    return handles


def grey_ramp(n, lo=0.08, hi=0.80):
    """Return ``n`` evenly spaced grey hex strings from dark (lo) to light (hi).

    Values are matplotlib greyscale levels (0 = black, 1 = white); the
    default range stays clear of pure black/white so every step is visible.
    """
    if n <= 1:
        return ["#1a1a1a"]
    out = []
    for k in range(n):
        v = lo + (hi - lo) * k / (n - 1)
        g = int(round(v * 255))
        out.append(f"#{g:02x}{g:02x}{g:02x}")
    return out


def apply_style():
    """Apply the global rcParams used by every figure."""
    plt.rcParams.update({
        "font.family": "serif",
        "font.serif": ["DejaVu Serif", "STIXGeneral", "Times New Roman"],
        "mathtext.fontset": "dejavuserif",
        # Enlarged per thesis-review feedback: axis titles, tick labels and
        # legend text are brought up close to body-text size so they remain
        # legible at the printed figure size (library rejects tiny axis text).
        "font.size": 15,
        "axes.titlesize": 17,
        "axes.titleweight": "bold",
        "axes.labelsize": 16,
        "xtick.labelsize": 14,
        "ytick.labelsize": 14,
        "legend.fontsize": 13,
        "figure.titlesize": 18,
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
        # Solid BLACK frame on all four sides (per review: axis lines must be
        # black solid, and figures need a black border). Individual figures
        # that need an open frame can still hide spines locally.
        "axes.spines.top": True,
        "axes.spines.right": True,
        "axes.spines.bottom": True,
        "axes.spines.left": True,
        "axes.edgecolor": "#000000",
        "axes.linewidth": 1.4,
        "xtick.color": "#000000",
        "ytick.color": "#000000",
        "xtick.labelcolor": "#000000",
        "ytick.labelcolor": "#000000",
        "axes.labelcolor": "#000000",
        "text.color": "#000000",
        "xtick.direction": "out",
        "ytick.direction": "out",
        "xtick.major.width": 1.3,
        "ytick.major.width": 1.3,
        "xtick.major.size": 5.5,
        "ytick.major.size": 5.5,
        # High-res raster; vector-friendly text.
        "figure.dpi": 200,
        "savefig.dpi": 300,
        "savefig.bbox": "tight",
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
        # Thicker default strokes so line/CDF figures stay legible when
        # projected on a slide.
        "lines.linewidth": 2.8,
        "lines.markersize": 8,
        "patch.linewidth": 0.6,
    })


def save(fig, path):
    """Save with tight bbox + small pad so nothing is clipped, then close."""
    fig.savefig(path, dpi=300, bbox_inches="tight", pad_inches=0.12,
                facecolor="white")
    plt.close(fig)
    print(f"  wrote {os.path.basename(path)}")


def finish(ax, frame=True):
    """Common per-axes cleanup: white bg, no grid, solid black frame.

    ``frame=True`` (default) draws all four spines as a solid black border
    (the "黑框" the review asks for). Pass ``frame=False`` for the few
    figures (ridge plots, diagrams) that intentionally hide a spine.
    """
    ax.set_facecolor("white")
    ax.grid(False)
    if frame:
        for side in ("top", "right", "bottom", "left"):
            ax.spines[side].set_visible(True)
            ax.spines[side].set_color("#000000")
            ax.spines[side].set_linewidth(1.4)


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
