"""Redraw all fig00 test-architecture diagrams as SIMPLE, large-font block
diagrams suitable for inclusion in a paper.

Design goals (per request):
  - Much larger fonts (title / box-title / sublabel) than the original
    detailed diagrams.
  - Strip clutter: equipment lists, methodology notes, per-line result
    bullets, full "Test Configuration" blocks, key-question boxes, etc.
  - Keep only the essential boxes and the arrows that show the data /
    thread flow.
  - Same white background + colour-blind-friendly palette as paper_style.

Each function overwrites the *current* fig00 PNG in its folder. The pristine
originals remain untouched in every folder's ``_original_figures/`` subdir,
so this is non-destructive to the source material.

Run:
    python3 redraw_architecture_simple.py          # rebuild all
    python3 redraw_architecture_simple.py fig01     # one folder (by number)
"""

import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from paper_style import apply_style, COLORS, save

apply_style()

ROOT = os.path.dirname(os.path.abspath(__file__))

# Large-font sizing used everywhere (the whole point of the redraw).
FS_TITLE   = 26   # figure title
FS_SUB     = 16   # one-line italic subtitle under the title
FS_SECTION = 19   # column / section heading
FS_BOXT    = 18   # box title (bold)
FS_BOXS    = 14   # box sublabel (one short line)
FS_NOTE    = 16   # arrow / speedup annotations


# ----------------------------------------------------------------------
# Generic helpers (0-100 coordinate canvas).
# ----------------------------------------------------------------------
def new_canvas(w, h):
    fig, ax = plt.subplots(figsize=(w, h))
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 100)
    ax.axis("off")
    fig.patch.set_facecolor("white")
    return fig, ax


def title(ax, main, sub=None, y=96, sub_gap=5.2):
    ax.text(50, y, main, ha="center", va="top",
            fontsize=FS_TITLE, fontweight="bold", color="#1a1a1a")
    if sub:
        ax.text(50, y - sub_gap, sub, ha="center", va="top",
                fontsize=FS_SUB, style="italic", color="#555555")


def box(ax, cx, cy, w, h, t, sub=None, fc="#eef3f9", ec=None,
        t_fs=FS_BOXT, s_fs=FS_BOXS, tc="#1a1a1a"):
    """Rounded box centred on (cx, cy) with a bold title and optional sub."""
    ec = ec or COLORS["blue"]
    p = FancyBboxPatch((cx - w / 2, cy - h / 2), w, h,
                       boxstyle="round,pad=0.3,rounding_size=1.4",
                       linewidth=2.0, edgecolor=ec, facecolor=fc, zorder=2)
    ax.add_patch(p)
    if sub:
        ax.text(cx, cy + h * 0.16, t, ha="center", va="center",
                fontsize=t_fs, fontweight="bold", color=tc, zorder=3)
        ax.text(cx, cy - h * 0.22, sub, ha="center", va="center",
                fontsize=s_fs, color="#444444", zorder=3)
    else:
        ax.text(cx, cy, t, ha="center", va="center",
                fontsize=t_fs, fontweight="bold", color=tc, zorder=3)


def varrow(ax, x, y0, y1, color="#555555"):
    ax.add_patch(FancyArrowPatch((x, y0), (x, y1), arrowstyle="-|>",
                 mutation_scale=26, linewidth=2.4, color=color, zorder=1))


def harrow(ax, x0, x1, y, color="#555555"):
    ax.add_patch(FancyArrowPatch((x0, y), (x1, y), arrowstyle="-|>",
                 mutation_scale=26, linewidth=2.4, color=color, zorder=1))


def section(ax, x, y, txt, color=None):
    ax.text(x, y, txt, ha="center", va="center", fontsize=FS_SECTION,
            fontweight="bold", color=color or COLORS["blue"])


# ----------------------------------------------------------------------
# 01 — FG mode-switching: three generations (vertical, with speedups).
# ----------------------------------------------------------------------
def fig01_fg_switching():
    fig, ax = new_canvas(9.5, 9.5)
    title(ax, "FG Mode-Switching Latency",
          "Three Generations of SCPI Command Strategy")

    gens = [
        ("Gen-1: Full Re-upload",      "~260 ms / switch", COLORS["tan2"],  COLORS["tan"]),
        ("Gen-2: Preloaded Multi-write", "~2-30 ms / switch", COLORS["row_blue"], COLORS["blue"]),
        ("Gen-3: Single Compound Write", "< 1 ms / switch",   "#dbe8d9",      COLORS["green"]),
    ]
    speed = ["~10x faster", "~5-10x faster"]
    w, h = 62, 14
    cy = [70, 47, 24]
    for i, (t, sub, fc, ec) in enumerate(gens):
        box(ax, 50, cy[i], w, h, t, sub, fc=fc, ec=ec)
        if i < 2:
            varrow(ax, 50, cy[i] - h / 2 - 0.5, cy[i + 1] + h / 2 + 0.5)
            ax.text(57, (cy[i] + cy[i + 1]) / 2, speed[i], ha="left",
                    va="center", fontsize=FS_NOTE, fontweight="bold",
                    color=gens[i][3])

    save(fig, os.path.join(ROOT, "01_fg_switching", "fig00_test_architecture.png"))


# ----------------------------------------------------------------------
# 02a — image processing pipeline (vertical flow, fewer/merged stages).
# ----------------------------------------------------------------------
def fig02a_pipeline():
    fig, ax = new_canvas(8.5, 11)
    title(ax, "Image Processing Pipeline", "Per-frame processing flow")

    stages = [
        ("Input Frame", COLORS["grey_l"], "#444444"),
        ("TPS Undistort + Rotate", COLORS["blue2"], "#27496d"),
        ("HSV Detection", COLORS["tan2"], "#9c5a16"),
        ("Kalman + EMA Smoothing", COLORS["tan2"], "#9c5a16"),
        ("Draw Overlay", COLORS["blue2"], "#27496d"),
        ("Output Result", COLORS["pass_green"], "#1f5c3a"),
    ]
    n = len(stages)
    w, h = 60, 8.5
    top, bot = 86, 6
    span = top - bot
    gap = (span - n * h) / (n - 1)
    for i, (t, fc, ec) in enumerate(stages):
        cy = top - i * (h + gap) - h / 2
        box(ax, 50, cy, w, h, t, fc=fc, ec=ec)
        if i < n - 1:
            varrow(ax, 50, cy - h / 2 - 0.4, cy - h / 2 - gap + 0.4)

    save(fig, os.path.join(ROOT, "02_process_thread", "fig00a_pipeline_architecture.png"))


# ----------------------------------------------------------------------
# 02b — three evaluation conditions (just titles + one short line).
# ----------------------------------------------------------------------
def fig02b_conditions():
    fig, ax = new_canvas(9, 8)
    title(ax, "Evaluation Conditions", "Three modes for latency characterization")

    modes = [
        ("Mode A: Offline",            "500 synthetic frames (no camera I/O)", "#dbe7f3", COLORS["blue"]),
        ("Mode B: Online Single-Pass", "500 live camera frames",               "#dcecdf", COLORS["green"]),
        ("Mode C: Online Multi-Round", "5 rounds x 200 frames",                "#f1e3cf", COLORS["tan"]),
    ]
    w, h = 78, 16
    cy = [72, 48, 24]
    for i, (t, sub, fc, ec) in enumerate(modes):
        box(ax, 50, cy[i], w, h, t, sub, fc=fc, ec=ec)

    save(fig, os.path.join(ROOT, "02_process_thread", "fig00b_test_conditions.png"))


# ----------------------------------------------------------------------
# 03 — end-to-end latency: three segments (horizontal flow).
# ----------------------------------------------------------------------
def fig03_e2e():
    fig, ax = new_canvas(13, 6.5)
    title(ax, "End-to-End Latency", "Capture -> Process -> FG Command",
          y=94, sub_gap=8)

    segs = [
        ("Capture",  "acquire one frame",     "#dbe7f3", COLORS["blue"]),
        ("Process",  "TPS + HSV + Kalman",    "#dbe8d9", COLORS["green"]),
        ("FG Write", "voltage command",       "#f3e7d6", COLORS["tan"]),
    ]
    w, h = 28, 24
    cx = [18, 50, 82]
    cy = 50
    for i, (t, sub, fc, ec) in enumerate(segs):
        box(ax, cx[i], cy, w, h, t, sub, fc=fc, ec=ec)
        if i < 2:
            harrow(ax, cx[i] + w / 2 + 0.5, cx[i + 1] - w / 2 - 0.5, cy)

    save(fig, os.path.join(ROOT, "03_end_to_end_latency", "fig00_test_architecture.png"))


# ----------------------------------------------------------------------
# 04 — combined latency: process thread vs main thread (isolation).
# ----------------------------------------------------------------------
def fig04_combined():
    fig, ax = new_canvas(11, 9.5)
    title(ax, "Combined Latency", "Process Thread vs FG Operations")

    # Left column: process thread (critical path).
    section(ax, 26, 84, "Process Thread", COLORS["green"])
    left = ["Input Frame", "TPS + HSV Detect", "Kalman / EMA", "Output Result"]
    lw, lh = 36, 11
    lcx = 26
    ltop = 76
    lgap = 4.5
    for i, t in enumerate(left):
        cy = ltop - i * (lh + lgap) - lh / 2
        box(ax, lcx, cy, lw, lh, t, fc="#e7f0ea", ec=COLORS["green"])
        if i < len(left) - 1:
            varrow(ax, lcx, cy - lh / 2 - 0.4, cy - lh / 2 - lgap + 0.4,
                   color=COLORS["green"])

    # Isolation boundary.
    ax.plot([50, 50], [10, 80], linestyle=(0, (6, 4)),
            color=COLORS["red"], linewidth=2.6)
    ax.text(50, 46, "Thread\nIsolation", ha="center", va="center",
            fontsize=FS_SUB, fontweight="bold", color=COLORS["red"],
            bbox=dict(boxstyle="round,pad=0.4", fc="white",
                      ec=COLORS["red"], lw=1.4))

    # Right column: main thread scenarios.
    section(ax, 76, 84, "Main Thread", COLORS["tan"])
    right = [
        ("A: Voltage Adjust",     COLORS["green"],  "#e7f0ea"),
        ("B: Same-Group Switch",  COLORS["blue"],   "#e7eff5"),
        ("C: Cross-Group Switch", COLORS["purple"], "#efeaf5"),
        ("D: Baseline (No FG)",   COLORS["grey"],   "#eeeeee"),
    ]
    rw, rh = 38, 11
    rcx = 76
    for i, (t, ec, fc) in enumerate(right):
        cy = ltop - i * (rh + lgap) - rh / 2
        box(ax, rcx, cy, rw, rh, t, fc=fc, ec=ec)

    save(fig, os.path.join(ROOT, "04_combined_latency", "fig00_test_architecture.png"))


# ----------------------------------------------------------------------
# 05 — pipeline throughput: 3-thread flow with queues (flow row only).
# ----------------------------------------------------------------------
def fig05_throughput():
    fig, ax = new_canvas(14, 6.5)
    title(ax, "Pipeline Throughput", "3-Thread Pipeline @ 120 fps",
          y=94, sub_gap=8)

    cy = 40
    blocks = [
        (11, 18, "Capture\nThread",  "#dbe7f3", COLORS["blue"]),
        (31, 12, "frame\nqueue",     "#eeeeee", "#9aa6b0"),
        (50, 18, "Process\nThread",  "#e3f0e6", COLORS["green"]),
        (69, 12, "result\nqueue",    "#eeeeee", "#9aa6b0"),
        (89, 18, "Main\nThread",     "#f3e7d6", COLORS["tan"]),
    ]
    h = 30
    for cx, w, t, fc, ec in blocks:
        box(ax, cx, cy, w, h, t, fc=fc, ec=ec, t_fs=FS_BOXT)
    # connecting arrows between adjacent box edges
    for i in range(len(blocks) - 1):
        x0 = blocks[i][0] + blocks[i][1] / 2 + 0.6
        x1 = blocks[i + 1][0] - blocks[i + 1][1] / 2 - 0.6
        harrow(ax, x0, x1, cy)

    save(fig, os.path.join(ROOT, "05_pipeline_throughput", "fig00_test_architecture.png"))


# ----------------------------------------------------------------------
# 06 — mode-switch impact: process thread vs FG switch thread (isolation).
# ----------------------------------------------------------------------
def fig06_mode_impact():
    fig, ax = new_canvas(11, 9.5)
    title(ax, "Mode Switch Impact", "Concurrent FG Switch -> Thread Isolation")

    section(ax, 26, 84, "Process Thread", COLORS["blue"])
    left = ["Input Frame", "TPS + HSV Detect", "Kalman / EMA", "Output Result"]
    lw, lh = 36, 11
    lcx = 26
    ltop = 76
    lgap = 4.5
    for i, t in enumerate(left):
        cy = ltop - i * (lh + lgap) - lh / 2
        box(ax, lcx, cy, lw, lh, t, fc=COLORS["row_blue"], ec=COLORS["blue"])
        if i < len(left) - 1:
            varrow(ax, lcx, cy - lh / 2 - 0.4, cy - lh / 2 - lgap + 0.4,
                   color=COLORS["blue"])

    ax.plot([50, 50], [14, 80], linestyle=(0, (6, 4)),
            color=COLORS["red"], linewidth=2.6)
    ax.text(50, 47, "Thread\nIsolation", ha="center", va="center",
            fontsize=FS_SUB, fontweight="bold", color=COLORS["red"],
            bbox=dict(boxstyle="round,pad=0.4", fc="white",
                      ec=COLORS["red"], lw=1.4))

    section(ax, 76, 84, "FG Switch Thread", COLORS["tan"])
    right = ["Wait 50 frames", "Cross-Group /\nPolarity Switch",
             "Gen-3 Compound\n(< 1 ms)"]
    rw, rh = 38, 14
    rcx = 76
    rtop = 76
    rgap = 7.5
    for i, t in enumerate(right):
        cy = rtop - i * (rh + rgap) - rh / 2
        box(ax, rcx, cy, rw, rh, t, fc=COLORS["tan2"], ec=COLORS["tan"])
        if i < len(right) - 1:
            varrow(ax, rcx, cy - rh / 2 - 0.4, cy - rh / 2 - rgap + 0.4,
                   color=COLORS["tan"])

    save(fig, os.path.join(ROOT, "06_mode_switch_impact", "fig00_test_architecture.png"))


# ----------------------------------------------------------------------
# 07 — TPS calibration: 3-stage rectification flow.
# ----------------------------------------------------------------------
def fig07_calibration():
    fig, ax = new_canvas(13, 6.5)
    title(ax, "TPS Calibration Quality", "Thin Plate Spline Rectification",
          y=94, sub_gap=8)

    stages = [
        ("Raw Image",        "640 x 480 px",        COLORS["row_blue"], COLORS["blue"]),
        ("TPS Grid Mapping", "308 control points",  COLORS["row_blue"], COLORS["blue"]),
        ("Rectified Output", "462 x 314 px",        "#dbe8d9",          COLORS["green"]),
    ]
    w, h = 27, 22
    cx = [18, 50, 82]
    cy = 50
    for i, (t, sub, fc, ec) in enumerate(stages):
        box(ax, cx[i], cy, w, h, t, sub, fc=fc, ec=ec)
        if i < 2:
            harrow(ax, cx[i] + w / 2 + 0.5, cx[i + 1] - w / 2 - 0.5, cy,
                   color=COLORS["tan"])

    save(fig, os.path.join(ROOT, "07_calibration_quality", "fig00_test_architecture.png"))


# ----------------------------------------------------------------------
# 08 — system time budget: 3 threads horizontal with budget values.
# ----------------------------------------------------------------------
def fig08_budget():
    fig, ax = new_canvas(13, 6.5)
    title(ax, "System Time Budget", "3-Thread Pipeline (budget = 8.33 ms / frame)",
          y=94, sub_gap=8)

    threads = [
        ("Capture Thread", "8.24 ms",          COLORS["blue"]),
        ("Process Thread", "3.01 ms (PASS)",   COLORS["green"]),
        ("Main Thread",    "0.13 ms (PASS)",   COLORS["tan"]),
    ]
    w, h = 26, 24
    cx = [19, 50, 81]
    cy = 50
    for i, (t, sub, ec) in enumerate(threads):
        box(ax, cx[i], cy, w, h, t, sub, fc="white", ec=ec, tc=ec)
        if i < 2:
            harrow(ax, cx[i] + w / 2 + 0.5, cx[i + 1] - w / 2 - 0.5, cy)

    save(fig, os.path.join(ROOT, "08_system_budget", "fig00_test_architecture.png"))


FIGS = {
    "fig01": fig01_fg_switching,
    "fig02a": fig02a_pipeline,
    "fig02b": fig02b_conditions,
    "fig03": fig03_e2e,
    "fig04": fig04_combined,
    "fig05": fig05_throughput,
    "fig06": fig06_mode_impact,
    "fig07": fig07_calibration,
    "fig08": fig08_budget,
}


if __name__ == "__main__":
    sel = sys.argv[1:] or sorted(FIGS)
    for key in sel:
        matches = [k for k in FIGS if k.startswith(key.replace(".png", ""))]
        if not matches:
            print(f"[warn] no figure matches '{key}'")
        for m in sorted(matches):
            print(f"-> {m}")
            FIGS[m]()
    print("Done.")
