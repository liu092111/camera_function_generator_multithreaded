"""Rebuild all top-level academic figures for 01_fg_switching.

Produces publication-quality PNGs: pure white background, no gridlines,
no "testXX"/"Test 0X" tokens, no overlapping text. Uses the shared
paper_style module. All data is computed from fg_switching_gen3_cleaned.csv;
the only hard-coded constants are the documented Gen-1 baseline numbers
(268 ms mean, 367 ms max) taken from report.txt.
"""

import sys
import pathlib

sys.path.insert(0, "/mnt/c/Users/liuuhua/Desktop/Git Repository/camera_function_generator_multithreaded/benchmarks")
from paper_style import apply_style, COLORS, save, finish, run_cli, line_style, grey_ramp, BAR_EDGE, BAR_EDGE_LW, LINE_BLACK, LW_MAIN, LW_MULTI, cdf_lines  # noqa: E402

import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch  # noqa: E402
from scipy.stats import gaussian_kde  # noqa: E402

apply_style()

HERE = pathlib.Path("/mnt/c/Users/liuuhua/Desktop/Git Repository/"
                    "camera_function_generator_multithreaded/benchmarks/"
                    "01_fg_switching")
OUT = HERE
CSV = HERE / "fg_switching_gen3_cleaned.csv"

# Documented Gen-1 baseline constants (report.txt). Not in the CSV.
GEN1_MEAN = 268.0
GEN1_MAX = 367.0
GEN1_N = 50

# ----------------------------------------------------------------------
# Column metadata: CSV column -> (short label, multiline label, color key)
# ----------------------------------------------------------------------
COLS = [
    "same_group_1_3_ms",
    "same_group_2_4_ms",
    "cross_group_1_to_2_ms",
    "cross_group_2_to_1_ms",
    "voltage_adjust_ms",
]
LABELS = {
    "same_group_1_3_ms":     "Same-Group (1↔3)",
    "same_group_2_4_ms":     "Same-Group (2↔4)",
    "cross_group_1_to_2_ms": "Cross-Group (1→2)",
    "cross_group_2_to_1_ms": "Cross-Group (2→1)",
    "voltage_adjust_ms":     "Voltage Adjust",
}
# Two-line versions for crowded x-axes.
LABELS2 = {
    "same_group_1_3_ms":     "Same-Group\n(1↔3)",
    "same_group_2_4_ms":     "Same-Group\n(2↔4)",
    "cross_group_1_to_2_ms": "Cross-Group\n(1→2)",
    "cross_group_2_to_1_ms": "Cross-Group\n(2→1)",
    "voltage_adjust_ms":     "Voltage\nAdjust",
}
# CDF / panel short legend labels.
SHORT = {
    "same_group_1_3_ms":     "Same (1↔3)",
    "same_group_2_4_ms":     "Same (2↔4)",
    "cross_group_1_to_2_ms": "Cross (1→2)",
    "cross_group_2_to_1_ms": "Cross (2→1)",
    "voltage_adjust_ms":     "Voltage",
}
CKEY = {
    "same_group_1_3_ms":     "blue",
    "same_group_2_4_ms":     "blue2",
    "cross_group_1_to_2_ms": "tan",
    "cross_group_2_to_1_ms": "green",
    "voltage_adjust_ms":     "purple",
}

df = pd.read_csv(CSV)
# Per-column clean (drop NaN) data arrays.
DATA = {c: df[c].dropna().to_numpy() for c in COLS}


def stats(arr):
    return {
        "n": int(arr.size),
        "mean": float(np.mean(arr)),
        "median": float(np.median(arr)),
        "std": float(np.std(arr, ddof=1)),
        "p95": float(np.percentile(arr, 95)),
        "p99": float(np.percentile(arr, 99)),
        "max": float(np.max(arr)),
    }


STATS = {c: stats(DATA[c]) for c in COLS}


# ======================================================================
# fig00 — architecture / methodology diagram
# ======================================================================
def fig00():
    fig, ax = plt.subplots(figsize=(11, 13))
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 100)
    ax.axis("off")
    fig.patch.set_facecolor("white")

    def box(x, y, w, h, lines, fc, ec, bold_first=True, fs=10.5,
            title_fs=11.5, text_color="#1a1a1a"):
        p = FancyBboxPatch((x, y), w, h,
                           boxstyle="round,pad=0.4,rounding_size=1.2",
                           linewidth=1.3, edgecolor=ec, facecolor=fc,
                           mutation_scale=1)
        ax.add_patch(p)
        n = len(lines)
        cy = y + h / 2
        # vertical layout of lines, centered
        line_h = h / (n + 0.6)
        start = cy + (n - 1) / 2 * line_h
        for i, ln in enumerate(lines):
            yy = start - i * line_h
            if i == 0 and bold_first:
                ax.text(x + w / 2, yy, ln, ha="center", va="center",
                        fontsize=title_fs, fontweight="bold",
                        color=text_color)
            else:
                ax.text(x + w / 2, yy, ln, ha="center", va="center",
                        fontsize=fs, color=text_color)

    def arrow(x1, y1, x2, y2):
        a = FancyArrowPatch((x1, y1), (x2, y2),
                            arrowstyle="-|>", mutation_scale=18,
                            linewidth=1.8, color="#444444")
        ax.add_patch(a)

    def section(y, txt):
        ax.text(50, y, txt, ha="center", va="center", fontsize=12.5,
                fontweight="bold", color=COLORS["blue"])

    # ---- Title block
    ax.text(50, 98, "Function Generator Mode-Switching Latency",
            ha="center", va="center", fontsize=17, fontweight="bold",
            color="#1a1a1a")
    ax.text(50, 95.0, "Benchmark Architecture & Methodology",
            ha="center", va="center", fontsize=12.5, color="#555555")

    # ---- Section: three generations
    section(91.5, "Three Generations of SCPI Command Strategy")

    # Top-left Equipment, top-right Methodology
    box(4, 81, 43, 8.2,
        ["Equipment", "Keysight 33622A (USB-TMC)",
         "Host PC: Windows + Python 3.14"],
        COLORS["row_blue"], COLORS["blue"])
    box(53, 81, 43, 8.2,
        ["Methodology", "50 trials/scenario + 10 warmup",
         "Timing: time.perf_counter() (sub-µs)"],
        COLORS["row_blue"], COLORS["blue"])

    # Gen boxes stacked, centered
    gx, gw = 18, 64
    box(gx, 69.5, gw, 8.0,
        ["Gen-1: Full Re-upload",
         "OUTP OFF → Read CSV → Upload → Configure → OUTP ON",
         "~260 ms per switch"],
        COLORS["tan2"], COLORS["tan"])
    arrow(50, 69.3, 50, 66.6)
    ax.text(53.5, 67.9, "~10×", ha="left", va="center", fontsize=11,
            fontweight="bold", color=COLORS["tan"])

    box(gx, 57.8, gw, 8.6,
        ["Gen-2: Preloaded + Multi-write",
         "Preload waveforms at init",
         "Multiple inst.write() + *WAI",
         "~2–30 ms per switch"],
        COLORS["row_blue"], COLORS["blue"])
    arrow(50, 57.6, 50, 54.9)
    ax.text(53.5, 56.2, "~5–10×", ha="left", va="center", fontsize=11,
            fontweight="bold", color=COLORS["blue"])

    box(gx, 46.1, gw, 8.6,
        ["Gen-3: Single Compound Write",
         "Preload waveforms at init",
         "All cmds joined by \";\" → 1 write",
         "< 1 ms per switch"],
        "#e4e4e4", COLORS["green"])

    # ---- Section: scenarios
    section(42.5, "Gen-3 Switching Scenarios (5 types)")

    scen = [
        ("1. Same-Group Polarity Switch (Mode 1↔3)", "OFF→POL→ON",
         "blue"),
        ("2. Same-Group Polarity Switch (Mode 2↔4)", "OFF→POL→ON",
         "blue2"),
        ("3. Cross-Group Waveform Switch (Mode 1→2)",
         "OFF→ARB→SRAT→FREQ→POL→ON", "tan"),
        ("4. Cross-Group Waveform Switch (Mode 2→1)",
         "OFF→ARB→SRAT→FREQ→POL→ON", "green"),
        ("5. Voltage Adjust (PID)", "VOLT X;VOLT Y", "purple"),
    ]
    sy = 35.4
    sh = 4.8
    sgap = 1.05
    for i, (t, sub, ck) in enumerate(scen):
        yy = sy - i * (sh + sgap)
        box(8, yy, 84, sh, [t, sub],
            COLORS["row_blue"], COLORS[ck], fs=9.8, title_fs=10.5)

    # ---- Bottom key-question box
    box(8, 1.5, 84, 8.0,
        ["Key Question",
         "Can Gen-3 compound SCPI keep ALL switching operations within",
         "the 120 fps frame budget (8.33 ms)?",
         "Target: P99 < 5 ms for every scenario"],
        "#f0f0f0", COLORS["tan"], fs=10.3, title_fs=12)

    save(fig, OUT / "fig00_test_architecture.png")


# ======================================================================
# fig01 — violin + jittered strip + mean diamonds
# ======================================================================
def fig01():
    fig, ax = plt.subplots(figsize=(10, 6))
    positions = np.arange(1, len(COLS) + 1)
    data = [DATA[c] for c in COLS]

    vp = ax.violinplot(data, positions=positions, showextrema=False,
                       widths=0.78)
    for body, c in zip(vp["bodies"], COLS):
        body.set_facecolor(COLORS[CKEY[c]])
        body.set_edgecolor(COLORS[CKEY[c]])
        body.set_alpha(0.35)

    rng = np.random.default_rng(42)
    for pos, c in zip(positions, COLS):
        arr = DATA[c]
        jit = rng.uniform(-0.13, 0.13, size=arr.size)
        ax.scatter(pos + jit, arr, s=12, color=COLORS[CKEY[c]],
                   alpha=0.55, edgecolors="none", zorder=3)
        # white diamond mean marker
        ax.scatter([pos], [np.mean(arr)], marker="D", s=70,
                   facecolor="white", edgecolors="#222222", linewidths=1.4,
                   zorder=5)

    ax.set_xticks(positions)
    ax.set_xticklabels([LABELS2[c] for c in COLS])
    ax.set_ylabel("Switching Latency (ms)")
    ax.set_title("Gen-3 Compound SCPI Switching Latency")
    ax.set_ylim(bottom=0)
    finish(ax)
    save(fig, OUT / "fig01_violin_strip.png")


# ======================================================================
# fig02 — bar + error bars + value labels
# ======================================================================
def fig02():
    fig, ax = plt.subplots(figsize=(9.5, 6))
    positions = np.arange(len(COLS))
    means = [STATS[c]["mean"] for c in COLS]
    stds = [STATS[c]["std"] for c in COLS]
    colors = [COLORS[CKEY[c]] for c in COLS]

    bars = ax.bar(positions, means, yerr=stds, capsize=6, width=0.62,
                  color=colors, edgecolor="#333333", linewidth=0.8,
                  error_kw=dict(ecolor="#333333", elinewidth=1.2,
                                capthick=1.2), zorder=3)

    ymax = max(m + s for m, s in zip(means, stds))
    for x, m, s in zip(positions, means, stds):
        ax.text(x, m + s + ymax * 0.04, f"{m:.3f}", ha="center", va="bottom",
                fontsize=10, fontweight="bold", color="#1a1a1a")

    ax.set_xticks(positions)
    ax.set_xticklabels([LABELS2[c] for c in COLS])
    ax.set_ylabel("Latency (ms)")
    ax.set_title("Gen-3 Switching Latency (Mean ± Std)")
    ax.set_ylim(0, ymax * 1.20)
    finish(ax)
    save(fig, OUT / "fig02_bar_errorbar.png")


# ======================================================================
# fig03 — empirical CDF
# ======================================================================
def fig03():
    fig, ax = plt.subplots(figsize=(9.5, 6))
    series = []
    for i, c in enumerate(COLS):
        arr = np.sort(DATA[c])
        cdf = np.arange(1, arr.size + 1) / arr.size * 100.0
        series.append({"x": arr, "y": cdf, "color": COLORS[CKEY[c]],
                       "label": SHORT[c], "style": line_style(i),
                       "step": "post"})
    cdf_lines(ax, series)

    ax.set_xlabel("Switching Latency (ms)")
    ax.set_ylabel("Cumulative Percentage (%)")
    ax.set_title("Empirical CDF — Gen-3 Switching Latency")
    ax.set_ylim(0, 102)
    ax.set_xlim(left=0)
    leg = ax.legend(loc="lower right", frameon=True)
    leg.get_frame().set_facecolor("white")
    finish(ax)
    save(fig, OUT / "fig03_cdf.png")


# ======================================================================
# fig04 — Gen-1 vs Gen-3, log y
# ======================================================================
def fig04():
    fig, ax = plt.subplots(figsize=(9, 6.2))

    same = np.concatenate([DATA["same_group_1_3_ms"],
                           DATA["same_group_2_4_ms"]])
    cross = np.concatenate([DATA["cross_group_1_to_2_ms"],
                            DATA["cross_group_2_to_1_ms"]])
    volt = DATA["voltage_adjust_ms"]

    labels = ["Gen-1\nFull Upload", "Gen-3\nSame-Group",
              "Gen-3\nCross-Group", "Gen-3\nVoltage Adj."]
    vals = [GEN1_MEAN, float(np.mean(same)), float(np.mean(cross)),
            float(np.mean(volt))]
    colors = [COLORS["tan"], COLORS["blue"], COLORS["blue"], COLORS["blue"]]

    positions = np.arange(len(labels))
    bars = ax.bar(positions, vals, width=0.6, color=colors,
                  edgecolor="#333333", linewidth=0.8, zorder=3)

    ax.set_yscale("log")
    ax.set_ylim(0.1, 1000)
    ax.set_ylabel("Latency (ms, log scale)")
    ax.set_title("Switching Latency: Gen-1 vs Gen-3")
    ax.set_xticks(positions)
    ax.set_xticklabels(labels)

    # Above-bar value labels (in log space, multiply for clearance).
    for x, v in zip(positions, vals):
        ax.text(x, v * 1.35, f"{v:.3f} ms" if v < 10 else f"{v:.1f} ms",
                ha="center", va="bottom", fontsize=10, fontweight="bold",
                color="#1a1a1a")

    # White in-bar speedup labels on the 3 Gen-3 bars.
    for x, v in zip(positions[1:], vals[1:]):
        speed = round(GEN1_MEAN / v)
        # place near vertical middle of the (log) bar
        ymid = np.sqrt(0.1 * v)
        ax.text(x, ymid, f"{speed}×", ha="center", va="center",
                fontsize=11.5, fontweight="bold", color="white", zorder=5)

    finish(ax)
    save(fig, OUT / "fig04_gen_comparison.png")


# ======================================================================
# fig05 — small-multiple histogram + KDE panels
# ======================================================================
def fig05():
    fig, axes = plt.subplots(2, 3, figsize=(13, 7.6),
                             constrained_layout=True)
    fig.suptitle("Gen-3 Switching Latency Distributions", fontsize=15,
                 fontweight="bold")
    axes = axes.ravel()

    for i, c in enumerate(COLS):
        ax = axes[i]
        arr = DATA[c]
        col = COLORS[CKEY[c]]
        ax.hist(arr, bins=18, density=True, color=col, alpha=0.45,
                edgecolor="white", linewidth=0.5, zorder=2)
        # KDE
        kde = gaussian_kde(arr)
        xs = np.linspace(arr.min(), arr.max(), 200)
        ax.plot(xs, kde(xs), color=LINE_BLACK, linewidth=LW_MAIN, zorder=3)
        mu = np.mean(arr)
        sig = np.std(arr, ddof=1)
        ax.axvline(mu, color="#222222", linestyle="--", linewidth=1.3,
                   zorder=4)
        ax.set_title(SHORT[c], fontsize=12)
        ax.set_xlabel("ms")
        ax.set_ylabel("Density")
        # stats box, anchored top-right inside axes
        ax.text(0.97, 0.95,
                f"µ={mu:.3f}\nσ={sig:.3f}",
                transform=ax.transAxes, ha="right", va="top", fontsize=9.5,
                bbox=dict(boxstyle="round,pad=0.35", facecolor="white",
                          edgecolor="#bbbbbb", linewidth=0.8))
        finish(ax)

    # hide unused 6th panel
    axes[5].axis("off")
    save(fig, OUT / "fig05_histogram_kde.png")


# ======================================================================
# fig06 — summary statistics table
# ======================================================================
def fig06():
    headers = ["", "n", "Mean (ms)", "Median (ms)", "Std (ms)",
               "P95 (ms)", "P99 (ms)", "Max (ms)"]

    def row_for(c, label):
        s = STATS[c]
        return [label, str(s["n"]), f"{s['mean']:.3f}", f"{s['median']:.3f}",
                f"{s['std']:.3f}", f"{s['p95']:.3f}", f"{s['p99']:.3f}",
                f"{s['max']:.3f}"]

    rows = [
        ["Gen-1 (Full Upload)", str(GEN1_N), f"{GEN1_MEAN:.3f}", "—",
         "—", "—", "—", f"{GEN1_MAX:.3f}"],
        row_for("same_group_1_3_ms",     "Gen-3 Same-Group (1↔3)"),
        row_for("same_group_2_4_ms",     "Gen-3 Same-Group (2↔4)"),
        row_for("cross_group_1_to_2_ms", "Gen-3 Cross-Group (1→2)"),
        row_for("cross_group_2_to_1_ms", "Gen-3 Cross-Group (2→1)"),
        row_for("voltage_adjust_ms",     "Gen-3 Voltage Adjust"),
    ]

    fig, ax = plt.subplots(figsize=(13, 4.4))
    ax.axis("off")
    ax.set_title("Summary Statistics: FG Switching Latency (3σ-cleaned)",
                 fontsize=14, fontweight="bold", pad=18)

    col_w = [0.255, 0.07, 0.115, 0.12, 0.10, 0.105, 0.105, 0.105]
    table = ax.table(cellText=rows, colLabels=headers, cellLoc="center",
                     colWidths=col_w, loc="center")
    table.auto_set_font_size(False)
    table.set_fontsize(10.5)
    table.scale(1, 2.4)

    n_rows = len(rows) + 1
    n_cols = len(headers)
    for (r, ccol), cell in table.get_celld().items():
        cell.set_edgecolor("#d0d0d0")
        cell.set_linewidth(0.6)
        if r == 0:
            cell.set_facecolor(COLORS["header_blue"])
            cell.set_text_props(color="white", fontweight="bold")
        else:
            # left-align label column
            if ccol == 0:
                cell.set_text_props(ha="left")
                cell._loc = "left"
                cell.PAD = 0.04
            # Gen-1 row light pink
            if r == 1:
                cell.set_facecolor("#e8e8e8")
            else:
                cell.set_facecolor("white" if (r % 2 == 1) else
                                   COLORS["row_blue"])

    save(fig, OUT / "fig06_summary_table.png")


if __name__ == "__main__":
    # Build all figures, or only the ones named on the command line.
    #   python rebuild_figures.py            # all
    #   python rebuild_figures.py fig03      # just fig03
    #   python rebuild_figures.py fig00 fig06 # a subset
    #   python rebuild_figures.py --list     # list figures
    run_cli(globals())
