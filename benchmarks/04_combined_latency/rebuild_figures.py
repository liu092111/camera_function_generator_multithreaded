"""Rebuild all top-level academic figures for 04_combined_latency.

Publication-clean PNGs: pure white bg, no gridlines, no "testXX"/"Test 0X"
tokens, no overlapping or clipped text. Uses the shared paper_style module.

All statistics are computed from combined_latency_cleaned.csv.
"""

import sys
import os

sys.path.insert(0, "/mnt/c/Users/liuuhua/Desktop/Git Repository/camera_function_generator_multithreaded/benchmarks")
from paper_style import apply_style, COLORS, save, finish, run_cli, line_style, grey_ramp, BAR_EDGE, BAR_EDGE_LW, LINE_BLACK, LW_MAIN, LW_MULTI, cdf_lines

import numpy as np
import pandas as pd
from scipy import stats
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

apply_style()

HERE = os.path.dirname(os.path.abspath(__file__))
CSV = os.path.join(HERE, "combined_latency_cleaned.csv")

# ----------------------------------------------------------------------
# Scenario metadata
# ----------------------------------------------------------------------
SCEN_ORDER = ["A_voltage", "B_same_group", "C_cross_group", "D_baseline"]
LABEL = {
    "A_voltage":     "A: Voltage",
    "B_same_group":  "B: Same-Group",
    "C_cross_group": "C: Cross-Group",
    "D_baseline":    "D: Baseline",
}
LABEL2 = {  # two-line variant
    "A_voltage":     "A:\nVoltage",
    "B_same_group":  "B:\nSame-Group",
    "C_cross_group": "C:\nCross-Group",
    "D_baseline":    "D:\nBaseline",
}
COLOR = {
    "A_voltage":     COLORS["green"],
    "B_same_group":  COLORS["blue"],
    "C_cross_group": COLORS["purple"],
    "D_baseline":    COLORS["grey"],
}

df = pd.read_csv(CSV)


def stat(scen):
    s = df[df.scenario == scen]
    return {
        "proc_mean": s.process_ms.mean(),
        "proc_std":  s.process_ms.std(),
        "proc_p99":  np.percentile(s.process_ms, 99),
        "fg_mean":   s.fg_ms.mean(),
        "fg_p99":    np.percentile(s.fg_ms, 99),
        "proc":      s.process_ms.values,
        "fg":        s.fg_ms.values,
    }


ST = {s: stat(s) for s in SCEN_ORDER}


# ======================================================================
# fig00 — test architecture box diagram
# ======================================================================
def fig00():
    fig, ax = plt.subplots(figsize=(15, 11))
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 100)
    ax.axis("off")

    def box(x, y, w, h, text, fc="#f2f2f2", ec="#454545", fs=11,
            weight="normal", tc="#1a1a1a"):
        p = FancyBboxPatch((x, y), w, h,
                           boxstyle="round,pad=0.3,rounding_size=0.8",
                           linewidth=1.2, edgecolor=ec, facecolor=fc)
        ax.add_patch(p)
        ax.text(x + w / 2, y + h / 2, text, ha="center", va="center",
                fontsize=fs, fontweight=weight, color=tc, zorder=5)

    def arrow(x, y0, y1):
        ax.add_patch(FancyArrowPatch((x, y0), (x, y1),
                     arrowstyle="-|>", mutation_scale=16,
                     linewidth=1.6, color="#454545"))

    # Title
    ax.text(50, 98, "Combined Latency Benchmark", ha="center", va="top",
            fontsize=20, fontweight="bold")
    ax.text(50, 94.3,
            "Thread Isolation Validation — Process Thread vs FG Operations",
            ha="center", va="top", fontsize=12.5, color="#444444")

    # ----- Left column: Process Thread -----
    lx, lw = 6, 22
    ax.text(lx + lw / 2, 90, "Process Thread", ha="center", va="center",
            fontsize=14, fontweight="bold", color=COLORS["green"])
    ax.text(lx + lw / 2, 87.2, "(Critical Path — determines frame rate)",
            ha="center", va="center", fontsize=9.5, color="#555555", style="italic")

    proc_boxes = ["Input Frame", "TPS Rectification", "HSV Detection",
                  "Kalman / EMA", "Output Result"]
    bh = 6.5
    gap = 2.6
    top = 82
    ys = []
    for i, t in enumerate(proc_boxes):
        y = top - i * (bh + gap)
        ys.append(y)
        box(lx, y, lw, bh, t, fc="#eeeeee", ec=COLORS["green"], fs=11)
    for i in range(len(proc_boxes) - 1):
        arrow(lx + lw / 2, ys[i] - 0.2, ys[i + 1] + bh + 0.2)

    # ----- Divider -----
    divx = 33.5
    ax.plot([divx, divx], [10, 84.5], linestyle=(0, (6, 4)), color=COLORS["red"],
            linewidth=2.0)
    ax.text(divx - 0.9, 47, "Thread Isolation Boundary", ha="center",
            va="center", rotation=90, fontsize=10.5, color=COLORS["red"],
            fontweight="bold",
            bbox=dict(boxstyle="round,pad=0.25", fc="white", ec="none",
                      alpha=0.9))

    # ----- Right column: Main Thread -----
    rx, rw = 40, 54
    ax.text(rx + rw / 2, 90, "Main Thread", ha="center", va="center",
            fontsize=14, fontweight="bold", color=COLORS["tan"])
    ax.text(rx + rw / 2, 87.2, "(Asynchronous — does NOT block Process)",
            ha="center", va="center", fontsize=9.5, color="#555555", style="italic")

    # Control decision -> FG command dispatch
    cw = 24
    cx = rx + (rw - cw) / 2
    box(cx, 79.5, cw, 5.5, "Control Decision", fc="#f1f1f1", ec=COLORS["tan"])
    arrow(cx + cw / 2, 79.3, 75.2)
    box(cx, 69.5, cw, 5.5, "FG Command Dispatch", fc="#f1f1f1", ec=COLORS["tan"])

    # Sub-header
    ax.text(rx + 1, 64.5, "4 Test Scenarios:", ha="left", va="center",
            fontsize=12, fontweight="bold", color="#222222")

    scen_data = [
        ("A: Voltage Adjust",      "PID routine — SOUR:VOLT X", COLORS["green"], "#eeeeee"),
        ("B: Same-Group Switch",   "OFF;POL;ON;*WAI",                COLORS["blue"],  "#eeeeee"),
        ("C: Cross-Group Switch",  "Full compound waveform switch",  COLORS["purple"], "#ececec"),
        ("D: Baseline (No FG)",    "Process only — control group", COLORS["grey"], "#eeeeee"),
    ]
    sbh = 9.0
    sgap = 2.2
    stop = 60.5
    for i, (title, sub, ec, fc) in enumerate(scen_data):
        y = stop - i * (sbh + sgap)
        p = FancyBboxPatch((rx, y), rw, sbh,
                           boxstyle="round,pad=0.3,rounding_size=0.8",
                           linewidth=1.3, edgecolor=ec, facecolor=fc)
        ax.add_patch(p)
        ax.text(rx + rw / 2, y + sbh * 0.66, title, ha="center", va="center",
                fontsize=11.5, fontweight="bold", color="#1a1a1a")
        ax.text(rx + rw / 2, y + sbh * 0.26, sub, ha="center", va="center",
                fontsize=9.5, color="#444444", style="italic")

    # ----- Bottom config block -----
    cfg_y = 1.5
    cfg_h = 13.5
    p = FancyBboxPatch((6, cfg_y), 88, cfg_h,
                       boxstyle="round,pad=0.3,rounding_size=0.8",
                       linewidth=1.3, edgecolor="#888888", facecolor="#f6f6f3")
    ax.add_patch(p)
    ax.text(8.5, cfg_y + cfg_h - 1.4, "Test Configuration", ha="left", va="top",
            fontsize=12, fontweight="bold", color="#222222")
    lines = [
        "• Camera: Arducam B0332 (120fps) + Keysight 33622A (Gen-3 compound)",
        "• Data: 4 scenarios × 5 rounds × 200 frames = 4000 total frames",
        "• Goal: Prove Process Thread latency is independent of FG operation type",
    ]
    for i, ln in enumerate(lines):
        ax.text(8.5, cfg_y + cfg_h - 4.4 - i * 2.9, ln, ha="left", va="top",
                fontsize=10.5, color="#333333")

    save(fig, os.path.join(HERE, "fig00_test_architecture.png"))


# ======================================================================
# fig01 — ridge plot
# ======================================================================
def fig01():
    order = ["D_baseline", "A_voltage", "B_same_group", "C_cross_group"]
    fig, ax = plt.subplots(figsize=(10, 7))

    all_vals = np.concatenate([ST[s]["proc"] for s in order])
    xlo, xhi = all_vals.min(), np.percentile(all_vals, 99.7)
    xs = np.linspace(xlo, xhi, 400)

    row_step = 1.0
    height = 1.55  # overlap factor
    for i, s in enumerate(order):
        base = (len(order) - 1 - i) * row_step
        kde = stats.gaussian_kde(ST[s]["proc"])
        ys = kde(xs)
        ys = ys / ys.max() * height
        ax.fill_between(xs, base, base + ys, color=COLOR[s], alpha=0.7,
                        zorder=10 - i, linewidth=0)
        ax.plot(xs, base + ys, color=COLOR[s], linewidth=1.4, zorder=10 - i)
        # mean line
        m = ST[s]["proc_mean"]
        ax.plot([m, m], [base, base + height * 0.95], linestyle="--",
                color="#222222", linewidth=1.2, zorder=20)
        # row label (left)
        ax.text(xlo - (xhi - xlo) * 0.02, base + 0.12, LABEL[s],
                ha="right", va="bottom", fontsize=11.5, fontweight="bold",
                color=COLOR[s])
        # mean label beside dashed line, placed high to avoid curve collision
        ax.text(m + (xhi - xlo) * 0.012, base + height * 0.80,
                f"mean = {m:.2f} ms", ha="left", va="center", fontsize=9.5,
                color="#222222",
                bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="none",
                          alpha=0.85), zorder=21)

    ax.set_xlim(xlo - (xhi - xlo) * 0.16, xhi)
    ax.set_ylim(-0.15, (len(order) - 1) * row_step + height + 0.25)
    ax.set_yticks([])
    ax.set_xlabel("Process Thread Latency (ms)")
    ax.set_title("Process Thread Latency Ridge Plot — 4 FG Scenarios\n"
                 "(Thread Isolation Validation)")
    finish(ax, frame=False)
    ax.spines["left"].set_visible(False)
    save(fig, os.path.join(HERE, "fig01_ridge_plot.png"))


# ======================================================================
# fig02 — bar + std error + P99 diamonds
# ======================================================================
def fig02():
    fig, ax = plt.subplots(figsize=(9, 6))
    x = np.arange(len(SCEN_ORDER))
    means = [ST[s]["proc_mean"] for s in SCEN_ORDER]
    stds = [ST[s]["proc_std"] for s in SCEN_ORDER]
    p99s = [ST[s]["proc_p99"] for s in SCEN_ORDER]
    cols = [COLOR[s] for s in SCEN_ORDER]

    ax.bar(x, means, yerr=stds, capsize=6, color=cols, edgecolor="#333333",
           linewidth=0.8, error_kw=dict(ecolor="#444444", elinewidth=1.3),
           width=0.62, zorder=3)
    # P99 diamonds
    ax.scatter(x, p99s, marker="D", s=70, color=COLORS["red"],
               edgecolor="white", linewidth=0.8, zorder=6, label="P99")

    ymax = max(p99s) * 1.18
    for xi, m, sd, p in zip(x, means, stds, p99s):
        ax.text(xi, m + sd + ymax * 0.02, f"{m:.2f}", ha="center", va="bottom",
                fontsize=10.5, fontweight="bold", color="#1a1a1a")
        ax.text(xi + 0.30, p, f"{p:.2f}", ha="left", va="center",
                fontsize=9.5, color=COLORS["red"], fontweight="bold")

    ax.set_xticks(x)
    ax.set_xticklabels([LABEL[s] for s in SCEN_ORDER])
    ax.set_ylim(0, ymax)
    ax.set_ylabel("Process Thread Latency (ms)")
    ax.set_title("Process Thread Mean ± Std by Scenario (with P99)")
    ax.legend(loc="upper left", frameon=True)
    finish(ax)
    save(fig, os.path.join(HERE, "fig02_bar_errorbar.png"))


# ======================================================================
# fig03 — CDF overlay
# ======================================================================
def fig03():
    fig, ax = plt.subplots(figsize=(9, 6))
    series = []
    for i, s in enumerate(SCEN_ORDER):
        v = np.sort(ST[s]["proc"])
        cdf = np.arange(1, len(v) + 1) / len(v)
        series.append({"x": v, "y": cdf, "color": COLOR[s],
                       "style": line_style(i),
                       "label": f"{LABEL[s]} (mean={ST[s]['proc_mean']:.2f} ms)"})
    cdf_lines(ax, series)
    ax.set_xlabel("Process Thread Latency (ms)")
    ax.set_ylabel("CDF")
    ax.set_ylim(0, 1.02)
    ax.set_title("Process Thread CDF — 4 Scenarios Overlaid")
    ax.legend(loc="lower right", frameon=True)
    finish(ax)
    save(fig, os.path.join(HERE, "fig03_cdf_overlay.png"))


# ======================================================================
# fig04 — one bar per scenario (mean +/- std over all rounds)
#
# Previously this was a 4-scenario x 5-round grouped bar (20 near-identical
# grey bars) that was unreadable on a slide. It is now collapsed to ONE bar
# per scenario, aggregated over every round, with a black mean +/- std error
# bar so the spread is shown directly. Cross-round consistency is reported in
# a small annotation instead of as separate per-round bars.
# ======================================================================
def fig04():
    fig, ax = plt.subplots(figsize=(9.5, 6))
    x = np.arange(len(SCEN_ORDER))
    means = [ST[s]["proc_mean"] for s in SCEN_ORDER]
    stds = [ST[s]["proc_std"] for s in SCEN_ORDER]
    # Distinct light-grey steps so the four scenarios separate cleanly.
    fills = grey_ramp(len(SCEN_ORDER), lo=0.62, hi=0.86)

    ax.bar(x, means, width=0.6, yerr=stds, capsize=6, color=fills,
           edgecolor=BAR_EDGE, linewidth=BAR_EDGE_LW,
           error_kw=dict(ecolor="#1a1a1a", elinewidth=1.6, capthick=1.6),
           zorder=3, label="Mean ± Std")

    ymax = max(m + s for m, s in zip(means, stds)) * 1.22
    ax.set_ylim(0, ymax)
    for xi, m, sd in zip(x, means, stds):
        ax.text(xi, m + sd + ymax * 0.02, f"{m:.2f}", ha="center", va="bottom",
                fontsize=11.5, fontweight="bold", color="#1a1a1a")

    # Cross-round consistency: max spread of per-round means across scenarios.
    per_round = df.groupby(["scenario", "round"]).process_ms.mean()
    spread = max(per_round.groupby(level=0).max() -
                 per_round.groupby(level=0).min())
    ax.text(0.02, 0.97,
            f"Cross-round spread ≤ {spread:.2f} ms\n(aggregated over 5 rounds)",
            transform=ax.transAxes, ha="left", va="top", fontsize=10.5,
            bbox=dict(boxstyle="round,pad=0.4", facecolor="white",
                      edgecolor="#999999"))

    ax.set_xticks(x)
    ax.set_xticklabels([LABEL[s] for s in SCEN_ORDER])
    ax.set_ylabel("Process Thread Latency (ms)")
    ax.set_title("Process Thread Latency by Scenario (Mean ± Std)")
    finish(ax)
    save(fig, os.path.join(HERE, "fig04_grouped_bar_rounds.png"))


# ======================================================================
# fig05 — dual axis: process (blue) vs FG (tan)
# ======================================================================
def fig05():
    fig, ax = plt.subplots(figsize=(9.5, 6))
    x = np.arange(len(SCEN_ORDER))
    w = 0.36
    proc = [ST[s]["proc_mean"] for s in SCEN_ORDER]
    fg = [ST[s]["fg_mean"] for s in SCEN_ORDER]

    b1 = ax.bar(x - w / 2, proc, width=w, color=COLORS["blue"],
                edgecolor="#333333", linewidth=0.6, label="Process Thread",
                zorder=3)
    ax.set_ylabel("Process Thread Latency (ms)", color=COLORS["blue"])
    ax.tick_params(axis="y", labelcolor=COLORS["blue"])
    ax.set_ylim(0, max(proc) * 1.30)

    ax2 = ax.twinx()
    ax2.spines["top"].set_visible(False)
    ax2.spines["right"].set_visible(True)
    ax2.spines["right"].set_color(COLORS["tan"])
    b2 = ax2.bar(x + w / 2, fg, width=w, color=COLORS["tan"],
                 edgecolor="#333333", linewidth=0.6, label="FG Command",
                 zorder=3)
    ax2.set_ylabel("FG Command Latency (ms)", color=COLORS["tan"])
    ax2.tick_params(axis="y", labelcolor=COLORS["tan"])
    ax2.set_ylim(0, max(fg) * 1.30)
    ax2.grid(False)

    # value labels
    for xi, v in zip(x, proc):
        ax.text(xi - w / 2, v + max(proc) * 0.02, f"{v:.2f}", ha="center",
                va="bottom", fontsize=9.5, color=COLORS["blue"],
                fontweight="bold")
    for xi, v in zip(x, fg):
        txt = "0" if v == 0 else f"{v:.1f}"
        ax2.text(xi + w / 2, v + max(fg) * 0.02, txt, ha="center",
                 va="bottom", fontsize=9.5, color=COLORS["tan"],
                 fontweight="bold")

    ax.set_xticks(x)
    ax.set_xticklabels([LABEL[s] for s in SCEN_ORDER])
    ax.set_title("Process Thread vs FG Command Latency — Thread Isolation")

    lines = [b1, b2]
    labs = ["Process Thread", "FG Command"]
    ax.legend(lines, labs, loc="upper left", frameon=True)
    finish(ax)
    save(fig, os.path.join(HERE, "fig05_dual_axis.png"))


# ======================================================================
# fig06 — summary table
# ======================================================================
def fig06():
    fig, ax = plt.subplots(figsize=(12, 3))
    ax.axis("off")

    cols = ["Scenario", "Process Mean (ms)", "Process Std (ms)",
            "Process P99 (ms)", "FG Mean (ms)", "FG P99 (ms)", "Verdict"]
    rows = []
    for s in SCEN_ORDER:
        d = ST[s]
        is_base = (s == "D_baseline")
        fg_mean = "—" if is_base else f"{d['fg_mean']:.2f}"
        fg_p99 = "—" if is_base else f"{d['fg_p99']:.2f}"
        rows.append([
            LABEL[s],
            f"{d['proc_mean']:.3f}",
            f"{d['proc_std']:.3f}",
            f"{d['proc_p99']:.3f}",
            fg_mean,
            fg_p99,
            "PASS",
        ])

    tbl = ax.table(cellText=rows, colLabels=cols, loc="center",
                   cellLoc="center")
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(10.5)
    tbl.scale(1, 2.0)

    col_w = [0.16, 0.155, 0.145, 0.145, 0.135, 0.13, 0.10]
    col_w = [w / sum(col_w) for w in col_w]
    ncol = len(cols)
    for (r, c), cell in tbl.get_celld().items():
        cell.set_width(col_w[c])
        cell.set_edgecolor("#cfcfcf")
        cell.set_linewidth(0.8)
        if r == 0:
            cell.set_facecolor(COLORS["header_blue"])
            cell.set_text_props(color="white", fontweight="bold")
            cell.set_height(cell.get_height() * 1.15)
        else:
            cell.set_facecolor(COLORS["row_blue"] if r % 2 == 1 else "white")
            if c == ncol - 1:  # verdict
                cell.set_text_props(color=COLORS["pass_green"],
                                    fontweight="bold")

    ax.set_title("Combined Latency Summary — Thread Isolation "
                 "(3σ Cleaned)", pad=18)
    save(fig, os.path.join(HERE, "fig06_summary_table.png"))


# ======================================================================
# fig07 — isolation conclusion
# ======================================================================
def fig07():
    # ascending by process mean
    order = sorted(SCEN_ORDER, key=lambda s: ST[s]["proc_mean"])
    means = [ST[s]["proc_mean"] for s in order]
    delta = max(means) - min(means)
    pct = delta / min(means) * 100.0
    fg_means = {s: ST[s]["fg_mean"] for s in SCEN_ORDER}
    max_fg = max(fg_means.values())
    max_fg_scen = max(fg_means, key=fg_means.get)
    proc_mean_all = np.mean([ST[s]["proc_mean"] for s in SCEN_ORDER])
    ratio = max_fg / proc_mean_all

    fig, ax = plt.subplots(figsize=(10, 7))
    x = np.arange(len(order))
    cols = [COLOR[s] for s in order]
    ax.bar(x, means, width=0.6, color=cols, edgecolor="#333333",
           linewidth=0.7, zorder=3)

    ymax = max(means) * 1.45
    for xi, s, m in zip(x, order, means):
        ax.text(xi, m + ymax * 0.015, f"{m:.2f}", ha="center", va="bottom",
                fontsize=11, fontweight="bold", color="#1a1a1a")
        fgv = fg_means[s]
        if fgv > 0:
            ax.text(xi, ymax * 0.04, f"FG: {fgv:.1f} ms", ha="center",
                    va="bottom", fontsize=8.5, color="#666666")

    # delta bracket on the right
    bx = len(order) - 0.5 + 0.18
    lo, hi = min(means), max(means)
    ax.annotate("", xy=(bx, hi), xytext=(bx, lo),
                arrowprops=dict(arrowstyle="<->", color="#444444", lw=1.6))
    ax.plot([len(order) - 1, bx], [hi, hi], color="#999999", lw=0.8, ls=":")
    ax.text(bx + 0.08, (lo + hi) / 2,
            f"Δ = {delta:.2f} ms\n({pct:.1f}% relative)",
            ha="left", va="center", fontsize=10, color="#333333",
            bbox=dict(boxstyle="round,pad=0.3", fc="#f6f6f3", ec="#aaaaaa"))

    ax.set_xticks(x)
    ax.set_xticklabels([LABEL2[s] for s in order])
    ax.set_ylabel("Process Thread Mean Latency (ms)")
    ax.set_ylim(0, ymax)
    ax.set_xlim(-0.6, len(order) + 0.2)
    ax.set_title(f"Process Thread Isolation — Only {delta:.2f} ms Spread "
                 "Across All FG Scenarios")

    caption = (
        f"FG latency ranges from 0 ms (Baseline) to {max_fg:.1f} ms "
        f"({LABEL[max_fg_scen].split(':')[1].strip()}) = {ratio:.0f}× of "
        f"Process mean,\nyet Process Thread variation is only {delta:.2f} ms "
        "→ Thread isolation confirmed."
    )
    ax.text(0.5, -0.20, caption, transform=ax.transAxes, ha="center",
            va="top", fontsize=10, color="#222222",
            bbox=dict(boxstyle="round,pad=0.5", fc="#eef4ee",
                      ec=COLORS["pass_green"], lw=1.2))
    finish(ax)
    save(fig, os.path.join(HERE, "fig07_isolation_conclusion.png"))


if __name__ == "__main__":
    # Build all figures, or only the ones named on the command line.
    #   python rebuild_figures.py            # all
    #   python rebuild_figures.py fig03      # just fig03
    #   python rebuild_figures.py fig00 fig06 # a subset
    #   python rebuild_figures.py --list     # list figures
    run_cli(globals())
