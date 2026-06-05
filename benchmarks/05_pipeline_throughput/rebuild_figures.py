"""Rebuild all top-level academic figures for 05_pipeline_throughput.

Produces clean publication PNGs:
  - pure white background
  - no gridlines
  - no "testXX"/"Test 0X" tokens
  - no overlapping text

Run from this folder (or anywhere); paths are resolved relative to the script.
"""

import sys
import pathlib

sys.path.insert(0, "/mnt/c/Users/liuuhua/Desktop/Git Repository/camera_function_generator_multithreaded/benchmarks")
from paper_style import apply_style, COLORS, save, finish, run_cli

import numpy as np
import pandas as pd
from scipy import stats
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

apply_style()

HERE = pathlib.Path(__file__).resolve().parent
CSV = HERE / "throughput_cleaned.csv"

df = pd.read_csv(CSV)
df = df.sort_values(["round", "frame_idx"]).reset_index(drop=True)
df["idx"] = np.arange(len(df))  # continuous frame index across concatenation

N = len(df)
ROUNDS = sorted(df["round"].unique())

active = df[df["fg_sent"] == 1]
idle = df[df["fg_sent"] == 0]
n_active = len(active)
n_idle = len(idle)


def stat_block(s):
    return dict(mean=s.mean(), std=s.std(), median=s.median(),
                p95=s.quantile(0.95), p99=s.quantile(0.99), max=s.max())


# ----------------------------------------------------------------------
# fig00 — architecture diagram + tables (axis off)
# ----------------------------------------------------------------------
def fig00():
    fig, ax = plt.subplots(figsize=(11, 8.5))
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 100)
    ax.axis("off")

    fig.suptitle("Pipeline Throughput Benchmark", fontsize=18, fontweight="bold",
                 y=0.985)
    ax.text(50, 96.5, "Sustained 120 fps Validation — 3-Thread Pipeline",
            ha="center", va="center", fontsize=12.5, color="#333333")

    # ---- Top flow row ----
    flow_y = 84
    box_h = 9.0
    proc_boxes = [
        dict(x=4, w=18, fc="#dbe7f3", title="Capture Thread",
             lines=["cap.read()", "120 fps blocking"]),
        dict(x=40, w=18, fc="#e3f0e6", title="Process Thread",
             lines=["TPS + HSV", "Kalman + EMA"]),
        dict(x=78, w=18, fc="#f3e7d6", title="Main Thread",
             lines=["PID + FG voltage", "(every 5 frames)"]),
    ]
    arrow_boxes = [
        dict(x=24, w=12, fc="#eeeeee", title="frame_queue", lines=["(size=120)", "FIFO buffer"]),
        dict(x=60, w=12, fc="#eeeeee", title="result_queue", lines=["FIFO buffer"]),
    ]

    def draw_box(x, w, y, h, fc, title, lines, ec="#5a6b7b"):
        box = FancyBboxPatch((x, y - h / 2), w, h,
                             boxstyle="round,pad=0.3,rounding_size=0.8",
                             linewidth=1.3, edgecolor=ec, facecolor=fc, zorder=2)
        ax.add_patch(box)
        cx = x + w / 2
        ax.text(cx, y + h / 2 - 1.9, title, ha="center", va="center",
                fontsize=10.5, fontweight="bold", zorder=3)
        for i, ln in enumerate(lines):
            ax.text(cx, y + h / 2 - 3.7 - i * 2.1, ln, ha="center", va="center",
                    fontsize=8.6, color="#444444", zorder=3)

    for b in proc_boxes:
        draw_box(b["x"], b["w"], flow_y, box_h, b["fc"], b["title"], b["lines"])
    for b in arrow_boxes:
        draw_box(b["x"], b["w"], flow_y, box_h - 1.5, b["fc"], b["title"], b["lines"],
                 ec="#9aa6b0")

    # arrows connecting the row
    seq_x = [(22, 24), (36, 40), (58, 60), (72, 78)]
    for x0, x1 in seq_x:
        ax.add_patch(FancyArrowPatch((x0, flow_y), (x1, flow_y),
                     arrowstyle="-|>", mutation_scale=16, linewidth=1.6,
                     color="#5a6b7b", zorder=1))

    # ---- Middle latency table ----
    tbl_top = 68
    ax.text(50, tbl_top + 2.5, "Per-Frame Latency Breakdown", ha="center",
            va="center", fontsize=12, fontweight="bold")
    lat_cols = ["Stage", "Mean (ms)", "P99 (ms)"]
    lat_rows = [
        ["Queue Wait", "0.16", "0.82"],
        ["Process", "2.47", "4.23"],
        ["FG Write (active)", "0.56", "0.90"],
        ["Total", "2.59", "4.65"],
    ]
    col_x = [26, 52, 72]
    row_h = 4.0
    header_y = tbl_top - 1.5
    # header band
    ax.add_patch(plt.Rectangle((20, header_y - row_h / 2), 60, row_h,
                 facecolor=COLORS["header_blue"], edgecolor="none", zorder=1))
    for cx, c in zip(col_x, lat_cols):
        ax.text(cx, header_y, c, ha="center", va="center", fontsize=9.8,
                color="white", fontweight="bold", zorder=3)
    for r, row in enumerate(lat_rows):
        ry = header_y - row_h * (r + 1)
        if r % 2 == 0:
            ax.add_patch(plt.Rectangle((20, ry - row_h / 2), 60, row_h,
                         facecolor=COLORS["row_blue"], edgecolor="none", zorder=0))
        bold = (row[0] == "Total")
        for cx, c in zip(col_x, row):
            ax.text(cx, ry, c, ha="center", va="center", fontsize=9.5,
                    fontweight="bold" if bold else "normal", zorder=3)
    # table border
    ax.add_patch(plt.Rectangle((20, header_y - row_h / 2 - row_h * len(lat_rows)),
                 60, row_h * (len(lat_rows) + 1), facecolor="none",
                 edgecolor="#9aa6b0", linewidth=1.1, zorder=2))

    # ---- Bottom configuration & results block ----
    blk_top = 34
    ax.add_patch(FancyBboxPatch((8, 6), 84, blk_top - 6,
                 boxstyle="round,pad=0.5,rounding_size=1.2",
                 linewidth=1.3, edgecolor="#5a6b7b", facecolor="#f7f9fb", zorder=1))
    ax.text(50, blk_top - 3.0, "Test Configuration & Results", ha="center",
            va="center", fontsize=12, fontweight="bold", zorder=3)
    cfg_lines = [
        "Data: 5 rounds × 500 frames = 2500 frames, FG command every 5 frames",
        "Effective FPS: 120.6 ± 0.1   |   Drop rate: 0.0%   |   "
        "Process P99: 4.23 ms < 8.33 ms",
        "Conclusion: Pipeline sustains 120 fps with zero frame drops",
    ]
    for i, ln in enumerate(cfg_lines):
        ax.text(50, blk_top - 8.5 - i * 5.5, ln, ha="center", va="center",
                fontsize=10.2, color="#222222", zorder=3,
                fontweight="bold" if i == 2 else "normal")

    save(fig, str(HERE / "fig00_test_architecture.png"))


# ----------------------------------------------------------------------
# fig01 — stacked decomposition
# ----------------------------------------------------------------------
def fig01():
    groups = {
        "All Frames": df,
        "FG-Active\n(every 5th)": active,
        "FG-Idle\n(remaining)": idle,
    }
    qw = [g["queue_wait_ms"].mean() for g in groups.values()]
    pr = [g["process_ms"].mean() for g in groups.values()]
    fg = [g["fg_ms"].mean() for g in groups.values()]

    fig, ax = plt.subplots(figsize=(7.2, 5.6))
    x = np.arange(len(groups))
    w = 0.55
    b1 = ax.bar(x, qw, w, label="Queue Wait", color=COLORS["grey"])
    b2 = ax.bar(x, pr, w, bottom=qw, label="Process", color=COLORS["green"])
    b3 = ax.bar(x, fg, w, bottom=np.array(qw) + np.array(pr), label="FG Write",
                color=COLORS["tan"])

    totals = np.array(qw) + np.array(pr) + np.array(fg)
    for xi, t in zip(x, totals):
        ax.text(xi, t + 0.07, f"{t:.2f}", ha="center", va="bottom",
                fontsize=10.5, fontweight="bold")

    ax.set_xticks(x)
    ax.set_xticklabels(list(groups.keys()))
    ax.set_ylabel("Latency (ms)")
    ax.set_ylim(0, totals.max() * 1.18)
    ax.set_title("Per-Frame Latency Decomposition")
    ax.legend(loc="upper right", frameon=True)
    finish(ax)
    fig.tight_layout()
    save(fig, str(HERE / "fig01_stacked_decomposition.png"))


# ----------------------------------------------------------------------
# fig02 — per-round process mean bar + P99 diamonds
# ----------------------------------------------------------------------
def fig02():
    means = [df[df["round"] == r]["process_ms"].mean() for r in ROUNDS]
    p99 = [df[df["round"] == r]["process_ms"].quantile(0.99) for r in ROUNDS]

    fig, ax = plt.subplots(figsize=(7.4, 5.6))
    x = np.arange(len(ROUNDS))
    ax.bar(x, means, 0.55, color=COLORS["blue"], label="Mean")
    ax.scatter(x, p99, marker="D", s=70, color=COLORS["red"], zorder=5,
               label="P99")

    for xi, m in zip(x, means):
        ax.text(xi, m + 0.05, f"{m:.2f}", ha="center", va="bottom",
                fontsize=10, fontweight="bold")
    for xi, p in zip(x, p99):
        ax.text(xi, p + 0.10, f"{p:.2f}", ha="center", va="bottom",
                fontsize=9.5, color=COLORS["red"], fontweight="bold")

    ax.set_xticks(x)
    ax.set_xticklabels([f"Round {r}" for r in ROUNDS])
    ax.set_ylabel("Process Latency (ms)")
    ax.set_ylim(0, max(p99) * 1.22)
    ax.set_title("Process Thread Latency Per Round (Mean + P99)")
    ax.legend(loc="upper right", frameon=True)
    finish(ax)
    fig.tight_layout()
    save(fig, str(HERE / "fig02_per_round_bar.png"))


# ----------------------------------------------------------------------
# fig03 — CDF of total_ms by round
# ----------------------------------------------------------------------
def fig03():
    fig, ax = plt.subplots(figsize=(7.6, 5.6))
    ramp = COLORS["ramp5"]
    for i, r in enumerate(ROUNDS):
        s = np.sort(df[df["round"] == r]["total_ms"].values)
        cdf = np.arange(1, len(s) + 1) / len(s)
        ax.plot(s, cdf, color=ramp[i % len(ramp)], linewidth=1.9,
                label=f"Round {r} (mean={s.mean():.2f}ms)")
    ax.set_xlabel("Total Frame Latency (ms)")
    ax.set_ylabel("CDF")
    ax.set_ylim(0, 1.02)
    ax.set_title("Total Frame Latency CDF — 5 Rounds Overlaid")
    ax.legend(loc="lower right", frameon=True)
    finish(ax)
    fig.tight_layout()
    save(fig, str(HERE / "fig03_cdf_rounds.png"))


# ----------------------------------------------------------------------
# fig04 — time series scatter
# ----------------------------------------------------------------------
def fig04():
    fig, ax = plt.subplots(figsize=(13, 6.2))
    nrm = df[df["fg_sent"] == 0]
    act = df[df["fg_sent"] == 1]
    ax.scatter(nrm["idx"], nrm["total_ms"], s=6, alpha=0.45,
               color=COLORS["blue"], label="Normal frame", edgecolors="none")
    ax.scatter(act["idx"], act["total_ms"], s=10, alpha=0.7,
               color=COLORS["tan"], label="FG-active frame", edgecolors="none")

    ymin = df["total_ms"].min() * 0.9
    ymax_data = df["total_ms"].max()
    ax.set_ylim(ymin, ymax_data * 1.22)

    omean = df["total_ms"].mean()
    ax.axhline(omean, color=COLORS["red"], linewidth=1.6, linestyle="-")
    # place the mean label just below the line, in clear space, away from points
    ax.text(N * 0.015, omean - (omean - ymin) * 0.22,
            f"mean={omean:.2f}ms", color=COLORS["red"], fontsize=11,
            fontweight="bold", va="top", ha="left",
            bbox=dict(boxstyle="round,pad=0.25", facecolor="white",
                      edgecolor="none", alpha=0.85))

    # round boundaries (cumulative counts)
    counts = [len(df[df["round"] == r]) for r in ROUNDS]
    cum = np.cumsum(counts)
    for b in cum[:-1]:
        ax.axvline(b, color=COLORS["grey"], linewidth=1.0, linestyle="--",
                   alpha=0.8)

    ax.set_xlim(-N * 0.01, N * 1.01)
    ax.set_xlabel("Frame Index")
    ax.set_ylabel("Total Latency (ms)")
    ax.set_title(f"Frame-by-Frame Latency ({N} Frames, Sustained Pipeline)",
                 pad=14)
    ax.legend(loc="upper right", ncol=2, frameon=True, markerscale=2.2)
    finish(ax)
    fig.tight_layout()
    save(fig, str(HERE / "fig04_timeseries.png"))


# ----------------------------------------------------------------------
# fig05 — FG-active vs FG-idle grouped bar
# ----------------------------------------------------------------------
def fig05():
    cols = ["queue_wait_ms", "process_ms", "fg_ms", "total_ms"]
    labels = ["Queue Wait", "Process", "FG Write", "Total"]
    av = [active[c].mean() for c in cols]
    iv = [idle[c].mean() for c in cols]

    fig, ax = plt.subplots(figsize=(8.2, 5.6))
    x = np.arange(len(cols))
    w = 0.38
    ba = ax.bar(x - w / 2, av, w, color=COLORS["tan"],
                label=f"FG-Active (n={n_active})")
    bi = ax.bar(x + w / 2, iv, w, color=COLORS["blue"],
                label=f"FG-Idle (n={n_idle})")

    ymax = max(max(av), max(iv))
    for bars, vals in ((ba, av), (bi, iv)):
        for b, v in zip(bars, vals):
            ax.text(b.get_x() + b.get_width() / 2, v + ymax * 0.012,
                    f"{v:.2f}", ha="center", va="bottom", fontsize=8.8)

    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylabel("Mean Latency (ms)")
    ax.set_ylim(0, ymax * 1.18)
    ax.set_title("FG-Active vs FG-Idle Frames — Overhead Comparison")
    ax.legend(loc="upper left", frameon=True)
    finish(ax)
    fig.tight_layout()
    save(fig, str(HERE / "fig05_fg_comparison.png"))


# ----------------------------------------------------------------------
# fig06 — summary table
# ----------------------------------------------------------------------
def fig06():
    metrics = [
        ("Queue Wait (ms)", stat_block(df["queue_wait_ms"])),
        ("Process (ms)", stat_block(df["process_ms"])),
        ("FG Write (ms)", stat_block(df["fg_ms"])),
        ("Total (ms)", stat_block(df["total_ms"])),
    ]
    header = ["Metric", "Mean", "Std", "Median", "P95", "P99", "Max"]

    # Per-round process means
    pr_means = [df[df["round"] == r]["process_ms"].mean() for r in ROUNDS]
    spread = max(pr_means) - min(pr_means)
    p99_proc = df["process_ms"].quantile(0.99)

    rows = []
    styles = []  # per-row style tag

    def fmt(d):
        return [f"{d['mean']:.2f}", f"{d['std']:.2f}", f"{d['median']:.2f}",
                f"{d['p95']:.2f}", f"{d['p99']:.2f}", f"{d['max']:.2f}"]

    for name, d in metrics:
        rows.append([name] + fmt(d))
        styles.append("data")

    # section separator + per-round process
    rows.append(["Per-Round Process Mean", "", "", "", "", "", ""])
    styles.append("section")
    rows.append([
        "R1–R5 (ms)",
        f"R1 {pr_means[0]:.2f}", f"R2 {pr_means[1]:.2f}", f"R3 {pr_means[2]:.2f}",
        f"R4 {pr_means[3]:.2f}", f"R5 {pr_means[4]:.2f}", f"Spread {spread:.2f}",
    ])
    styles.append("data")

    rows.append(["System Metrics", "Value", "Standard", "Result", "", "", ""])
    styles.append("section")
    rows.append(["Effective FPS", "120.6", "≥ 114", "PASS", "", "", ""])
    styles.append("sys")
    rows.append(["Drop Rate", "0.0%", "< 5%", "PASS", "", "", ""])
    styles.append("sys")
    rows.append(["Process P99", f"{p99_proc:.2f} ms", "< 8.33 ms", "PASS",
                 "", "", ""])
    styles.append("sys")

    ncol = len(header)
    nrow = len(rows) + 1  # +header

    fig, ax = plt.subplots(figsize=(11.5, 6.2))
    ax.axis("off")
    fig.suptitle("Pipeline Throughput Summary (3σ Cleaned)", fontsize=15,
                 fontweight="bold", y=0.97)

    col_w = [0.26, 0.135, 0.135, 0.135, 0.115, 0.105, 0.115]
    col_w = [c / sum(col_w) for c in col_w]
    col_x = np.concatenate([[0], np.cumsum(col_w)])

    top = 0.90
    bottom = 0.06
    rh = (top - bottom) / nrow

    def cell(r, c, text, **kw):
        x0 = col_x[c]
        cx = x0 + col_w[c] / 2
        cy = top - (r + 0.5) * rh
        ax.text(cx, cy, text, ha="center", va="center",
                transform=ax.transAxes, **kw)

    # header band
    ax.add_patch(plt.Rectangle((0, top - rh), 1, rh, transform=ax.transAxes,
                 facecolor=COLORS["header_blue"], edgecolor="none"))
    for c, h in enumerate(header):
        cell(0, c, h, color="white", fontsize=10.5, fontweight="bold")

    for ri, (row, st) in enumerate(zip(rows, styles), start=1):
        y0 = top - (ri + 1) * rh
        if st == "section":
            ax.add_patch(plt.Rectangle((0, y0), 1, rh, transform=ax.transAxes,
                         facecolor="#d3deea", edgecolor="none"))
            cell(ri, 0, row[0], fontsize=10, fontweight="bold",
                 color="#1f3a5a")
            for c in range(1, ncol):
                if row[c]:
                    cell(ri, c, row[c], fontsize=9.5, fontweight="bold",
                         color="#1f3a5a")
        else:
            if ri % 2 == 0:
                ax.add_patch(plt.Rectangle((0, y0), 1, rh, transform=ax.transAxes,
                             facecolor=COLORS["row_blue"], edgecolor="none"))
            for c in range(ncol):
                if not row[c]:
                    continue
                kw = dict(fontsize=9.6)
                if c == 0:
                    kw.update(fontweight="bold")
                if st == "sys" and row[c] == "PASS":
                    kw.update(color=COLORS["pass_green"], fontweight="bold")
                cell(ri, c, row[c], **kw)

    # grid lines
    for ri in range(nrow + 1):
        y = top - ri * rh
        ax.plot([0, 1], [y, y], transform=ax.transAxes, color="#c2ccd6",
                linewidth=0.7)
    ax.plot([0, 0], [bottom, top], transform=ax.transAxes, color="#c2ccd6",
            linewidth=0.7)
    ax.plot([1, 1], [bottom, top], transform=ax.transAxes, color="#c2ccd6",
            linewidth=0.7)

    save(fig, str(HERE / "fig06_summary_table.png"))


# ----------------------------------------------------------------------
# fig07 — stability & correlation (3 panels)
# ----------------------------------------------------------------------
def fig07():
    fig, axes = plt.subplots(1, 3, figsize=(16, 5.2), constrained_layout=True)
    fig.suptitle("Pipeline Stability & Correlation Analysis", fontsize=15,
                 fontweight="bold")

    # (a) Temporal stability
    ax = axes[0]
    finish(ax)
    proc = df["process_ms"].values
    idx = df["idx"].values
    omean = proc.mean()
    ostd = proc.std()
    cv = ostd / omean * 100
    roll = pd.Series(proc).rolling(50, min_periods=1).mean().values
    ax.scatter(idx, proc, s=4, alpha=0.25, color=COLORS["blue2"],
               edgecolors="none")
    ax.fill_between(idx, omean - ostd, omean + ostd, color=COLORS["green"],
                    alpha=0.15, label="±1σ band")
    ax.plot(idx, roll, color=COLORS["green"], linewidth=1.8,
            label="Rolling mean (50)")
    ax.axhline(omean, color=COLORS["red"], linewidth=1.4, linestyle="--",
               label=f"Overall mean = {omean:.2f} ms")
    ax.text(0.97, 0.95, f"CV = {cv:.1f}%", transform=ax.transAxes, ha="right",
            va="top", fontsize=10, fontweight="bold",
            bbox=dict(boxstyle="round,pad=0.4", facecolor="white",
                      edgecolor="#bbbbbb"))
    ax.set_xlabel("Frame Index")
    ax.set_ylabel("Process Latency (ms)")
    ax.set_title("Temporal Stability")
    ax.legend(loc="lower right", frameon=True, fontsize=8.5)

    # (b) Queue Wait vs Process
    ax = axes[1]
    finish(ax)
    r, p = stats.pearsonr(df["queue_wait_ms"], df["process_ms"])
    ax.scatter(df["queue_wait_ms"], df["process_ms"], s=6, alpha=0.3,
               color=COLORS["purple"], edgecolors="none")
    ptxt = "p < 0.001" if p < 0.001 else f"p = {p:.3f}"
    ax.text(0.05, 0.95, f"r = {r:.2f} / {ptxt}", transform=ax.transAxes,
            ha="left", va="top", fontsize=10, fontweight="bold",
            bbox=dict(boxstyle="round,pad=0.4", facecolor="white",
                      edgecolor="#bbbbbb"))
    ax.set_xlabel("Queue Wait (ms)")
    ax.set_ylabel("Process Latency (ms)")
    ax.set_title("Queue Wait vs Process")

    # (c) Round consistency
    ax = axes[2]
    finish(ax)
    means = [df[df["round"] == rr]["process_ms"].mean() for rr in ROUNDS]
    stds = [df[df["round"] == rr]["process_ms"].std() for rr in ROUNDS]
    p99 = [df[df["round"] == rr]["process_ms"].quantile(0.99) for rr in ROUNDS]
    x = np.arange(len(ROUNDS))
    ax.bar(x, means, 0.55, yerr=stds, capsize=4, color=COLORS["blue"],
           label="Mean ± Std",
           error_kw=dict(ecolor="#333333", elinewidth=1.1))
    ax.scatter(x, p99, marker="D", s=60, color=COLORS["red"], zorder=5,
               label="P99")
    spread = max(means) - min(means)
    spread_pct = spread / np.mean(means) * 100
    ax.text(0.5, 0.96, f"Mean spread = {spread:.2f} ms ({spread_pct:.1f}%)",
            transform=ax.transAxes, ha="center", va="top", fontsize=9.5,
            fontweight="bold",
            bbox=dict(boxstyle="round,pad=0.4", facecolor="white",
                      edgecolor="#bbbbbb"))
    ax.set_xticks(x)
    ax.set_xticklabels([f"R{rr}" for rr in ROUNDS])
    ax.set_ylabel("Latency (ms)")
    ax.set_ylim(0, max(p99) * 1.25)
    ax.set_title("Round Consistency")
    ax.legend(loc="lower right", frameon=True, fontsize=8.5)

    save(fig, str(HERE / "fig07_stability_correlation.png"))


if __name__ == "__main__":
    # Build all figures, or only the ones named on the command line.
    #   python rebuild_figures.py            # all
    #   python rebuild_figures.py fig03      # just fig03
    #   python rebuild_figures.py fig00 fig06 # a subset
    #   python rebuild_figures.py --list     # list figures
    run_cli(globals())
