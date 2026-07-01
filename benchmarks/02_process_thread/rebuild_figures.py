"""Rebuild all top-level academic figures for 02_process_thread.

Publication-clean PNGs: pure white background, no gridlines, no "testXX"
tokens, no overlapping text. All quantities computed from the cleaned CSV.
"""

import sys
import os

sys.path.insert(0, "/mnt/c/Users/liuuhua/Desktop/Git Repository/camera_function_generator_multithreaded/benchmarks")
from paper_style import apply_style, COLORS, save, finish, run_cli, line_style, grey_ramp, BAR_EDGE, BAR_EDGE_LW, LINE_BLACK, LW_MAIN, LW_MULTI, cdf_lines

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

apply_style()

HERE = os.path.dirname(os.path.abspath(__file__))
CSV = os.path.join(HERE, "process_thread_cleaned.csv")

df = pd.read_csv(CSV)

# Online frames = mode != 'offline'  (online_single + online_round).
online = df[df["mode"] != "offline"].copy()
# Multi-round = rows with round in 1..5.
multi = df[df["round"].isin([1, 2, 3, 4, 5])].copy()

# Five aggregated pipeline-step series (per-frame), derived from columns.
STEP_LABELS = ["TPS Undistort", "Rotate + Flip", "HSV + Morph",
               "Kalman + EMA", "Draw Overlay"]


def step_series(frame):
    """Return a DataFrame with the 5 aggregated step columns."""
    return pd.DataFrame({
        "TPS Undistort": frame["undistort_ms"].values,
        "Rotate + Flip": frame["rotate_flip_ms"].values,
        "HSV + Morph":   frame["hsv_detect_ms"].values,
        "Kalman + EMA":  (frame["kalman_ms"] + frame["ema_ms"]).values,
        "Draw Overlay":  frame["draw_ms"].values,
    })


# ----------------------------------------------------------------------
# fig00a — pipeline architecture (vertical box diagram)
# ----------------------------------------------------------------------
def fig00a():
    boxes = [
        ("Input Frame", "640x480, Monochrome, 120 fps (Arducam B0332)", COLORS["grey_l"]),
        ("Step 1: TPS Undistort", "cv2.remap with pre-computed float32 maps", COLORS["blue2"]),
        ("Step 2: Rotate 90deg", "cv2.rotate (ROTATE_90_CLOCKWISE)", COLORS["blue2"]),
        ("Step 3: Flip V + H", "cv2.flip (vertical + horizontal)", COLORS["blue2"]),
        ("Step 4: Kalman Predict", "4-state linear Kalman filter (x, y, vx, vy)", COLORS["tan2"]),
        ("Step 5: HSV Detection", "Color mask + Morphology + findContours + minAreaRect", COLORS["tan2"]),
        ("Step 6: Kalman Correct + EMA Smoothing",
         "Update Kalman state + exponential moving average", COLORS["tan2"]),
        ("Step 7: Draw Overlay", "polylines + circle + putText on frame", COLORS["blue2"]),
        ("Output", "Processed result, 462x314", COLORS["pass_green"]),
    ]

    fig, ax = plt.subplots(figsize=(8.5, 13))
    ax.set_xlim(0, 10)
    n = len(boxes)
    # vertical layout
    top = 9.3
    bottom = 0.4
    box_h = 0.78
    gap = (top - bottom - n * box_h) / (n - 1)
    ax.set_ylim(0, 10)
    ax.axis("off")

    # header + title
    ax.text(5, 9.92, "Pipeline Architecture (per frame)", ha="center", va="top",
            fontsize=11.5, style="italic", color="#555555")
    ax.text(5, 9.72, "Image Processing Pipeline", ha="center", va="top",
            fontsize=16, fontweight="bold", color="#222222")

    centers = []
    for i, (title, sub, color) in enumerate(boxes):
        y = top - i * (box_h + gap) - box_h
        cx = 5.0
        w = 7.2
        patch = FancyBboxPatch((cx - w / 2, y), w, box_h,
                               boxstyle="round,pad=0.02,rounding_size=0.12",
                               linewidth=1.1, edgecolor="#444444",
                               facecolor=color, alpha=0.92)
        ax.add_patch(patch)
        ax.text(cx, y + box_h * 0.62, title, ha="center", va="center",
                fontsize=11.5, fontweight="bold", color="#1a1a1a")
        ax.text(cx, y + box_h * 0.24, sub, ha="center", va="center",
                fontsize=9, color="#333333")
        centers.append((cx, y, y + box_h))

    # arrows between boxes
    for i in range(n - 1):
        _, y_lo_top, _ = centers[i]
        cx, _, _ = centers[i + 1]
        y_top_next = centers[i + 1][2]
        arr = FancyArrowPatch((cx, centers[i][1]), (cx, y_top_next),
                              arrowstyle="-|>", mutation_scale=16,
                              linewidth=1.6, color="#555555")
        ax.add_patch(arr)

    ax.text(5, 0.12,
            "Camera: Arducam B0332 (OV9281, global shutter)  |  "
            "Host: Windows PC, Python 3.14, OpenCV 4.13",
            ha="center", va="bottom", fontsize=9, style="italic", color="#666666")

    save(fig, os.path.join(HERE, "fig00a_pipeline_architecture.png"))


# ----------------------------------------------------------------------
# fig00b — evaluation conditions
# ----------------------------------------------------------------------
def fig00b():
    fig, ax = plt.subplots(figsize=(9, 11))
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 12)
    ax.axis("off")

    ax.text(5, 11.7, "Evaluation Conditions", ha="center", va="top",
            fontsize=16, fontweight="bold", color="#222222")
    ax.text(5, 11.15, "Three modes for pipeline latency characterization",
            ha="center", va="top", fontsize=11.5, style="italic", color="#555555")

    panels = [
        ("Mode A: Offline", "#e5e5e5",
         ["500 synthetic frames (pre-generated grayscale images)",
          "No camera I/O - isolates pure computation cost",
          "Purpose: Establish processing baseline without hardware variance"]),
        ("Mode B: Online Single-Pass", "#e8e8e8",
         ["500 live camera frames (Arducam B0332 @ 120 fps, MJPG)",
          "Single continuous acquisition run",
          "Purpose: Measure real-world latency including OS scheduling jitter"]),
        ("Mode C: Online Multi-Round", "#e5e5e5",
         ["5 rounds x 200 frames (total 1000 frames)",
          "Inter-round pause and pipeline restart between each round",
          "Purpose: Verify temporal consistency and repeatability"]),
    ]

    top = 10.4
    box_h = 2.3
    gap = 0.45
    for i, (title, color, bullets) in enumerate(panels):
        y = top - i * (box_h + gap) - box_h
        w = 8.6
        cx = 5.0
        patch = FancyBboxPatch((cx - w / 2, y), w, box_h,
                               boxstyle="round,pad=0.02,rounding_size=0.08",
                               linewidth=1.2, edgecolor="#555555",
                               facecolor=color)
        ax.add_patch(patch)
        ax.text(cx - w / 2 + 0.3, y + box_h - 0.32, title, ha="left", va="top",
                fontsize=13, fontweight="bold", color="#1a1a1a")
        for j, b in enumerate(bullets):
            ax.text(cx - w / 2 + 0.5, y + box_h - 0.85 - j * 0.46,
                    "• " + b, ha="left", va="top", fontsize=10.3,
                    color="#222222")

    # bottom summary box
    y = top - 3 * (box_h + gap) - 0.7
    w = 8.6
    cx = 5.0
    patch = FancyBboxPatch((cx - w / 2, y), w, 0.7,
                           boxstyle="round,pad=0.02,rounding_size=0.08",
                           linewidth=1.2, edgecolor="#444444",
                           facecolor="#e9e9e9")
    ax.add_patch(patch)
    ax.text(cx, y + 0.35,
            "Total: 2000 frames  |  Timing: time.perf_counter() (sub-us resolution)",
            ha="center", va="center", fontsize=11, fontweight="bold", color="#222222")

    save(fig, os.path.join(HERE, "fig00b_test_conditions.png"))


# ----------------------------------------------------------------------
# fig01 — grouped bar: 5 step groups x 5 rounds (mean +/- std)
# ----------------------------------------------------------------------
def fig01():
    # order groups by descending overall mean
    overall = step_series(multi).mean()
    order = ["HSV + Morph", "TPS Undistort", "Rotate + Flip",
             "Draw Overlay", "Kalman + EMA"]
    rounds = [1, 2, 3, 4, 5]
    ramp = COLORS["ramp5"]

    means = np.zeros((len(order), len(rounds)))
    stds = np.zeros((len(order), len(rounds)))
    for ri, r in enumerate(rounds):
        sub = step_series(multi[multi["round"] == r])
        for gi, g in enumerate(order):
            means[gi, ri] = sub[g].mean()
            stds[gi, ri] = sub[g].std()

    fig, ax = plt.subplots(figsize=(11, 5.5))
    x = np.arange(len(order))
    nb = len(rounds)
    width = 0.15
    for ri, r in enumerate(rounds):
        offset = (ri - (nb - 1) / 2) * width
        ax.bar(x + offset, means[:, ri], width, yerr=stds[:, ri],
               color=ramp[ri], edgecolor="#33333344", linewidth=0.5,
               capsize=2.5, error_kw=dict(elinewidth=0.8, ecolor="#555555"),
               label=f"Round {r}")

    ax.set_xticks(x)
    ax.set_xticklabels(order)
    ax.set_ylabel("Processing Time (ms)")
    ax.set_title("Per-Step Latency x 5 Rounds (Mean +/- Std, 200 frames/round)")
    ax.legend(loc="upper right", ncol=1, frameon=True)
    finish(ax)
    fig.tight_layout()
    save(fig, os.path.join(HERE, "fig01_grouped_bar_steps.png"))


# ----------------------------------------------------------------------
# fig02 — empirical CDF of total_ms per round
# ----------------------------------------------------------------------
def fig02():
    rounds = [1, 2, 3, 4, 5]
    ramp = COLORS["ramp5"]
    fig, ax = plt.subplots(figsize=(9, 5.5))
    series = []
    for ri, r in enumerate(rounds):
        vals = np.sort(multi[multi["round"] == r]["total_ms"].values)
        n = len(vals)
        y = np.arange(1, n + 1) / n * 100.0
        mean = vals.mean()
        series.append({"x": vals, "y": y, "color": ramp[ri],
                       "style": line_style(ri),
                       "label": f"Round {r} (µ={mean:.3f}, n={n})"})
    cdf_lines(ax, series)
    ax.set_xlabel("Total Processing Time (ms)")
    ax.set_ylabel("Cumulative Percentage (%)")
    ax.set_title("Empirical CDF - Total Latency per Round")
    ax.legend(loc="lower right", frameon=True)
    ax.set_ylim(0, 102)
    finish(ax)
    fig.tight_layout()
    save(fig, os.path.join(HERE, "fig02_cdf_5rounds.png"))


# ----------------------------------------------------------------------
# fig03 — round bar + std error + P99 diamonds
# ----------------------------------------------------------------------
def fig03():
    rounds = [1, 2, 3, 4, 5]
    means, stds, p99s, ns = [], [], [], []
    for r in rounds:
        v = multi[multi["round"] == r]["total_ms"].values
        means.append(v.mean())
        stds.append(v.std())
        p99s.append(np.percentile(v, 99))
        ns.append(len(v))
    means = np.array(means)
    cross_sigma = np.std(means)

    fig, ax = plt.subplots(figsize=(9, 5.5))
    x = np.arange(len(rounds))
    bars = ax.bar(x, means, 0.55, yerr=stds, color=COLORS["blue"],
                  edgecolor="#444444", linewidth=0.6, capsize=4,
                  error_kw=dict(elinewidth=1.0, ecolor="#333333"),
                  label="Mean +/- Std")
    ax.scatter(x, p99s, marker="D", s=90, color=COLORS["red"], zorder=5,
               label="P99", edgecolor="#323232", linewidth=0.6)

    # Value labels: each number sits at the element it describes so the
    # printed value always matches its Y position. The MEAN is labelled on
    # the bar (just above the upper error-bar cap); the P99 is labelled in
    # red on its diamond. (Previously the mean was mistakenly printed above
    # the P99 marker, so the number contradicted the axis — thesis review.)
    for xi, (m, sd, p) in enumerate(zip(means, stds, p99s)):
        ax.text(xi, m + sd + 0.015 * max(p99s), f"{m:.3f}", ha="center",
                va="bottom", fontsize=11, fontweight="bold", color="#1a1a1a")
        ax.text(xi + 0.16, p, f"P99 {p:.3f}", ha="left", va="center",
                fontsize=10.5, color=COLORS["red"], fontweight="bold")

    ax.set_xticks(x)
    ax.set_xticklabels([f"Round {r}\n(n={n})" for r, n in zip(rounds, ns)])
    ax.set_ylabel("Processing Time (ms)")
    ax.set_title("5-Round Total Latency (Mean +/- Std, P99)")
    ax.legend(loc="upper right", frameon=True)

    # headroom + sigma annotation in empty upper-left, clear of bars/caps/P99
    ymax = max(p99s) * 1.25
    ax.set_ylim(0, ymax)
    ax.text(0.02, 0.97,
            f"Cross-round σ = {cross_sigma:.4f} ms",
            transform=ax.transAxes, ha="left", va="top", fontsize=10,
            color="#444444",
            bbox=dict(boxstyle="round,pad=0.35", facecolor="#f2f2f2",
                      edgecolor="#bbbbbb"))
    finish(ax)
    fig.tight_layout()
    save(fig, os.path.join(HERE, "fig03_round_bar_errorbar.png"))


# ----------------------------------------------------------------------
# fig04 — horizontal per-step breakdown (online combined)
# ----------------------------------------------------------------------
def fig04():
    s = step_series(online)
    means = s.mean()
    total = means.sum()
    # sort ascending so largest at bottom
    ordered = means.sort_values(ascending=True)
    labels = list(ordered.index)
    vals = ordered.values
    pct = vals / total * 100.0

    fig, ax = plt.subplots(figsize=(9, 5))
    y = np.arange(len(labels))
    ax.barh(y, vals, color=COLORS["blue"], edgecolor="#444444", linewidth=0.6)
    ax.set_yticks(y)
    ax.set_yticklabels(labels)
    ax.set_xlabel("Mean Processing Time (ms)")
    ax.set_title(f"Per-Step Time Breakdown (Online Combined, total = {total:.3f} ms)")
    xmax = vals.max() * 1.18
    ax.set_xlim(0, xmax)
    for yi, (v, p) in enumerate(zip(vals, pct)):
        ax.text(v + xmax * 0.012, yi, f"{v:.3f} ms · {p:.1f}%",
                va="center", ha="left", fontsize=9.5, color="#222222")
    finish(ax)
    fig.tight_layout()
    save(fig, os.path.join(HERE, "fig04_step_breakdown.png"))


# ----------------------------------------------------------------------
# fig05 — histogram of total_ms (online) + P99 line
# ----------------------------------------------------------------------
def fig05():
    v = online["total_ms"].values
    n = len(v)
    p99 = np.percentile(v, 99)
    lo = np.floor(v.min() * 10) / 10
    hi = np.ceil(v.max() * 10) / 10
    bins = np.arange(lo, hi + 0.1, 0.1)

    fig, ax = plt.subplots(figsize=(9, 5.5))
    counts, _, _ = ax.hist(v, bins=bins, color=COLORS["blue2"],
                           edgecolor="#444444", linewidth=0.5)
    ax.axvline(p99, color=COLORS["red"], linestyle="--", linewidth=1.8)
    ymax = counts.max() * 1.12
    ax.set_ylim(0, ymax)
    # place annotation to the right of the line, clear of it, near top
    ax.annotate(f"P99 = {p99:.3f} ms", xy=(p99, ymax * 0.92),
                xytext=(p99 + (hi - lo) * 0.06, ymax * 0.92),
                ha="left", va="center", fontsize=10, color=COLORS["red"],
                fontweight="bold")
    ax.set_xlabel("Total Processing Time (ms)")
    ax.set_ylabel("Count")
    ax.set_title(f"Latency Histogram (Online, n={n}, bin = 0.1 ms)")
    finish(ax)
    fig.tight_layout()
    save(fig, os.path.join(HERE, "fig05_histogram.png"))


# ----------------------------------------------------------------------
# fig06 — summary statistics table
# ----------------------------------------------------------------------
def _stats(v):
    return [len(v), v.mean(), np.median(v), v.std(),
            np.percentile(v, 95), np.percentile(v, 99), v.max()]


def fig06():
    cols = ["", "n", "Mean", "Median", "Std", "P95", "P99", "Max"]
    rows = []
    rowlabels = []

    off = df[df["mode"] == "offline"]["total_ms"].values
    rows.append(_stats(off)); rowlabels.append("Offline")
    for r in [1, 2, 3, 4, 5]:
        v = online[online["round"] == r]["total_ms"].values
        rows.append(_stats(v)); rowlabels.append(f"Round {r}")
    rows.append(_stats(online["total_ms"].values)); rowlabels.append("Online Combined")

    table_data = []
    for lab, st in zip(rowlabels, rows):
        table_data.append([lab, f"{int(st[0])}"] + [f"{x:.3f}" for x in st[1:]])

    fig, ax = plt.subplots(figsize=(10, 4.2))
    ax.axis("off")
    ax.set_title("Summary Statistics - All Rounds (ms, 3σ-cleaned)",
                 fontweight="bold", pad=16)

    tbl = ax.table(cellText=table_data, colLabels=cols,
                   cellLoc="center", loc="center")
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(10)
    tbl.scale(1, 1.6)

    ncols = len(cols)
    # widen the row-label column so "Online Combined" is not truncated
    nrows_tbl = len(rowlabels) + 1
    for rr in range(nrows_tbl):
        tbl[rr, 0].set_width(0.22)
        for cc in range(1, ncols):
            tbl[rr, cc].set_width(0.78 / (ncols - 1))
    # header style
    for c in range(ncols):
        cell = tbl[0, c]
        cell.set_facecolor(COLORS["header_blue"])
        cell.set_text_props(color="white", fontweight="bold")
        cell.set_edgecolor("white")
    # body styling
    for ri, lab in enumerate(rowlabels, start=1):
        for c in range(ncols):
            cell = tbl[ri, c]
            cell.set_edgecolor("#cccccc")
            if lab == "Offline":
                cell.set_facecolor("#ededed")
            elif lab == "Online Combined":
                cell.set_facecolor(COLORS["row_blue"])
            else:
                cell.set_facecolor("white")
            if c == 0:
                cell.set_text_props(ha="left")
                cell.PAD = 0.04

    save(fig, os.path.join(HERE, "fig06_summary_table.png"))


# ----------------------------------------------------------------------
# fig07 — strip / jitter scatter of 6 individual steps over 5 rounds
# ----------------------------------------------------------------------
def fig07():
    steps = [
        ("TPS Undistort", "undistort_ms"),
        ("Rotate+Flip", "rotate_flip_ms"),
        ("Kalman", "kalman_ms"),
        ("HSV Detect", "hsv_detect_ms"),
        ("EMA", "ema_ms"),
        ("Draw", "draw_ms"),
    ]
    rounds = [1, 2, 3, 4, 5]
    ramp = COLORS["ramp5"]

    fig, ax = plt.subplots(figsize=(11, 5))
    rng = np.random.default_rng(42)

    ymax = 0.0
    for si, (label, col) in enumerate(steps):
        for ri, r in enumerate(rounds):
            vals = multi[multi["round"] == r][col].values
            if len(vals) == 0:
                continue
            jitter = (rng.random(len(vals)) - 0.5) * 0.6
            xs = np.full(len(vals), si) + (ri - 2) * 0.06 + jitter * 0.0 + jitter
            ax.scatter(xs, vals, s=6, color=ramp[ri], alpha=0.18,
                       edgecolors="none",
                       label=f"Round {r}" if si == 0 else None)
            ymax = max(ymax, vals.max())
        # mean diamond for the step (across all multi rounds)
        m = multi[col].mean()
        ax.scatter([si], [m], marker="D", s=70, color="#111111",
                   edgecolor="white", linewidth=0.8, zorder=6,
                   label="Mean" if si == 0 else None)

    ax.set_xticks(np.arange(len(steps)))
    ax.set_xticklabels([s[0] for s in steps])
    ax.set_ylabel("Latency (ms)")
    ax.set_title("Per-Step Latency Distribution - Strip Plot (5 Rounds)")
    # headroom so legend (upper-right) does not cover dense points
    ax.set_ylim(-0.02, ymax * 1.45)

    handles, labels = ax.get_legend_handles_labels()
    leg = ax.legend(handles, labels, loc="upper right", frameon=True,
                    ncol=3, markerscale=1.6)
    for lh in leg.legend_handles:
        try:
            lh.set_alpha(1.0)
        except Exception:
            pass
    finish(ax)
    fig.tight_layout()
    save(fig, os.path.join(HERE, "fig07_strip_jitter.png"))


if __name__ == "__main__":
    # Build all figures, or only the ones named on the command line.
    #   python rebuild_figures.py            # all
    #   python rebuild_figures.py fig03      # just fig03
    #   python rebuild_figures.py fig00 fig06 # a subset
    #   python rebuild_figures.py --list     # list figures
    run_cli(globals())
