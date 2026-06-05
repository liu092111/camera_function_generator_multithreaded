"""Rebuild publication figures for the end-to-end latency benchmark.

Pure white background, no gridlines, no test tokens, no overlapping text.
All statistics are computed from e2e_latency_cleaned.csv with pandas/numpy.
Color convention: Gen-3 = blue, Gen-2 = tan.
"""

import os
import sys

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
from matplotlib.lines import Line2D

sys.path.insert(0, "/mnt/c/Users/liuuhua/Desktop/Git Repository/camera_function_generator_multithreaded/benchmarks")
from paper_style import apply_style, COLORS, save, finish, run_cli

apply_style()

HERE = os.path.dirname(os.path.abspath(__file__))
CSV = os.path.join(HERE, "e2e_latency_cleaned.csv")

df = pd.read_csv(CSV)
df = df.dropna(subset=["generation"])

g3 = df[df["generation"] == "gen3"]
g2 = df[df["generation"] == "gen2"]

C_G3 = COLORS["blue"]
C_G2 = COLORS["tan"]


def _stat(series):
    return dict(
        n=int(series.count()),
        mean=float(series.mean()),
        median=float(series.median()),
        p99=float(np.percentile(series.dropna(), 99)),
    )


# ----------------------------------------------------------------------
# fig00 — architecture / box diagram
# ----------------------------------------------------------------------
def fig00():
    fig, ax = plt.subplots(figsize=(13.5, 10.5))
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 100)
    ax.axis("off")
    finish(ax)

    fig.suptitle("End-to-End Latency Benchmark", fontsize=20, fontweight="bold",
                 y=0.985)
    ax.text(50, 95.2, "Complete Control Loop:  Capture → Process → FG Command",
            ha="center", va="center", fontsize=13, style="italic", color="#333333")

    def box(x, y, w, h, lines, fc, ec, fs=10.5, head=False):
        b = FancyBboxPatch((x, y), w, h,
                           boxstyle="round,pad=0.3,rounding_size=1.2",
                           linewidth=1.3, edgecolor=ec, facecolor=fc,
                           mutation_aspect=1.0)
        ax.add_patch(b)
        cy = y + h / 2.0
        if isinstance(lines, str):
            lines = [lines]
        # first line bold (title), rest regular
        n = len(lines)
        line_h = h / (n + 0.6)
        start = cy + (n - 1) * line_h / 2.0
        for i, ln in enumerate(lines):
            weight = "bold" if (i == 0 and head) else ("bold" if i == 0 else "normal")
            size = fs + 1 if (i == 0 and head) else fs
            ax.text(x + w / 2.0, start - i * line_h, ln, ha="center", va="center",
                    fontsize=size, fontweight=weight, color="#1a1a1a")

    def darrow(x, y0, y1):
        ax.add_patch(FancyArrowPatch((x, y0), (x, y1),
                     arrowstyle="-|>", mutation_scale=18,
                     linewidth=1.6, color=COLORS["grey"]))

    # ---- Left column: E2E Latency Definition ----
    lx, lw = 4, 40
    ax.text(lx + lw / 2, 89, "E2E Latency Definition", ha="center", va="center",
            fontsize=13.5, fontweight="bold", color=COLORS["blue"])
    seg_specs = [
        (["Seg 1: Capture", "cap.read() — acquire one frame"], COLORS["blue2"]),
        (["Seg 2: Process", "TPS Undistort + HSV Detect + Kalman + EMA"], COLORS["green"]),
        (["Seg 3: FG Write", "Send voltage command to Keysight 33622A"], COLORS["tan"]),
    ]
    bh = 8.5
    gap = 5.0
    top = 84
    for i, (lines, col) in enumerate(seg_specs):
        y = top - i * (bh + gap) - bh
        box(lx, y, lw, bh, lines, fc="white", ec=col, fs=10.5)
        if i < len(seg_specs) - 1:
            darrow(lx + lw / 2, y - 0.4, y - gap + 0.4)

    # ---- Right column: FG Command Strategy ----
    rx, rw = 52, 44
    ax.text(rx + rw / 2, 89, "FG Command Strategy", ha="center", va="center",
            fontsize=13.5, fontweight="bold", color=COLORS["tan"])

    box(rx, 70, rw, 12,
        ["Gen-3: Compound Single Write",
         "SOUR1:VOLT (X);SOUR2:VOLT (Y)",
         "1 SCPI transaction → ~6.5 ms typical"],
        fc="#eef3f9", ec=COLORS["blue"], fs=10.0)

    box(rx, 50, rw, 16,
        ["Gen-2: Multi-Write Sequence",
         "OUTP1 OFF → SOUR1:VOLT (X) → OUTP1 ON → *WAI",
         "OUTP2 OFF → SOUR2:VOLT (Y) → OUTP2 ON → *WAI",
         "4+ SCPI transactions → ~40 ms typical"],
        fc="#fdf3e9", ec=COLORS["tan"], fs=10.0)

    # ---- Bottom block: Test Configuration ----
    bx, bw, by, bhh = 4, 92, 4, 33
    b = FancyBboxPatch((bx, by), bw, bhh,
                       boxstyle="round,pad=0.4,rounding_size=1.5",
                       linewidth=1.3, edgecolor="#888888", facecolor="#f7f7f7")
    ax.add_patch(b)
    ax.text(bx + 2.5, by + bhh - 3.2, "Test Configuration", ha="left", va="center",
            fontsize=13, fontweight="bold", color="#1a1a1a")
    cfg = [
        "• Camera: Arducam B0332 (120fps, MJPG, DirectShow)",
        "• Function Generator: Keysight 33622A (USB-VISA)",
        "• Calibration: TPS Undistort enabled",
        "• Data: 5 rounds × 200 frames per generation = 2000 total frames",
        "• Measurement: per-frame timing of each segment (Capture / Process / FG Write)",
    ]
    for i, line in enumerate(cfg):
        ax.text(bx + 3.0, by + bhh - 8.5 - i * 4.7, line, ha="left", va="center",
                fontsize=11, color="#1a1a1a")

    save(fig, os.path.join(HERE, "fig00_test_architecture.png"))


# ----------------------------------------------------------------------
# fig01 — CDF of fg_write_ms
# ----------------------------------------------------------------------
def fig01():
    fig, ax = plt.subplots(figsize=(9, 6))
    finish(ax)

    s3 = _stat(g3["fg_write_ms"])
    s2 = _stat(g2["fg_write_ms"])

    for series, col, lab in [
        (g3["fg_write_ms"], C_G3, f"Gen-3 (n={s3['n']}, mean={s3['mean']:.2f}ms)"),
        (g2["fg_write_ms"], C_G2, f"Gen-2 (n={s2['n']}, mean={s2['mean']:.2f}ms)"),
    ]:
        x = np.sort(series.dropna().values)
        y = np.arange(1, len(x) + 1) / len(x)
        ax.plot(x, y, color=col, linewidth=2.2, label=lab)

    ax.set_xlabel("FG Write Latency (ms)")
    ax.set_ylabel("CDF")
    ax.set_title("FG Write Latency CDF — Gen-3 vs Gen-2 (All Frames)", pad=12)
    ax.set_ylim(0, 1.02)
    ax.set_xlim(left=0)
    ax.legend(loc="center right", frameon=True)

    # light annotation noting Gen-3 near-zero cluster, in empty space (upper-left)
    g3_frac = float((g3["fg_write_ms"] < 1.0).mean())
    ax.annotate(f"Gen-3 cluster: {g3_frac*100:.0f}% of frames < 1 ms",
                xy=(0.5, 0.5), xytext=(0.30, 0.86), textcoords="axes fraction",
                fontsize=9.5, color=C_G3,
                arrowprops=dict(arrowstyle="->", color=C_G3, lw=1.2,
                                connectionstyle="arc3,rad=0.2"))

    save(fig, os.path.join(HERE, "fig01_cdf_fg_write.png"))


# ----------------------------------------------------------------------
# fig02 — grouped boxplot of three segments
# ----------------------------------------------------------------------
def fig02():
    fig, ax = plt.subplots(figsize=(10, 6.5))
    finish(ax)

    cols = ["capture_ms", "process_ms", "fg_write_ms"]
    labels = ["Capture", "Process", "FG Write"]
    centers = np.arange(len(cols))
    off = 0.2
    bw = 0.34

    for gi, (grp, col) in enumerate([(g3, C_G3), (g2, C_G2)]):
        pos = centers + (off if gi == 1 else -off)
        data = [grp[c].dropna().values for c in cols]
        bp = ax.boxplot(data, positions=pos, widths=bw, patch_artist=True,
                        showfliers=False, showmeans=False,
                        medianprops=dict(color="#222222", linewidth=1.3),
                        whiskerprops=dict(color=col, linewidth=1.1),
                        capprops=dict(color=col, linewidth=1.1),
                        boxprops=dict(facecolor=col, edgecolor=col, alpha=0.65,
                                      linewidth=1.0))
        # mean diamonds + labels
        for j, c in enumerate(cols):
            m = float(grp[c].mean())
            ax.scatter(pos[j], m, marker="D", s=55, color="#222222",
                       zorder=6, edgecolor="white", linewidth=0.6)

    ax.set_xticks(centers)
    ax.set_xticklabels(labels)
    ax.set_ylabel("Latency (ms)")
    ax.set_title("E2E Latency by Segment — Gen-3 vs Gen-2", pad=12)

    # expand y-limit to give headroom for mean labels
    ymax = max(g2["fg_write_ms"].quantile(0.98), g3["fg_write_ms"].quantile(0.98),
               g2["capture_ms"].quantile(0.98))
    ax.set_ylim(-3, ymax * 1.18)

    # mean value labels with vertical offsets sized to avoid box collision
    for gi, (grp, col) in enumerate([(g3, C_G3), (g2, C_G2)]):
        pos = centers + (off if gi == 1 else -off)
        for j, c in enumerate(cols):
            m = float(grp[c].mean())
            q3 = float(grp[c].quantile(0.75))
            # place label above the upper-quartile region; if small, clamp to a
            # readable band below the diamond for the dominant FG-Write box
            if c == "fg_write_ms" and gi == 1:
                ytxt = m + ymax * 0.05
            else:
                ytxt = max(m, q3) + ymax * 0.05
            ax.annotate(f"{m:.2f}", xy=(pos[j], m), xytext=(pos[j], ytxt),
                        ha="center", va="bottom", fontsize=8.5,
                        fontweight="bold", color=col)

    legend_handles = [
        plt.Rectangle((0, 0), 1, 1, fc=C_G3, alpha=0.65, ec=C_G3, label="Gen-3"),
        plt.Rectangle((0, 0), 1, 1, fc=C_G2, alpha=0.65, ec=C_G2, label="Gen-2"),
        Line2D([0], [0], marker="D", color="w", markerfacecolor="#222222",
               markeredgecolor="white", markersize=8, label="Mean"),
    ]
    ax.legend(handles=legend_handles, loc="upper left", frameon=True)

    save(fig, os.path.join(HERE, "fig02_boxplot_segments.png"))


# ----------------------------------------------------------------------
# fig03 — per-round grouped bar of mean fg_write_ms
# ----------------------------------------------------------------------
def fig03():
    fig, ax = plt.subplots(figsize=(10, 6.5))
    finish(ax)

    rounds = sorted(df["round"].dropna().unique().astype(int))
    g3_means = [g3[g3["round"] == r]["fg_write_ms"].mean() for r in rounds]
    g2_means = [g2[g2["round"] == r]["fg_write_ms"].mean() for r in rounds]

    x = np.arange(len(rounds))
    bw = 0.36
    b3 = ax.bar(x - bw / 2, g3_means, bw, color=C_G3, label="Gen-3")
    b2 = ax.bar(x + bw / 2, g2_means, bw, color=C_G2, label="Gen-2")

    ax.set_xticks(x)
    ax.set_xticklabels([f"Round {r}" for r in rounds])
    ax.set_ylabel("FG Write Mean Latency (ms)")
    ax.set_title("FG Write Latency Per Round — Gen-3 vs Gen-2", pad=26)

    ymax = max(g2_means)
    ax.set_ylim(0, ymax * 1.30)  # ~30% headroom for speedup labels + title pad

    # value labels on bars
    for rects, col in [(b3, C_G3), (b2, C_G2)]:
        for r in rects:
            h = r.get_height()
            ax.text(r.get_x() + r.get_width() / 2, h + ymax * 0.015,
                    f"{h:.2f}", ha="center", va="bottom", fontsize=8,
                    color=col)

    # speedup labels above each group, with clear headroom below title
    for i, r in enumerate(rounds):
        sp = g2_means[i] / g3_means[i] if g3_means[i] else float("nan")
        ytxt = ymax * 1.18
        ax.text(x[i], ytxt, f"{sp:.1f}×", ha="center", va="center",
                fontsize=11, fontweight="bold", color="#1a1a1a",
                bbox=dict(boxstyle="round,pad=0.3", fc="#eef3f9",
                          ec=COLORS["blue"], lw=0.8))

    # legend in the empty mid-left band (above the short Gen-3 bars, below the
    # speedup annotations) so it never collides with the title or the labels
    ax.legend(loc="center left", frameon=True, bbox_to_anchor=(0.005, 0.62))

    save(fig, os.path.join(HERE, "fig03_per_round_fg.png"))


# ----------------------------------------------------------------------
# fig04 — stacked bar of mean segment composition
# ----------------------------------------------------------------------
def fig04():
    fig, ax = plt.subplots(figsize=(8.5, 7))
    finish(ax)

    segs = ["capture_ms", "process_ms", "fg_write_ms"]
    seg_labels = ["Capture", "Process", "FG Write"]
    seg_colors = [COLORS["blue2"], COLORS["green"], COLORS["tan"]]

    groups = [("Gen-3", g3), ("Gen-2", g2)]
    x = np.arange(len(groups))
    bw = 0.5

    means = {name: [grp[s].mean() for s in segs] for name, grp in groups}
    totals = {name: sum(means[name]) for name, _ in groups}
    ymax = max(totals.values())

    thin_threshold = ymax * 0.06
    for gi, (name, _) in enumerate(groups):
        bottom = 0.0
        vals = means[name]
        thin = []  # (si, mid, value) for segments needing an outside leader
        for si, s in enumerate(segs):
            v = vals[si]
            ax.bar(x[gi], v, bw, bottom=bottom, color=seg_colors[si],
                   edgecolor="white", linewidth=0.8,
                   label=seg_labels[si] if gi == 0 else None)
            mid = bottom + v / 2.0
            if v >= thin_threshold:
                txtcol = "white" if si != 0 else "#1a1a1a"
                ax.text(x[gi], mid, f"{v:.2f}", ha="center", va="center",
                        fontsize=9, fontweight="bold", color=txtcol)
            else:
                thin.append((si, mid, v))
            bottom += v

        # lay out thin-segment leaders with guaranteed vertical separation so
        # stacked thin segments (e.g. Gen-2 capture + process) never collide
        if thin:
            step = ymax * 0.07
            base = max(m for _, m, _ in thin) + step
            base = max(base, ymax * 0.10)
            for k, (si, mid, v) in enumerate(sorted(thin, key=lambda t: t[1])):
                ytxt = base + k * step
                xt = x[gi] + bw / 2 + 0.18
                ax.annotate(f"{seg_labels[si]}: {v:.2f}",
                            xy=(x[gi] + bw / 2, mid), xytext=(xt, ytxt),
                            ha="left", va="center", fontsize=8.5,
                            color=seg_colors[si],
                            arrowprops=dict(arrowstyle="-", color=seg_colors[si],
                                            lw=0.9,
                                            connectionstyle="arc3,rad=0.0"))

        ax.text(x[gi], totals[name] + ymax * 0.025, f"Total: {totals[name]:.2f}ms",
                ha="center", va="bottom", fontsize=11, fontweight="bold",
                color="#1a1a1a")

    ax.set_xticks(x)
    ax.set_xticklabels([n for n, _ in groups])
    ax.set_ylabel("Latency (ms)")
    ax.set_title("E2E Total Latency Composition — Gen-3 vs Gen-2", pad=12)
    ax.set_ylim(0, ymax * 1.16)
    ax.set_xlim(-0.7, 1.9)
    ax.legend(loc="upper left", frameon=True)

    save(fig, os.path.join(HERE, "fig04_stacked_bar.png"))


# ----------------------------------------------------------------------
# fig05 — stacked histograms of fg_write_ms
# ----------------------------------------------------------------------
def fig05():
    fig, axes = plt.subplots(2, 1, figsize=(9, 8), constrained_layout=True)

    specs = [
        (axes[0], g3["fg_write_ms"], C_G3,
         "Gen-3: FG Write Distribution (compound write)"),
        (axes[1], g2["fg_write_ms"], C_G2,
         "Gen-2: FG Write Distribution (multi-write sequence)"),
    ]
    for ax, series, col, title in specs:
        finish(ax)
        s = series.dropna()
        ax.hist(s, bins=40, color=col, alpha=0.78, edgecolor="white", linewidth=0.4)
        med = float(s.median())
        ax.axvline(med, color="#c0392b", linestyle="--", linewidth=1.6)
        ymax = ax.get_ylim()[1]
        ax.text(med, ymax * 0.96, f"median={med:.2f}ms", color="#c0392b",
                fontsize=9, ha="left" if med < s.max() * 0.6 else "right",
                va="top", rotation=0,
                bbox=dict(boxstyle="round,pad=0.2", fc="white",
                          ec="#c0392b", lw=0.8))
        ax.set_title(title, pad=8)
        ax.set_ylabel("Count")
        st = _stat(s)
        ax.text(0.98, 0.78, f"n={st['n']}\nmean={st['mean']:.2f}ms",
                transform=ax.transAxes, ha="right", va="top", fontsize=9.5,
                bbox=dict(boxstyle="round,pad=0.4", fc="#f5f5f5",
                          ec="#888888", lw=0.9))

    axes[1].set_xlabel("FG Write Latency (ms)")
    save(fig, os.path.join(HERE, "fig05_histogram_fg.png"))


# ----------------------------------------------------------------------
# fig06 — summary table
# ----------------------------------------------------------------------
def fig06():
    s3w = _stat(g3["fg_write_ms"])
    s2w = _stat(g2["fg_write_ms"])
    s3p = _stat(g3["process_ms"])
    s2p = _stat(g2["process_ms"])
    t3 = float(g3["total_ms"].mean())
    t2 = float(g2["total_ms"].mean())

    def sp(g2v, g3v):
        return f"{g2v / g3v:.2f}×" if g3v else "—"

    rows = [
        ["N (frames)", f"{s3w['n']}", f"{s2w['n']}", "—"],
        ["FG Write Mean (ms)", f"{s3w['mean']:.2f}", f"{s2w['mean']:.2f}",
         sp(s2w['mean'], s3w['mean'])],
        ["FG Write Median (ms)", f"{s3w['median']:.2f}", f"{s2w['median']:.2f}",
         sp(s2w['median'], s3w['median'])],
        ["FG Write P99 (ms)", f"{s3w['p99']:.2f}", f"{s2w['p99']:.2f}",
         sp(s2w['p99'], s3w['p99'])],
        ["Process Mean (ms)", f"{s3p['mean']:.2f}", f"{s2p['mean']:.2f}", "~1.0×"],
        ["Process P99 (ms)", f"{s3p['p99']:.2f}", f"{s2p['p99']:.2f}", "~1.0×"],
        ["Total Mean (ms)", f"{t3:.2f}", f"{t2:.2f}", sp(t2, t3)],
    ]
    headers = ["Metric", "Gen-3", "Gen-2", "Speedup"]

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.axis("off")
    finish(ax)
    ax.set_title("End-to-End Latency Summary — Gen-3 vs Gen-2 (3σ Cleaned)",
                 pad=18, fontsize=14)

    tbl = ax.table(cellText=rows, colLabels=headers,
                   cellLoc="center", loc="center",
                   colWidths=[0.34, 0.22, 0.22, 0.22])
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(11)
    tbl.scale(1, 2.0)

    header_fill = COLORS["header_blue"]
    zebra = COLORS["row_blue"]
    ncols = len(headers)
    for (r, c), cell in tbl.get_celld().items():
        cell.set_edgecolor("#bbbbbb")
        cell.set_linewidth(0.8)
        if r == 0:
            cell.set_facecolor(header_fill)
            cell.set_text_props(color="white", fontweight="bold")
            cell.set_height(cell.get_height() * 1.05)
        else:
            cell.set_facecolor(zebra if r % 2 == 1 else "white")
            if c == 0:
                cell.set_text_props(ha="left", fontweight="bold")
                cell.PAD = 0.04
            if c == ncols - 1:
                cell.set_text_props(color=COLORS["blue"], fontweight="bold")

    save(fig, os.path.join(HERE, "fig06_summary_table.png"))


if __name__ == "__main__":
    # Build all figures, or only the ones named on the command line.
    #   python rebuild_figures.py            # all
    #   python rebuild_figures.py fig03      # just fig03
    #   python rebuild_figures.py fig00 fig06 # a subset
    #   python rebuild_figures.py --list     # list figures
    run_cli(globals())
