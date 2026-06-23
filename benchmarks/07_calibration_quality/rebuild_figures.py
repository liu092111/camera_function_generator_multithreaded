"""Rebuild top-level academic figures for the TPS Calibration Quality benchmark.

Clean publication PNGs: pure white background, no gridlines, no test tokens,
no overlapping text. All statistics are computed directly from
calibration_raw.csv (1504 points).

Only the top-level figXX_*.png are rebuilt; calibration_viz/, figures/, and
_original_figures/ are left untouched.
"""

import sys
import pathlib

sys.path.insert(0, "/mnt/c/Users/liuuhua/Desktop/Git Repository/camera_function_generator_multithreaded/benchmarks")
from paper_style import apply_style, COLORS, save, finish, run_cli, line_style, grey_ramp, BAR_EDGE, BAR_EDGE_LW  # noqa: E402

apply_style()

import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from scipy.interpolate import griddata  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
CSV = HERE / "calibration_raw.csv"

ZONE_NAMES = ["Center", "Mid-inner", "Mid-outer", "Edge"]


# ----------------------------------------------------------------------
# Load + derive statistics
# ----------------------------------------------------------------------
df = pd.read_csv(CSV)
res = df["residual_px"].to_numpy()
aff = df["affine_residual_px"].to_numpy()
n = len(df)

tps_mean = res.mean()
tps_med = np.median(res)
tps_std = res.std(ddof=1)
tps_p95 = np.percentile(res, 95)
tps_p99 = np.percentile(res, 99)
tps_max = res.max()

aff_mean = aff.mean()
aff_med = np.median(aff)
aff_std = aff.std(ddof=1)
aff_p95 = np.percentile(aff, 95)
aff_p99 = np.percentile(aff, 99)
aff_max = aff.max()

pct_lt_01 = 100.0 * np.mean(res < 0.1)
pct_lt_03 = 100.0 * np.mean(res < 0.3)
pct_lt_05 = 100.0 * np.mean(res < 0.5)

tps_lt_05 = 100.0 * np.mean(res < 0.5)
aff_lt_05 = 100.0 * np.mean(aff < 0.5)

improvement = aff_mean / tps_mean

# Radial zones (normalized distance from grid center)
gx = df["grid_x"].to_numpy()
gy = df["grid_y"].to_numpy()
cx = 0.5 * (gx.min() + gx.max())
cy = 0.5 * (gy.min() + gy.max())
halfw = 0.5 * (gx.max() - gx.min())
halfh = 0.5 * (gy.max() - gy.min())
rad = np.sqrt(((gx - cx) / halfw) ** 2 + ((gy - cy) / halfh) ** 2)

q = np.quantile(rad, [0.25, 0.50, 0.75])
zone_idx = np.digitize(rad, q)  # 0..3
df["zone"] = [ZONE_NAMES[i] for i in zone_idx]

zone_stats = {}
for i, name in enumerate(ZONE_NAMES):
    s = res[zone_idx == i]
    zone_stats[name] = {
        "mean": s.mean(),
        "std": s.std(ddof=1),
        "p99": np.percentile(s, 99),
        "n": len(s),
    }

print("Derived stats:")
print("  TPS  mean=%.4f p95=%.4f p99=%.4f max=%.4f" % (tps_mean, tps_p95, tps_p99, tps_max))
print("  Aff  mean=%.4f improvement=%.2fx" % (aff_mean, improvement))
for nm in ZONE_NAMES:
    z = zone_stats[nm]
    print("  %-10s mean=%.4f p99=%.4f n=%d" % (nm, z["mean"], z["p99"], z["n"]))


# ----------------------------------------------------------------------
# fig00 — architecture / overview diagram
# ----------------------------------------------------------------------
def fig00():
    fig, ax = plt.subplots(figsize=(11.0, 8.6))
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 100)
    ax.axis("off")

    # Title block
    ax.text(50, 97, "TPS Calibration Quality Benchmark",
            ha="center", va="top", fontsize=20, fontweight="bold",
            color=COLORS["blue"])
    ax.text(50, 92.5,
            "Thin Plate Spline Rectification — Sub-Pixel Accuracy Validation",
            ha="center", va="top", fontsize=12.5, style="italic",
            color="#444444")

    # ---- Pipeline section ----
    ax.text(6, 86, "TPS Calibration Pipeline", ha="left", va="top",
            fontsize=14, fontweight="bold", color="#222222")

    boxes = [
        ("Raw Image", "640×480 px"),
        ("TPS Grid Mapping", "5 mm grid, 308 control pts"),
        ("Rectified Output", "462×314 px (cropped)"),
    ]
    box_w, box_h = 24, 11
    centers_x = [17, 50, 83]
    box_cy = 75
    for (title, sub), bx in zip(boxes, centers_x):
        fb = FancyBboxPatch((bx - box_w / 2, box_cy - box_h / 2), box_w, box_h,
                            boxstyle="round,pad=0.4,rounding_size=1.2",
                            linewidth=1.4, edgecolor=COLORS["blue"],
                            facecolor=COLORS["row_blue"])
        ax.add_patch(fb)
        ax.text(bx, box_cy + 2.0, title, ha="center", va="center",
                fontsize=12, fontweight="bold", color="#1a1a1a")
        ax.text(bx, box_cy - 2.6, sub, ha="center", va="center",
                fontsize=9.5, color="#444444")

    for i in range(2):
        x0 = centers_x[i] + box_w / 2
        x1 = centers_x[i + 1] - box_w / 2
        ar = FancyArrowPatch((x0 + 0.5, box_cy), (x1 - 0.5, box_cy),
                             arrowstyle="-|>", mutation_scale=18,
                             linewidth=1.8, color=COLORS["tan"])
        ax.add_patch(ar)

    # ---- Quality measurement section ----
    ax.text(6, 63, "Quality Measurement: Local Linearity", ha="left", va="top",
            fontsize=14, fontweight="bold", color="#222222")
    qm = [
        "Sample 1504 points in valid region (47 cols × 32 rows, 10 px spacing)",
        "For each point: compute neighbor consistency of TPS mapping",
        "Residual = deviation from perfect local linearity (px)",
        "Pass criterion: P99 < 2.0 px (sub-pixel accuracy)",
    ]
    y = 58.5
    for line in qm:
        ax.text(8, y, "•  " + line, ha="left", va="top", fontsize=11.5,
                color="#1a1a1a")
        y -= 4.6

    # ---- Key results section ----
    ax.text(6, 36, "Key Results", ha="left", va="top",
            fontsize=14, fontweight="bold", color="#222222")
    kr = [
        "Mean residual: %.3f px  |  P99: %.3f px  |  Max: %.3f px"
        % (tps_mean, tps_p99, tps_max),
        "%.1f%% of points have residual < 0.5 px (sub-pixel accuracy)" % pct_lt_05,
        "Spatial pattern: center (%.3f px) → edge (%.3f px) "
        "— classic lens distortion residual"
        % (zone_stats["Center"]["mean"], zone_stats["Edge"]["mean"]),
        "TPS vs Affine: %.1f× improvement (%.3f vs %.3f px)"
        % (improvement, tps_mean, aff_mean),
    ]
    y = 31.5
    for line in kr:
        ax.text(8, y, "•  " + line, ha="left", va="top", fontsize=11.5,
                color="#1a1a1a")
        y -= 4.8

    save(fig, str(HERE / "fig00_test_architecture.png"))


# ----------------------------------------------------------------------
# fig01 — sorted residual vs percentile
# ----------------------------------------------------------------------
def fig01():
    fig, ax = plt.subplots(figsize=(9.0, 5.6))
    finish(ax)

    sorted_r = np.sort(res)
    pct = np.linspace(0, 100, n)
    ax.plot(pct, sorted_r, color=COLORS["blue"], linewidth=2.0,
            label="Sorted residual")

    ax.set_title("TPS Calibration Residual — Sorted (1504 Points)")
    ax.set_xlabel("Percentile (%)")
    ax.set_ylabel("Residual (px)")
    ax.set_xlim(0, 100)
    ax.set_ylim(0, tps_max * 1.12)

    # Dashed horizontal lines for 0.1 / 0.3 / 0.5 px thresholds.
    thresholds = [
        (0.5, pct_lt_05, COLORS["red"]),
        (0.3, pct_lt_03, COLORS["tan"]),
        (0.1, pct_lt_01, COLORS["green"]),
    ]
    for val, p, col in thresholds:
        ax.axhline(val, ls="--", lw=1.3, color=col, alpha=0.9)
        ax.text(2.0, val + 0.008, "%.1f px (%.1f%% below)" % (val, p),
                fontsize=9.5, color=col, va="bottom", ha="left",
                fontweight="bold")

    # P99 diamond marker.
    ax.plot(99, tps_p99, marker="D", markersize=10, color=COLORS["red"],
            markeredgecolor="white", markeredgewidth=1.0, zorder=5)
    ax.annotate("P99 = %.3f px" % tps_p99,
                xy=(99, tps_p99), xytext=(78, tps_p99 - 0.12),
                fontsize=10, color=COLORS["red"], fontweight="bold",
                ha="center",
                arrowprops=dict(arrowstyle="->", color=COLORS["red"], lw=1.2))

    save(fig, str(HERE / "fig01_sorted_residual.png"))


# ----------------------------------------------------------------------
# fig02 — spatial heatmap
# ----------------------------------------------------------------------
def fig02():
    fig, ax = plt.subplots(figsize=(8.6, 6.2))
    finish(ax)

    # Interpolate onto a regular grid covering the data extent.
    xi = np.linspace(gx.min(), gx.max(), 240)
    yi = np.linspace(gy.min(), gy.max(), 200)
    XI, YI = np.meshgrid(xi, yi)
    ZI = griddata((gx, gy), res, (XI, YI), method="cubic")
    # Fill any NaN holes with nearest so the data region is fully colored.
    ZN = griddata((gx, gy), res, (XI, YI), method="nearest")
    ZI = np.where(np.isnan(ZI), ZN, ZI)

    # Greyscale heatmap: light = low residual, dark = high residual.
    im = ax.imshow(ZI, origin="upper", cmap="Greys",
                   extent=[gx.min(), gx.max(), gy.max(), gy.min()],
                   aspect="auto", vmin=0, vmax=tps_p99)

    ax.set_title("TPS Residual Spatial Distribution (px)")
    ax.set_xlabel("Grid X (px)")
    ax.set_ylabel("Grid Y (px)")

    cb = fig.colorbar(im, ax=ax, fraction=0.046, pad=0.03)
    cb.set_label("Residual (px)")
    cb.outline.set_edgecolor("#333333")
    cb.outline.set_linewidth(0.8)

    save(fig, str(HERE / "fig02_spatial_heatmap.png"))


# ----------------------------------------------------------------------
# fig03 — zone bar chart
# ----------------------------------------------------------------------
def fig03():
    fig, ax = plt.subplots(figsize=(8.6, 5.8))
    finish(ax)

    means = [zone_stats[z]["mean"] for z in ZONE_NAMES]
    sds = [zone_stats[z]["std"] for z in ZONE_NAMES]
    p99s = [zone_stats[z]["p99"] for z in ZONE_NAMES]
    ns = [zone_stats[z]["n"] for z in ZONE_NAMES]
    x = np.arange(len(ZONE_NAMES))

    ax.bar(x, means, width=0.62, yerr=sds, capsize=5, color=COLORS["blue"],
           edgecolor="#484848", linewidth=0.8, zorder=2,
           error_kw=dict(ecolor="#1a1a1a", elinewidth=1.2, capthick=1.2))

    # P99 diamonds.
    ax.plot(x, p99s, "D", markersize=10, color=COLORS["red"],
            markeredgecolor="white", markeredgewidth=1.0, zorder=4,
            linestyle="none")

    ymax = max(p99s) * 1.18
    ax.set_ylim(0, ymax)

    for xi, m, sd, p, nn in zip(x, means, sds, p99s, ns):
        # bold mean label above the upper error-bar cap
        ax.text(xi, m + sd + ymax * 0.012, "%.3f" % m, ha="center", va="bottom",
                fontsize=10.5, fontweight="bold", color="#1a1a1a")
        # red P99 label above diamond
        ax.text(xi, p + ymax * 0.018, "P99 %.3f" % p, ha="center", va="bottom",
                fontsize=9, color=COLORS["red"], fontweight="bold")
        # grey n-count inside bar
        ax.text(xi, m * 0.5, "n=%d" % nn, ha="center", va="center",
                fontsize=9, color="white")

    ax.set_xticks(x)
    ax.set_xticklabels(ZONE_NAMES)
    ax.set_title("Calibration Residual by Zone (Center → Edge)")
    ax.set_ylabel("Residual (px)")

    legend_el = [Line2D([0], [0], marker="D", color="none",
                        markerfacecolor=COLORS["red"],
                        markeredgecolor="white", markersize=10, label="P99")]
    ax.legend(handles=legend_el, loc="upper left", frameon=True)

    save(fig, str(HERE / "fig03_zone_bar.png"))


# ----------------------------------------------------------------------
# fig04 — CDF overlay TPS vs Affine
# ----------------------------------------------------------------------
def fig04():
    fig, ax = plt.subplots(figsize=(9.0, 5.6))
    finish(ax)

    def cdf(vals):
        s = np.sort(vals)
        c = np.arange(1, len(s) + 1) / len(s)
        return s, c

    s_t, c_t = cdf(res)
    s_a, c_a = cdf(aff)

    ax.plot(s_t, c_t, color=COLORS["blue"], lw=2.2, linestyle=line_style(0),
            label="TPS (mean=%.3f px)" % tps_mean)
    ax.plot(s_a, c_a, color=COLORS["tan"], lw=2.2, linestyle=line_style(1),
            label="Affine (mean=%.3f px)" % aff_mean)

    ax.set_title("Calibration Accuracy: TPS vs Affine Transform")
    ax.set_xlabel("Residual (px)")
    ax.set_ylabel("CDF")
    ax.set_xlim(0, np.percentile(aff, 99.5))
    ax.set_ylim(0, 1.02)
    ax.legend(loc="center right", frameon=True)

    # Boxed annotation in lower-right empty space.
    ax.text(0.97, 0.18,
            "TPS improvement: %.1f×" % improvement,
            transform=ax.transAxes, ha="right", va="center",
            fontsize=11, fontweight="bold", color=COLORS["blue"],
            bbox=dict(boxstyle="round,pad=0.5", facecolor="white",
                      edgecolor=COLORS["blue"], linewidth=1.3))

    save(fig, str(HERE / "fig04_tps_vs_affine.png"))


# ----------------------------------------------------------------------
# fig05 — histogram + cumulative dual axis
# ----------------------------------------------------------------------
def fig05():
    fig, ax = plt.subplots(figsize=(9.2, 5.7))
    finish(ax)

    bins = np.linspace(0, tps_max, 40)
    counts, edges = np.histogram(res, bins=bins)
    centers = 0.5 * (edges[:-1] + edges[1:])
    width = edges[1] - edges[0]
    ax.bar(centers, counts, width=width * 0.95, color=COLORS["blue2"],
           edgecolor=BAR_EDGE, linewidth=0.5, zorder=2)
    ax.set_xlabel("Residual (px)")
    ax.set_ylabel("Count")
    ax.set_xlim(0, tps_max * 1.02)
    ax.set_ylim(0, counts.max() * 1.18)

    # Cumulative on right axis.
    ax2 = ax.twinx()
    ax2.set_facecolor("white")
    ax2.grid(False)
    ax2.spines["top"].set_visible(False)
    sorted_r = np.sort(res)
    cum = 100.0 * np.arange(1, n + 1) / n
    line_cum, = ax2.plot(sorted_r, cum, color=COLORS["red"], lw=2.0,
                         label="Cumulative %", zorder=3)
    ax2.set_ylabel("Cumulative (%)")
    ax2.set_ylim(0, 105)

    # Vertical dashed lines at P95 / P99, labels placed in the empty
    # mid-height band to the side of each line (clear of bars and curve).
    ax.axvline(tps_p95, ls="--", lw=1.3, color=COLORS["tan"], zorder=4)
    ax.axvline(tps_p99, ls="--", lw=1.3, color=COLORS["red"], zorder=4)
    ax.text(tps_p95 - tps_max * 0.015, counts.max() * 0.62,
            "P95=%.3f" % tps_p95,
            color=COLORS["tan"], fontsize=9.5, fontweight="bold",
            ha="right", va="center")
    ax.text(tps_p99 + tps_max * 0.015, counts.max() * 0.46,
            "P99=%.3f" % tps_p99,
            color=COLORS["red"], fontsize=9.5, fontweight="bold",
            ha="left", va="center")

    ax.set_title("TPS Residual Distribution (Histogram + Cumulative)")
    ax2.legend(handles=[line_cum], loc="center right", frameon=True)

    save(fig, str(HERE / "fig05_histogram_cumulative.png"))


# ----------------------------------------------------------------------
# fig06 — summary table
# ----------------------------------------------------------------------
def fig06():
    fig = plt.figure(figsize=(10.5, 8.4))
    fig.patch.set_facecolor("white")

    fig.suptitle("TPS Calibration Quality Summary", fontsize=16,
                 fontweight="bold", color=COLORS["blue"], y=0.97)

    # ---- main metrics table ----
    ax1 = fig.add_axes([0.05, 0.50, 0.90, 0.40])
    ax1.axis("off")

    main_cols = ["Metric", "TPS Rectification", "Affine (baseline)", "Improvement"]
    rows = [
        ["N (points)", "%d" % n, "%d" % n, "—"],
        ["Mean (px)", "%.4f" % tps_mean, "%.4f" % aff_mean,
         "%.1f×" % (aff_mean / tps_mean)],
        ["Median (px)", "%.4f" % tps_med, "%.4f" % aff_med,
         "%.1f×" % (aff_med / tps_med)],
        ["Std (px)", "%.4f" % tps_std, "%.4f" % aff_std,
         "%.1f×" % (aff_std / tps_std)],
        ["P95 (px)", "%.4f" % tps_p95, "%.4f" % aff_p95,
         "%.1f×" % (aff_p95 / tps_p95)],
        ["P99 (px)", "%.4f" % tps_p99, "%.4f" % aff_p99,
         "%.1f×" % (aff_p99 / tps_p99)],
        ["Max (px)", "%.4f" % tps_max, "%.4f" % aff_max,
         "%.1f×" % (aff_max / tps_max)],
        ["< 0.5 px", "%.1f%%" % tps_lt_05, "%.1f%%" % aff_lt_05, "—"],
    ]

    t1 = ax1.table(cellText=rows, colLabels=main_cols, loc="center",
                   cellLoc="center")
    _style_table(t1, main_cols, len(rows))

    # ---- zone analysis sub-table ----
    ax2 = fig.add_axes([0.18, 0.06, 0.64, 0.34])
    ax2.axis("off")
    ax2.text(0.5, 1.04, "Zone Analysis", ha="center", va="bottom",
             transform=ax2.transAxes, fontsize=13, fontweight="bold",
             color="#222222")

    zcols = ["Zone", "Mean (px)", "P99 (px)", "N"]
    zrows = []
    for z in ZONE_NAMES:
        s = zone_stats[z]
        zrows.append([z, "%.4f" % s["mean"], "%.4f" % s["p99"], "%d" % s["n"]])
    t2 = ax2.table(cellText=zrows, colLabels=zcols, loc="center",
                   cellLoc="center")
    _style_table(t2, zcols, len(zrows))

    save(fig, str(HERE / "fig06_summary_table.png"))


def _style_table(tbl, cols, nrows):
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(10.5)
    tbl.scale(1.0, 1.7)
    ncol = len(cols)
    for (r, c), cell in tbl.get_celld().items():
        cell.set_edgecolor("#cccccc")
        cell.set_linewidth(0.6)
        if r == 0:
            cell.set_facecolor(COLORS["header_blue"])
            cell.set_text_props(color="white", fontweight="bold")
        else:
            cell.set_facecolor(COLORS["row_blue"] if (r % 2 == 0) else "white")
            if c == 0:
                cell.set_text_props(fontweight="bold", color="#1a1a1a")


if __name__ == "__main__":
    # Build all figures, or only the ones named on the command line.
    #   python rebuild_figures.py            # all
    #   python rebuild_figures.py fig03      # just fig03
    #   python rebuild_figures.py fig00 fig06 # a subset
    #   python rebuild_figures.py --list     # list figures
    run_cli(globals())
