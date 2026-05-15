"""07 — Calibration Quality Benchmark Charts

Reads: calibration_raw.csv
Outputs: calibration_sorted_residual.png, calibration_heatmap.png,
         calibration_by_row.png, calibration_vectors.png
"""

import os
import glob
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

plt.rcParams.update({
    'font.family': 'serif',
    'font.size': 10,
    'axes.titlesize': 11,
    'axes.labelsize': 10,
    'xtick.labelsize': 9,
    'ytick.labelsize': 9,
    'figure.dpi': 200,
    'savefig.dpi': 200,
    'figure.facecolor': 'white',
    'axes.facecolor': '#fafafa',
    'axes.spines.top': False,
    'axes.spines.right': False,
    'axes.grid': False,
})

C1 = '#6b8cae'
C2 = '#a3c4dc'
C3 = '#d4a574'
C4 = '#8fae8f'
C5 = '#b8a9c9'
C_GRAY = '#9e9e9e'

BASE_DIR = Path(__file__).parent
OUT_DIR = BASE_DIR / "figures"
OUT_DIR.mkdir(exist_ok=True)


def clean_output_dir():
    """Remove all old figures from the output directory."""
    for f in glob.glob(str(OUT_DIR / "*.png")):
        os.remove(f)


def savefig(fig, name):
    """Save figure and close."""
    fig.savefig(OUT_DIR / name, bbox_inches='tight')
    plt.close(fig)


def main():
    clean_output_dir()

    csv_path = BASE_DIR / "calibration_raw.csv"
    if not os.path.exists(csv_path):
        print(f"[WARNING] {csv_path} not found. Skipping all charts.")
        return

    df = pd.read_csv(csv_path)

    # Chart 1: Sorted residual plot (deterministic data, no outlier spikes)
    fig, ax = plt.subplots(figsize=(8, 4.5))
    sorted_res = np.sort(df['residual_px'].values)
    x = np.arange(1, len(sorted_res) + 1)
    ax.plot(x, sorted_res, '-', color=C1, linewidth=0.9)
    mean_val = sorted_res.mean()
    ax.axvline(x=len(sorted_res), color='none')  # placeholder for spacing
    # Add mean as horizontal dashed line (appropriate for calibration)
    ax.axhline(y=mean_val, color=C3, linewidth=0.8, linestyle='--',
               alpha=0.7, label=f'Mean ({mean_val:.4f} px)')
    p99 = np.percentile(sorted_res, 99)
    ax.axhline(y=p99, color=C_GRAY, linewidth=0.8, linestyle='--',
               alpha=0.5, label=f'P99 ({p99:.4f} px)')
    ax.set_xlabel('Point (sorted)')
    ax.set_ylabel('Residual (px)')
    ax.set_title(f'TPS Calibration Residual (n={len(df)}, Sorted)')
    ax.legend(fontsize=9)
    savefig(fig, "calibration_sorted_residual.png")

    # Chart 2: Spatial heatmap with jet colormap
    fig, ax = plt.subplots(figsize=(9, 5))
    pivot = df.pivot_table(values='residual_px', index='grid_y',
                           columns='grid_x', aggfunc='mean')
    im = ax.imshow(pivot.values, cmap='jet', aspect='auto',
                   interpolation='bilinear', origin='lower')
    ax.set_xlabel('Grid X')
    ax.set_ylabel('Grid Y')
    ax.set_title('Residual Spatial Distribution (px)')
    cbar = plt.colorbar(im, ax=ax, shrink=0.85)
    cbar.set_label('Residual (px)')
    savefig(fig, "calibration_heatmap.png")

    # Chart 3: Scatter per row + mean line
    fig, ax = plt.subplots(figsize=(10, 4.5))
    rows = sorted(df['grid_y'].unique())
    for row in rows:
        row_vals = df[df['grid_y'] == row]['residual_px'].values
        ax.scatter([row] * len(row_vals), row_vals, s=6, color=C1, alpha=0.3)

    row_means = df.groupby('grid_y')['residual_px'].mean()
    ax.plot(row_means.index, row_means.values, '-', color=C3, linewidth=1.5,
            label='Row mean')
    ax.set_xlabel('Grid Row (Y)')
    ax.set_ylabel('Residual (px)')
    ax.set_title('Calibration Residual by Row')
    ax.legend(fontsize=9)
    savefig(fig, "calibration_by_row.png")

    # Chart 4: Quiver plot of distortion vectors with jet colormap
    fig, ax = plt.subplots(figsize=(8, 6))
    gx = df['grid_x'].values.astype(float)
    gy = df['grid_y'].values.astype(float)
    sx = df['src_x'].values
    sy = df['src_y'].values
    residuals = df['residual_px'].values

    # Subsample for clarity
    step = max(1, len(df) // 400)
    idx = np.arange(0, len(df), step)

    # Compute distortion vectors
    dx = np.zeros(len(df))
    dy = np.zeros(len(df))
    gx_unique = np.sort(df['grid_x'].unique())
    gy_unique = np.sort(df['grid_y'].unique())
    if len(gx_unique) > 1 and len(gy_unique) > 1:
        x_min, x_max = sx.min(), sx.max()
        y_min, y_max = sy.min(), sy.max()
        expected_x = x_min + (gx - gx.min()) / (gx.max() - gx.min()) * (x_max - x_min)
        expected_y = y_min + (gy - gy.min()) / (gy.max() - gy.min()) * (y_max - y_min)
        dx = expected_x - sx
        dy = expected_y - sy

    q = ax.quiver(sx[idx], sy[idx], dx[idx], dy[idx],
                  residuals[idx], cmap='jet', scale_units='xy',
                  scale=0.5, width=0.003, alpha=0.8)
    ax.set_xlabel('Source X (px)')
    ax.set_ylabel('Source Y (px)')
    ax.set_title('Distortion Vectors (src position, colored by residual)')
    ax.invert_yaxis()
    ax.set_aspect('equal')
    plt.colorbar(q, ax=ax, shrink=0.8, label='Residual (px)')
    savefig(fig, "calibration_vectors.png")

    print(f"Done: 4 charts saved to {OUT_DIR}")


if __name__ == '__main__':
    main()
