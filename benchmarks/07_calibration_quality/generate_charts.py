"""07 — Calibration Quality Benchmark Charts"""

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
C3 = '#d4a574'

BASE_DIR = Path(__file__).parent
OUT_DIR = BASE_DIR / "figures"
OUT_DIR.mkdir(exist_ok=True)


def savefig(fig, name):
    fig.savefig(OUT_DIR / name, bbox_inches='tight', pad_inches=0.1)
    plt.close(fig)


def main():
    df = pd.read_csv(BASE_DIR / "calibration_raw.csv")

    # Chart 1: Sorted residual (every point)
    fig, ax = plt.subplots(figsize=(8, 4.5))
    sorted_res = np.sort(df['residual_px'].values) * 1000
    idx = np.arange(1, len(sorted_res)+1)
    ax.plot(idx, sorted_res, '-', color=C1, linewidth=1)
    mean_res = sorted_res.mean()
    ax.axhline(y=mean_res, color=C3, linewidth=0.8, linestyle='--',
               alpha=0.7, label=f'Mean ({mean_res:.3f}×10⁻³ px)')
    ax.set_xlabel('Point (sorted by residual)')
    ax.set_ylabel('Residual (×10⁻³ px)')
    ax.set_title(f'TPS Calibration Residual — All Points (n={len(df)})')
    ax.legend(fontsize=9)
    savefig(fig, "calibration_sorted_residual.png")

    # Chart 2: Spatial heatmap (rainbow colormap)
    fig, ax = plt.subplots(figsize=(9, 5))
    pivot = df.pivot_table(values='residual_px', index='row', columns='col', aggfunc='mean')
    im = ax.imshow(pivot.values * 1000, cmap='jet', aspect='auto', interpolation='bilinear')
    ax.set_xlabel('Column')
    ax.set_ylabel('Row')
    ax.set_title('Residual Spatial Distribution (×10⁻³ px)')
    cbar = plt.colorbar(im, ax=ax, shrink=0.85)
    cbar.set_label('Residual (×10⁻³ px)')
    savefig(fig, "calibration_heatmap.png")

    # Chart 3: Residual per row (scatter + mean line)
    fig, ax = plt.subplots(figsize=(10, 4.5))
    rows = sorted(df['row'].unique())
    for row in rows:
        row_vals = df[df['row'] == row]['residual_px'].values * 1000
        ax.scatter([row] * len(row_vals), row_vals, s=6, color=C1, alpha=0.4)

    row_means = df.groupby('row')['residual_px'].mean().values * 1000
    ax.plot(rows, row_means, '-', color=C3, linewidth=1.2, label='Row mean')
    ax.set_xlabel('Grid Row')
    ax.set_ylabel('Residual (×10⁻³ px)')
    ax.set_title('Calibration Residual by Row (each dot = one grid point)')
    ax.legend(fontsize=9)
    savefig(fig, "calibration_by_row.png")

    # Chart 4: Distortion vectors (rainbow by magnitude)
    fig, ax = plt.subplots(figsize=(8, 6))
    sample = df.iloc[::4]
    dx = sample['dst_x'].values - sample['src_x'].values
    dy = sample['dst_y'].values - sample['src_y'].values
    mag = np.sqrt(dx**2 + dy**2)
    q = ax.quiver(sample['src_x'], sample['src_y'], dx, dy,
                  mag, cmap='jet', scale=800, width=0.003, alpha=0.8)
    ax.set_xlabel('X (px)')
    ax.set_ylabel('Y (px)')
    ax.set_title('Distortion Correction Vectors (src → dst)')
    ax.invert_yaxis()
    ax.set_aspect('equal')
    plt.colorbar(q, ax=ax, shrink=0.8, label='Displacement (px)')
    savefig(fig, "calibration_vectors.png")

    print("Done: 4 figures in", OUT_DIR)


if __name__ == '__main__':
    main()
