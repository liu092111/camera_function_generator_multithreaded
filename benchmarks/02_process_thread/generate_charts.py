"""02 — Process Thread Per-Frame Benchmark Charts

Reads: process_thread_raw.csv
Outputs: process_histogram.png, process_step_breakdown.png,
         process_per_round_histogram.png, process_round_errorbar.png
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

    csv_path = BASE_DIR / "process_thread_raw.csv"
    if not os.path.exists(csv_path):
        print(f"[WARNING] {csv_path} not found. Skipping all charts.")
        return

    df = pd.read_csv(csv_path)

    # Filter online frames only (exclude offline)
    df_online = df[df['mode'] != 'offline'].copy()

    # Chart 1: Histogram of total_ms for online frames
    fig, ax = plt.subplots(figsize=(8, 4.5))
    data = df_online['total_ms'].dropna().values
    mean_val = data.mean()
    ax.hist(data, bins=50, color=C1, alpha=0.7, edgecolor='none')
    ax.axvline(x=mean_val, color=C3, linestyle='--', linewidth=1,
               label=f'Mean ({mean_val:.2f} ms)')
    ax.set_xlabel('Time (ms)')
    ax.set_ylabel('Count')
    ax.set_title(f'Process Thread Latency Distribution (n={len(data)})')
    ax.legend(fontsize=9)
    savefig(fig, "process_histogram.png")

    # Chart 2: Horizontal bar of per-step mean time
    step_cols = ['undistort_ms', 'rotate_flip_ms', 'kalman_ms',
                 'hsv_detect_ms', 'ema_ms', 'draw_ms']
    step_labels = ['Undistort (TPS)', 'Rotate + Flip', 'Kalman',
                   'HSV Detect', 'EMA + Angle', 'Draw Overlay']
    step_means = [df_online[col].mean() for col in step_cols]

    fig, ax = plt.subplots(figsize=(7, 4))
    y_pos = np.arange(len(step_labels))
    colors_bar = [C1, C2, C3, C4, C5, C_GRAY]
    bars = ax.barh(y_pos, step_means, height=0.55, color=colors_bar,
                   alpha=0.8, edgecolor='none')
    ax.set_yticks(y_pos)
    ax.set_yticklabels(step_labels)
    ax.set_xlabel('Time (ms)')
    ax.set_title('Process Thread Per-Step Mean Latency (Online)')
    for bar, val in zip(bars, step_means):
        if val >= 0.01:
            ax.text(bar.get_width() + max(step_means) * 0.02,
                    bar.get_y() + bar.get_height() / 2,
                    f'{val:.3f} ms', va='center', fontsize=8, color='#444444')
    ax.set_xlim(0, max(step_means) * 1.35)
    savefig(fig, "process_step_breakdown.png")

    # Chart 3: 5 rounds as overlaid semi-transparent histograms
    rounds = sorted(df_online['round'].unique())
    round_colors = [C1, C2, C3, C4, C5]

    fig, ax = plt.subplots(figsize=(8, 4.5))
    for i, rnd in enumerate(rounds[:5]):
        df_rnd = df_online[df_online['round'] == rnd]
        data_rnd = df_rnd['total_ms'].dropna().values
        color = round_colors[i % len(round_colors)]
        ax.hist(data_rnd, bins=50, color=color, alpha=0.5, edgecolor='none',
                label=f'Round {rnd} (mean={data_rnd.mean():.2f} ms)')

    ax.set_xlabel('Time (ms)')
    ax.set_ylabel('Count')
    ax.set_title('Process Thread Latency per Round (Histogram)')
    ax.legend(fontsize=8, loc='upper right', framealpha=0.9)
    savefig(fig, "process_per_round_histogram.png")

    # Chart 4: Mean +/- std for each round (errorbar plot)
    round_means = []
    round_stds = []
    round_labels = []
    for rnd in rounds:
        df_rnd = df_online[df_online['round'] == rnd]
        round_means.append(df_rnd['total_ms'].mean())
        round_stds.append(df_rnd['total_ms'].std())
        round_labels.append(f'R{rnd}')

    fig, ax = plt.subplots(figsize=(6, 4))
    x_pos = np.arange(len(round_labels))
    ax.errorbar(x_pos, round_means, yerr=round_stds, fmt='o',
                color=C1, ecolor=C2, capsize=4, markersize=6,
                elinewidth=1.5, capthick=1.2)
    ax.set_xticks(x_pos)
    ax.set_xticklabels(round_labels)
    ax.set_xlabel('Round')
    ax.set_ylabel('Time (ms)')
    ax.set_title('Process Thread Latency: Per-Round Mean +/- Std')
    savefig(fig, "process_round_errorbar.png")

    print(f"Done: 4 charts saved to {OUT_DIR}")


if __name__ == '__main__':
    main()
