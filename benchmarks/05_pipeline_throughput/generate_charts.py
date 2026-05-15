"""05 — Pipeline Throughput Benchmark Charts

Reads: throughput_raw.csv
Outputs: throughput_process_histogram.png, throughput_round_fps.png,
         throughput_timeline.png
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

    csv_path = BASE_DIR / "throughput_raw.csv"
    if not os.path.exists(csv_path):
        print(f"[WARNING] {csv_path} not found. Skipping all charts.")
        return

    df = pd.read_csv(csv_path)

    # Chart 1: Histogram of process_ms (2500 frames)
    fig, ax = plt.subplots(figsize=(8, 4.5))
    data = df['process_ms'].dropna().values
    mean_val = data.mean()
    ax.hist(data, bins=50, color=C1, alpha=0.7, edgecolor='none')
    ax.axvline(x=mean_val, color=C3, linestyle='--', linewidth=1,
               label=f'Mean ({mean_val:.2f} ms)')
    ax.set_xlabel('Time (ms)')
    ax.set_ylabel('Count')
    ax.set_title(f'Pipeline Process Latency Distribution (n={len(data)})')
    ax.legend(fontsize=9)
    savefig(fig, "throughput_process_histogram.png")

    # Chart 2: Bar chart with error bars showing FPS per round
    rounds = sorted(df['round'].unique())
    round_fps_means = []
    round_fps_stds = []
    for rnd in rounds:
        df_rnd = df[df['round'] == rnd]
        fps_vals = 1000.0 / df_rnd['total_ms'].replace(0, np.nan).dropna().values
        round_fps_means.append(fps_vals.mean())
        round_fps_stds.append(fps_vals.std())

    fig, ax = plt.subplots(figsize=(6, 4))
    x_pos = np.arange(len(rounds))
    ax.bar(x_pos, round_fps_means, yerr=round_fps_stds, capsize=4,
           color=C1, alpha=0.8, edgecolor='none', ecolor=C_GRAY)
    ax.set_xticks(x_pos)
    ax.set_xticklabels([f'R{r}' for r in rounds])
    ax.set_xlabel('Round')
    ax.set_ylabel('FPS')
    ax.set_title('Throughput per Round (Frames Per Second)')
    savefig(fig, "throughput_round_fps.png")

    # Chart 3: Time series of process_ms for first 200 frames (raw order)
    fig, ax = plt.subplots(figsize=(10, 4))
    df_first_round = df[df['round'] == rounds[0]].head(200)
    ax.plot(df_first_round['frame_idx'].values,
            df_first_round['process_ms'].values,
            '-', color=C1, linewidth=0.7, alpha=0.9)
    ax.set_xlabel('Frame Index')
    ax.set_ylabel('Time (ms)')
    ax.set_title('Process Latency Timeline (First 200 Frames)')
    savefig(fig, "throughput_timeline.png")

    print(f"Done: 3 charts saved to {OUT_DIR}")


if __name__ == '__main__':
    main()
