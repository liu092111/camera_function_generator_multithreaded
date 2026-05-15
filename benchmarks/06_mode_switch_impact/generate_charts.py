"""06 — Mode Switch Impact Benchmark Charts

Reads: mode_impact_raw.csv
Outputs: mode_impact_histogram.png, mode_impact_round_errorbar.png
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

    csv_path = BASE_DIR / "mode_impact_raw.csv"
    if not os.path.exists(csv_path):
        print(f"[WARNING] {csv_path} not found. Skipping all charts.")
        return

    df = pd.read_csv(csv_path)

    # Chart 1: Histogram of all 2500 frames process_ms
    fig, ax = plt.subplots(figsize=(8, 4.5))
    data = df['process_ms'].dropna().values
    mean_val = data.mean()
    ax.hist(data, bins=50, color=C1, alpha=0.7, edgecolor='none',
            label=f'All frames (n={len(data)}, mean={mean_val:.2f} ms)')
    ax.axvline(x=mean_val, color=C3, linestyle='--', linewidth=1,
               label=f'Mean ({mean_val:.2f} ms)')
    ax.set_xlabel('Time (ms)')
    ax.set_ylabel('Count')
    ax.set_title('Process Latency During Mode-Switch Test (Histogram)')
    ax.legend(fontsize=9)
    savefig(fig, "mode_impact_histogram.png")

    # Chart 2: Per-round mean +/- std (error bar plot)
    rounds = sorted(df['round'].unique())
    round_means = []
    round_stds = []
    for rnd in rounds:
        df_rnd = df[df['round'] == rnd]
        round_means.append(df_rnd['process_ms'].mean())
        round_stds.append(df_rnd['process_ms'].std())

    fig, ax = plt.subplots(figsize=(6, 4))
    x_pos = np.arange(len(rounds))
    ax.errorbar(x_pos, round_means, yerr=round_stds, fmt='o',
                color=C1, ecolor=C2, capsize=4, markersize=7,
                elinewidth=1.5, capthick=1.2)
    ax.set_xticks(x_pos)
    ax.set_xticklabels([f'R{r}' for r in rounds])
    ax.set_xlabel('Round')
    ax.set_ylabel('Time (ms)')
    ax.set_title('Process Latency Per Round: Mean +/- Std')
    savefig(fig, "mode_impact_round_errorbar.png")

    print(f"Done: 2 charts saved to {OUT_DIR}")


if __name__ == '__main__':
    main()
