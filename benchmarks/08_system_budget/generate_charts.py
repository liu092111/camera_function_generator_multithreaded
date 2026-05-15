"""08 — System Budget Benchmark Charts

Reads: system_budget_raw.csv
Outputs: system_process_histogram.png, system_round_errorbar.png,
         system_budget_bar.png
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

    csv_path = BASE_DIR / "system_budget_raw.csv"
    if not os.path.exists(csv_path):
        print(f"[WARNING] {csv_path} not found. Skipping all charts.")
        return

    df = pd.read_csv(csv_path)

    # Chart 1: Histogram of process_ms from all 2500 frames
    fig, ax = plt.subplots(figsize=(8, 4.5))
    data = df['process_ms'].dropna().values
    mean_val = data.mean()
    ax.hist(data, bins=50, color=C1, alpha=0.7, edgecolor='none')
    ax.axvline(x=mean_val, color=C3, linestyle='--', linewidth=1,
               label=f'Mean ({mean_val:.2f} ms)')
    ax.set_xlabel('Time (ms)')
    ax.set_ylabel('Count')
    ax.set_title(f'System Process Latency Distribution (n={len(data)})')
    ax.legend(fontsize=9)
    savefig(fig, "system_process_histogram.png")

    # Chart 2: Per-round process mean +/- std (error bar plot)
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
    ax.set_ylabel('Process Time (ms)')
    ax.set_title('Process Thread Latency Per Round: Mean +/- Std')
    savefig(fig, "system_round_errorbar.png")

    # Chart 3: Horizontal bar showing per-thread used time (no budget line)
    thread_cols = ['capture_ms', 'process_ms', 'main_ms']
    thread_labels = ['Capture Thread', 'Process Thread', 'Main Thread']
    thread_colors = [C1, C3, C4]

    thread_means = [df[col].mean() for col in thread_cols]

    fig, ax = plt.subplots(figsize=(8, 3))
    y_pos = np.arange(len(thread_labels))
    bars = ax.barh(y_pos, thread_means, height=0.45, color=thread_colors,
                   alpha=0.8, edgecolor='none')
    ax.set_yticks(y_pos)
    ax.set_yticklabels(thread_labels)
    ax.set_xlabel('Time (ms)')
    ax.set_title('Per-Thread Mean Latency')
    for i, (bar, val) in enumerate(zip(bars, thread_means)):
        ax.text(bar.get_width() + max(thread_means) * 0.02, i,
                f'{val:.2f} ms', va='center', fontsize=8, color='#444444')
    ax.set_xlim(0, max(thread_means) * 1.25)
    savefig(fig, "system_budget_bar.png")

    print(f"Done: 3 charts saved to {OUT_DIR}")


if __name__ == '__main__':
    main()
