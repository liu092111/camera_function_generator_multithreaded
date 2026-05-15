"""04 — Combined Latency (Process + FG) Benchmark Charts

Reads: combined_latency_raw.csv
Outputs: combined_process_histogram.png, combined_round_consistency.png,
         combined_fg_histogram.png
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

    csv_path = BASE_DIR / "combined_latency_raw.csv"
    if not os.path.exists(csv_path):
        print(f"[WARNING] {csv_path} not found. Skipping all charts.")
        return

    df = pd.read_csv(csv_path)

    scenarios = ['A_voltage', 'B_same_group', 'C_cross_group', 'D_baseline']
    scenario_labels = ['A: Voltage Adj', 'B: Same Group',
                       'C: Cross Group', 'D: Baseline']
    colors = [C3, C1, C5, C4]

    # Chart 1: 4 scenarios overlaid histograms (process_ms)
    fig, ax = plt.subplots(figsize=(8, 4.5))
    for scenario, label, color in zip(scenarios, scenario_labels, colors):
        df_s = df[df['scenario'] == scenario]
        data = df_s['process_ms'].dropna().values
        if len(data) > 0:
            ax.hist(data, bins=50, color=color, alpha=0.5, edgecolor='none',
                    label=f'{label} (mean={data.mean():.2f} ms)')

    ax.set_xlabel('Time (ms)')
    ax.set_ylabel('Count')
    ax.set_title('Process Latency Distribution by Scenario')
    ax.legend(fontsize=8, loc='upper right', framealpha=0.9)
    savefig(fig, "combined_process_histogram.png")

    # Chart 2: Line plot: 5 rounds x 4 scenarios (process mean per round)
    fig, ax = plt.subplots(figsize=(7, 4.5))
    for scenario, label, color in zip(scenarios, scenario_labels, colors):
        df_s = df[df['scenario'] == scenario]
        rounds = sorted(df_s['round'].unique())
        round_means = [df_s[df_s['round'] == r]['process_ms'].mean()
                       for r in rounds]
        ax.plot(rounds, round_means, '-o', color=color, linewidth=1.2,
                markersize=5, label=label)

    ax.set_xlabel('Round')
    ax.set_ylabel('Mean Process Time (ms)')
    ax.set_title('Per-Round Process Consistency (4 Scenarios)')
    ax.set_xticks(sorted(df['round'].unique()))
    ax.legend(fontsize=8, framealpha=0.9)
    savefig(fig, "combined_round_consistency.png")

    # Chart 3: FG latency histograms for scenarios A, B, C overlaid
    fig, ax = plt.subplots(figsize=(8, 4.5))
    fg_scenarios = ['A_voltage', 'B_same_group', 'C_cross_group']
    fg_labels = ['A: Voltage Adj', 'B: Same Group', 'C: Cross Group']
    fg_colors = [C3, C1, C5]

    for scenario, label, color in zip(fg_scenarios, fg_labels, fg_colors):
        df_s = df[df['scenario'] == scenario]
        data = df_s['fg_ms'].dropna().values
        if len(data) > 0:
            ax.hist(data, bins=50, color=color, alpha=0.5, edgecolor='none',
                    label=f'{label} (mean={data.mean():.2f} ms)')

    ax.set_xlabel('Time (ms)')
    ax.set_ylabel('Count')
    ax.set_title('FG Latency Distribution (Scenarios A, B, C)')
    ax.legend(fontsize=8, loc='upper right', framealpha=0.9)
    savefig(fig, "combined_fg_histogram.png")

    print(f"Done: 3 charts saved to {OUT_DIR}")


if __name__ == '__main__':
    main()
