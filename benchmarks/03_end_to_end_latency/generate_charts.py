"""03 — End-to-End Latency Benchmark Charts

Reads: e2e_latency_raw.csv
Outputs: e2e_gen2_vs_gen3_histogram.png, e2e_process_histogram.png,
         e2e_round_errorbar.png
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

    csv_path = BASE_DIR / "e2e_latency_raw.csv"
    if not os.path.exists(csv_path):
        print(f"[WARNING] {csv_path} not found. Skipping all charts.")
        return

    df = pd.read_csv(csv_path)

    df_gen3 = df[df['generation'] == 'gen3'].copy()
    df_gen2 = df[df['generation'] == 'gen2'].copy()

    # Chart 1: Gen-2 vs Gen-3 FG write overlaid histograms
    fig, ax = plt.subplots(figsize=(8, 4.5))
    if not df_gen3.empty:
        data_gen3 = df_gen3['fg_write_ms'].dropna().values
        ax.hist(data_gen3, bins=50, color=C1, alpha=0.6, edgecolor='none',
                label=f'Gen-3 FG Write (mean={data_gen3.mean():.2f} ms)')
    if not df_gen2.empty:
        data_gen2 = df_gen2['fg_write_ms'].dropna().values
        ax.hist(data_gen2, bins=50, color=C3, alpha=0.6, edgecolor='none',
                label=f'Gen-2 FG Write (mean={data_gen2.mean():.2f} ms)')

    ax.set_xlabel('Time (ms)')
    ax.set_ylabel('Count')
    ax.set_title('FG Write Latency Distribution: Gen-2 vs Gen-3')
    ax.legend(fontsize=9)
    savefig(fig, "e2e_gen2_vs_gen3_histogram.png")

    # Chart 2: Histogram of process_ms (Gen-3 data)
    fig, ax = plt.subplots(figsize=(8, 4.5))
    if not df_gen3.empty:
        data_proc = df_gen3['process_ms'].dropna().values
        mean_val = data_proc.mean()
        ax.hist(data_proc, bins=50, color=C1, alpha=0.7, edgecolor='none')
        ax.axvline(x=mean_val, color=C3, linestyle='--', linewidth=1,
                   label=f'Mean ({mean_val:.2f} ms)')

    ax.set_xlabel('Time (ms)')
    ax.set_ylabel('Count')
    ax.set_title(f'Process Latency Distribution (Gen-3, n={len(df_gen3)})')
    ax.legend(fontsize=9)
    savefig(fig, "e2e_process_histogram.png")

    # Chart 3: Per-round mean +/- std for FG write (Gen-2 and Gen-3 side by side)
    fig, ax = plt.subplots(figsize=(7, 4.5))
    width = 0.35

    rounds_gen3 = sorted(df_gen3['round'].unique()) if not df_gen3.empty else []
    rounds_gen2 = sorted(df_gen2['round'].unique()) if not df_gen2.empty else []
    all_rounds = sorted(set(rounds_gen3) | set(rounds_gen2))
    x_pos = np.arange(len(all_rounds))

    # Gen-3
    means_gen3 = []
    stds_gen3 = []
    for rnd in all_rounds:
        rnd_data = df_gen3[df_gen3['round'] == rnd]['fg_write_ms']
        means_gen3.append(rnd_data.mean() if not rnd_data.empty else 0)
        stds_gen3.append(rnd_data.std() if not rnd_data.empty else 0)

    # Gen-2
    means_gen2 = []
    stds_gen2 = []
    for rnd in all_rounds:
        rnd_data = df_gen2[df_gen2['round'] == rnd]['fg_write_ms']
        means_gen2.append(rnd_data.mean() if not rnd_data.empty else 0)
        stds_gen2.append(rnd_data.std() if not rnd_data.empty else 0)

    ax.errorbar(x_pos - width / 2, means_gen3, yerr=stds_gen3, fmt='o',
                color=C1, ecolor=C2, capsize=4, markersize=6,
                elinewidth=1.5, capthick=1.2, label='Gen-3')
    ax.errorbar(x_pos + width / 2, means_gen2, yerr=stds_gen2, fmt='s',
                color=C3, ecolor=C5, capsize=4, markersize=6,
                elinewidth=1.5, capthick=1.2, label='Gen-2')

    ax.set_xticks(x_pos)
    ax.set_xticklabels([f'R{r}' for r in all_rounds])
    ax.set_xlabel('Round')
    ax.set_ylabel('FG Write Time (ms)')
    ax.set_title('FG Write Latency Per Round: Gen-2 vs Gen-3')
    ax.legend(fontsize=9)
    savefig(fig, "e2e_round_errorbar.png")

    print(f"Done: 3 charts saved to {OUT_DIR}")


if __name__ == '__main__':
    main()
