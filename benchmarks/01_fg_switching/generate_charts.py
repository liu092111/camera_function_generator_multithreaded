"""01 — Function Generator Switching Benchmark Charts

Reads: fg_switching_gen3_raw.csv, fg_switching_3gen_raw.csv (optional)
Outputs: fg_gen3_histogram.png, fg_3gen_comparison.png, fg_speedup.png
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

    # --- Load Gen-3 data ---
    gen3_csv = BASE_DIR / "fg_switching_gen3_raw.csv"
    if not os.path.exists(gen3_csv):
        print(f"[WARNING] {gen3_csv} not found. Skipping Gen-3 charts.")
        return

    df = pd.read_csv(gen3_csv)

    cols = ['same_group_1_3_ms', 'same_group_2_4_ms',
            'cross_group_1_to_2_ms', 'cross_group_2_to_1_ms',
            'voltage_adjust_ms']
    labels = ['Same Group (1-3)', 'Same Group (2-4)',
              'Cross Group (1->2)', 'Cross Group (2->1)',
              'Voltage Adjust']
    colors = [C1, C2, C3, C4, C5]

    # Chart 1: Overlaid histograms for 5 Gen-3 scenarios
    fig, ax = plt.subplots(figsize=(8, 4.5))
    for col, label, color in zip(cols, labels, colors):
        data = df[col].dropna().values
        mean_val = data.mean()
        ax.hist(data, bins=30, color=color, alpha=0.5, edgecolor='none',
                label=f'{label} (mean={mean_val:.2f} ms)')

    ax.set_xlabel('Time (ms)')
    ax.set_ylabel('Count')
    ax.set_title('Gen-3 FG Mode-Switching Latency Distribution')
    ax.legend(fontsize=8, loc='upper right', framealpha=0.9)
    savefig(fig, "fg_gen3_histogram.png")

    # Chart 2: 3-generation comparison histogram (if file exists)
    gen3gen_csv = BASE_DIR / "fg_switching_3gen_raw.csv"
    if os.path.exists(gen3gen_csv):
        df3 = pd.read_csv(gen3gen_csv)
        fig, ax = plt.subplots(figsize=(8, 4.5))

        gen_cols = {
            'Gen-1 Full': ('gen1_full_ms', C3),
            'Gen-2 Same': ('gen2_same_ms', C1),
            'Gen-2 Cross': ('gen2_cross_ms', C2),
            'Gen-3 Same': ('gen3_same_ms', C4),
            'Gen-3 Cross': ('gen3_cross_ms', C5),
            'Gen-3 Voltage': ('gen3_voltage_ms', C_GRAY),
        }
        for label, (col, color) in gen_cols.items():
            if col in df3.columns:
                data = df3[col].dropna().values
                if len(data) > 0:
                    ax.hist(data, bins=30, color=color, alpha=0.5,
                            edgecolor='none',
                            label=f'{label} (mean={data.mean():.2f} ms)')

        ax.set_xlabel('Time (ms)')
        ax.set_ylabel('Count')
        ax.set_title('FG Switching: Gen-1 vs Gen-2 vs Gen-3')
        ax.legend(fontsize=8, loc='upper right', framealpha=0.9)
        savefig(fig, "fg_3gen_comparison.png")
    else:
        print("[WARNING] fg_switching_3gen_raw.csv not found. Skipping 3-gen chart.")

    # Chart 3: Horizontal dot plot showing mean latency per scenario
    fig, ax = plt.subplots(figsize=(6, 3.5))
    means = {col: df[col].dropna().mean() for col in cols}
    scenario_means = [means[col] for col in cols]
    y_pos = np.arange(len(labels))

    ax.scatter(scenario_means, y_pos, s=80, color=C1, zorder=5)
    ax.hlines(y_pos, 0, scenario_means, colors=C2, linewidth=2)
    ax.set_yticks(y_pos)
    ax.set_yticklabels(labels)
    ax.set_xlabel('Mean Switching Time (ms)')
    ax.set_title('Gen-3 Switching Latency by Scenario')
    for x_val, y_val in zip(scenario_means, y_pos):
        ax.text(x_val + max(scenario_means) * 0.03, y_val,
                f'{x_val:.2f} ms', va='center', fontsize=8)
    ax.set_xlim(0, max(scenario_means) * 1.3)
    savefig(fig, "fg_speedup.png")

    print(f"Done: charts saved to {OUT_DIR}")


if __name__ == '__main__':
    main()
