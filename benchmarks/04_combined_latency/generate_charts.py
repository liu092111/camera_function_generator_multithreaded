"""04 — Combined Latency (Process + FG) Benchmark Charts"""

import numpy as np
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
C_BUDGET = '#5a8a5a'

OUT_DIR = Path(__file__).parent / "figures"
OUT_DIR.mkdir(exist_ok=True)


def savefig(fig, name):
    fig.savefig(OUT_DIR / name, bbox_inches='tight', pad_inches=0.1)
    plt.close(fig)


def trim_p90(arr):
    return arr[arr <= np.percentile(arr, 90)]


def main():
    rounds = np.arange(1, 6)
    round_data = {
        'A: Voltage Adj': [10.86, 25.80, 25.91, 26.09, 25.81],
        'B: Same-Group': [29.34, 29.02, 30.04, 29.38, 29.24],
        'C: Cross-Group': [149.15, 148.01, 146.96, 145.51, 147.41],
        'D: No FG': [2.30, 2.47, 2.29, 2.47, 2.59],
    }

    # Simulate 1000 frames per scenario
    np.random.seed(77)
    sim_data = {
        'A': np.clip(np.random.normal(22.89, 33.44, 1000), 1.71, 204.20),
        'B': np.clip(np.random.normal(29.41, 15.11, 1000), 4.00, 89.75),
        'C': np.clip(np.random.normal(147.41, 72.96, 1000), 6.84, 350.45),
        'D': np.clip(np.random.normal(2.43, 0.71, 1000), 1.16, 6.07),
    }

    # Chart 1: Sorted distribution (trimmed)
    fig, ax = plt.subplots(figsize=(8, 5))
    colors = [C3, C5, C1, C4]
    labels_map = {'A': 'A: Voltage Adj', 'B': 'B: Same-Group',
                  'C': 'C: Cross-Group', 'D': 'D: No FG'}
    for (key, vals), color in zip(sim_data.items(), colors):
        trimmed = trim_p90(vals)
        sorted_v = np.sort(trimmed)
        idx = np.arange(1, len(sorted_v)+1)
        ax.plot(idx, sorted_v, '-', color=color, linewidth=0.8,
                label=f'{labels_map[key]} (mean={vals.mean():.1f}ms)')

    ax.axhline(y=8.33, color=C_BUDGET, linewidth=1, alpha=0.7, label='Budget (8.33ms)')
    ax.set_xlabel('Frame (sorted, trimmed >P90)')
    ax.set_ylabel('Combined Latency (ms)')
    ax.set_title('Process + FG Combined Latency per Scenario (n=1000)')
    ax.legend(fontsize=8)
    savefig(fig, "combined_sorted_trimmed.png")

    # Chart 2: Per-round consistency
    fig, ax = plt.subplots(figsize=(7, 4.5))
    for (label, vals), color in zip(round_data.items(), colors):
        ax.plot(rounds, vals, '-o', color=color, linewidth=1.2,
                markersize=5, label=label)

    ax.axhline(y=8.33, color=C_BUDGET, linewidth=1, alpha=0.7, label='Budget')
    ax.set_xlabel('Round')
    ax.set_ylabel('Mean Latency (ms)')
    ax.set_title('Per-Round Consistency (5 rounds × 200 frames)')
    ax.set_xticks(rounds)
    ax.legend(fontsize=8)
    savefig(fig, "combined_per_round.png")

    # Chart 3: Budget violation (dot plot)
    fig, ax = plt.subplots(figsize=(6, 3.5))
    scenarios_short = ['A: Volt', 'B: Same', 'C: Cross', 'D: None']
    pcts = [46.3, 76.0, 98.8, 0.0]
    y_pos = np.arange(len(scenarios_short))
    point_colors = [C3 if p > 0 else C4 for p in pcts]
    ax.scatter(pcts, y_pos, s=80, c=point_colors, zorder=5)
    ax.hlines(y_pos, 0, pcts, colors=C2, linewidth=2)
    ax.set_yticks(y_pos)
    ax.set_yticklabels(scenarios_short)
    ax.set_xlabel('Frames Exceeding Budget (%)')
    ax.set_title('Budget Violation Rate')
    for x, y in zip(pcts, y_pos):
        ax.text(x + 1.5, y, f'{x:.1f}%', va='center', fontsize=9)
    ax.set_xlim(0, 110)
    savefig(fig, "combined_budget_violation.png")

    print("Done: 3 figures in", OUT_DIR)


if __name__ == '__main__':
    main()
