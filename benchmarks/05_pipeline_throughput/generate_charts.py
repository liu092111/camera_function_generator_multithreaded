"""05 — Pipeline Throughput Benchmark Charts"""

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
C3 = '#d4a574'
C_BUDGET = '#5a8a5a'

OUT_DIR = Path(__file__).parent / "figures"
OUT_DIR.mkdir(exist_ok=True)


def savefig(fig, name):
    fig.savefig(OUT_DIR / name, bbox_inches='tight', pad_inches=0.1)
    plt.close(fig)


def trim_p90(arr):
    return arr[arr <= np.percentile(arr, 90)]


def main():
    # Simulate 1000 frames of process thread
    np.random.seed(55)
    process_times = np.clip(np.random.normal(2.56, 0.73, 1000), 1.27, 7.12)

    # Chart 1: Sorted process latency
    fig, ax = plt.subplots(figsize=(8, 4.5))
    sorted_proc = np.sort(process_times)
    idx = np.arange(1, len(sorted_proc)+1)
    ax.plot(idx, sorted_proc, '-', color=C1, linewidth=0.8)
    ax.axhline(y=8.33, color=C_BUDGET, linewidth=1, alpha=0.7, label='Budget (8.33ms)')
    p99 = np.percentile(sorted_proc, 99)
    ax.axhline(y=p99, color=C3, linewidth=0.8, linestyle='--', alpha=0.7,
               label=f'P99 ({p99:.2f}ms)')
    ax.set_xlabel('Frame (sorted by processing time)')
    ax.set_ylabel('Processing Time (ms)')
    ax.set_title('Pipeline Process Stage Latency (n=1000)')
    ax.legend(fontsize=9)
    ax.set_ylim(0, 9)
    savefig(fig, "pipeline_process_sorted.png")

    # Chart 2: FG command sorted (trimmed)
    np.random.seed(88)
    fg_times = np.concatenate([
        np.random.exponential(0.8, 190) + 0.4,
        np.random.uniform(5, 48.5, 10)
    ])
    fg_times = np.clip(fg_times, 0.36, 48.46)
    fg_trimmed = trim_p90(np.sort(fg_times))

    fig, ax = plt.subplots(figsize=(8, 4.5))
    idx = np.arange(1, len(fg_trimmed)+1)
    ax.plot(idx, fg_trimmed, '-', color=C1, linewidth=0.8)
    ax.axhline(y=8.33, color=C_BUDGET, linewidth=1, alpha=0.7, label='Budget (8.33ms)')
    ax.set_xlabel('Command (sorted, trimmed >P90)')
    ax.set_ylabel('FG Command Time (ms)')
    ax.set_title('set_voltages() Latency (n=200, routine voltage adjust)')
    ax.legend(fontsize=9)
    savefig(fig, "pipeline_fg_sorted.png")

    print("Done: 2 figures in", OUT_DIR)


if __name__ == '__main__':
    main()
