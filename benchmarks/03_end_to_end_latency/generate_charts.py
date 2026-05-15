"""03 — End-to-End Latency Benchmark Charts"""

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
C_GRAY = '#9e9e9e'
C_BUDGET = '#5a8a5a'

BASE_DIR = Path(__file__).parent
OUT_DIR = BASE_DIR / "figures"
OUT_DIR.mkdir(exist_ok=True)


def savefig(fig, name):
    fig.savefig(OUT_DIR / name, bbox_inches='tight', pad_inches=0.1)
    plt.close(fig)


def main():
    df = pd.read_csv(BASE_DIR / "latency_benchmark_raw.csv")

    # Chart 1: Sorted total latency
    sorted_total = np.sort(df['total'].values)
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(np.arange(1, len(sorted_total)+1), sorted_total, '-', color=C1, linewidth=0.8)
    ax.axhline(y=8.33, color=C_BUDGET, linewidth=1, alpha=0.7, label='Budget (8.33ms)')
    mean_val = df['total'].mean()
    ax.axhline(y=mean_val, color=C3, linewidth=0.8, linestyle='--',
               alpha=0.7, label=f'Mean ({mean_val:.1f}ms)')
    ax.set_xlabel('Frame (sorted by latency)')
    ax.set_ylabel('End-to-End Latency (ms)')
    ax.set_title('End-to-End Latency Distribution (n=500)')
    ax.legend(fontsize=9)
    savefig(fig, "e2e_sorted_latency.png")

    # Chart 2: Segment sorted individually
    fig, ax = plt.subplots(figsize=(8, 4.5))
    seg3_sorted = np.sort(df['seg3'].values)
    seg2_sorted = np.sort(df['seg2'].values)
    idx = np.arange(1, len(df)+1)
    ax.plot(idx, seg3_sorted, '-', color=C1, linewidth=0.8,
            label=f'Main+FG (mean={df["seg3"].mean():.1f}ms)')
    ax.plot(idx, seg2_sorted, '-', color=C3, linewidth=0.8,
            label=f'Process+Queue (mean={df["seg2"].mean():.2f}ms)')
    ax.axhline(y=8.33, color=C_BUDGET, linewidth=1, alpha=0.7, label='Budget')
    ax.set_xlabel('Frame (sorted independently per segment)')
    ax.set_ylabel('Latency (ms)')
    ax.set_title('Latency by Segment (sorted)')
    ax.legend(fontsize=9)
    savefig(fig, "e2e_segment_sorted.png")

    # Chart 3: Time series
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(df['frame_id'], df['total'], '-', color=C1, linewidth=0.5, alpha=0.8)
    ax.axhline(y=8.33, color=C_BUDGET, linewidth=1, alpha=0.7, label='Budget (8.33ms)')
    ax.set_xlabel('Frame ID')
    ax.set_ylabel('Latency (ms)')
    ax.set_title('End-to-End Latency Time Series (500 consecutive frames)')
    ax.legend(fontsize=9)
    savefig(fig, "e2e_timeseries.png")

    # Chart 4: CDF
    fig, ax = plt.subplots(figsize=(6, 4.5))
    cdf = np.arange(1, len(sorted_total)+1) / len(sorted_total)
    ax.plot(sorted_total, cdf, '-', color=C1, linewidth=1.5)
    ax.axvline(x=8.33, color=C_BUDGET, linewidth=1, alpha=0.7, label='Budget')
    p50 = np.percentile(df['total'], 50)
    p95 = np.percentile(df['total'], 95)
    ax.axhline(y=0.50, color=C_GRAY, linewidth=0.4, linestyle=':')
    ax.axhline(y=0.95, color=C_GRAY, linewidth=0.4, linestyle=':')
    ax.text(sorted_total[-1]*0.55, 0.49, f'P50={p50:.1f}ms', fontsize=8, color=C_GRAY)
    ax.text(sorted_total[-1]*0.55, 0.94, f'P95={p95:.1f}ms', fontsize=8, color=C_GRAY)
    ax.set_xlabel('Latency (ms)')
    ax.set_ylabel('Cumulative Probability')
    ax.set_title('End-to-End Latency CDF')
    ax.legend(fontsize=9)
    savefig(fig, "e2e_cdf.png")

    print("Done: 4 figures in", OUT_DIR)


if __name__ == '__main__':
    main()
