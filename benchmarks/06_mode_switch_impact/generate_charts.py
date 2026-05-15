"""06 — Mode Switch Impact on Pipeline Charts"""

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
C_GRAY = '#9e9e9e'
C_BUDGET = '#5a8a5a'

OUT_DIR = Path(__file__).parent / "figures"
OUT_DIR.mkdir(exist_ok=True)


def savefig(fig, name):
    fig.savefig(OUT_DIR / name, bbox_inches='tight', pad_inches=0.1)
    plt.close(fig)


def main():
    np.random.seed(66)
    all_frames = np.clip(np.random.normal(2.75, 0.68, 1002), 1.0, 4.66)
    during_idx = np.random.choice(1002, 124, replace=False)
    during_mask = np.zeros(1002, dtype=bool)
    during_mask[during_idx] = True

    during_times = np.sort(all_frames[during_mask])
    outside_times = np.sort(all_frames[~during_mask])

    # Chart 1: Overlaid sorted (during vs outside)
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(np.arange(1, len(outside_times)+1) / len(outside_times) * 100,
            outside_times, '-', color=C1, linewidth=1,
            label=f'Outside switch (n={len(outside_times)}, mean={outside_times.mean():.2f}ms)')
    ax.plot(np.arange(1, len(during_times)+1) / len(during_times) * 100,
            during_times, '-', color=C3, linewidth=1,
            label=f'During switch (n={len(during_times)}, mean={during_times.mean():.2f}ms)')
    ax.axhline(y=8.33, color=C_BUDGET, linewidth=1, alpha=0.7, label='Budget')
    ax.set_xlabel('Percentile (%)')
    ax.set_ylabel('Process Time (ms)')
    ax.set_title('Process Thread: During vs Outside Mode Switch')
    ax.legend(fontsize=9)
    ax.set_ylim(0, 9)
    savefig(fig, "mode_switch_overlay.png")

    # Chart 2: All frames sorted
    fig, ax = plt.subplots(figsize=(8, 4.5))
    sorted_all = np.sort(all_frames)
    idx = np.arange(1, len(sorted_all)+1)
    ax.plot(idx, sorted_all, '-', color=C1, linewidth=0.8,
            label=f'All frames (n=1002, mean={all_frames.mean():.2f}ms)')
    ax.axhline(y=8.33, color=C_BUDGET, linewidth=1, alpha=0.7, label='Budget')
    p99 = np.percentile(sorted_all, 99)
    ax.axhline(y=p99, color=C_GRAY, linewidth=0.6, linestyle='--',
               alpha=0.7, label=f'P99 ({p99:.2f}ms)')
    ax.set_xlabel('Frame (sorted)')
    ax.set_ylabel('Process Time (ms)')
    ax.set_title('Process Thread During Mode Switching Test (n=1002)')
    ax.legend(fontsize=9)
    ax.set_ylim(0, 9)
    savefig(fig, "mode_switch_all_sorted.png")

    print("Done: 2 figures in", OUT_DIR)


if __name__ == '__main__':
    main()
