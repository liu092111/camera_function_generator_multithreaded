"""08 — System Budget Overview Charts"""

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
C_LIGHT = '#d9e2ec'
C_GRAY = '#9e9e9e'
C_BUDGET = '#5a8a5a'

OUT_DIR = Path(__file__).parent / "figures"
OUT_DIR.mkdir(exist_ok=True)


def savefig(fig, name):
    fig.savefig(OUT_DIR / name, bbox_inches='tight', pad_inches=0.1)
    plt.close(fig)


def main():
    # Chart 1: Thread budget utilization
    fig, ax = plt.subplots(figsize=(8, 3))
    budget = 8.33
    threads = ['Capture Thread', 'Process Thread', 'Main Thread']
    used = [0.031, 2.53, 1.20]
    headroom = [budget - u for u in used]

    y_pos = np.arange(len(threads))
    ax.barh(y_pos, used, height=0.45, color=C1, alpha=0.7, label='Used')
    ax.barh(y_pos, headroom, left=used, height=0.45, color=C_LIGHT, label='Headroom')
    ax.axvline(x=budget, color=C_BUDGET, linewidth=1.2)
    ax.set_yticks(y_pos)
    ax.set_yticklabels(threads)
    ax.set_xlabel('Time (ms)')
    ax.set_title('Per-Thread Budget Utilization (120fps)')
    ax.legend(fontsize=8, loc='lower right')
    for i, val in enumerate(used):
        pct = val / budget * 100
        ax.text(val + 0.15, i, f'{val:.2f}ms ({pct:.0f}%)', va='center', fontsize=8)
    ax.set_xlim(0, budget * 1.05)
    savefig(fig, "system_budget.png")

    # Chart 2: Component timing range (dot + range line)
    fig, ax = plt.subplots(figsize=(7, 4))
    components = ['Capture', 'Process', 'FG Same-Grp', 'FG Cross-Grp']
    typical = [0.031, 2.53, 6.0, 16.5]
    worst = [0.239, 4.67, 36.4, 117.9]

    y_pos = np.arange(len(components))
    ax.scatter(typical, y_pos, s=60, color=C1, zorder=5, label='Typical')
    ax.scatter(worst, y_pos, s=60, color=C3, marker='D', zorder=5, label='Worst-case')
    ax.hlines(y_pos, typical, worst, colors=C_GRAY, linewidth=1.5, alpha=0.5)
    ax.axvline(x=8.33, color=C_BUDGET, linewidth=1, alpha=0.7, label='Budget')
    ax.set_yticks(y_pos)
    ax.set_yticklabels(components)
    ax.set_xlabel('Time (ms)')
    ax.set_title('System Component Timing Range')
    ax.legend(fontsize=8)
    ax.set_xscale('log')
    savefig(fig, "system_timing_range.png")

    print("Done: 2 figures in", OUT_DIR)


if __name__ == '__main__':
    main()
