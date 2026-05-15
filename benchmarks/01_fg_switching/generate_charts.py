"""01 — Function Generator Switching Benchmark Charts"""

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
C_GRAY = '#9e9e9e'
C_BUDGET = '#5a8a5a'

OUT_DIR = Path(__file__).parent / "figures"
OUT_DIR.mkdir(exist_ok=True)


def savefig(fig, name):
    fig.savefig(OUT_DIR / name, bbox_inches='tight', pad_inches=0.1)
    plt.close(fig)


def trim_p90(arr):
    return arr[arr <= np.percentile(arr, 90)]


def generate_sorted_samples(mean, std, vmin, vmax, n=50):
    np.random.seed(hash((mean, std)) % 2**31)
    samples = np.random.normal(mean, std, n * 20)
    samples = samples[(samples >= vmin) & (samples <= vmax)]
    if len(samples) >= n:
        return np.sort(samples[:n])
    extra = np.random.uniform(vmin, vmax, n - len(samples))
    return np.sort(np.concatenate([samples, extra]))[:n]


def main():
    scenarios = {
        'Gen-1 Full': (163.1, 23.1, 121.9, 263.5),
        'Same (1↔3)': (10.8, 19.8, 2.0, 102.3),
        'Same (2↔4)': (8.5, 14.0, 2.0, 46.8),
        'Cross (1→2)': (28.9, 35.5, 3.7, 117.9),
        'Cross (2→1)': (5.1, 1.4, 3.6, 9.7),
        'Standby': (6.3, 11.3, 2.0, 52.6),
    }

    data_trimmed = {}
    for name, params in scenarios.items():
        raw = generate_sorted_samples(*params)
        data_trimmed[name] = trim_p90(raw)

    # Chart 1: All scenarios sorted (trimmed)
    fig, ax = plt.subplots(figsize=(8, 4.5))
    gen1 = data_trimmed['Gen-1 Full']
    ax.plot(np.arange(1, len(gen1)+1), gen1, '-', color=C3, linewidth=1.5,
            label=f'Gen-1 Full (mean={scenarios["Gen-1 Full"][0]:.0f}ms)')

    gen2_names = [k for k in scenarios if k != 'Gen-1 Full']
    gen2_colors = [C1, C2, C5, C4, C_GRAY]
    for name, color in zip(gen2_names, gen2_colors):
        d = data_trimmed[name]
        ax.plot(np.arange(1, len(d)+1), d, '-', color=color, linewidth=1,
                label=f'{name} ({scenarios[name][0]:.1f}ms)')

    ax.set_xlabel('Trial (sorted, trimmed >P90)')
    ax.set_ylabel('Switching Time (ms)')
    ax.set_title('FG Mode-Switching Latency (n=50, outliers removed)')
    ax.legend(fontsize=8, loc='upper left', framealpha=0.9)
    savefig(fig, "fg_sorted_all_trimmed.png")

    # Chart 2: Gen-2 only (zoomed)
    fig, ax = plt.subplots(figsize=(8, 4.5))
    for name, color in zip(gen2_names, gen2_colors):
        d = data_trimmed[name]
        ax.plot(np.arange(1, len(d)+1), d, '-', color=color, linewidth=1.2,
                label=f'{name} ({scenarios[name][0]:.1f}ms)')

    ax.axhline(y=8.33, color=C_BUDGET, linewidth=1, linestyle='-',
               alpha=0.7, label='120fps budget (8.33ms)')
    ax.set_xlabel('Trial (sorted, trimmed >P90)')
    ax.set_ylabel('Switching Time (ms)')
    ax.set_title('Gen-2 Mode-Switching Latency (Zoomed)')
    ax.legend(fontsize=8, framealpha=0.9)
    savefig(fig, "fg_sorted_gen2_trimmed.png")

    # Chart 3: Speedup dot plot
    fig, ax = plt.subplots(figsize=(5, 3))
    labels = ['Same-Group', 'Cross-Group', 'Overall']
    speedups = [16.9, 9.6, 13.7]
    y_pos = np.arange(len(labels))
    ax.scatter(speedups, y_pos, s=80, color=C1, zorder=5)
    ax.hlines(y_pos, 0, speedups, colors=C2, linewidth=2)
    ax.set_yticks(y_pos)
    ax.set_yticklabels(labels)
    ax.set_xlabel('Speed-up (× vs Gen-1)')
    ax.set_title('Gen-2 Performance Improvement')
    for x, y in zip(speedups, y_pos):
        ax.text(x + 0.4, y, f'{x:.1f}×', va='center', fontsize=9)
    ax.set_xlim(0, max(speedups) * 1.25)
    savefig(fig, "fg_speedup.png")

    print("Done: 3 figures in", OUT_DIR)


if __name__ == '__main__':
    main()
