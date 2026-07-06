#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
generate_charts_v2.py — Test 01: Architecture Diagram + Figure A + Figure C

Outputs (saved to figures_v2/):
  fig0_architecture.png  — Gen-1/2/3 workflow + 5 test scenarios
  figA_generation_comparison.png — Log-scale bar: Gen-1 vs Gen-3, Gen-2 band
  figC_cdf.png           — CDF curves with stats table
"""

import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch
from matplotlib.gridspec import GridSpec
from pathlib import Path

# ── Global style ───────────────────────────────────────────────
matplotlib.rcParams.update({
    'font.family':        'serif',
    'font.size':          9,
    'axes.titlesize':     11,
    'axes.labelsize':     10,
    'xtick.labelsize':    9,
    'ytick.labelsize':    9,
    'figure.facecolor':   'white',
    'axes.facecolor':     'white',
    'axes.spines.top':    False,
    'axes.spines.right':  False,
    'savefig.dpi':        300,
    'figure.dpi':         120,
})

BASE_DIR = Path(__file__).parent
OUT_DIR  = BASE_DIR / 'figures_v2'
OUT_DIR.mkdir(exist_ok=True)

# Colours
C_GEN1   = '#C0392B'   # deep red
C_GEN2   = '#E67E22'   # amber
C_GEN3   = '#27AE60'   # green
C_GRAY   = '#7F8C8D'
C_DARK   = '#2C3E50'
PALETTE5 = ['#2E86AB', '#E84855', '#3BB273', '#F18F01', '#7B2D8B']


def savefig(fig, name: str):
    path = OUT_DIR / name
    fig.savefig(path, bbox_inches='tight', dpi=300)
    plt.close(fig)
    print(f'  Saved: {path}')


# ── Outlier removal (IQR, symmetric) ─────────────────────────
def clean_col(s: pd.Series, k: float = 2.5) -> pd.Series:
    q1, q3 = s.quantile(0.25), s.quantile(0.75)
    iqr = q3 - q1
    lo, hi = q1 - k * iqr, q3 + k * iqr
    return s.where((s >= lo) & (s <= hi))


# ─────────────────────────────────────────────────────────────
# Figure 0: Architecture diagram
# ─────────────────────────────────────────────────────────────
def _box(ax, x, y, w, h, text, fc, ec='white', fs=8, fc_text='white',
         lw=1.4, alpha=0.93, bold=False, mono=False):
    patch = FancyBboxPatch(
        (x, y), w, h,
        boxstyle='round,pad=0.06',
        facecolor=fc, edgecolor=ec,
        linewidth=lw, alpha=alpha, zorder=2
    )
    ax.add_patch(patch)
    family = 'monospace' if mono else None
    ax.text(
        x + w / 2, y + h / 2, text,
        ha='center', va='center',
        fontsize=fs, color=fc_text,
        fontweight='bold' if bold else 'normal',
        family=family,
        multialignment='center',
        linespacing=1.45,
        zorder=3,
        wrap=False,
    )


def _arrow(ax, x1, y1, x2, y2, color='#555555', lw=1.4):
    ax.annotate(
        '', xy=(x2, y2), xytext=(x1, y1),
        arrowprops=dict(arrowstyle='->', color=color,
                        lw=lw, mutation_scale=11),
        zorder=4
    )


def make_architecture_diagram():
    fig = plt.figure(figsize=(16, 10.5))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, 16)
    ax.set_ylim(0, 10.5)
    ax.axis('off')
    fig.patch.set_facecolor('white')

    # ── Title ──────────────────────────────────────────────────
    ax.text(8, 10.1,
            'Function Generator Mode-Switching Strategy: Gen-1 → Gen-2 → Gen-3',
            ha='center', va='center', fontsize=13, fontweight='bold', color=C_DARK)

    # ── Column geometry ────────────────────────────────────────
    CX  = [0.25, 5.65, 11.05]   # left-edge x of each column
    CW  = 4.55                   # column width
    BH  = 0.72                   # standard box height
    GAP = 0.22                   # vertical gap between boxes

    gen_colors = [C_GEN1, C_GEN2, C_GEN3]
    gen_titles = [
        'Gen-1\n(Full Re-upload per Switch)',
        'Gen-2\n(Preloaded · Multi-write + *WAI)',
        'Gen-3\n(Preloaded · Single Compound Write)',
    ]
    gen_times = ['≈ 268 ms', '≈ 2 – 40 ms', '< 1 ms']

    # ── Column headers ─────────────────────────────────────────
    for i in range(3):
        _box(ax, CX[i], 9.0, CW, 0.82,
             gen_titles[i], gen_colors[i],
             fs=9.5, bold=True)
        ax.text(CX[i] + CW / 2, 8.76,
                f'Typical latency: {gen_times[i]}',
                ha='center', va='center',
                fontsize=8, color=gen_colors[i],
                fontstyle='italic', fontweight='bold')

    # ─── Gen-1 steps ──────────────────────────────────────────
    g1_steps = [
        (C_DARK,    'Mode Switch Triggered'),
        (C_GEN1,    'OUTP1 OFF / OUTP2 OFF\n*WAI'),
        ('#7B241C',  'Read CSV Waveform Files\nfrom Disk  ← bottleneck'),
        ('#922B21',  'Upload Waveforms\nSOUR1:DATA:ARB  /  SOUR2:DATA:ARB\n*WAI  ← ~200 ms'),
        (C_GEN1,    'Configure 16 SCPI Parameters\n(FUNC, ARB, SRAT, FREQ, VOLT …)\n+ multiple *WAI'),
        (C_GEN1,    'OUTP1 ON / OUTP2 ON\n*WAI'),
    ]
    y = 8.35
    for j, (fc, txt) in enumerate(g1_steps):
        h = BH * 1.35 if j in (3, 4) else BH
        _box(ax, CX[0], y - h, CW, h, txt, fc, fs=8)
        if j > 0:
            _arrow(ax, CX[0] + CW / 2, y, CX[0] + CW / 2, y - GAP * 0.55)
        y -= h + GAP

    # ─── Gen-2 steps ──────────────────────────────────────────
    # One-time preload header
    _box(ax, CX[1], 8.35 - BH, CW, BH,
         'Startup: Preload All Waveforms Once\n(one-time cost — not counted in latency)',
         C_GRAY, fs=7.8)
    ax.text(CX[1] + CW / 2, 8.35 - BH - 0.18,
            '↓  per-switch path below',
            ha='center', fontsize=7.5, color='#888888', fontstyle='italic')

    g2_steps = [
        (C_DARK,  'Mode Switch Triggered'),
        (C_GEN2,  'OUTP1 OFF / OUTP2 OFF'),
        (C_GEN2,  'Re-select ARB waveform\nSet SRAT, FREQ, PHAS:SYNC\n*WAI  ←  USB write #1\n[cross-group only]'),
        ('#D35400', '5 ms inter-write delay\n(wait for *WAI to settle)'),
        (C_GEN2,  'Set Polarity (OUTP:POL)\nOUTP1 ON / OUTP2 ON\n*WAI  ←  USB write #2'),
    ]
    y = 7.25
    for j, (fc, txt) in enumerate(g2_steps):
        h = BH * 1.35 if j in (2, 3, 4) else BH
        _box(ax, CX[1], y - h, CW, h, txt, fc, fs=7.8)
        if j > 0:
            _arrow(ax, CX[1] + CW / 2, y, CX[1] + CW / 2, y - GAP * 0.55)
        y -= h + GAP

    # ─── Gen-3 steps ──────────────────────────────────────────
    _box(ax, CX[2], 8.35 - BH, CW, BH,
         'Startup: Preload All Waveforms Once\n(one-time cost — not counted in latency)',
         C_GRAY, fs=7.8)
    ax.text(CX[2] + CW / 2, 8.35 - BH - 0.18,
            '↓  per-switch path below',
            ha='center', fontsize=7.5, color='#888888', fontstyle='italic')

    _box(ax, CX[2], 7.25 - BH, CW, BH,
         'Mode Switch Triggered', C_DARK, fs=8)
    _arrow(ax, CX[2] + CW / 2, 7.25 - BH,
           CX[2] + CW / 2, 7.25 - BH - GAP * 0.6)

    # Big single-write box
    cmd_txt = (
        'Single USB Write  (all cmds semicoloned):\n\n'
        '"OUTP1 OFF ; OUTP2 OFF ;\n'
        ' SOUR1:FUNC:ARB {name1} ; SOUR2:FUNC:ARB {name2} ;\n'
        ' SOUR1:FUNC:ARB:SRAT {srate} ; SOUR2:FUNC:ARB:SRAT {srate} ;\n'
        ' SOUR1:FREQ {freq} ; SOUR2:FREQ {freq} ;\n'
        ' OUTP1:POL {pol1} ; OUTP2:POL {pol2} ;\n'
        ' OUTP1 ON ; OUTP2 ON ; *WAI"'
    )
    box_top = 7.25 - BH - GAP * 0.6
    box_h   = 3.15
    _box(ax, CX[2], box_top - box_h, CW, box_h,
         cmd_txt, C_GEN3, fs=7.6, mono=True, lw=2)

    ax.text(CX[2] + CW / 2, box_top - box_h - 0.26,
            '→ Eliminates per-call USB overhead entirely',
            ha='center', fontsize=8, color=C_GEN3,
            fontweight='bold', fontstyle='italic')

    # ── Speed-up arrows between columns ───────────────────────
    mid_y = 5.8
    for xi, label, color in [
        (CX[0] + CW + 0.04, '~10 – 80×\nfaster', C_GEN2),
        (CX[1] + CW + 0.04, '~5 – 10×\nfaster', C_GEN3),
    ]:
        next_x = xi + (CX[1] - CX[0] - CW - 0.12)
        ax.annotate(
            '', xy=(xi + 0.85, mid_y), xytext=(xi, mid_y),
            arrowprops=dict(arrowstyle='->', color=color, lw=2.0,
                            mutation_scale=14),
            zorder=5
        )
        ax.text(xi + 0.42, mid_y + 0.25, label,
                ha='center', fontsize=8, color=color,
                fontweight='bold', multialignment='center')

    # ── 5 Scenarios banner ────────────────────────────────────
    banner_y = 2.12
    _box(ax, 0.15, banner_y, 15.7, 0.45,
         '5 Benchmark Scenarios  (Gen-3,  n = 100 trials each,  latency budget: 5 ms)',
         '#1A252F', ec='none', fs=9.5, bold=True)

    scenarios = [
        ('S1', 'Same-Group\nPolarity Switch\n(Mode 1 ↔ 3)',
         '25 kHz waveform\nremains — only\npolarities flip'),
        ('S2', 'Same-Group\nPolarity Switch\n(Mode 2 ↔ 4)',
         '47 kHz waveform\nremains — only\npolarities flip'),
        ('S3', 'Cross-Group\nWaveform Switch\n(1 → 2,  25k → 47k)',
         'Waveform set\nchanges from\n25 kHz to 47 kHz'),
        ('S4', 'Cross-Group\nWaveform Switch\n(2 → 1,  47k → 25k)',
         'Waveform set\nchanges from\n47 kHz to 25 kHz'),
        ('S5', 'Voltage Adjust\n(PID Routine)',
         'Amplitude-only\nupdate — no\nwaveform change'),
    ]
    sc_colors = ['#2471A3', '#2471A3', '#1A5276', '#1A5276', '#6C3483']
    sw = 2.86
    sh = 1.72
    gap_s = 0.195
    sx0 = (16 - 5 * sw - 4 * gap_s) / 2
    sy = 0.18

    for k, (sid, title, desc) in enumerate(scenarios):
        sx = sx0 + k * (sw + gap_s)
        _box(ax, sx, sy, sw, sh, '', sc_colors[k], lw=1.2)
        ax.text(sx + sw / 2, sy + sh * 0.68, title,
                ha='center', va='center', fontsize=8,
                color='white', fontweight='bold',
                multialignment='center', linespacing=1.4)
        ax.text(sx + sw / 2, sy + sh * 0.24, desc,
                ha='center', va='center', fontsize=7,
                color='#D5E8F5',
                multialignment='center', linespacing=1.35,
                fontstyle='italic')
        ax.text(sx + 0.12, sy + sh - 0.13, sid,
                ha='left', va='top', fontsize=9,
                color='white', fontweight='bold', alpha=0.7)

    savefig(fig, 'fig0_architecture.png')


# ─────────────────────────────────────────────────────────────
# Figure A: Gen-1 vs Gen-3 bar chart (log scale) + Gen-2 band
# ─────────────────────────────────────────────────────────────
def make_figure_a(df_clean: pd.DataFrame):
    COLS  = ['same_group_1_3_ms', 'same_group_2_4_ms',
             'cross_group_1_to_2_ms', 'cross_group_2_to_1_ms',
             'voltage_adjust_ms']
    XLABELS = ['Same-Group\n(1↔3)', 'Same-Group\n(2↔4)',
               'Cross-Group\n(1→2)', 'Cross-Group\n(2→1)',
               'Voltage\nAdjust']

    gen3_mean = np.array([df_clean[c].mean() for c in COLS])
    gen3_p99  = np.array([df_clean[c].quantile(0.99) for c in COLS])
    gen3_std  = np.array([df_clean[c].std() for c in COLS])

    # Gen-1: fixed 268 ms for all scenarios (always full re-upload)
    GEN1_MEAN = 268.0
    # Gen-2 band range from benchmark analysis (not raw-data)
    GEN2_LO, GEN2_HI = 2.0, 40.0

    x  = np.arange(len(XLABELS))
    w  = 0.32

    fig, ax = plt.subplots(figsize=(10, 5.8))

    # ── Gen-2 reference band ──────────────────────────────────
    ax.axhspan(GEN2_LO, GEN2_HI,
               color=C_GEN2, alpha=0.13, zorder=0,
               label=f'Gen-2 range  ({GEN2_LO}–{GEN2_HI} ms, multi-write + *WAI)')
    ax.axhline(GEN2_HI, color=C_GEN2, linewidth=0.8, linestyle='--', alpha=0.5)
    ax.axhline(GEN2_LO, color=C_GEN2, linewidth=0.8, linestyle='--', alpha=0.5)

    # ── Gen-1 bars ────────────────────────────────────────────
    bars1 = ax.bar(x - w / 2, [GEN1_MEAN] * len(x), w,
                   label=f'Gen-1  (mean = {GEN1_MEAN:.0f} ms, full re-upload)',
                   color=C_GEN1, alpha=0.82, edgecolor='white', linewidth=0.6,
                   zorder=3)

    # ── Gen-3 bars ────────────────────────────────────────────
    bars3 = ax.bar(x + w / 2, gen3_mean, w,
                   label='Gen-3  (compound write, measured)',
                   color=C_GEN3, alpha=0.88, edgecolor='white', linewidth=0.6,
                   zorder=3,
                   yerr=gen3_std, error_kw=dict(ecolor='#1E8449', capsize=4,
                                                 elinewidth=1.2, capthick=1.2))

    # ── Value labels on Gen-3 bars ───────────────────────────
    for xi, (m, p) in enumerate(zip(gen3_mean, gen3_p99)):
        ax.text(xi + w / 2, m * 1.65,
                f'{m:.3f} ms\n(P99={p:.2f})',
                ha='center', va='bottom',
                fontsize=7.5, color='#1E8449', fontweight='bold',
                multialignment='center', linespacing=1.3)

    # ── Gen-1 label on first bar ──────────────────────────────
    ax.text(0 - w / 2, GEN1_MEAN * 1.12,
            '268 ms', ha='center', va='bottom',
            fontsize=8, color=C_GEN1, fontweight='bold')

    # ── Log scale & decoration ────────────────────────────────
    ax.set_yscale('log')
    ax.set_ylim(0.08, 2000)
    ax.yaxis.grid(True, which='both', alpha=0.25, linestyle='--', zorder=0)
    ax.set_axisbelow(True)
    ax.set_xticks(x)
    ax.set_xticklabels(XLABELS)
    ax.set_xlabel('Scenario')
    ax.set_ylabel('Mode-Switching Latency (ms)  [log scale]')
    ax.set_title('FG Mode-Switching Latency: Gen-1 vs Gen-2 vs Gen-3',
                 fontweight='bold', pad=10)

    # ── Speedup annotation box ────────────────────────────────
    overall_speedup = GEN1_MEAN / gen3_mean.mean()
    annot_txt = (f'Speedup  Gen-3 vs Gen-1:\n'
                 f'  mean  {overall_speedup:.0f}×\n'
                 f'  max   {GEN1_MEAN / gen3_mean.min():.0f}×')
    ax.text(0.985, 0.97, annot_txt,
            transform=ax.transAxes,
            fontsize=9, fontweight='bold', color=C_DARK,
            va='top', ha='right',
            bbox=dict(boxstyle='round,pad=0.45',
                      facecolor='#F8F9F9', edgecolor='#BDC3C7', alpha=0.92))

    ax.legend(loc='upper left', fontsize=8.5, framealpha=0.92, edgecolor='#BDC3C7')
    plt.tight_layout()
    savefig(fig, 'figA_generation_comparison.png')


# ─────────────────────────────────────────────────────────────
# Figure C: CDF + stats table
# ─────────────────────────────────────────────────────────────
def make_figure_c(df_clean: pd.DataFrame):
    COLS   = ['same_group_1_3_ms', 'same_group_2_4_ms',
              'cross_group_1_to_2_ms', 'cross_group_2_to_1_ms',
              'voltage_adjust_ms']
    LABELS = ['Same-Group (1↔3)', 'Same-Group (2↔4)',
              'Cross-Group (1→2)', 'Cross-Group (2→1)',
              'Voltage Adjust']

    fig, ax = plt.subplots(figsize=(9, 5.5))

    p50_all, p99_all = [], []

    for col, lbl, clr in zip(COLS, LABELS, PALETTE5):
        data = np.sort(df_clean[col].dropna().values)
        n    = len(data)
        cdf  = np.arange(1, n + 1) / n
        ax.plot(data, cdf, '-', color=clr, linewidth=2.2, label=lbl, zorder=3)

        p50 = np.percentile(data, 50)
        p99 = np.percentile(data, 99)
        p50_all.append(p50)
        p99_all.append(p99)

        ax.plot(p50, 0.50, 'o', color=clr, markersize=6, zorder=5,
                markeredgecolor='white', markeredgewidth=0.8)
        ax.plot(p99, 0.99, 's', color=clr, markersize=6, zorder=5,
                markeredgecolor='white', markeredgewidth=0.8)

    # ── P50 / P99 reference lines ─────────────────────────────
    ax.axhline(0.50, color='#888888', linestyle=':',  linewidth=1.1, alpha=0.8, zorder=1)
    ax.axhline(0.99, color='#888888', linestyle='--', linewidth=1.1, alpha=0.8, zorder=1)
    ax.text(ax.get_xlim()[1] * 0.98 if ax.get_xlim()[1] > 0 else 3.5,
            0.505, 'P50', va='bottom', ha='right', fontsize=8, color='#888888')
    ax.text(ax.get_xlim()[1] * 0.98 if ax.get_xlim()[1] > 0 else 3.5,
            0.995, 'P99', va='top',    ha='right', fontsize=8, color='#888888')

    # ── Decoration ────────────────────────────────────────────
    ax.set_xlabel('Switching Latency (ms)')
    ax.set_ylabel('Cumulative Probability')
    ax.set_title('CDF of Gen-3 FG Mode-Switching Latency  (Outliers Removed)',
                 fontweight='bold', pad=10)
    ax.yaxis.set_major_formatter(
        matplotlib.ticker.FuncFormatter(lambda y, _: f'{y*100:.0f}%'))
    ax.set_xlim(left=0)
    ax.set_ylim(-0.01, 1.03)
    ax.yaxis.grid(True, alpha=0.25, linestyle='--', zorder=0)
    ax.set_axisbelow(True)
    ax.legend(loc='lower right', fontsize=8.5, framealpha=0.92,
              edgecolor='#BDC3C7')

    # ── Inset stats table ─────────────────────────────────────
    short_names = ['SG 1↔3', 'SG 2↔4', 'CG 1→2', 'CG 2→1', 'Volt.']
    col_headers = ['Scenario', 'n', 'Mean', 'Median', 'P99', 'Max']
    rows = []
    for col, sn in zip(COLS, short_names):
        d = df_clean[col].dropna()
        rows.append([
            sn, str(len(d)),
            f'{d.mean():.3f}',
            f'{d.median():.3f}',
            f'{d.quantile(0.99):.3f}',
            f'{d.max():.3f}',
        ])

    tbl = ax.table(
        cellText=rows,
        colLabels=col_headers,
        loc='upper left',
        bbox=[0.01, 0.34, 0.52, 0.40],
    )
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(7.5)
    for (r, c), cell in tbl.get_celld().items():
        cell.set_edgecolor('#CCCCCC')
        if r == 0:
            cell.set_facecolor('#2C3E50')
            cell.set_text_props(color='white', fontweight='bold')
        elif r % 2 == 0:
            cell.set_facecolor('#F2F3F4')
        else:
            cell.set_facecolor('white')
    ax.text(0.015, 0.755, 'Statistics after outlier removal  (unit: ms)',
            transform=ax.transAxes, fontsize=7.5, color='#555555',
            fontstyle='italic')

    plt.tight_layout()
    savefig(fig, 'figC_cdf.png')


# ─────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────
def main():
    csv_path = BASE_DIR / 'fg_switching_gen3_raw.csv'
    df = pd.read_csv(csv_path)

    COLS = ['same_group_1_3_ms', 'same_group_2_4_ms',
            'cross_group_1_to_2_ms', 'cross_group_2_to_1_ms',
            'voltage_adjust_ms']

    # ── Outlier removal (IQR k=2.5) ──────────────────────────
    df_clean = df.copy()
    print('Outlier removal (IQR k=2.5):')
    total_removed = 0
    for col in COLS:
        before = df_clean[col].notna().sum()
        df_clean[col] = clean_col(df_clean[col], k=2.5)
        removed = before - df_clean[col].notna().sum()
        total_removed += removed
        if removed:
            print(f'  {col}: removed {removed} outlier(s)')
    print(f'  Total removed: {total_removed} / {len(df) * len(COLS)} observations\n')

    # ── Print clean stats ─────────────────────────────────────
    print('Clean statistics (ms):')
    print(f'  {"Scenario":<30}  {"n":>4}  {"mean":>7}  {"median":>7}  '
          f'{"P99":>7}  {"max":>7}')
    print('  ' + '-' * 64)
    for col in COLS:
        d = df_clean[col].dropna()
        print(f'  {col:<30}  {len(d):>4}  {d.mean():>7.3f}  '
              f'{d.median():>7.3f}  {d.quantile(0.99):>7.3f}  {d.max():>7.3f}')
    print()

    # ── Generate figures ──────────────────────────────────────
    print('Generating figures...')
    make_architecture_diagram()
    make_figure_a(df_clean)
    make_figure_c(df_clean)
    print(f'\nAll figures saved to: {OUT_DIR}')


if __name__ == '__main__':
    main()
