#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Benchmark 01: Function Generator Switching - Three Generations Comparison

Gen-1: Full re-upload per switch (~260ms)
Gen-2: Preloaded, multiple sequential writes (~2-30ms)
Gen-3: Single compound SCPI write (<1ms)
"""

import os
import sys
import time
import csv
import numpy as np
from pathlib import Path
from datetime import datetime

PROJECT_ROOT = str(Path(__file__).resolve().parents[2])
sys.path.insert(0, PROJECT_ROOT)

from config import FG_RESOURCE_STRING, FG_MODE_CONFIGS, HAVE_VISA

if not HAVE_VISA:
    print("ERROR: PyVISA not available.")
    sys.exit(1)

import pyvisa as visa

N_TRIALS = 50
N_WARMUP = 10
BUDGET_MS = 5.0


# -- Waveform Utilities --

def load_waveform_csv(filename):
    filepath = os.path.join(PROJECT_ROOT, filename)
    times, values = [], []
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split(',') if ',' in line else line.split()
            if len(parts) >= 2:
                try:
                    times.append(float(parts[0]))
                    values.append(float(parts[1]))
                except ValueError:
                    continue
    return np.array(times), np.array(values)


def align_waveforms(file1, file2):
    t1, v1 = load_waveform_csv(file1)
    t2, v2 = load_waveform_csv(file2)
    t_start = max(t1[0], t2[0])
    t_end = min(t1[-1], t2[-1])
    dt = min(np.mean(np.diff(t1)), np.mean(np.diff(t2)))
    t_unified = np.arange(t_start, t_end + dt, dt)
    s1 = np.interp(t_unified, t1, v1).astype('f4')
    s2 = np.interp(t_unified, t2, v2).astype('f4')
    srate = 1.0 / dt
    return s1, s2, srate, len(t_unified)


def wait_ready(inst):
    """Wait for instrument to be ready (replaces *OPC? which can timeout)."""
    inst.write('*WAI')
    time.sleep(0.3)


def preload_all_waveforms(inst):
    inst.write('SOUR1:DATA:VOL:CLE')
    inst.write('SOUR2:DATA:VOL:CLE')
    inst.write('*WAI')
    time.sleep(0.5)
    sampling_rates = {}
    uploaded = set()
    for mode_num, config in FG_MODE_CONFIGS.items():
        sig1, sig2, srate, npts = align_waveforms(config['file1'], config['file2'])
        freq = srate / npts
        sampling_rates[mode_num] = {
            'sRate': srate, 'points': npts, 'freq': freq,
            'name1': config['name1'], 'name2': config['name2']
        }
        wave_key = config['name1'][:5]
        if wave_key not in uploaded:
            inst.write_binary_values(
                f'SOUR1:DATA:ARB {config["name1"]},', sig1, datatype='f', is_big_endian=False)
            inst.write('*WAI')
            inst.write_binary_values(
                f'SOUR2:DATA:ARB {config["name2"]},', sig2, datatype='f', is_big_endian=False)
            inst.write('*WAI')
            uploaded.add(wave_key)
    wait_ready(inst)
    return sampling_rates


# -- Gen-1: Full re-upload per switch --

def gen1_switch(inst, mode_num):
    file1 = FG_MODE_CONFIGS[mode_num]['file1']
    file2 = FG_MODE_CONFIGS[mode_num]['file2']
    ch1_pol = FG_MODE_CONFIGS[mode_num]['ch1_pol']
    ch2_pol = FG_MODE_CONFIGS[mode_num]['ch2_pol']

    inst.write('OUTP1 OFF')
    inst.write('OUTP2 OFF')
    inst.write('SOUR2:TRACK OFF')
    inst.write('*WAI')

    sig1, sig2, srate, npts = align_waveforms(file1, file2)

    inst.write('SOUR1:DATA:VOL:CLE')
    inst.write_binary_values('SOUR1:DATA:ARB MODAL_CH1,', sig1, datatype='f', is_big_endian=False)
    inst.write('*WAI')
    inst.write('SOUR2:DATA:VOL:CLE')
    inst.write_binary_values('SOUR2:DATA:ARB MODAL_CH2,', sig2, datatype='f', is_big_endian=False)
    inst.write('*WAI')

    freq = srate / npts
    inst.write('SOUR1:FUNC ARB')
    inst.write('SOUR1:FUNC:ARB MODAL_CH1')
    inst.write(f'SOUR1:FUNC:ARB:SRAT {srate:.0f}')
    inst.write('SOUR1:VOLT 1.2')
    inst.write('SOUR1:VOLT:OFFS 0')
    inst.write('SOUR2:FUNC ARB')
    inst.write('SOUR2:FUNC:ARB MODAL_CH2')
    inst.write(f'SOUR2:FUNC:ARB:SRAT {srate:.0f}')
    inst.write('SOUR2:VOLT 1.2')
    inst.write('SOUR2:VOLT:OFFS 0')
    inst.write(f'SOUR1:FREQ {freq}')
    inst.write(f'SOUR2:FREQ {freq}')
    inst.write('SOUR1:PHAS 0')
    inst.write('SOUR2:PHAS 0')
    inst.write('*WAI')
    inst.write('SOUR1:TRACK OFF')
    inst.write('SOUR2:TRACK OFF')
    inst.write('SOUR2:TRACK ON')
    inst.write('SOUR2:PHAS:SYNC')
    inst.write('*WAI')
    inst.write(f'OUTP1:POL {ch1_pol}')
    inst.write(f'OUTP2:POL {ch2_pol}')
    inst.write('OUTP1 ON')
    inst.write('OUTP2 ON')
    inst.write('*WAI')


# -- Gen-2: Preloaded, multiple writes --

def gen2_setup(inst, sampling_rates, mode_num):
    config = FG_MODE_CONFIGS[mode_num]
    mode_data = sampling_rates[mode_num]
    inst.write('SOUR1:FUNC ARB')
    inst.write('SOUR2:FUNC ARB')
    inst.write(f'SOUR1:FUNC:ARB {mode_data["name1"]}')
    inst.write(f'SOUR2:FUNC:ARB {mode_data["name2"]}')
    inst.write(f'SOUR1:FUNC:ARB:SRAT {mode_data["sRate"]:.0f}')
    inst.write(f'SOUR2:FUNC:ARB:SRAT {mode_data["sRate"]:.0f}')
    inst.write(f'SOUR1:FREQ {mode_data["freq"]}')
    inst.write(f'SOUR2:FREQ {mode_data["freq"]}')
    inst.write('SOUR1:PHAS 0')
    inst.write('SOUR2:PHAS 0')
    inst.write('SOUR1:VOLT 1.2')
    inst.write('SOUR2:VOLT 1.2')
    inst.write('SOUR1:VOLT:OFFS 0')
    inst.write('SOUR2:VOLT:OFFS 0')
    inst.write('*WAI')
    inst.write('SOUR1:TRACK OFF')
    inst.write('SOUR2:TRACK OFF')
    inst.write('SOUR2:TRACK ON')
    inst.write('SOUR2:PHAS:SYNC')
    inst.write('*WAI')
    inst.write(f'OUTP1:POL {config["ch1_pol"]}')
    inst.write(f'OUTP2:POL {config["ch2_pol"]}')
    inst.write('*WAI')
    inst.write('OUTP1 ON')
    inst.write('OUTP2 ON')
    inst.write('*WAI')
    time.sleep(0.3)


def gen2_same_group_switch(inst, mode_num):
    """Gen-2 style: two separate USB writes (OFF+POL then ON+WAI)."""
    config = FG_MODE_CONFIGS[mode_num]
    inst.write(f'OUTP1 OFF;OUTP2 OFF;OUTP1:POL {config["ch1_pol"]};OUTP2:POL {config["ch2_pol"]}')
    inst.write('OUTP1 ON;OUTP2 ON;*WAI')


def gen2_cross_group_switch(inst, sampling_rates, mode_num):
    """Gen-2 style: two separate USB writes with *WAI between them."""
    config = FG_MODE_CONFIGS[mode_num]
    mode_data = sampling_rates[mode_num]
    # First batch: turn off + reconfigure waveform
    inst.write(
        f'OUTP1 OFF;OUTP2 OFF;'
        f'SOUR1:FUNC:ARB {mode_data["name1"]};SOUR2:FUNC:ARB {mode_data["name2"]};'
        f'SOUR1:FUNC:ARB:SRAT {mode_data["sRate"]:.0f};SOUR2:FUNC:ARB:SRAT {mode_data["sRate"]:.0f};'
        f'SOUR1:FREQ {mode_data["freq"]};SOUR2:FREQ {mode_data["freq"]};'
        f'SOUR2:PHAS:SYNC;*WAI'
    )
    time.sleep(0.005)  # Wait for *WAI to complete before next write
    # Second batch: set polarity + turn on
    inst.write(
        f'OUTP1:POL {config["ch1_pol"]};OUTP2:POL {config["ch2_pol"]};'
        f'OUTP1 ON;OUTP2 ON;*WAI'
    )


# -- Gen-3: Single compound SCPI write --

def gen3_setup(inst, sampling_rates, mode_num):
    gen2_setup(inst, sampling_rates, mode_num)


def gen3_same_group_switch(inst, mode_num):
    config = FG_MODE_CONFIGS[mode_num]
    inst.write(
        f'OUTP1 OFF;OUTP2 OFF;'
        f'OUTP1:POL {config["ch1_pol"]};OUTP2:POL {config["ch2_pol"]};'
        f'OUTP1 ON;OUTP2 ON;*WAI'
    )


def gen3_cross_group_switch(inst, sampling_rates, mode_num):
    config = FG_MODE_CONFIGS[mode_num]
    mode_data = sampling_rates[mode_num]
    inst.write(
        f'OUTP1 OFF;OUTP2 OFF;'
        f'SOUR1:FUNC:ARB {mode_data["name1"]};SOUR2:FUNC:ARB {mode_data["name2"]};'
        f'SOUR1:FUNC:ARB:SRAT {mode_data["sRate"]:.0f};SOUR2:FUNC:ARB:SRAT {mode_data["sRate"]:.0f};'
        f'SOUR1:FREQ {mode_data["freq"]};SOUR2:FREQ {mode_data["freq"]};'
        f'SOUR2:PHAS:SYNC;'
        f'OUTP1:POL {config["ch1_pol"]};OUTP2:POL {config["ch2_pol"]};'
        f'OUTP1 ON;OUTP2 ON;*WAI'
    )


def gen3_voltage_adjust(inst, v1, v2):
    inst.write(f'SOUR1:VOLT {v1:.3f};SOUR2:VOLT {v2:.3f}')


# -- Runner --

def run_timed(fn, n, warmup=N_WARMUP, inter_delay=0.05):
    for _ in range(warmup):
        fn()
        time.sleep(inter_delay)
    time.sleep(1.0)
    times = []
    for _ in range(n):
        t0 = time.perf_counter()
        fn()
        t1 = time.perf_counter()
        times.append((t1 - t0) * 1000.0)
        time.sleep(inter_delay)
    return times


def stats_str(arr):
    a = np.array(arr)
    return (f"mean={np.mean(a):7.2f}ms  std={np.std(a):6.2f}ms  "
            f"min={np.min(a):6.2f}ms  max={np.max(a):7.2f}ms  "
            f"p95={np.percentile(a,95):6.2f}ms  p99={np.percentile(a,99):6.2f}ms")


# -- Main --

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(PROJECT_ROOT)

    print("=" * 72)
    print("  Benchmark 01: FG Switching - Gen-1 vs Gen-2 vs Gen-3")
    print(f"  Device: {FG_RESOURCE_STRING}")
    print(f"  Trials: {N_TRIALS} per scenario (+ {N_WARMUP} warmup)")
    print(f"  Timestamp: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 72)

    rm = visa.ResourceManager()
    inst = rm.open_resource(FG_RESOURCE_STRING)
    inst.timeout = 60000
    try:
        inst.control_ren(6)
    except Exception:
        pass
    idn = inst.query('*IDN?').strip()
    print(f"  Connected: {idn}\n")

    inst.write('OUTP1 OFF;OUTP2 OFF')
    inst.write('SOUR1:TRACK OFF;SOUR2:TRACK OFF')
    inst.write('*CLS')
    inst.write('FORM:BORD SWAP')
    wait_ready(inst)

    # Preload
    print("Preloading waveforms...")
    sampling_rates = preload_all_waveforms(inst)
    print("  Done.\n")

    # === GEN-1 ===
    print("-" * 72)
    print("  GEN-1: Full re-upload per switch")
    print("-" * 72)
    gen1_times = run_timed(lambda: gen1_switch(inst, 1), N_TRIALS, warmup=3)
    print(f"  {stats_str(gen1_times)}")

    # Reset for Gen-2/3
    inst.write('OUTP1 OFF;OUTP2 OFF;*WAI')
    inst.write('SOUR1:DATA:VOL:CLE')
    inst.write('SOUR2:DATA:VOL:CLE')
    inst.write('*WAI')
    time.sleep(2)

    print("\nRe-preloading waveforms...")
    sampling_rates = preload_all_waveforms(inst)
    print("  Done.\n")

    # === GEN-2 ===
    print("-" * 72)
    print("  GEN-2: Preloaded, multiple sequential writes")
    print("-" * 72)

    # Same-group
    gen2_setup(inst, sampling_rates, 1)
    cur = [1]

    def g2_same():
        t = 3 if cur[0] == 1 else 1
        gen2_same_group_switch(inst, t)
        cur[0] = t

    gen2_same_times = run_timed(g2_same, N_TRIALS)
    print(f"  Same-group: {stats_str(gen2_same_times)}")

    # Cross-group (alternate 1->2->1->2...)
    gen2_setup(inst, sampling_rates, 1)
    cur[0] = 1

    def g2_cross():
        t = 2 if cur[0] == 1 else 1
        gen2_cross_group_switch(inst, sampling_rates, t)
        cur[0] = t

    gen2_cross_times = run_timed(g2_cross, N_TRIALS)
    print(f"  Cross-group: {stats_str(gen2_cross_times)}")

    # === GEN-3 ===
    print("\n" + "-" * 72)
    print("  GEN-3: Single compound SCPI write")
    print("-" * 72)

    # Same-group
    gen3_setup(inst, sampling_rates, 1)
    cur[0] = 1

    def g3_same():
        t = 3 if cur[0] == 1 else 1
        gen3_same_group_switch(inst, t)
        cur[0] = t

    gen3_same_times = run_timed(g3_same, N_TRIALS)
    print(f"  Same-group: {stats_str(gen3_same_times)}")

    # Cross-group (alternate 1->2->1->2...)
    gen3_setup(inst, sampling_rates, 1)
    cur[0] = 1

    def g3_cross():
        t = 2 if cur[0] == 1 else 1
        gen3_cross_group_switch(inst, sampling_rates, t)
        cur[0] = t

    gen3_cross_times = run_timed(g3_cross, N_TRIALS)
    print(f"  Cross-group: {stats_str(gen3_cross_times)}")

    # Voltage only
    gen3_setup(inst, sampling_rates, 1)
    gen3_volt_times = run_timed(
        lambda: gen3_voltage_adjust(inst, 1.0 + 0.3 * np.random.rand(), 1.0 + 0.3 * np.random.rand()),
        N_TRIALS
    )
    print(f"  Voltage adj: {stats_str(gen3_volt_times)}")

    # Shutdown
    inst.write('OUTP1 OFF;OUTP2 OFF;SOUR1:VOLT 0;SOUR2:VOLT 0;*WAI')
    time.sleep(0.5)
    inst.close()
    print("\nInstrument closed.")

    # === Save CSV ===
    csv_path = os.path.join(script_dir, 'fg_switching_3gen_raw.csv')
    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'trial', 'gen1_full_ms', 'gen2_same_ms', 'gen2_cross_ms',
            'gen3_same_ms', 'gen3_cross_ms', 'gen3_voltage_ms'
        ])
        for i in range(N_TRIALS):
            writer.writerow([
                i + 1,
                f"{gen1_times[i]:.4f}",
                f"{gen2_same_times[i]:.4f}",
                f"{gen2_cross_times[i]:.4f}",
                f"{gen3_same_times[i]:.4f}",
                f"{gen3_cross_times[i]:.4f}",
                f"{gen3_volt_times[i]:.4f}",
            ])
    print(f"Raw CSV: {csv_path}")

    # === Report ===
    report = []
    report.append("=" * 72)
    report.append("  Benchmark 01: FG Switching - Gen-1 vs Gen-2 vs Gen-3")
    report.append(f"  Device: {idn}")
    report.append(f"  Trials: {N_TRIALS} (+ {N_WARMUP} warmup)")
    report.append(f"  Timestamp: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    report.append("=" * 72)
    report.append("")

    scenarios = [
        ("Gen-1  Full re-upload", gen1_times),
        ("Gen-2  Same-group (multi-write)", gen2_same_times),
        ("Gen-2  Cross-group (multi-write)", gen2_cross_times),
        ("Gen-3  Same-group (compound)", gen3_same_times),
        ("Gen-3  Cross-group (compound)", gen3_cross_times),
        ("Gen-3  Voltage adjust only", gen3_volt_times),
    ]

    for label, data in scenarios:
        a = np.array(data)
        verdict = "PASS" if np.percentile(a, 99) < BUDGET_MS else "---"
        report.append(f"  {label}")
        report.append(f"    {stats_str(data)}")
        report.append(f"    [{verdict}] (budget={BUDGET_MS}ms)")
        report.append("")

    gen1_mean = np.mean(gen1_times)
    report.append("-" * 72)
    report.append("  Speed-up vs Gen-1:")
    report.append(f"    Gen-1 mean:         {gen1_mean:.1f} ms (baseline)")
    report.append(f"    Gen-2 same-group:   {np.mean(gen2_same_times):.2f} ms ({gen1_mean/np.mean(gen2_same_times):.0f}x)")
    report.append(f"    Gen-2 cross-group:  {np.mean(gen2_cross_times):.2f} ms ({gen1_mean/np.mean(gen2_cross_times):.0f}x)")
    report.append(f"    Gen-3 same-group:   {np.mean(gen3_same_times):.2f} ms ({gen1_mean/np.mean(gen3_same_times):.0f}x)")
    report.append(f"    Gen-3 cross-group:  {np.mean(gen3_cross_times):.2f} ms ({gen1_mean/np.mean(gen3_cross_times):.0f}x)")
    report.append(f"    Gen-3 voltage:      {np.mean(gen3_volt_times):.2f} ms ({gen1_mean/np.mean(gen3_volt_times):.0f}x)")
    report.append("=" * 72)

    report_text = "\n".join(report)
    print(f"\n{report_text}")

    report_path = os.path.join(script_dir, 'fg_switching_3gen_report.txt')
    with open(report_path, 'w', encoding='utf-8') as f:
        f.write(report_text + "\n")
    print(f"\nReport: {report_path}")


if __name__ == "__main__":
    main()
