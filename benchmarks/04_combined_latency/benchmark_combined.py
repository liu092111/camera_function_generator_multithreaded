#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Benchmark 04: Combined Latency (Process + Various FG Commands)

Tests four scenarios combining image processing with different FG operations:
  A: Process + set_voltages (routine PID) -- should PASS
  B: Process + same-group polarity switch -- should PASS with Gen-3
  C: Process + cross-group switch -- may exceed 8.33ms, designed for async
  D: Process only (baseline)

Target: 8.33ms per frame (120fps budget)
"""

import os
import sys
import time
import csv
import numpy as np
import cv2
from pathlib import Path
from datetime import datetime

# Add project root to path
PROJECT_ROOT = str(Path(__file__).resolve().parents[2])
sys.path.insert(0, PROJECT_ROOT)

from config import (
    CAM_WIDTH, CAM_HEIGHT,
    ENABLE_UNDISTORT, FLIP_VERTICAL, FLIP_HORIZONTAL,
    EMA_ALPHA_POS, EMA_ALPHA_ANGLE, CALIBRATION_DATA_PATH,
    FG_RESOURCE_STRING, FG_MODE_CONFIGS, HAVE_VISA
)
from signal_processing import make_kalman, ema
from image_processing import find_target_and_angle
from undistort import UndistortCorrector


# ── Configuration ──────────────────────────────────────────────────────────
N_ROUNDS = 5
N_FRAMES_PER_ROUND = 200
BUDGET_MS = 8.33


# ── FG Gen-3 Controller ───────────────────────────────────────────────────

class FGBenchmarkController:
    """
    Gen-3 FG controller for combined benchmarks.
    Falls back to simulated timing if VISA unavailable.
    """

    def __init__(self):
        self.inst = None
        self.simulated = False
        self.sampling_rates = {}

    def connect(self):
        if not HAVE_VISA:
            self.simulated = True
            print("  FG: simulated mode (no PyVISA)")
            return True
        try:
            import pyvisa as visa
            rm = visa.ResourceManager()
            self.inst = rm.open_resource(FG_RESOURCE_STRING)
            self.inst.timeout = 30000
            try:
                self.inst.control_ren(6)
            except Exception:
                pass
            print(f"  FG: {self.inst.query('*IDN?').strip()}")
            self._preload_and_setup()
            return True
        except Exception as e:
            print(f"  FG connection failed: {e}, using simulated mode")
            self.simulated = True
            return True

    def _preload_and_setup(self):
        """Preload waveforms and set initial state."""
        self.inst.write('OUTP1 OFF;OUTP2 OFF')
        self.inst.write('SOUR1:DATA:VOL:CLE;SOUR2:DATA:VOL:CLE')
        self.inst.write('*WAI')

        uploaded = set()
        for mode_num, config in FG_MODE_CONFIGS.items():
            sig1, sig2, srate, npts = self._align_waveforms(config['file1'], config['file2'])
            freq = srate / npts
            self.sampling_rates[mode_num] = {
                'sRate': srate, 'points': npts, 'freq': freq,
                'name1': config['name1'], 'name2': config['name2']
            }
            wave_key = config['name1'][:5]
            if wave_key not in uploaded:
                self.inst.write_binary_values(
                    f'SOUR1:DATA:ARB {config["name1"]},', sig1,
                    datatype='f', is_big_endian=False)
                self.inst.write('*WAI')
                self.inst.write_binary_values(
                    f'SOUR2:DATA:ARB {config["name2"]},', sig2,
                    datatype='f', is_big_endian=False)
                self.inst.write('*WAI')
                uploaded.add(wave_key)

        # Setup Mode 1 as initial
        md = self.sampling_rates[1]
        self.inst.write('SOUR1:FUNC ARB;SOUR2:FUNC ARB')
        self.inst.write(f'SOUR1:FUNC:ARB {md["name1"]};SOUR2:FUNC:ARB {md["name2"]}')
        self.inst.write(f'SOUR1:FUNC:ARB:SRAT {md["sRate"]:.0f};SOUR2:FUNC:ARB:SRAT {md["sRate"]:.0f}')
        self.inst.write(f'SOUR1:FREQ {md["freq"]};SOUR2:FREQ {md["freq"]}')
        self.inst.write('SOUR1:PHAS 0;SOUR2:PHAS 0')
        self.inst.write('SOUR1:VOLT:OFFS 0;SOUR2:VOLT:OFFS 0')
        self.inst.write('SOUR1:VOLT 1.5;SOUR2:VOLT 1.5')
        self.inst.write('*WAI')
        self.inst.write('SOUR1:TRACK OFF;SOUR2:TRACK OFF')
        self.inst.write('SOUR2:TRACK ON;SOUR2:PHAS:SYNC')
        self.inst.write('*WAI')
        self.inst.write('OUTP1:POL NORM;OUTP2:POL INV')
        self.inst.write('OUTP1 ON;OUTP2 ON')
        self.inst.write('*WAI')
        time.sleep(0.5)

    def _align_waveforms(self, file1, file2):
        t1, v1 = self._load_csv(file1)
        t2, v2 = self._load_csv(file2)
        t_start = max(t1[0], t2[0])
        t_end = min(t1[-1], t2[-1])
        dt = min(np.mean(np.diff(t1)), np.mean(np.diff(t2)))
        t_unified = np.arange(t_start, t_end + dt, dt)
        s1 = np.interp(t_unified, t1, v1).astype('f4')
        s2 = np.interp(t_unified, t2, v2).astype('f4')
        return s1, s2, 1.0 / dt, len(t_unified)

    def _load_csv(self, filename):
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

    def voltage_adjust(self, ch1_v, ch2_v):
        """Gen-3: voltage only, no *WAI."""
        if self.simulated:
            time.sleep(0.0003)
        else:
            ch1_v = max(0.0, min(3.0, ch1_v))
            ch2_v = max(0.0, min(3.0, ch2_v))
            self.inst.write(f'SOUR1:VOLT {ch1_v:.3f};SOUR2:VOLT {ch2_v:.3f}')

    def same_group_polarity_switch(self, mode_num):
        """Gen-3: compound OFF;POL;ON;*WAI in single write."""
        config = FG_MODE_CONFIGS[mode_num]
        if self.simulated:
            time.sleep(0.0004)
        else:
            self.inst.write(
                f'OUTP1 OFF;OUTP2 OFF;'
                f'OUTP1:POL {config["ch1_pol"]};OUTP2:POL {config["ch2_pol"]};'
                f'OUTP1 ON;OUTP2 ON;*WAI'
            )

    def cross_group_switch(self, mode_num):
        """Gen-3: compound command with single *WAI."""
        config = FG_MODE_CONFIGS[mode_num]
        md = self.sampling_rates[mode_num]
        if self.simulated:
            time.sleep(0.003)
        else:
            cmds = [
                'OUTP1 OFF', 'OUTP2 OFF',
                f'SOUR1:FUNC:ARB {md["name1"]}', f'SOUR2:FUNC:ARB {md["name2"]}',
                f'SOUR1:FUNC:ARB:SRAT {md["sRate"]:.0f}', f'SOUR2:FUNC:ARB:SRAT {md["sRate"]:.0f}',
                f'SOUR1:FREQ {md["freq"]}', f'SOUR2:FREQ {md["freq"]}',
                'SOUR2:PHAS:SYNC',
                f'OUTP1:POL {config["ch1_pol"]}', f'OUTP2:POL {config["ch2_pol"]}',
                'OUTP1 ON', 'OUTP2 ON', '*WAI'
            ]
            self.inst.write(';'.join(cmds))

    def disconnect(self):
        if self.inst:
            try:
                self.inst.write('OUTP1 OFF;OUTP2 OFF')
                self.inst.write('SOUR1:VOLT 0;SOUR2:VOLT 0')
                self.inst.write('*WAI')
                self.inst.close()
            except Exception:
                pass


# ── Process Pipeline ──────────────────────────────────────────────────────

class ProcessPipeline:
    """Minimal process pipeline for combined benchmark."""

    def __init__(self):
        self.kf = make_kalman()
        self.ema_x = None
        self.ema_y = None
        self.corrector = None

        if ENABLE_UNDISTORT:
            cal_path = os.path.join(PROJECT_ROOT, CALIBRATION_DATA_PATH)
            self.corrector = UndistortCorrector(cal_path)
            if not self.corrector.load_calibration():
                self.corrector = None

    def process(self, frame):
        """Full process pipeline, returns (fx, fy)."""
        if self.corrector is not None:
            frame = self.corrector.undistort(frame)
        frame_rotated = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        if FLIP_VERTICAL:
            frame_rotated = cv2.flip(frame_rotated, 0)
        if FLIP_HORIZONTAL:
            frame_rotated = cv2.flip(frame_rotated, 1)

        pred = self.kf.predict()
        px_pred, py_pred = float(pred[0, 0]), float(pred[1, 0])

        m = find_target_and_angle(frame_rotated)
        if m is not None:
            cx, cy, angle_deg, box = m
            est = self.kf.correct(np.array([[cx], [cy]], dtype=np.float32))
            fx_raw, fy_raw = float(est[0, 0]), float(est[1, 0])
        else:
            fx_raw, fy_raw = px_pred, py_pred

        self.ema_x = ema(self.ema_x, fx_raw, EMA_ALPHA_POS)
        self.ema_y = ema(self.ema_y, fy_raw, EMA_ALPHA_POS)
        fx = float(self.ema_x) if self.ema_x is not None else px_pred
        fy = float(self.ema_y) if self.ema_y is not None else py_pred
        return fx, fy


# ── Synthetic Frame ───────────────────────────────────────────────────────

def create_frame(idx=0):
    frame = np.zeros((CAM_HEIGHT, CAM_WIDTH, 3), dtype=np.uint8)
    cx = CAM_WIDTH // 2 + int(20 * np.sin(idx * 0.02))
    cy = CAM_HEIGHT // 2 + int(15 * np.cos(idx * 0.015))
    rect = ((cx, cy), (90, 60), 5.0 * np.sin(idx * 0.01))
    box = cv2.boxPoints(rect).astype(int)
    cv2.fillPoly(frame, [box], (0, 255, 255))
    return frame


# ── Main ──────────────────────────────────────────────────────────────────

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(PROJECT_ROOT)

    print("=" * 70)
    print("  Benchmark 04: Combined Latency (Process + FG)")
    print(f"  Rounds: {N_ROUNDS} x {N_FRAMES_PER_ROUND} frames per scenario")
    print(f"  Budget: {BUDGET_MS} ms per frame")
    print(f"  Timestamp: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 70)

    # Setup
    print("\nInitializing...")
    fg = FGBenchmarkController()
    fg.connect()
    pipeline = ProcessPipeline()

    all_results = []  # flat list of all measurements
    round_summaries = {}  # {scenario: [{round, proc_mean, proc_p99, fg_mean}]}

    scenarios_def = [
        ('A_voltage', 'Process + voltage adjust (PID)'),
        ('B_same_group', 'Process + same-group polarity switch'),
        ('C_cross_group', 'Process + cross-group switch'),
        ('D_baseline', 'Process only (baseline)'),
    ]

    for scenario_key, scenario_label in scenarios_def:
        print(f"\n[{scenario_key}] {scenario_label} ({N_ROUNDS} rounds)...")
        round_summaries[scenario_key] = []

        for r in range(N_ROUNDS):
            cur_mode = 1
            round_data = []

            for i in range(N_FRAMES_PER_ROUND):
                frame = create_frame(r * N_FRAMES_PER_ROUND + i)

                t0 = time.perf_counter()
                fx, fy = pipeline.process(frame)
                t_proc = time.perf_counter()

                if scenario_key == 'A_voltage':
                    fg.voltage_adjust(1.5 + 0.01 * (fx - 240), 1.5 - 0.01 * (fx - 240))
                elif scenario_key == 'B_same_group':
                    target = 3 if cur_mode == 1 else 1
                    fg.same_group_polarity_switch(target)
                    cur_mode = target
                elif scenario_key == 'C_cross_group':
                    target = 2 if cur_mode == 1 else 1
                    fg.cross_group_switch(target)
                    cur_mode = target
                # D_baseline: no FG command

                t_end = time.perf_counter()

                row = {
                    'scenario': scenario_key,
                    'round': r + 1,
                    'frame_idx': i,
                    'process_ms': (t_proc - t0) * 1000.0,
                    'fg_ms': (t_end - t_proc) * 1000.0 if scenario_key != 'D_baseline' else 0.0,
                    'total_ms': (t_end - t0) * 1000.0,
                }
                round_data.append(row)

            all_results.extend(round_data)
            procs = [d['process_ms'] for d in round_data]
            fgs = [d['fg_ms'] for d in round_data]
            round_summaries[scenario_key].append({
                'round': r + 1,
                'proc_mean': np.mean(procs),
                'proc_p99': np.percentile(procs, 99),
                'fg_mean': np.mean(fgs),
            })
            print(f"  Round {r+1}: proc={np.mean(procs):.3f}ms  FG={np.mean(fgs):.3f}ms")

    fg.disconnect()

    # ── Save CSV ──────────────────────────────────────────────────────────
    csv_path = os.path.join(script_dir, 'combined_latency_raw.csv')
    fields = ['scenario', 'round', 'frame_idx', 'process_ms', 'fg_ms', 'total_ms']
    with open(csv_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for r in all_results:
            row = {k: (f"{r[k]:.4f}" if isinstance(r[k], float) else r[k]) for k in fields}
            writer.writerow(row)
    print(f"\nRaw data saved: {csv_path}")

    # ── Generate Report ───────────────────────────────────────────────────
    report_lines = []
    report_lines.append("=" * 72)
    report_lines.append("  Benchmark 04: Combined Latency (Process + FG)")
    report_lines.append(f"  Rounds: {N_ROUNDS} x {N_FRAMES_PER_ROUND} frames per scenario")
    report_lines.append(f"  Total frames: {len(all_results)}")
    report_lines.append(f"  Budget: {BUDGET_MS} ms per frame (120fps)")
    report_lines.append(f"  FG: {'Real VISA' if not fg.simulated else 'Simulated'}")
    report_lines.append(f"  Timestamp: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    report_lines.append("=" * 72)

    for scenario_key, scenario_label in scenarios_def:
        scenario_data = [r for r in all_results if r['scenario'] == scenario_key]
        procs = np.array([r['process_ms'] for r in scenario_data])
        fgs = np.array([r['fg_ms'] for r in scenario_data])
        proc_p99 = np.percentile(procs, 99)
        proc_verdict = "PASS" if proc_p99 < BUDGET_MS else "FAIL"

        report_lines.append(f"\n  [{scenario_key}] {scenario_label}")
        report_lines.append(f"    Process:  mean={np.mean(procs):.3f} +/- {np.std(procs):.3f}  p99={proc_p99:.3f}ms  [{proc_verdict}]")
        report_lines.append(f"    FG cmd:   mean={np.mean(fgs):.3f} +/- {np.std(fgs):.3f}ms")

        # Per-round consistency
        summaries = round_summaries[scenario_key]
        proc_means = [s['proc_mean'] for s in summaries]
        report_lines.append(f"    Rounds:   proc mean = {np.mean(proc_means):.3f} +/- {np.std(proc_means):.3f} ms")

    report_lines.append(f"\n{'-'*72}")
    report_lines.append("  Architecture note:")
    report_lines.append("    Process and FG run on separate threads in the real system.")
    report_lines.append("    Process Thread is the critical path for 120fps compliance.")
    report_lines.append("")
    all_proc = np.array([r['process_ms'] for r in all_results])
    proc_overall_p99 = np.percentile(all_proc, 99)
    overall_verdict = "PASS" if proc_overall_p99 < BUDGET_MS else "FAIL"
    report_lines.append(f"  Process Thread Overall: [{overall_verdict}] (P99={proc_overall_p99:.3f}ms < {BUDGET_MS}ms)")
    report_lines.append("=" * 72)

    report_text = "\n".join(report_lines)
    print(f"\n{report_text}")

    report_path = os.path.join(script_dir, 'combined_latency_report.txt')
    with open(report_path, 'w', encoding='utf-8') as f:
        f.write(report_text + "\n")
    print(f"\nReport saved: {report_path}")


if __name__ == "__main__":
    main()
