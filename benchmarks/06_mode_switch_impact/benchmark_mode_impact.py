#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Benchmark 06: Mode Switch Impact on Process Thread (Thread Isolation)

Tests whether FG mode switching affects process thread timing.
Runs process thread continuously while injecting mode switches from another thread.
Marks which frames were processed during a mode switch event.

Target: Process thread P99 < 8.33ms regardless of concurrent FG activity.
"""

import os
import sys
import time
import csv
import threading
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
N_FRAMES_PER_ROUND = 500
SWITCH_INTERVAL_FRAMES = 50  # Inject a mode switch every N frames
BUDGET_MS = 8.33


# ── FG Controller (thread-safe) ───────────────────────────────────────────

class FGSwitchInjector:
    """
    FG controller that injects mode switches from a separate thread.
    Thread-safe with lock protection on VISA writes.
    """

    def __init__(self):
        self.inst = None
        self.simulated = False
        self.lock = threading.Lock()
        self.switch_active = threading.Event()  # Set during a switch
        self.switch_log = []  # (start_time, end_time, mode)

    def connect(self):
        if not HAVE_VISA:
            self.simulated = True
            print("  FG: simulated mode")
            return True
        try:
            import pyvisa as visa
            rm = visa.ResourceManager()
            self.inst = rm.open_resource(FG_RESOURCE_STRING)
            self.inst.timeout = 10000
            try:
                self.inst.control_ren(6)
            except Exception:
                pass
            print(f"  FG: {self.inst.query('*IDN?').strip()}")
            return True
        except Exception as e:
            print(f"  FG failed: {e}, simulated mode")
            self.simulated = True
            return True

    def do_cross_group_switch(self, mode_num):
        """Perform a cross-group switch (the slowest operation)."""
        config = FG_MODE_CONFIGS[mode_num]
        t_start = time.perf_counter()
        self.switch_active.set()

        with self.lock:
            if self.simulated:
                time.sleep(0.003)  # Simulate ~3ms switch
            else:
                md_name1 = f'WF_25K_84' if mode_num in [1, 3] else 'WF_47K_57'
                md_name2 = f'WF_25K_264' if mode_num in [1, 3] else 'WF_47K_237'
                cmds = [
                    'OUTP1 OFF', 'OUTP2 OFF',
                    f'SOUR1:FUNC:ARB {md_name1}', f'SOUR2:FUNC:ARB {md_name2}',
                    f'OUTP1:POL {config["ch1_pol"]}', f'OUTP2:POL {config["ch2_pol"]}',
                    'OUTP1 ON', 'OUTP2 ON', '*WAI'
                ]
                self.inst.write(';'.join(cmds))

        t_end = time.perf_counter()
        self.switch_active.clear()
        self.switch_log.append((t_start, t_end, mode_num))

    def do_polarity_switch(self, mode_num):
        """Same-group polarity switch (fast)."""
        config = FG_MODE_CONFIGS[mode_num]
        t_start = time.perf_counter()
        self.switch_active.set()

        with self.lock:
            if self.simulated:
                time.sleep(0.0004)
            else:
                self.inst.write(f'OUTP1:POL {config["ch1_pol"]};OUTP2:POL {config["ch2_pol"]}')

        t_end = time.perf_counter()
        self.switch_active.clear()
        self.switch_log.append((t_start, t_end, mode_num))

    def disconnect(self):
        if self.inst:
            try:
                self.inst.write('OUTP1 OFF;OUTP2 OFF;*WAI')
                self.inst.close()
            except Exception:
                pass


# ── Process Pipeline ──────────────────────────────────────────────────────

class ProcessPipeline:
    """Process pipeline for isolation testing."""

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
        """Full process pipeline."""
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
            self.kf.correct(np.array([[cx], [cy]], dtype=np.float32))
            fx_raw, fy_raw = cx, cy
        else:
            fx_raw, fy_raw = px_pred, py_pred

        self.ema_x = ema(self.ema_x, fx_raw, EMA_ALPHA_POS)
        self.ema_y = ema(self.ema_y, fy_raw, EMA_ALPHA_POS)


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
    print("  Benchmark 06: Mode Switch Impact on Process Thread")
    print(f"  Rounds: {N_ROUNDS} x {N_FRAMES_PER_ROUND} frames")
    print(f"  Switch injection interval: every {SWITCH_INTERVAL_FRAMES} frames")
    print(f"  Budget: {BUDGET_MS} ms per frame")
    print(f"  Timestamp: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 70)

    # Setup
    print("\nInitializing...")
    fg = FGSwitchInjector()
    fg.connect()
    pipeline = ProcessPipeline()

    # ── Run benchmark (multi-round) ─────────────────────────────────────
    print(f"\nRunning {N_ROUNDS} rounds x {N_FRAMES_PER_ROUND} frames with concurrent FG switches...")

    results = []
    round_summaries = []

    for rnd in range(N_ROUNDS):
        switch_threads = []
        mode_sequence = [1, 2, 3, 4, 1, 2, 3, 4]
        switch_idx = 0
        round_results = []

        for i in range(N_FRAMES_PER_ROUND):
            frame = create_frame(rnd * N_FRAMES_PER_ROUND + i)
            during_switch = fg.switch_active.is_set()

            if i > 0 and i % SWITCH_INTERVAL_FRAMES == 0:
                target_mode = mode_sequence[switch_idx % len(mode_sequence)]
                switch_idx += 1
                if switch_idx % 2 == 0:
                    t = threading.Thread(target=fg.do_cross_group_switch, args=(target_mode,))
                else:
                    t = threading.Thread(target=fg.do_polarity_switch, args=(target_mode,))
                t.start()
                switch_threads.append(t)

            t0 = time.perf_counter()
            pipeline.process(frame)
            t1 = time.perf_counter()

            during_switch_after = fg.switch_active.is_set()
            overlapped = during_switch or during_switch_after

            round_results.append({
                'round': rnd + 1,
                'frame_idx': i,
                'process_ms': (t1 - t0) * 1000.0,
                'during_switch': int(overlapped),
            })

        for t in switch_threads:
            t.join(timeout=5.0)

        proc_times = [r['process_ms'] for r in round_results]
        n_switch = sum(1 for r in round_results if r['during_switch'])
        round_summaries.append({
            'round': rnd + 1,
            'proc_mean': np.mean(proc_times),
            'proc_p99': np.percentile(proc_times, 99),
            'frames_during_switch': n_switch,
        })
        results.extend(round_results)
        print(f"  Round {rnd+1}: proc={np.mean(proc_times):.3f}ms p99={np.percentile(proc_times,99):.3f}ms | {n_switch} frames during switch")

    fg.disconnect()

    total_switches = len(fg.switch_log)
    frames_during_switch = sum(1 for r in results if r['during_switch'])
    print(f"\n  Total switches: {total_switches}")
    print(f"  Frames during switch: {frames_during_switch}/{len(results)}")

    # ── Analysis ──────────────────────────────────────────────────────────
    normal_times = [r['process_ms'] for r in results if not r['during_switch']]
    switch_times = [r['process_ms'] for r in results if r['during_switch']]

    # ── Save CSV ──────────────────────────────────────────────────────────
    csv_path = os.path.join(script_dir, 'mode_impact_raw.csv')
    fields = ['round', 'frame_idx', 'process_ms', 'during_switch']
    with open(csv_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for r in results:
            writer.writerow({
                'round': r['round'],
                'frame_idx': r['frame_idx'],
                'process_ms': f"{r['process_ms']:.4f}",
                'during_switch': r['during_switch'],
            })
    print(f"\nRaw data saved: {csv_path}")

    # ── Generate Report ───────────────────────────────────────────────────
    report_lines = []
    report_lines.append("=" * 72)
    report_lines.append("  Benchmark 06: Mode Switch Impact on Process Thread")
    report_lines.append(f"  Rounds: {N_ROUNDS} x {N_FRAMES_PER_ROUND} = {len(results)} frames")
    report_lines.append(f"  FG switches injected: {total_switches}")
    report_lines.append(f"  Budget: {BUDGET_MS} ms per frame")
    report_lines.append(f"  FG: {'Real VISA' if not fg.simulated else 'Simulated'}")
    report_lines.append(f"  Timestamp: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    report_lines.append("=" * 72)
    report_lines.append("")
    report_lines.append("  Per-Round Consistency:")
    report_lines.append(f"  {'Round':<8} {'Proc Mean':>10} {'Proc P99':>10} {'During SW':>10}")
    report_lines.append(f"  {'-'*8} {'-'*10} {'-'*10} {'-'*10}")
    for s in round_summaries:
        report_lines.append(f"  {s['round']:<8} {s['proc_mean']:>9.3f}  {s['proc_p99']:>9.3f}  {s['frames_during_switch']:>9}")
    proc_means = [s['proc_mean'] for s in round_summaries]
    report_lines.append(f"  {'Avg':<8} {np.mean(proc_means):>9.3f} +/- {np.std(proc_means):.3f} ms")
    report_lines.append("")

    all_times = np.array([r['process_ms'] for r in results])
    report_lines.append(f"  ALL FRAMES ({len(all_times)}):")
    report_lines.append(f"    Mean: {np.mean(all_times):.3f} ms | Std: {np.std(all_times):.3f} ms")
    report_lines.append(f"    Max:  {np.max(all_times):.3f} ms | P95: {np.percentile(all_times, 95):.3f} ms | P99: {np.percentile(all_times, 99):.3f} ms")
    report_lines.append("")

    if normal_times:
        normal_arr = np.array(normal_times)
        report_lines.append(f"  NORMAL FRAMES (no concurrent switch) ({len(normal_times)}):")
        report_lines.append(f"    Mean: {np.mean(normal_arr):.3f} ms | Std: {np.std(normal_arr):.3f} ms")
        report_lines.append(f"    Max:  {np.max(normal_arr):.3f} ms | P99: {np.percentile(normal_arr, 99):.3f} ms")
        report_lines.append("")

    if switch_times:
        switch_arr = np.array(switch_times)
        report_lines.append(f"  FRAMES DURING FG SWITCH ({len(switch_times)}):")
        report_lines.append(f"    Mean: {np.mean(switch_arr):.3f} ms | Std: {np.std(switch_arr):.3f} ms")
        report_lines.append(f"    Max:  {np.max(switch_arr):.3f} ms | P99: {np.percentile(switch_arr, 99):.3f} ms")
        report_lines.append("")

    # Isolation test: compare means
    if normal_times and switch_times:
        mean_diff = abs(np.mean(switch_times) - np.mean(normal_times))
        isolation_pct = (mean_diff / np.mean(normal_times)) * 100.0
        report_lines.append(f"  ISOLATION ANALYSIS:")
        report_lines.append(f"    Mean difference: {mean_diff:.3f} ms ({isolation_pct:.1f}% impact)")
        isolation_pass = isolation_pct < 20.0  # Less than 20% impact
        report_lines.append(f"    Isolation: {'GOOD' if isolation_pass else 'POOR'} (<20% impact)")
        report_lines.append("")

    # Verdict
    p99_all = np.percentile(all_times, 99)
    verdict = "PASS" if p99_all < BUDGET_MS else "FAIL"

    report_lines.append("-" * 72)
    report_lines.append(f"  Process P99 (all frames): {p99_all:.3f} ms")
    report_lines.append(f"  VERDICT: [{verdict}] (P99 < {BUDGET_MS}ms)")
    report_lines.append("=" * 72)

    report_text = "\n".join(report_lines)
    print(f"\n{report_text}")

    report_path = os.path.join(script_dir, 'mode_impact_report.txt')
    with open(report_path, 'w', encoding='utf-8') as f:
        f.write(report_text + "\n")
    print(f"\nReport saved: {report_path}")


if __name__ == "__main__":
    main()
