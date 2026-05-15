#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Benchmark 03: End-to-End Latency (Capture -> Process -> FG Command)

Compares Gen-2 vs Gen-3 FG voltage command approach:
  Gen-2: OUTP OFF -> VOLT -> OUTP ON -> *WAI (4 separate writes)
  Gen-3: SOUR1:VOLT X;SOUR2:VOLT Y (1 compound write, no *WAI)

Runs 5 rounds x 200 frames for each generation.
Architecture: In real system, Seg1/Seg2/Seg3 run on separate threads.
Process Thread (Seg2) is the critical path for 120fps compliance.
"""

import os
import sys
import time
import csv
import numpy as np
import cv2
from pathlib import Path
from datetime import datetime

PROJECT_ROOT = str(Path(__file__).resolve().parents[2])
sys.path.insert(0, PROJECT_ROOT)
os.chdir(PROJECT_ROOT)

from config import (
    CAM_WIDTH, CAM_HEIGHT, CAMERA_INDEX, CAM_FPS_REQ,
    ENABLE_UNDISTORT, FLIP_VERTICAL, FLIP_HORIZONTAL,
    EMA_ALPHA_POS, EMA_ALPHA_ANGLE, CALIBRATION_DATA_PATH,
    FG_RESOURCE_STRING, HAVE_VISA
)
from signal_processing import make_kalman, ema
from image_processing import find_target_and_angle
from undistort import UndistortCorrector

N_ROUNDS = 5
N_FRAMES_PER_ROUND = 200
N_WARMUP = 30
BUDGET_MS = 8.33


# -- FG Controller --

class FGController:
    def __init__(self):
        self.inst = None

    def connect(self):
        if not HAVE_VISA:
            return False
        import pyvisa as visa
        rm = visa.ResourceManager()
        self.inst = rm.open_resource(FG_RESOURCE_STRING)
        self.inst.timeout = 30000
        try:
            self.inst.control_ren(6)
        except Exception:
            pass
        print(f"  FG: {self.inst.query('*IDN?').strip()}")
        self._setup()
        return True

    def _setup(self):
        self.inst.write('SOUR1:FUNC SIN;SOUR2:FUNC SIN')
        self.inst.write('SOUR1:FREQ 25000;SOUR2:FREQ 25000')
        self.inst.write('SOUR1:VOLT 1.5;SOUR2:VOLT 1.5')
        self.inst.write('SOUR1:VOLT:OFFS 0;SOUR2:VOLT:OFFS 0')
        self.inst.write('OUTP1 ON;OUTP2 ON;*WAI')
        time.sleep(1.0)
        # Warmup USB-TMC bus
        for _ in range(30):
            self.inst.write('SOUR1:VOLT 1.500;SOUR2:VOLT 1.500')
            time.sleep(0.02)
        print("  FG warmed up")

    def gen3_voltage(self, v1, v2):
        """Gen-3: single compound write, no *WAI."""
        self.inst.write(f'SOUR1:VOLT {v1:.3f};SOUR2:VOLT {v2:.3f}')

    def gen2_voltage(self, v1, v2):
        """Gen-2: OUTP OFF -> set voltage -> OUTP ON -> *WAI (4 writes)."""
        self.inst.write('OUTP1 OFF')
        self.inst.write('OUTP2 OFF')
        self.inst.write(f'SOUR1:VOLT {v1:.3f}')
        self.inst.write(f'SOUR2:VOLT {v2:.3f}')
        self.inst.write('OUTP1 ON')
        self.inst.write('OUTP2 ON')
        self.inst.write('*WAI')

    def disconnect(self):
        if self.inst:
            try:
                self.inst.write('OUTP1 OFF;OUTP2 OFF;*WAI')
                time.sleep(0.3)
                self.inst.close()
            except Exception:
                pass


# -- Process Pipeline --

class Pipeline:
    def __init__(self):
        self.kf = make_kalman()
        self.ema_x = None
        self.ema_y = None
        self.corrector = None
        cal_path = os.path.join(PROJECT_ROOT, CALIBRATION_DATA_PATH)
        self.corrector = UndistortCorrector(cal_path)
        if not self.corrector.load_calibration():
            self.corrector = None

    def reset(self):
        self.kf = make_kalman()
        self.ema_x = None
        self.ema_y = None

    def process(self, frame):
        if self.corrector:
            frame = self.corrector.undistort(frame)
        frame_r = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        if FLIP_VERTICAL:
            frame_r = cv2.flip(frame_r, 0)
        if FLIP_HORIZONTAL:
            frame_r = cv2.flip(frame_r, 1)
        pred = self.kf.predict()
        px, py = float(pred[0, 0]), float(pred[1, 0])
        m = find_target_and_angle(frame_r)
        if m is not None:
            cx, cy, angle_deg, box = m
            self.kf.correct(np.array([[cx], [cy]], dtype=np.float32))
            self.ema_x = ema(self.ema_x, cx, EMA_ALPHA_POS)
            self.ema_y = ema(self.ema_y, cy, EMA_ALPHA_POS)
        fx = float(self.ema_x) if self.ema_x else px
        fy = float(self.ema_y) if self.ema_y else py
        return fx, fy


# -- Main --

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))

    print("=" * 70)
    print("  Benchmark 03: End-to-End Latency (Gen-2 vs Gen-3)")
    print(f"  Rounds: {N_ROUNDS} x {N_FRAMES_PER_ROUND} frames per generation")
    print(f"  Budget: {BUDGET_MS} ms (Process Thread critical path)")
    print(f"  Timestamp: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 70)

    # Connect FG
    print("\nInitializing FG...")
    fg = FGController()
    if not fg.connect():
        print("  ERROR: Cannot connect FG")
        return

    # Open camera
    print("\nOpening camera...")
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print("  ERROR: Camera not available")
        fg.disconnect()
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, CAM_FPS_REQ)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    for _ in range(N_WARMUP):
        cap.read()
    print("  Camera ready.")

    all_results = []

    # === Gen-3: 5 rounds ===
    print(f"\n{'='*70}")
    print("  GEN-3: Compound voltage write (no *WAI)")
    print("=" * 70)
    pipeline = Pipeline()
    for r in range(N_ROUNDS):
        pipeline.reset()
        round_results = []
        for i in range(N_FRAMES_PER_ROUND):
            # Seg1: Capture
            t0 = time.perf_counter()
            ret, frame = cap.read()
            if not ret:
                break
            t1 = time.perf_counter()

            # Seg2: Process
            t2 = time.perf_counter()
            fx, fy = pipeline.process(frame)
            t3 = time.perf_counter()

            # Seg3: FG voltage write (Gen-3)
            v = 1.5 + 0.01 * (fx - 150)
            t4 = time.perf_counter()
            fg.gen3_voltage(v, v)
            t5 = time.perf_counter()

            round_results.append({
                'generation': 'gen3',
                'round': r + 1,
                'frame_idx': i,
                'capture_ms': (t1 - t0) * 1000,
                'process_ms': (t3 - t2) * 1000,
                'fg_write_ms': (t5 - t4) * 1000,
                'total_ms': (t5 - t0) * 1000,
            })
        all_results.extend(round_results)
        totals = [d['process_ms'] for d in round_results]
        fg_times = [d['fg_write_ms'] for d in round_results]
        print(f"  Round {r+1}: process mean={np.mean(totals):.3f}ms p99={np.percentile(totals,99):.3f}ms | "
              f"FG mean={np.mean(fg_times):.3f}ms p99={np.percentile(fg_times,99):.3f}ms")
        time.sleep(0.5)

    # === Gen-2: 5 rounds ===
    print(f"\n{'='*70}")
    print("  GEN-2: OUTP OFF/ON + *WAI (4 separate writes)")
    print("=" * 70)
    pipeline2 = Pipeline()
    for r in range(N_ROUNDS):
        pipeline2.reset()
        round_results = []
        for i in range(N_FRAMES_PER_ROUND):
            t0 = time.perf_counter()
            ret, frame = cap.read()
            if not ret:
                break
            t1 = time.perf_counter()

            t2 = time.perf_counter()
            fx, fy = pipeline2.process(frame)
            t3 = time.perf_counter()

            v = 1.5 + 0.01 * (fx - 150)
            t4 = time.perf_counter()
            fg.gen2_voltage(v, v)
            t5 = time.perf_counter()

            round_results.append({
                'generation': 'gen2',
                'round': r + 1,
                'frame_idx': i,
                'capture_ms': (t1 - t0) * 1000,
                'process_ms': (t3 - t2) * 1000,
                'fg_write_ms': (t5 - t4) * 1000,
                'total_ms': (t5 - t0) * 1000,
            })
        all_results.extend(round_results)
        totals = [d['process_ms'] for d in round_results]
        fg_times = [d['fg_write_ms'] for d in round_results]
        print(f"  Round {r+1}: process mean={np.mean(totals):.3f}ms p99={np.percentile(totals,99):.3f}ms | "
              f"FG mean={np.mean(fg_times):.3f}ms p99={np.percentile(fg_times,99):.3f}ms")
        time.sleep(0.5)

    # Cleanup
    cap.release()
    fg.disconnect()
    print("\nHardware released.")

    # === Save CSV ===
    csv_path = os.path.join(script_dir, 'e2e_latency_raw.csv')
    fields = ['generation', 'round', 'frame_idx', 'capture_ms', 'process_ms', 'fg_write_ms', 'total_ms']
    with open(csv_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for r in all_results:
            row = {}
            for k in fields:
                v = r[k]
                row[k] = f"{v:.4f}" if isinstance(v, float) else v
            writer.writerow(row)
    print(f"Raw CSV: {csv_path}")

    # === Report ===
    gen3_data = [r for r in all_results if r['generation'] == 'gen3']
    gen2_data = [r for r in all_results if r['generation'] == 'gen2']

    report = []
    report.append("=" * 72)
    report.append("  Benchmark 03: End-to-End Latency — Gen-2 vs Gen-3")
    report.append(f"  Rounds: {N_ROUNDS} x {N_FRAMES_PER_ROUND} frames per generation")
    report.append(f"  Total: {len(gen3_data)} Gen-3 + {len(gen2_data)} Gen-2 = {len(all_results)} frames")
    report.append(f"  Budget: {BUDGET_MS} ms (Process Thread)")
    report.append(f"  Timestamp: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    report.append("=" * 72)

    for label, data in [("GEN-3 (compound, no WAI)", gen3_data), ("GEN-2 (OUTP OFF/ON + WAI)", gen2_data)]:
        if not data:
            continue
        report.append(f"\n  [{label}] Frames: {len(data)}")
        report.append(f"  {'Segment':<16} {'Mean':>8} {'Std':>8} {'P95':>8} {'P99':>8} {'Max':>8}")
        report.append(f"  {'-'*16} {'-'*8} {'-'*8} {'-'*8} {'-'*8} {'-'*8}")
        for key, seg_label in [('capture_ms','Capture'), ('process_ms','Process'), ('fg_write_ms','FG Write'), ('total_ms','Total')]:
            arr = np.array([r[key] for r in data])
            report.append(f"  {seg_label:<16} {np.mean(arr):>7.3f}  {np.std(arr):>7.3f}  "
                          f"{np.percentile(arr,95):>7.3f}  {np.percentile(arr,99):>7.3f}  {np.max(arr):>7.3f}")

    # Per-round summary
    report.append(f"\n  Per-Round FG Write Comparison:")
    report.append(f"  {'Round':<8} {'Gen-3 mean':>12} {'Gen-3 p99':>12} {'Gen-2 mean':>12} {'Gen-2 p99':>12}")
    report.append(f"  {'-'*8} {'-'*12} {'-'*12} {'-'*12} {'-'*12}")
    for r_num in range(1, N_ROUNDS + 1):
        g3r = [d['fg_write_ms'] for d in gen3_data if d['round'] == r_num]
        g2r = [d['fg_write_ms'] for d in gen2_data if d['round'] == r_num]
        report.append(f"  {r_num:<8} {np.mean(g3r):>11.3f}  {np.percentile(g3r,99):>11.3f}  "
                      f"{np.mean(g2r):>11.3f}  {np.percentile(g2r,99):>11.3f}")

    # Verdict
    gen3_fg = np.array([r['fg_write_ms'] for r in gen3_data])
    gen2_fg = np.array([r['fg_write_ms'] for r in gen2_data])
    gen3_proc = np.array([r['process_ms'] for r in gen3_data])

    report.append(f"\n{'-'*72}")
    report.append(f"  FG Write Improvement: Gen-2 mean={np.mean(gen2_fg):.2f}ms -> Gen-3 mean={np.mean(gen3_fg):.2f}ms")
    report.append(f"  Speed-up: {np.mean(gen2_fg)/np.mean(gen3_fg):.1f}x faster")
    report.append(f"")
    proc_p99 = np.percentile(gen3_proc, 99)
    verdict = "PASS" if proc_p99 < BUDGET_MS else "FAIL"
    report.append(f"  Process Thread (critical path): [{verdict}] P99={proc_p99:.3f}ms < {BUDGET_MS}ms")
    report.append("=" * 72)

    report_text = "\n".join(report)
    print(f"\n{report_text}")

    report_path = os.path.join(script_dir, 'e2e_latency_report.txt')
    with open(report_path, 'w', encoding='utf-8') as f:
        f.write(report_text + "\n")
    print(f"\nReport: {report_path}")


if __name__ == "__main__":
    main()
