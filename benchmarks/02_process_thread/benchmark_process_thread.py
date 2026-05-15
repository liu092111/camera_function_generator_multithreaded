#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Benchmark 02: Process Thread Per-Frame Pipeline Timing

Measures per-frame processing pipeline:
  Undistort (TPS remap) -> Rotate+Flip -> Kalman predict -> HSV+Morphology+Contour -> EMA -> Draw

Mode: 5 rounds x 200 frames (online + offline)
Also runs a single 500-frame pass for dense data.

Target: < 8.33ms per frame (120fps budget)
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
    EMA_ALPHA_POS, EMA_ALPHA_ANGLE, CALIBRATION_DATA_PATH
)
from signal_processing import make_kalman, ema
from image_processing import find_target_and_angle
from undistort import UndistortCorrector


# -- Configuration --
N_ROUNDS = 5
N_FRAMES_PER_ROUND = 200
N_SINGLE_PASS = 500
N_WARMUP = 50
BUDGET_MS = 8.33


# -- Synthetic Frame Generator --

def create_synthetic_frame(width=CAM_WIDTH, height=CAM_HEIGHT, frame_idx=0):
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    for x in range(0, width, 50):
        cv2.line(frame, (x, 0), (x, height), (30, 30, 30), 1)
    for y in range(0, height, 50):
        cv2.line(frame, (0, y), (width, y), (30, 30, 30), 1)
    cx = width // 2 + int(20 * np.sin(frame_idx * 0.02))
    cy = height // 2 + int(15 * np.cos(frame_idx * 0.015))
    angle = 5.0 * np.sin(frame_idx * 0.01)
    rect = ((cx, cy), (90, 60), angle)
    box = cv2.boxPoints(rect).astype(int)
    cv2.fillPoly(frame, [box], (0, 255, 255))
    return frame


# -- Process Pipeline --

class ProcessPipeline:
    def __init__(self):
        self.kf = make_kalman()
        self.ema_x = None
        self.ema_y = None
        self.ema_ang = None
        self.corrector = UndistortCorrector(
            os.path.join(PROJECT_ROOT, CALIBRATION_DATA_PATH)
        )
        self.has_undistort = self.corrector.load_calibration()
        if not self.has_undistort:
            print("  WARNING: Undistort calibration not loaded, skipping undistort step")

    def reset_state(self):
        self.kf = make_kalman()
        self.ema_x = None
        self.ema_y = None
        self.ema_ang = None

    def process_frame(self, frame):
        timings = {}

        t0 = time.perf_counter()
        if self.has_undistort:
            frame = self.corrector.undistort(frame)
        t1 = time.perf_counter()
        timings['undistort_ms'] = (t1 - t0) * 1000.0

        t0 = time.perf_counter()
        frame_r = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        if FLIP_VERTICAL:
            frame_r = cv2.flip(frame_r, 0)
        if FLIP_HORIZONTAL:
            frame_r = cv2.flip(frame_r, 1)
        t1 = time.perf_counter()
        timings['rotate_flip_ms'] = (t1 - t0) * 1000.0

        t0 = time.perf_counter()
        pred = self.kf.predict()
        px, py = float(pred[0, 0]), float(pred[1, 0])
        t1 = time.perf_counter()
        timings['kalman_ms'] = (t1 - t0) * 1000.0

        t0 = time.perf_counter()
        m = find_target_and_angle(frame_r)
        t1 = time.perf_counter()
        timings['hsv_detect_ms'] = (t1 - t0) * 1000.0

        t0 = time.perf_counter()
        if m is not None:
            cx, cy, angle_deg, box = m
            self.kf.correct(np.array([[cx], [cy]], dtype=np.float32))
            self.ema_x = ema(self.ema_x, cx, EMA_ALPHA_POS)
            self.ema_y = ema(self.ema_y, cy, EMA_ALPHA_POS)
            if np.isfinite(angle_deg):
                self.ema_ang = ema(self.ema_ang, angle_deg, EMA_ALPHA_ANGLE)
        else:
            box = None
        t1 = time.perf_counter()
        timings['ema_ms'] = (t1 - t0) * 1000.0

        t0 = time.perf_counter()
        display = frame_r.copy()
        fx = float(self.ema_x) if self.ema_x else px
        fy = float(self.ema_y) if self.ema_y else py
        if box is not None:
            cv2.polylines(display, [box], True, (0, 0, 255), 2)
        if np.isfinite(fx) and np.isfinite(fy):
            cv2.circle(display, (int(fx), int(fy)), 5, (0, 255, 0), -1)
        cv2.putText(display, f"({fx:.1f},{fy:.1f})", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
        t1 = time.perf_counter()
        timings['draw_ms'] = (t1 - t0) * 1000.0

        timings['total_ms'] = sum(timings.values())
        return timings


# -- Main --

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))

    print("=" * 70)
    print("  Benchmark 02: Process Thread Per-Frame Pipeline")
    print(f"  Multi-round: {N_ROUNDS} rounds x {N_FRAMES_PER_ROUND} frames")
    print(f"  Single-pass: {N_SINGLE_PASS} frames")
    print(f"  Budget: {BUDGET_MS} ms per frame (120fps)")
    print(f"  Timestamp: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 70)

    # === OFFLINE (synthetic) - Single pass ===
    print("\n[Offline] Single pass with synthetic frames...")
    pipeline = ProcessPipeline()
    offline_results = []
    for i in range(N_SINGLE_PASS):
        frame = create_synthetic_frame(frame_idx=i)
        timings = pipeline.process_frame(frame)
        timings['frame_idx'] = i
        timings['round'] = 0
        offline_results.append(timings)
    offline_totals = [r['total_ms'] for r in offline_results]
    print(f"  Mean={np.mean(offline_totals):.3f}ms  P99={np.percentile(offline_totals,99):.3f}ms  Max={np.max(offline_totals):.3f}ms")

    # === ONLINE - Open camera ===
    print(f"\n[Online] Opening camera (index={CAMERA_INDEX})...")
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print("  ERROR: Camera not available. Skipping online tests.")
        cap = None
    else:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, CAM_FPS_REQ)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        # Warmup
        for _ in range(N_WARMUP):
            cap.read()
        print("  Camera ready.")

    # === ONLINE - Single pass (500 frames) ===
    online_single_results = []
    if cap:
        print(f"\n[Online] Single pass ({N_SINGLE_PASS} frames)...")
        pipeline_s = ProcessPipeline()
        for i in range(N_SINGLE_PASS):
            ret, frame = cap.read()
            if not ret:
                break
            timings = pipeline_s.process_frame(frame)
            timings['frame_idx'] = i
            timings['round'] = 0
            online_single_results.append(timings)
        totals = [r['total_ms'] for r in online_single_results]
        print(f"  Frames: {len(online_single_results)}")
        print(f"  Mean={np.mean(totals):.3f}ms  P99={np.percentile(totals,99):.3f}ms  Max={np.max(totals):.3f}ms")

    # === ONLINE - Multi-round (5 x 200) ===
    online_round_results = []
    round_summaries = []
    if cap:
        print(f"\n[Online] Multi-round ({N_ROUNDS} rounds x {N_FRAMES_PER_ROUND} frames)...")
        for r in range(N_ROUNDS):
            pipeline_r = ProcessPipeline()
            round_data = []
            for i in range(N_FRAMES_PER_ROUND):
                ret, frame = cap.read()
                if not ret:
                    break
                timings = pipeline_r.process_frame(frame)
                timings['frame_idx'] = i
                timings['round'] = r + 1
                round_data.append(timings)
            online_round_results.extend(round_data)
            totals_r = [d['total_ms'] for d in round_data]
            mean_r = np.mean(totals_r)
            p99_r = np.percentile(totals_r, 99)
            max_r = np.max(totals_r)
            round_summaries.append({'round': r+1, 'mean': mean_r, 'p99': p99_r, 'max': max_r})
            print(f"  Round {r+1}: mean={mean_r:.3f}ms  p99={p99_r:.3f}ms  max={max_r:.3f}ms")

    if cap:
        cap.release()

    # === Save CSV (all data) ===
    csv_path = os.path.join(script_dir, 'process_thread_raw.csv')
    fields = ['mode', 'round', 'frame_idx', 'undistort_ms', 'rotate_flip_ms',
              'kalman_ms', 'hsv_detect_ms', 'ema_ms', 'draw_ms', 'total_ms']
    with open(csv_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for r in offline_results:
            row = {k: (f"{r[k]:.4f}" if isinstance(r.get(k), float) else r.get(k, ''))
                   for k in fields if k != 'mode'}
            row['mode'] = 'offline'
            writer.writerow(row)
        for r in online_single_results:
            row = {k: (f"{r[k]:.4f}" if isinstance(r.get(k), float) else r.get(k, ''))
                   for k in fields if k != 'mode'}
            row['mode'] = 'online_single'
            writer.writerow(row)
        for r in online_round_results:
            row = {k: (f"{r[k]:.4f}" if isinstance(r.get(k), float) else r.get(k, ''))
                   for k in fields if k != 'mode'}
            row['mode'] = 'online_round'
            writer.writerow(row)
    print(f"\nRaw CSV: {csv_path}")

    # === Report ===
    report = []
    report.append("=" * 72)
    report.append("  Benchmark 02: Process Thread Per-Frame Pipeline")
    report.append(f"  Budget: {BUDGET_MS} ms per frame (120fps)")
    report.append(f"  Timestamp: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    report.append("=" * 72)

    step_keys = ['undistort_ms', 'rotate_flip_ms', 'kalman_ms',
                 'hsv_detect_ms', 'ema_ms', 'draw_ms', 'total_ms']

    def add_table(label, results):
        if not results:
            return
        report.append(f"\n  [{label}] Frames: {len(results)}")
        report.append(f"  {'Step':<20} {'Mean':>8} {'Std':>8} {'Max':>8} {'P95':>8} {'P99':>8}")
        report.append(f"  {'-'*20} {'-'*8} {'-'*8} {'-'*8} {'-'*8} {'-'*8}")
        for key in step_keys:
            arr = np.array([r[key] for r in results])
            report.append(
                f"  {key:<20} {np.mean(arr):>7.3f}  {np.std(arr):>7.3f}  "
                f"{np.max(arr):>7.3f}  {np.percentile(arr,95):>7.3f}  {np.percentile(arr,99):>7.3f}"
            )
        total_arr = np.array([r['total_ms'] for r in results])
        verdict = "PASS" if np.percentile(total_arr, 99) < BUDGET_MS else "FAIL"
        report.append(f"  Verdict: [{verdict}] (P99={np.percentile(total_arr,99):.3f}ms)")

    add_table("OFFLINE single-pass (500 frames)", offline_results)
    add_table("ONLINE single-pass (500 frames)", online_single_results)
    add_table(f"ONLINE multi-round ({N_ROUNDS}x{N_FRAMES_PER_ROUND}={N_ROUNDS*N_FRAMES_PER_ROUND} frames)", online_round_results)

    if round_summaries:
        report.append(f"\n  Per-Round Consistency:")
        report.append(f"  {'Round':<8} {'Mean':>8} {'P99':>8} {'Max':>8}")
        report.append(f"  {'-'*8} {'-'*8} {'-'*8} {'-'*8}")
        for s in round_summaries:
            report.append(f"  {s['round']:<8} {s['mean']:>7.3f}  {s['p99']:>7.3f}  {s['max']:>7.3f}")
        means = [s['mean'] for s in round_summaries]
        report.append(f"  {'Avg':<8} {np.mean(means):>7.3f} +/- {np.std(means):.3f} ms")

    # Overall
    all_online = [r['total_ms'] for r in online_single_results + online_round_results]
    if all_online:
        overall_p99 = np.percentile(all_online, 99)
        overall_verdict = "PASS" if overall_p99 < BUDGET_MS else "FAIL"
    else:
        overall_p99 = np.percentile(offline_totals, 99)
        overall_verdict = "PASS" if overall_p99 < BUDGET_MS else "FAIL"

    report.append(f"\n{'='*72}")
    report.append(f"  OVERALL: [{overall_verdict}] (P99={overall_p99:.3f}ms < {BUDGET_MS}ms)")
    report.append("=" * 72)

    report_text = "\n".join(report)
    print(f"\n{report_text}")

    report_path = os.path.join(script_dir, 'process_thread_report.txt')
    with open(report_path, 'w', encoding='utf-8') as f:
        f.write(report_text + "\n")
    print(f"\nReport: {report_path}")


if __name__ == "__main__":
    main()
