#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Benchmark 05: Pipeline Throughput

Full pipeline: capture -> process -> voltage command every N frames.
Measures sustained throughput over 1000 frames:
  - Drop rate
  - Effective FPS
  - Per-stage timing breakdown

Target: >= 120 FPS effective throughput (< 8.33ms per frame)
"""

import os
import sys
import time
import csv
import queue
import threading
import numpy as np
import cv2
from pathlib import Path
from datetime import datetime

# Add project root to path
PROJECT_ROOT = str(Path(__file__).resolve().parents[2])
sys.path.insert(0, PROJECT_ROOT)

from config import (
    CAM_WIDTH, CAM_HEIGHT, CAMERA_INDEX, CAM_FPS_REQ,
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
FG_COMMAND_INTERVAL = 5  # Send voltage command every N frames
BUDGET_MS = 8.33
FRAME_QUEUE_SIZE = 120


# ── FG Voltage Stub ───────────────────────────────────────────────────────

class FGVoltageController:
    """Minimal FG controller for voltage commands."""

    def __init__(self):
        self.inst = None
        self.simulated = False

    def connect(self):
        if not HAVE_VISA:
            self.simulated = True
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
            # Setup continuous output
            self.inst.write('SOUR1:FUNC SIN;SOUR2:FUNC SIN')
            self.inst.write('SOUR1:FREQ 25000;SOUR2:FREQ 25000')
            self.inst.write('SOUR1:VOLT 1.5;SOUR2:VOLT 1.5')
            self.inst.write('OUTP1 ON;OUTP2 ON;*WAI')
            time.sleep(0.5)
            # Warmup USB bus
            for _ in range(20):
                self.inst.write('SOUR1:VOLT 1.500;SOUR2:VOLT 1.500')
                time.sleep(0.02)
            return True
        except Exception:
            self.simulated = True
            return True

    def voltage_write(self, ch1_v, ch2_v):
        if self.simulated:
            time.sleep(0.0003)
        else:
            ch1_v = max(0.0, min(3.0, ch1_v))
            ch2_v = max(0.0, min(3.0, ch2_v))
            self.inst.write(f'SOUR1:VOLT {ch1_v:.3f};SOUR2:VOLT {ch2_v:.3f}')

    def disconnect(self):
        if self.inst:
            try:
                self.inst.write('SOUR1:VOLT 0;SOUR2:VOLT 0;*WAI')
                self.inst.close()
            except Exception:
                pass


# ── Synthetic Frame Generator (threaded) ──────────────────────────────────

def synthetic_capture_thread(frame_queue, running, n_frames, stats):
    """
    Simulates camera capture at 120fps by generating frames at timed intervals.
    """
    frame_interval = 1.0 / CAM_FPS_REQ  # ~8.33ms
    for i in range(n_frames):
        if not running[0]:
            break
        t_start = time.perf_counter()

        # Generate synthetic frame
        frame = np.zeros((CAM_HEIGHT, CAM_WIDTH, 3), dtype=np.uint8)
        cx = CAM_WIDTH // 2 + int(20 * np.sin(i * 0.02))
        cy = CAM_HEIGHT // 2 + int(15 * np.cos(i * 0.015))
        rect = ((cx, cy), (90, 60), 5.0 * np.sin(i * 0.01))
        box = cv2.boxPoints(rect).astype(int)
        cv2.fillPoly(frame, [box], (0, 255, 255))

        timestamp = time.perf_counter()

        try:
            frame_queue.put_nowait((frame, timestamp, i))
            stats['captured'] += 1
        except queue.Full:
            stats['dropped'] += 1

        # Maintain frame rate
        elapsed = time.perf_counter() - t_start
        sleep_time = frame_interval - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

    running[0] = False


def camera_capture_thread(frame_queue, running, n_frames, stats, cap):
    """Real camera capture thread."""
    for i in range(n_frames):
        if not running[0]:
            break
        ret, frame = cap.read()
        if not ret:
            stats['read_errors'] += 1
            continue
        timestamp = time.perf_counter()
        try:
            frame_queue.put_nowait((frame, timestamp, i))
            stats['captured'] += 1
        except queue.Full:
            stats['dropped'] += 1

    running[0] = False


# ── Main ──────────────────────────────────────────────────────────────────

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(PROJECT_ROOT)

    print("=" * 70)
    print("  Benchmark 05: Pipeline Throughput")
    print(f"  Rounds: {N_ROUNDS} x {N_FRAMES_PER_ROUND} frames")
    print(f"  FG command every {FG_COMMAND_INTERVAL} frames")
    print(f"  Budget: {BUDGET_MS} ms per frame (120fps)")
    print(f"  Timestamp: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 70)

    # Initialize
    fg = FGVoltageController()
    fg.connect()
    print(f"  FG: {'Real' if not fg.simulated else 'Simulated'}")

    # Initialize undistort
    corrector = None
    if ENABLE_UNDISTORT:
        cal_path = os.path.join(PROJECT_ROOT, CALIBRATION_DATA_PATH)
        corrector = UndistortCorrector(cal_path)
        if not corrector.load_calibration():
            corrector = None

    # Try camera
    use_camera = False
    cap = None
    try:
        cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
            cap.set(cv2.CAP_PROP_FPS, CAM_FPS_REQ)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            ret, _ = cap.read()
            if ret:
                use_camera = True
                print(f"  Camera: connected ({CAM_WIDTH}x{CAM_HEIGHT})")
            else:
                cap.release()
                cap = None
    except Exception:
        pass

    if not use_camera:
        print("  Camera: using synthetic frames")

    # Multi-round execution
    all_frame_results = []
    round_summaries = []
    total_dropped = 0
    total_captured = 0

    print(f"\nRunning {N_ROUNDS} rounds x {N_FRAMES_PER_ROUND} frames...")

    for rnd in range(N_ROUNDS):
        frame_queue = queue.Queue(maxsize=FRAME_QUEUE_SIZE)
        running = [True]
        capture_stats = {'captured': 0, 'dropped': 0, 'read_errors': 0}

        if use_camera:
            cap_thread = threading.Thread(
                target=camera_capture_thread,
                args=(frame_queue, running, N_FRAMES_PER_ROUND, capture_stats, cap),
                daemon=True
            )
        else:
            cap_thread = threading.Thread(
                target=synthetic_capture_thread,
                args=(frame_queue, running, N_FRAMES_PER_ROUND, capture_stats),
                daemon=True
            )

        kf = make_kalman()
        ema_x = None
        ema_y = None
        frame_results = []
        frames_processed = 0
        fg_commands_sent = 0

        t_round_start = time.perf_counter()
        cap_thread.start()

        while frames_processed < N_FRAMES_PER_ROUND:
            try:
                frame, timestamp, idx = frame_queue.get(timeout=2.0)
            except queue.Empty:
                if not running[0]:
                    break
                continue

            t_proc_start = time.perf_counter()

            if corrector is not None:
                frame = corrector.undistort(frame)

            frame_rotated = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
            if FLIP_VERTICAL:
                frame_rotated = cv2.flip(frame_rotated, 0)
            if FLIP_HORIZONTAL:
                frame_rotated = cv2.flip(frame_rotated, 1)

            pred = kf.predict()
            px_pred, py_pred = float(pred[0, 0]), float(pred[1, 0])
            m = find_target_and_angle(frame_rotated)
            if m is not None:
                cx, cy, angle_deg, box = m
                kf.correct(np.array([[cx], [cy]], dtype=np.float32))
                fx_raw, fy_raw = cx, cy
            else:
                fx_raw, fy_raw = px_pred, py_pred

            ema_x = ema(ema_x, fx_raw, EMA_ALPHA_POS)
            ema_y = ema(ema_y, fy_raw, EMA_ALPHA_POS)

            t_proc_end = time.perf_counter()

            t_fg_start = time.perf_counter()
            fg_sent = False
            if frames_processed % FG_COMMAND_INTERVAL == 0:
                fx = float(ema_x) if ema_x is not None else px_pred
                v = 1.5 + 0.005 * (fx - 240)
                fg.voltage_write(v, v)
                fg_commands_sent += 1
                fg_sent = True
            t_fg_end = time.perf_counter()

            frame_results.append({
                'round': rnd + 1,
                'frame_idx': idx,
                'queue_wait_ms': (t_proc_start - timestamp) * 1000.0,
                'process_ms': (t_proc_end - t_proc_start) * 1000.0,
                'fg_ms': (t_fg_end - t_fg_start) * 1000.0 if fg_sent else 0.0,
                'total_ms': (t_fg_end - t_proc_start) * 1000.0,
                'fg_sent': fg_sent,
            })
            frames_processed += 1

        t_round_end = time.perf_counter()
        cap_thread.join(timeout=5.0)

        round_time = t_round_end - t_round_start
        round_fps = frames_processed / round_time if round_time > 0 else 0
        round_drops = capture_stats['dropped']
        total_dropped += round_drops
        total_captured += capture_stats['captured']

        proc_times = [r['process_ms'] for r in frame_results]
        round_summaries.append({
            'round': rnd + 1,
            'fps': round_fps,
            'drops': round_drops,
            'proc_mean': np.mean(proc_times),
            'proc_p99': np.percentile(proc_times, 99),
        })
        all_frame_results.extend(frame_results)
        print(f"  Round {rnd+1}: {round_fps:.1f} fps | drops={round_drops} | "
              f"proc={np.mean(proc_times):.3f}ms (p99={np.percentile(proc_times,99):.3f}ms)")

    if cap is not None:
        cap.release()
    fg.disconnect()

    # Compute overall metrics
    total_frames = len(all_frame_results)
    total_time_s = sum(s['round'] for s in round_summaries)  # placeholder
    fps_values = [s['fps'] for s in round_summaries]
    effective_fps = np.mean(fps_values)
    drop_rate = total_dropped / max(1, total_captured + total_dropped)
    frame_results = all_frame_results  # for CSV/report compatibility

    print(f"\n  Total frames: {total_frames}")
    print(f"  Mean FPS: {effective_fps:.1f} +/- {np.std(fps_values):.1f}")
    print(f"  Total dropped: {total_dropped} ({drop_rate*100:.1f}%)")

    # ── Save CSV ──────────────────────────────────────────────────────────
    csv_path = os.path.join(script_dir, 'throughput_raw.csv')
    fields = ['round', 'frame_idx', 'queue_wait_ms', 'process_ms', 'fg_ms', 'total_ms', 'fg_sent']
    with open(csv_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for r in frame_results:
            writer.writerow({
                'round': r['round'],
                'frame_idx': r['frame_idx'],
                'queue_wait_ms': f"{r['queue_wait_ms']:.4f}",
                'process_ms': f"{r['process_ms']:.4f}",
                'fg_ms': f"{r['fg_ms']:.4f}",
                'total_ms': f"{r['total_ms']:.4f}",
                'fg_sent': int(r['fg_sent']),
            })
    print(f"\nRaw data saved: {csv_path}")

    # ── Generate Report ───────────────────────────────────────────────────
    report_lines = []
    report_lines.append("=" * 72)
    report_lines.append("  Benchmark 05: Pipeline Throughput")
    report_lines.append(f"  Source: {'Camera' if use_camera else 'Synthetic'}")
    report_lines.append(f"  FG: {'Real VISA' if not fg.simulated else 'Simulated'}")
    report_lines.append(f"  Rounds: {N_ROUNDS} x {N_FRAMES_PER_ROUND} = {total_frames} frames")
    report_lines.append(f"  FG command interval: every {FG_COMMAND_INTERVAL} frames")
    report_lines.append(f"  Timestamp: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    report_lines.append("=" * 72)
    report_lines.append("")
    report_lines.append(f"  Throughput Metrics:")
    report_lines.append(f"    Effective FPS:  {effective_fps:.1f} +/- {np.std(fps_values):.1f}")
    report_lines.append(f"    Target FPS:     {CAM_FPS_REQ}")
    report_lines.append(f"    Total frames:   {total_frames}")
    report_lines.append(f"    Dropped frames: {total_dropped} ({drop_rate*100:.1f}%)")
    report_lines.append("")
    report_lines.append(f"  Per-Round Consistency:")
    report_lines.append(f"  {'Round':<8} {'FPS':>8} {'Drops':>8} {'Proc Mean':>10} {'Proc P99':>10}")
    report_lines.append(f"  {'-'*8} {'-'*8} {'-'*8} {'-'*10} {'-'*10}")
    for s in round_summaries:
        report_lines.append(f"  {s['round']:<8} {s['fps']:>7.1f}  {s['drops']:>7}  {s['proc_mean']:>9.3f}  {s['proc_p99']:>9.3f}")
    report_lines.append("")

    if frame_results:
        report_lines.append(f"  Per-Frame Timing:")
        report_lines.append(f"  {'Stage':<16} {'Mean':>8} {'Std':>8} {'Max':>8} {'P95':>8} {'P99':>8}")
        report_lines.append(f"  {'-'*16} {'-'*8} {'-'*8} {'-'*8} {'-'*8} {'-'*8}")

        for key, label in [('queue_wait_ms', 'Queue wait'),
                           ('process_ms', 'Process'),
                           ('fg_ms', 'FG write'),
                           ('total_ms', 'Total')]:
            if key == 'fg_ms':
                # Only frames with FG command
                arr = np.array([r[key] for r in frame_results if r['fg_sent']])
            else:
                arr = np.array([r[key] for r in frame_results])
            if len(arr) == 0:
                continue
            report_lines.append(
                f"  {label:<16} {np.mean(arr):>7.3f}  {np.std(arr):>7.3f}  "
                f"{np.max(arr):>7.3f}  {np.percentile(arr, 95):>7.3f}  "
                f"{np.percentile(arr, 99):>7.3f}"
            )

    # Verdict
    process_times = np.array([r['process_ms'] for r in frame_results])
    p99_proc = np.percentile(process_times, 99) if len(process_times) > 0 else 0
    fps_pass = effective_fps >= (CAM_FPS_REQ * 0.95)  # Allow 5% tolerance
    proc_pass = p99_proc < BUDGET_MS
    drop_pass = drop_rate < 0.05  # Less than 5% drops

    overall_pass = fps_pass and proc_pass and drop_pass
    verdict = "PASS" if overall_pass else "FAIL"

    report_lines.append("")
    report_lines.append("-" * 72)
    report_lines.append(f"  FPS check:    {'PASS' if fps_pass else 'FAIL'} ({effective_fps:.1f} >= {CAM_FPS_REQ * 0.95:.0f})")
    report_lines.append(f"  Process P99:  {'PASS' if proc_pass else 'FAIL'} ({p99_proc:.3f}ms < {BUDGET_MS}ms)")
    report_lines.append(f"  Drop rate:    {'PASS' if drop_pass else 'FAIL'} ({drop_rate*100:.1f}% < 5%)")
    report_lines.append(f"  OVERALL:      [{verdict}]")
    report_lines.append("=" * 72)

    report_text = "\n".join(report_lines)
    print(f"\n{report_text}")

    report_path = os.path.join(script_dir, 'throughput_report.txt')
    with open(report_path, 'w', encoding='utf-8') as f:
        f.write(report_text + "\n")
    print(f"\nReport saved: {report_path}")


if __name__ == "__main__":
    main()
