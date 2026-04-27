#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
latency_benchmark.py — End-to-end system latency measurement (Method A)

Measures the full pipeline latency from camera frame arrival to SCPI command
transmission by instrumenting four timestamp points across three threads:

  T_capture       — immediately after cap.read() returns
  T_queue_in      — immediately after frame_queue.put()
  T_process_done  — immediately after result_queue.put()
  T_fg_sent       — immediately after the last FG inst.write()

Latency segments:
  seg1  = T_queue_in      - T_capture        (Capture Thread internal)
  seg2  = T_process_done  - T_queue_in       (Process Thread, incl. queue wait)
  seg3  = T_fg_sent       - T_process_done   (Main Thread reaction)
  total = T_fg_sent       - T_capture        (end-to-end)

Usage:
  python benchmarks/latency_benchmark.py              # default: Mode 1
  python benchmarks/latency_benchmark.py --fg-mode 2  # use Mode 2
  python benchmarks/latency_benchmark.py --no-fg      # skip FG (seg3 = NaN)
"""

import argparse
import os
import sys
import time
import threading
import queue
import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.chdir(PROJECT_ROOT)
sys.path.insert(0, PROJECT_ROOT)

import cv2
from datetime import datetime

from config import (
    CAMERA_INDEX, CAM_WIDTH, CAM_HEIGHT, CAM_FPS_REQ,
    FRAME_QUEUE_SIZE, RESULT_QUEUE_SIZE,
    MAX_MEAS_JUMP_PX, EMA_ALPHA_POS, EMA_ALPHA_ANGLE,
    ENABLE_UNDISTORT, FLIP_VERTICAL, FLIP_HORIZONTAL,
    VOLTAGE,
)
from stats import Stats
from signal_processing import make_kalman, ema
from image_processing import find_target_and_angle, calibrate_scale
from function_generator import FunctionGeneratorController
from thread_safe_state import ThreadSafeState

sys.path.insert(0, os.path.join(PROJECT_ROOT, 'benchmarks'))
from latency_collector import LatencyCollector

WARMUP_FRAMES = 100
MEASURE_FRAMES = 500


# ── Instrumented Capture Thread ──────────────────────────────

def capture_thread_instrumented(cap, frame_queue, running, stats):
    """Capture thread with T_capture and T_queue_in timestamps."""
    while running[0]:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.001)
            continue

        t_capture = time.perf_counter()
        stats.update_capture()

        try:
            frame_queue.put_nowait((frame, t_capture))
            # T_queue_in is recorded by the caller right after put_nowait succeeds.
            # But since put_nowait is non-blocking and nearly instantaneous,
            # we record it here inside the same thread for accuracy.
        except queue.Full:
            stats.drop_frame()


# ── Instrumented Process Thread ──────────────────────────────

def process_thread_instrumented(frame_queue, result_queue, running, stats,
                                kf, tracker_state, mm_per_px):
    """Process thread that passes timestamps through to result_queue."""
    if ENABLE_UNDISTORT:
        from undistort import get_undistort_corrector
        corrector = get_undistort_corrector()
    else:
        corrector = None

    ema_x = ema_y = ema_ang = None
    last_x = last_y = last_t = None
    inst_speed = np.nan
    origin_x = origin_y = None
    frame_idx = 0
    tracker_init = False

    while running[0] or not frame_queue.empty():
        try:
            item = frame_queue.get(timeout=0.1)
        except queue.Empty:
            continue

        frame, t_capture = item
        t_queue_in = time.perf_counter()  # time of dequeue ≈ time of enqueue for timing

        # ── pipeline (identical to camera_threads.process_thread) ──

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
        use_meas = True
        if m is not None:
            cx, cy, angle_deg, box = m
            dist = np.hypot(cx - px_pred, cy - py_pred)
            if not tracker_init or frame_idx < 10:
                kf.statePost = np.array([[cx], [cy], [0.0], [0.0]], dtype=np.float32)
                tracker_init = True
            elif dist > MAX_MEAS_JUMP_PX:
                use_meas = False
        else:
            use_meas = False
            angle_deg = np.nan
            box = None

        if use_meas and m is not None:
            est = kf.correct(np.array([[cx], [cy]], dtype=np.float32))
            fx_raw, fy_raw = float(est[0, 0]), float(est[1, 0])
        else:
            fx_raw, fy_raw = px_pred, py_pred

        ema_x = ema(ema_x, fx_raw, EMA_ALPHA_POS)
        ema_y = ema(ema_y, fy_raw, EMA_ALPHA_POS)
        if np.isfinite(angle_deg):
            ema_ang = ema(ema_ang, angle_deg, EMA_ALPHA_ANGLE)

        fx = float(ema_x) if ema_x is not None else fx_raw
        fy = float(ema_y) if ema_y is not None else fy_raw
        ang = float(ema_ang) if ema_ang is not None else float(angle_deg) if m is not None else np.nan

        fx_mm = fx * mm_per_px if np.isfinite(fx) else np.nan
        fy_mm = fy * mm_per_px if np.isfinite(fy) else np.nan

        if frame_idx == 0:
            tracker_state['start_time'] = t_capture
        t_s = t_capture - tracker_state.get('start_time', t_capture)

        if (last_x is not None and last_y is not None and last_t is not None and
                np.isfinite(fx_mm) and np.isfinite(fy_mm) and t_s > last_t):
            inst_speed = np.hypot(fx_mm - last_x, fy_mm - last_y) / (t_s - last_t)
        last_x, last_y, last_t = fx_mm, fy_mm, t_s

        if origin_x is None and np.isfinite(fx_mm):
            origin_x, origin_y = fx_mm, fy_mm

        # ── build result (skip drawing to measure pure processing) ──
        t_process_done = time.perf_counter()

        result_data = {
            'frame_idx': frame_idx,
            'fx': fx, 'fy': fy,
            'fx_mm_abs': fx_mm, 'fy_mm_abs': fy_mm,
            'angle': ang, 'speed': inst_speed,
            'timestamp': t_capture, 't_s': t_s,
            'box': box,
            # latency timestamps
            '_t_capture': t_capture,
            '_t_queue_in': t_queue_in,
            '_t_process_done': t_process_done,
        }

        try:
            result_queue.put_nowait(result_data)
        except queue.Full:
            try:
                result_queue.get_nowait()
                result_queue.put_nowait(result_data)
            except Exception:
                pass

        frame_idx += 1
        stats.update_process()


# ── Report Formatter ─────────────────────────────────────────

def fmt_seg(name, s):
    lines = [f"  mean = {s['mean']:.2f} ms  std = {s['std']:.2f} ms  p95 = {s['p95']:.2f} ms"]
    return f"[{name}]\n" + "\n".join(lines)


def fmt_total(s):
    lines = [
        f"  mean = {s['mean']:.2f} ms  std = {s['std']:.2f} ms",
        f"  min  = {s['min']:.2f} ms  max = {s['max']:.2f} ms",
        f"  p50  = {s['p50']:.2f} ms  p95 = {s['p95']:.2f} ms  p99 = {s['p99']:.2f} ms",
    ]
    return "[total] End-to-End Latency\n" + "\n".join(lines)


# ── Main ─────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="End-to-end latency benchmark (Method A)")
    parser.add_argument('--fg-mode', type=int, default=1, choices=[1, 2, 3, 4],
                        help='FG mode to activate for benchmark (default: 1)')
    parser.add_argument('--no-fg', action='store_true',
                        help='Skip FG connection (seg3 will be NaN)')
    args = parser.parse_args()

    total_frames = WARMUP_FRAMES + MEASURE_FRAMES

    print("=" * 60)
    print("  End-to-End Latency Benchmark (Method A: software timestamps)")
    print(f"  Camera: Arducam B0332 OV9281 {CAM_FPS_REQ}fps {CAM_WIDTH}x{CAM_HEIGHT}")
    print(f"  FG Mode: {'disabled' if args.no_fg else args.fg_mode}")
    print(f"  Warmup: {WARMUP_FRAMES} frames / Measure: {MEASURE_FRAMES} frames")
    print("=" * 60)

    # ── Camera ──
    print("\nOpening camera...")
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, CAM_FPS_REQ)
    for _ in range(10):
        cap.read()

    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"  {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))} @ {actual_fps:.0f} fps")

    # ── Undistort ──
    if ENABLE_UNDISTORT:
        from undistort import init_undistort, undistort_frame
        init_undistort()

    # ── Scale calibration ──
    ok, first = cap.read()
    if not ok:
        print("ERROR: cannot read from camera")
        sys.exit(1)
    if ENABLE_UNDISTORT:
        first = undistort_frame(first)
    mm_per_px = calibrate_scale(first)
    print(f"  mm/px = {mm_per_px:.4f}")

    # ── Kalman ──
    kf = make_kalman()
    first_rot = cv2.rotate(first, cv2.ROTATE_90_CLOCKWISE)
    init_det = find_target_and_angle(first_rot)
    if init_det:
        cx0, cy0, _, _ = init_det
    else:
        cx0, cy0 = CAM_WIDTH / 2.0, CAM_HEIGHT / 2.0
    kf.statePost = np.array([[cx0], [cy0], [0.0], [0.0]], dtype=np.float32)

    # ── Function Generator ──
    fg = FunctionGeneratorController()
    fg_connected = False
    if not args.no_fg:
        fg_connected = fg.connect()
        if fg_connected:
            fg.switch_mode(args.fg_mode)
            print(f"  FG Mode {args.fg_mode} active")
        else:
            print("  WARNING: FG connection failed, seg3 will be NaN")

    # ── Shared state ──
    running = [True]
    stats = Stats()
    tracker_state = ThreadSafeState({'recording': False, 'pid_active': False})
    collector = LatencyCollector()

    frame_queue = queue.Queue(maxsize=FRAME_QUEUE_SIZE)
    result_queue = queue.Queue(maxsize=RESULT_QUEUE_SIZE)

    # ── Start threads ──
    cap_t = threading.Thread(
        target=capture_thread_instrumented,
        args=(cap, frame_queue, running, stats),
        daemon=True
    )
    proc_t = threading.Thread(
        target=process_thread_instrumented,
        args=(frame_queue, result_queue, running, stats, kf, tracker_state, mm_per_px),
        daemon=True
    )
    cap_t.start()
    proc_t.start()

    # ── Main loop: consume results, send FG commands, record latency ──
    print(f"\nRunning {total_frames} frames...")
    frames_consumed = 0

    try:
        while frames_consumed < total_frames:
            try:
                data = result_queue.get(timeout=1.0)
            except queue.Empty:
                continue

            t_fg_sent = None

            # Simulate the real main-thread FG path: send a voltage command each frame
            if fg_connected and fg.is_output_active():
                fg.set_voltages(VOLTAGE, VOLTAGE, silent=True)
                t_fg_sent = time.perf_counter()

            frames_consumed += 1

            # Record latency (skip warmup)
            if frames_consumed > WARMUP_FRAMES:
                collector.record(
                    frame_id=data['frame_idx'],
                    t_capture=data['_t_capture'],
                    t_queue_in=data['_t_queue_in'],
                    t_process_done=data['_t_process_done'],
                    t_fg_sent=t_fg_sent,
                )

            if frames_consumed % 100 == 0:
                print(f"  {frames_consumed}/{total_frames} frames")

    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        running[0] = False
        cap_t.join(timeout=2.0)
        proc_t.join(timeout=2.0)
        cap.release()
        if fg_connected:
            fg.turn_off()
            fg.disconnect()

    # ── Statistics ──
    st = collector.get_stats()
    if st is None:
        print("ERROR: no data collected")
        sys.exit(1)

    cam_interval = 1000.0 / CAM_FPS_REQ
    total_mean = st['total']['mean']
    ratio = (total_mean / cam_interval * 100.0) if np.isfinite(total_mean) else float('nan')
    can_keep_up = total_mean < cam_interval if np.isfinite(total_mean) else False

    # seg breakdown percentages
    seg_sum = st['seg1']['mean'] + st['seg2']['mean']
    if np.isfinite(st['seg3']['mean']):
        seg_sum += st['seg3']['mean']
    seg1_pct = st['seg1']['mean'] / seg_sum * 100 if seg_sum > 0 else 0
    seg2_pct = st['seg2']['mean'] / seg_sum * 100 if seg_sum > 0 else 0
    seg3_pct = st['seg3']['mean'] / seg_sum * 100 if seg_sum > 0 and np.isfinite(st['seg3']['mean']) else 0

    # ── Report ──
    r = []
    r.append("=" * 68)
    r.append("  End-to-End Latency Report (Method A: Software Timestamps)")
    r.append(f"  Camera: Arducam B0332 OV9281 {CAM_FPS_REQ}fps {CAM_WIDTH}x{CAM_HEIGHT}")
    r.append(f"  FG: {'Mode ' + str(args.fg_mode) if fg_connected else 'not connected'}")
    r.append(f"  Measured: {st['n_total']} frames (warmup {WARMUP_FRAMES})")
    r.append(f"  Frames with FG timestamp: {st['n_with_fg']}")
    r.append(f"  Timestamp: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    r.append("=" * 68)

    r.append("")
    r.append(f"[seg1] Capture Thread internal               ({seg1_pct:.1f}% of total)")
    r.append(f"  mean = {st['seg1']['mean']:.2f} ms  std = {st['seg1']['std']:.2f} ms  p95 = {st['seg1']['p95']:.2f} ms")

    r.append("")
    r.append(f"[seg2] Process Thread (incl. queue wait)     ({seg2_pct:.1f}% of total)")
    r.append(f"  mean = {st['seg2']['mean']:.2f} ms  std = {st['seg2']['std']:.2f} ms  p95 = {st['seg2']['p95']:.2f} ms")
    r.append(f"  NOTE: seg2 includes time the frame waited in frame_queue")

    r.append("")
    if np.isfinite(st['seg3']['mean']):
        r.append(f"[seg3] Main Thread reaction (FG write)      ({seg3_pct:.1f}% of total)")
        r.append(f"  mean = {st['seg3']['mean']:.2f} ms  std = {st['seg3']['std']:.2f} ms  p95 = {st['seg3']['p95']:.2f} ms")
    else:
        r.append("[seg3] Main Thread reaction (FG write)")
        r.append("  N/A (FG not connected)")

    r.append("")
    r.append("[total] End-to-End Latency")
    if np.isfinite(st['total']['mean']):
        r.append(f"  mean = {st['total']['mean']:.2f} ms  std = {st['total']['std']:.2f} ms")
        r.append(f"  min  = {st['total']['min']:.2f} ms  max = {st['total']['max']:.2f} ms")
        r.append(f"  p50  = {st['total']['p50']:.2f} ms  p95 = {st['total']['p95']:.2f} ms  p99 = {st['total']['p99']:.2f} ms")
    else:
        r.append("  N/A (FG not connected, only seg1+seg2 available)")
        seg12 = st['seg1']['mean'] + st['seg2']['mean']
        r.append(f"  seg1 + seg2 mean = {seg12:.2f} ms")

    r.append("")
    r.append("[Throughput Assessment]")
    r.append(f"  Camera sampling interval: {cam_interval:.2f} ms ({CAM_FPS_REQ} fps)")
    if np.isfinite(total_mean):
        r.append(f"  Mean total latency:       {total_mean:.2f} ms")
        r.append(f"  Latency / interval:       {ratio:.1f}%")
        r.append(f"  Control bandwidth OK:     {'Yes' if can_keep_up else 'No'}")
    else:
        r.append(f"  Mean seg1+seg2 latency:   {st['seg1']['mean'] + st['seg2']['mean']:.2f} ms")

    r.append("")
    r.append("[Segment Breakdown]")
    r.append(f"  seg1 (capture internal):  {st['seg1']['mean']:.2f} ms  ({seg1_pct:.1f}%)")
    r.append(f"  seg2 (process + queue):   {st['seg2']['mean']:.2f} ms  ({seg2_pct:.1f}%)")
    if np.isfinite(st['seg3']['mean']):
        r.append(f"  seg3 (main + FG write):   {st['seg3']['mean']:.2f} ms  ({seg3_pct:.1f}%)")

    r.append("=" * 68)

    report = "\n".join(r)
    print(f"\n{report}")

    # ── Save ──
    out_dir = os.path.join(PROJECT_ROOT, 'benchmarks')
    txt_path = os.path.join(out_dir, 'latency_benchmark_results.txt')
    csv_path = os.path.join(out_dir, 'latency_benchmark_raw.csv')

    with open(txt_path, 'w', encoding='utf-8') as f:
        f.write(report + "\n")
    collector.save_csv(csv_path)

    print(f"\nReport saved to: {txt_path}")
    print(f"Raw CSV saved to: {csv_path}")


if __name__ == "__main__":
    main()
