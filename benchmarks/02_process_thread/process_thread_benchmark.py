#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
process_thread_benchmark.py — Process Thread per-frame timing benchmark

Measures the image-processing pipeline that runs inside process_thread():
  undistort -> rotate/flip -> Kalman predict -> HSV + morphology + contour
  -> Kalman correct -> EMA -> coordinate/speed calc -> draw overlay

Two modes:
  offline  — synthetic frames (no camera required)
  online   — live Arducam B0332 OV9281 frames
"""

import argparse
import os
import sys
import time
import numpy as np

# Ensure project root is on sys.path so we can import project modules
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.chdir(PROJECT_ROOT)
sys.path.insert(0, PROJECT_ROOT)

import cv2
from datetime import datetime

from config import (
    CAMERA_INDEX, CAM_WIDTH, CAM_HEIGHT, CAM_FPS_REQ,
    MAX_MEAS_JUMP_PX, EMA_ALPHA_POS, EMA_ALPHA_ANGLE,
    ENABLE_UNDISTORT, FLIP_VERTICAL, FLIP_HORIZONTAL,
    KF_PROCESS_NOISE, KF_MEASURE_NOISE,
    HSV_YELLOW_LO, HSV_YELLOW_HI,
    HSV_WHITE_LO, HSV_WHITE_HI,
    MIN_CONTOUR_AREA,
    CALIBRATION_DATA_PATH,
)
from signal_processing import ema
from image_processing import find_target_and_angle

# ── Constants ─────────────────────────────────────────────────
WARMUP_FRAMES = 50
MEASURE_FRAMES = 500
MM_PER_PX = 0.1  # dummy scale for benchmark


# ── Helpers ───────────────────────────────────────────────────

def make_kalman():
    """Identical to signal_processing.make_kalman()."""
    kf = cv2.KalmanFilter(4, 2)
    kf.transitionMatrix = np.array([
        [1, 0, 1, 0],
        [0, 1, 0, 1],
        [0, 0, 1, 0],
        [0, 0, 0, 1]], dtype=np.float32)
    kf.measurementMatrix = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0]], dtype=np.float32)
    kf.processNoiseCov = np.eye(4, dtype=np.float32) * KF_PROCESS_NOISE
    kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * KF_MEASURE_NOISE
    kf.errorCovPost = np.eye(4, dtype=np.float32)
    return kf


def load_undistort_corrector():
    """Load TPS / traditional undistort maps from the project calibration file."""
    if not ENABLE_UNDISTORT:
        return None
    from undistort import UndistortCorrector
    c = UndistortCorrector(CALIBRATION_DATA_PATH)
    if c.load_calibration():
        return c
    return None


def generate_synthetic_frame():
    """640x480 BGR random uint8 frame, same shape as camera output."""
    return np.random.randint(0, 256, (CAM_HEIGHT, CAM_WIDTH, 3), dtype=np.uint8)


# ── Single-frame pipeline (mirrors camera_threads.process_thread) ─

def process_one_frame(frame, kf, state, corrector):
    """
    Execute the full process_thread pipeline on a single frame.
    Returns a dict of per-step timings (seconds).
    """
    timings = {}

    # ── 1. Undistort ──
    t0 = time.perf_counter()
    if corrector is not None:
        frame = corrector.undistort(frame)
    t1 = time.perf_counter()
    timings['undistort'] = t1 - t0

    # ── 2. Rotate + Flip ──
    frame_rotated = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    if FLIP_VERTICAL:
        frame_rotated = cv2.flip(frame_rotated, 0)
    if FLIP_HORIZONTAL:
        frame_rotated = cv2.flip(frame_rotated, 1)
    t2 = time.perf_counter()
    timings['rotate_flip'] = t2 - t1

    # ── 3. Kalman predict ──
    pred = kf.predict()
    px_pred = float(pred[0, 0])
    py_pred = float(pred[1, 0])
    t3 = time.perf_counter()
    timings['kalman_predict'] = t3 - t2

    # ── 4. HSV mask + morphology + contour (find_target_and_angle) ──
    m = find_target_and_angle(frame_rotated)
    t4 = time.perf_counter()
    timings['hsv_morph_contour'] = t4 - t3

    # ── 5. Kalman correct ──
    use_meas = True
    if m is not None:
        cx, cy, angle_deg, box = m
        dist = np.hypot(cx - px_pred, cy - py_pred)
        if not state['tracker_init'] or state['frame_idx'] < 10:
            kf.statePost = np.array([[cx], [cy], [0.0], [0.0]], dtype=np.float32)
            state['tracker_init'] = True
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
    t5 = time.perf_counter()
    timings['kalman_correct'] = t5 - t4

    # ── 6. EMA + angle + coordinate calc ──
    state['ema_x'] = ema(state['ema_x'], fx_raw, EMA_ALPHA_POS)
    state['ema_y'] = ema(state['ema_y'], fy_raw, EMA_ALPHA_POS)
    if np.isfinite(angle_deg):
        state['ema_ang'] = ema(state['ema_ang'], angle_deg, EMA_ALPHA_ANGLE)

    fx = float(state['ema_x']) if state['ema_x'] is not None else fx_raw
    fy = float(state['ema_y']) if state['ema_y'] is not None else fy_raw
    ang = float(state['ema_ang']) if state['ema_ang'] is not None else float(angle_deg) if m is not None else np.nan

    fx_mm = fx * MM_PER_PX
    fy_mm = fy * MM_PER_PX
    t6 = time.perf_counter()
    timings['ema_angle_calc'] = t6 - t5

    # ── 7. Draw overlay (polylines, putText, etc.) ──
    display_frame = frame_rotated.copy()
    if box is None:
        sz = 20
        bx = np.array([
            [fx - sz, fy - sz], [fx + sz, fy - sz],
            [fx + sz, fy + sz], [fx - sz, fy + sz]], dtype=int)
        box_draw = bx
    else:
        box_draw = box
    cv2.polylines(display_frame, [box_draw], True, (0, 0, 255), 3)
    if np.isfinite(fx) and np.isfinite(fy):
        cv2.circle(display_frame, (int(round(fx)), int(round(fy))), 6, (0, 255, 0), -1)
    cv2.putText(display_frame, f"Pos(mm): ({fx_mm:.2f}, {fy_mm:.2f})",
                (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
    if np.isfinite(ang):
        cv2.putText(display_frame, f"Angle: {ang:+.2f} deg",
                    (10, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
    t7 = time.perf_counter()
    timings['draw_overlay'] = t7 - t6

    timings['total'] = t7 - t0
    state['frame_idx'] += 1
    return timings


# ── Main ──────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Process Thread Benchmark")
    parser.add_argument('--mode', choices=['offline', 'online'], default='offline')
    args = parser.parse_args()

    mode = args.mode
    total_frames = WARMUP_FRAMES + MEASURE_FRAMES

    print("=" * 56)
    print("  Process Thread Benchmark")
    print(f"  Mode: {mode}")
    if mode == 'online':
        print(f"  Camera: Arducam B0332 OV9281 120fps {CAM_WIDTH}x{CAM_HEIGHT}")
    print(f"  Warmup: {WARMUP_FRAMES} frames / Measure: {MEASURE_FRAMES} frames")
    print("=" * 56)

    # ── Load undistort corrector ──
    print("\nLoading undistort corrector...")
    corrector = load_undistort_corrector()
    if corrector is not None:
        print("  Undistort corrector loaded")
    else:
        print("  Undistort disabled or failed to load")

    # ── Open camera (online mode) ──
    cap = None
    if mode == 'online':
        print(f"\nOpening camera (index={CAMERA_INDEX})...")
        cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, CAM_FPS_REQ)

        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = cap.get(cv2.CAP_PROP_FPS)
        print(f"  Actual: {actual_w}x{actual_h} @ {actual_fps:.1f} fps")

        ret, test_frame = cap.read()
        if not ret:
            print("  ERROR: Cannot read from camera")
            cap.release()
            sys.exit(1)
        print(f"  Frame shape: {test_frame.shape}")

    # ── Initialize Kalman + tracker state ──
    kf = make_kalman()
    state = {
        'ema_x': None,
        'ema_y': None,
        'ema_ang': None,
        'tracker_init': False,
        'frame_idx': 0,
    }

    # ── Run pipeline ──
    step_keys = [
        'undistort', 'rotate_flip', 'kalman_predict',
        'hsv_morph_contour', 'kalman_correct', 'ema_angle_calc',
        'draw_overlay',
    ]
    all_timings = {k: [] for k in step_keys}
    all_timings['total'] = []

    print(f"\nRunning {total_frames} frames...")
    for i in range(total_frames):
        if mode == 'online':
            ret, frame = cap.read()
            if not ret:
                print(f"  Frame read failed at #{i}, retrying...")
                continue
        else:
            frame = generate_synthetic_frame()

        t = process_one_frame(frame, kf, state, corrector)

        if i >= WARMUP_FRAMES:
            for k in step_keys:
                all_timings[k].append(t[k])
            all_timings['total'].append(t['total'])

        if (i + 1) % 100 == 0:
            print(f"  {i + 1}/{total_frames} frames done")

    if cap is not None:
        cap.release()

    # ── Statistics ──
    def stats(arr_s):
        arr_ms = np.array(arr_s) * 1000.0
        return {
            'mean': float(np.mean(arr_ms)),
            'std': float(np.std(arr_ms)),
            'min': float(np.min(arr_ms)),
            'max': float(np.max(arr_ms)),
            'p50': float(np.percentile(arr_ms, 50)),
            'p95': float(np.percentile(arr_ms, 95)),
            'p99': float(np.percentile(arr_ms, 99)),
        }

    total_stats = stats(all_timings['total'])
    step_stats = {k: stats(all_timings[k]) for k in step_keys}
    step_sum = sum(s['mean'] for s in step_stats.values())

    step_labels = {
        'undistort':         'Undistort (TPS remap)',
        'rotate_flip':       'Rotate + Flip',
        'kalman_predict':    'Kalman Predict',
        'hsv_morph_contour': 'HSV + Morphology + Contour',
        'kalman_correct':    'Kalman Correct',
        'ema_angle_calc':    'EMA + Angle Calc',
        'draw_overlay':      'Draw Overlay',
    }

    max_fps = 1000.0 / total_stats['mean'] if total_stats['mean'] > 0 else float('inf')
    can_keep_up = max_fps >= CAM_FPS_REQ

    # ── Build report ──
    r = []
    r.append("=" * 60)
    r.append("  Process Thread Benchmark Report")
    r.append(f"  Mode: {mode}")
    if mode == 'online':
        r.append(f"  Camera: Arducam B0332 OV9281 {CAM_FPS_REQ}fps {CAM_WIDTH}x{CAM_HEIGHT}")
    else:
        r.append(f"  Frame: synthetic random {CAM_WIDTH}x{CAM_HEIGHT} uint8")
    r.append(f"  Undistort: {'enabled (TPS)' if corrector is not None else 'disabled'}")
    r.append(f"  Measured: {MEASURE_FRAMES} frames (warmup {WARMUP_FRAMES})")
    r.append(f"  Timestamp: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    r.append("=" * 60)

    r.append("")
    r.append("[Per-frame Total Time]")
    r.append(f"  mean  = {total_stats['mean']:.2f} ms")
    r.append(f"  std   = {total_stats['std']:.2f} ms")
    r.append(f"  min   = {total_stats['min']:.2f} ms")
    r.append(f"  max   = {total_stats['max']:.2f} ms")
    r.append(f"  p50   = {total_stats['p50']:.2f} ms")
    r.append(f"  p95   = {total_stats['p95']:.2f} ms")
    r.append(f"  p99   = {total_stats['p99']:.2f} ms")

    r.append("")
    r.append("[Per-step Breakdown]")
    for k in step_keys:
        s = step_stats[k]
        pct = (s['mean'] / step_sum * 100.0) if step_sum > 0 else 0
        lbl = step_labels[k]
        r.append(f"  {lbl:<30s} {s['mean']:6.2f} ms  ({pct:5.1f}%)")
    r.append(f"  {'─' * 48}")
    r.append(f"  {'Total':<30s} {step_sum:6.2f} ms  (100.0%)")

    r.append("")
    r.append("[Throughput Assessment]")
    r.append(f"  Max processing FPS:    {max_fps:.1f} fps")
    r.append(f"  Camera input FPS:      {CAM_FPS_REQ} fps")
    r.append(f"  Can keep up:           {'Yes' if can_keep_up else 'No'}")
    if not can_keep_up:
        headroom = (1000.0 / CAM_FPS_REQ) - total_stats['mean']
        r.append(f"  Budget per frame:      {1000.0 / CAM_FPS_REQ:.2f} ms")
        r.append(f"  Over budget by:        {-headroom:.2f} ms")
    else:
        headroom = (1000.0 / CAM_FPS_REQ) - total_stats['mean']
        r.append(f"  Budget per frame:      {1000.0 / CAM_FPS_REQ:.2f} ms")
        r.append(f"  Headroom:              {headroom:.2f} ms")

    r.append("=" * 60)

    report_text = "\n".join(r)
    print(f"\n{report_text}")

    out_path = os.path.join(PROJECT_ROOT, 'benchmarks', 'process_thread_benchmark_results.txt')
    with open(out_path, 'w', encoding='utf-8') as f:
        f.write(report_text + "\n")
    print(f"\nReport saved to: {out_path}")


if __name__ == "__main__":
    main()
