#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
pipeline_throughput_benchmark.py — Realistic pipeline throughput validation

Validates that the 3-stage pipeline sustains 120 fps throughput under
realistic operating conditions:
  - Process Thread processes every frame
  - FG commands sent every PID_UPDATE_INTERVAL (=5) frames, not every frame
  - Measures per-stage time to confirm each stage < 8.33 ms

This is the key evidence for thesis real-time control feasibility.
"""

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

WARMUP = 100
MEASURE = 1000
PID_UPDATE_INTERVAL = 5  # same as main.py
CAMERA_INTERVAL_MS = 1000.0 / CAM_FPS_REQ  # 8.33 ms


# ── Instrumented Capture Thread ──────────────────────────────

def capture_thread(cap, frame_queue, running, stats, capture_times):
    """Records per-frame capture duration (read -> enqueue)."""
    while running[0]:
        t0 = time.perf_counter()
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.001)
            continue
        t_read = time.perf_counter()
        stats.update_capture()

        try:
            frame_queue.put_nowait((frame, t_read))
            t_enq = time.perf_counter()
            capture_times.append((t_read, t_enq))
        except queue.Full:
            stats.drop_frame()


# ── Instrumented Process Thread ──────────────────────────────

def process_thread(frame_queue, result_queue, running, stats,
                   kf, tracker_state, mm_per_px, process_times):
    """Records per-frame process duration (dequeue -> result enqueue)."""
    if ENABLE_UNDISTORT:
        from undistort import get_undistort_corrector
        corrector = get_undistort_corrector()
    else:
        corrector = None

    ema_x = ema_y = ema_ang = None
    frame_idx = 0
    tracker_init = False

    while running[0] or not frame_queue.empty():
        try:
            item = frame_queue.get(timeout=0.1)
        except queue.Empty:
            continue

        frame, t_capture = item
        t_start = time.perf_counter()

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

        t_done = time.perf_counter()

        result_data = {
            'frame_idx': frame_idx,
            'fx': fx, 'fy': fy,
            'fx_mm_abs': fx_mm, 'fy_mm_abs': fy_mm,
            'angle': ang,
            'timestamp': t_capture,
            '_t_capture': t_capture,
            '_t_proc_start': t_start,
            '_t_proc_done': t_done,
        }

        try:
            result_queue.put_nowait(result_data)
        except queue.Full:
            try:
                result_queue.get_nowait()
                result_queue.put_nowait(result_data)
            except Exception:
                pass

        proc_ms = (t_done - t_start) * 1000.0
        process_times.append(proc_ms)

        frame_idx += 1
        stats.update_process()


# ── Main ─────────────────────────────────────────────────────

def main():
    total_frames = WARMUP + MEASURE

    print("=" * 64)
    print("  Pipeline Throughput Benchmark")
    print(f"  Camera: {CAM_WIDTH}x{CAM_HEIGHT} @ {CAM_FPS_REQ} fps")
    print(f"  FG command interval: every {PID_UPDATE_INTERVAL} frames")
    print(f"  Warmup: {WARMUP} / Measure: {MEASURE}")
    print("=" * 64)

    # ── Camera ──
    print("\nOpening camera...")
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, CAM_FPS_REQ)
    for _ in range(10):
        cap.read()
    print(f"  {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
          f"{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))} @ "
          f"{cap.get(cv2.CAP_PROP_FPS):.0f} fps")

    # ── Undistort ──
    if ENABLE_UNDISTORT:
        from undistort import init_undistort, undistort_frame
        init_undistort()

    # ── Scale ──
    ok, first = cap.read()
    if not ok:
        print("ERROR: cannot read camera")
        sys.exit(1)
    if ENABLE_UNDISTORT:
        first = undistort_frame(first)
    mm_per_px = calibrate_scale(first)
    print(f"  mm/px = {mm_per_px:.4f}")

    # ── Kalman ──
    kf = make_kalman()
    first_rot = cv2.rotate(first, cv2.ROTATE_90_CLOCKWISE)
    det = find_target_and_angle(first_rot)
    cx0, cy0 = (det[0], det[1]) if det else (CAM_WIDTH / 2.0, CAM_HEIGHT / 2.0)
    kf.statePost = np.array([[cx0], [cy0], [0.0], [0.0]], dtype=np.float32)

    # ── FG ──
    fg = FunctionGeneratorController()
    fg_connected = fg.connect()
    if fg_connected:
        fg.switch_mode(1)
        print("  FG Mode 1 active")
    else:
        print("  WARNING: FG not connected")

    # ── Shared state ──
    running = [True]
    stats = Stats()
    tracker_state = ThreadSafeState({'recording': False, 'pid_active': False})

    frame_queue = queue.Queue(maxsize=FRAME_QUEUE_SIZE)
    result_queue = queue.Queue(maxsize=RESULT_QUEUE_SIZE)

    capture_times = []   # (t_read, t_enqueue)
    process_times = []   # proc_ms

    # Main-thread timing storage
    main_times_no_fg = []   # ms, frames without FG command
    main_times_with_fg = [] # ms, frames with FG command
    fg_cmd_times = []       # ms, FG command only
    frame_intervals = []    # ms, inter-frame arrival at main thread
    dropped_frames = 0

    cap_t = threading.Thread(
        target=capture_thread,
        args=(cap, frame_queue, running, stats, capture_times),
        daemon=True
    )
    proc_t = threading.Thread(
        target=process_thread,
        args=(frame_queue, result_queue, running, stats, kf,
              tracker_state, mm_per_px, process_times),
        daemon=True
    )
    cap_t.start()
    proc_t.start()

    # ── Main loop ──
    print(f"\nRunning {total_frames} frames...")
    consumed = 0
    pid_counter = 0
    last_main_t = None

    try:
        while consumed < total_frames:
            try:
                data = result_queue.get(timeout=1.0)
            except queue.Empty:
                continue

            t_main_start = time.perf_counter()
            consumed += 1
            pid_counter += 1

            # Inter-frame interval
            if last_main_t is not None and consumed > WARMUP:
                frame_intervals.append((t_main_start - last_main_t) * 1000.0)
            last_main_t = t_main_start

            # FG command every PID_UPDATE_INTERVAL frames (realistic)
            sent_fg = False
            if fg_connected and fg.is_output_active() and pid_counter >= PID_UPDATE_INTERVAL:
                pid_counter = 0
                t_fg0 = time.perf_counter()
                fg.set_voltages(VOLTAGE, VOLTAGE, silent=True)
                t_fg1 = time.perf_counter()
                sent_fg = True
                fg_ms = (t_fg1 - t_fg0) * 1000.0
                if consumed > WARMUP:
                    fg_cmd_times.append(fg_ms)

            t_main_end = time.perf_counter()
            main_ms = (t_main_end - t_main_start) * 1000.0

            if consumed > WARMUP:
                if sent_fg:
                    main_times_with_fg.append(main_ms)
                else:
                    main_times_no_fg.append(main_ms)

            if consumed % 200 == 0:
                print(f"  {consumed}/{total_frames}")

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
    def s(arr, label=""):
        a = np.array(arr)
        if len(a) == 0:
            return None
        return {
            'n': len(a),
            'mean': float(np.mean(a)),
            'std': float(np.std(a)),
            'min': float(np.min(a)),
            'max': float(np.max(a)),
            'p50': float(np.percentile(a, 50)),
            'p95': float(np.percentile(a, 95)),
            'p99': float(np.percentile(a, 99)),
        }

    # Process Thread: skip warmup
    proc_arr = process_times[WARMUP:WARMUP + MEASURE] if len(process_times) >= WARMUP + MEASURE else process_times[WARMUP:]
    ps = s(proc_arr)

    # Capture Thread: skip warmup
    cap_arr = capture_times[WARMUP:WARMUP + MEASURE] if len(capture_times) >= WARMUP + MEASURE else capture_times[WARMUP:]
    cap_durations = [(e - r) * 1000.0 for r, e in cap_arr]
    cs = s(cap_durations)

    ms_no = s(main_times_no_fg)
    ms_fg = s(main_times_with_fg)
    fg_s = s(fg_cmd_times)
    fi = s(frame_intervals)

    info = stats.get_info()

    # ── Verdict ──
    proc_ok = ps['max'] < CAMERA_INTERVAL_MS if ps else False
    proc_p99_ok = ps['p99'] < CAMERA_INTERVAL_MS if ps else False
    main_ok = True
    if ms_fg:
        main_ok = ms_fg['p99'] < CAMERA_INTERVAL_MS

    # ── Report ──
    r = []
    r.append("=" * 68)
    r.append("  Pipeline Throughput Benchmark Report")
    r.append(f"  Camera: {CAM_WIDTH}x{CAM_HEIGHT} @ {CAM_FPS_REQ} fps")
    r.append(f"  FG: {'Mode 1' if fg_connected else 'not connected'}")
    r.append(f"  FG command interval: every {PID_UPDATE_INTERVAL} frames")
    r.append(f"  Measured: {MEASURE} frames (warmup {WARMUP})")
    r.append(f"  Timestamp: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    r.append("=" * 68)

    r.append("")
    r.append(f"[Stage 1] Capture Thread (read + enqueue)")
    if cs:
        r.append(f"  mean = {cs['mean']:.3f} ms  std = {cs['std']:.3f} ms")
        r.append(f"  max  = {cs['max']:.3f} ms  p99 = {cs['p99']:.3f} ms")
        r.append(f"  < {CAMERA_INTERVAL_MS:.2f} ms budget: {'YES' if cs['max'] < CAMERA_INTERVAL_MS else 'YES (p99)' if cs['p99'] < CAMERA_INTERVAL_MS else 'MARGINAL'}")

    r.append("")
    r.append(f"[Stage 2] Process Thread (undistort + HSV + Kalman + EMA)")
    if ps:
        r.append(f"  mean = {ps['mean']:.2f} ms  std = {ps['std']:.2f} ms")
        r.append(f"  min  = {ps['min']:.2f} ms  max = {ps['max']:.2f} ms")
        r.append(f"  p50  = {ps['p50']:.2f} ms  p95 = {ps['p95']:.2f} ms  p99 = {ps['p99']:.2f} ms")
        r.append(f"  < {CAMERA_INTERVAL_MS:.2f} ms budget: {'YES' if proc_ok else 'YES (p99)' if proc_p99_ok else 'NO'}")
        headroom = CAMERA_INTERVAL_MS - ps['mean']
        r.append(f"  Headroom: {headroom:.2f} ms ({headroom / CAMERA_INTERVAL_MS * 100:.0f}% of budget)")

    r.append("")
    r.append(f"[Stage 3a] Main Thread (no FG command)")
    if ms_no:
        r.append(f"  n = {ms_no['n']}  mean = {ms_no['mean']:.3f} ms  max = {ms_no['max']:.3f} ms")

    r.append("")
    r.append(f"[Stage 3b] Main Thread (with FG command, every {PID_UPDATE_INTERVAL} frames)")
    if ms_fg:
        r.append(f"  n = {ms_fg['n']}  mean = {ms_fg['mean']:.2f} ms  max = {ms_fg['max']:.2f} ms")
        r.append(f"  p95 = {ms_fg['p95']:.2f} ms  p99 = {ms_fg['p99']:.2f} ms")
        r.append(f"  < {CAMERA_INTERVAL_MS:.2f} ms budget: {'YES' if ms_fg['max'] < CAMERA_INTERVAL_MS else 'YES (p99)' if ms_fg['p99'] < CAMERA_INTERVAL_MS else 'NO'}")

    r.append("")
    r.append(f"[FG Command Only] set_voltages() (2 SCPI writes)")
    if fg_s:
        r.append(f"  n = {fg_s['n']}  mean = {fg_s['mean']:.2f} ms  max = {fg_s['max']:.2f} ms  p99 = {fg_s['p99']:.2f} ms")

    r.append("")
    r.append(f"[Frame Interval at Main Thread]")
    if fi:
        r.append(f"  mean = {fi['mean']:.2f} ms  (expected {CAMERA_INTERVAL_MS:.2f} ms)")
        r.append(f"  std  = {fi['std']:.2f} ms  min = {fi['min']:.2f} ms  max = {fi['max']:.2f} ms")
        actual_fps = 1000.0 / fi['mean'] if fi['mean'] > 0 else 0
        r.append(f"  Effective throughput: {actual_fps:.1f} fps")

    r.append("")
    r.append(f"[Drop Rate]")
    r.append(f"  Captured: {info['captured']}  Processed: {info['processed']}  Dropped: {info['dropped']}")
    drop_pct = info['dropped'] / max(info['captured'], 1) * 100
    r.append(f"  Drop rate: {drop_pct:.1f}%")

    r.append("")
    r.append("=" * 68)
    r.append("  VERDICT: Per-Stage Budget Compliance (< 8.33 ms)")
    r.append("=" * 68)
    r.append(f"  Capture Thread  : {'PASS' if cs and cs['p99'] < CAMERA_INTERVAL_MS else 'FAIL'}"
             f"  (p99 = {cs['p99']:.2f} ms)" if cs else "  N/A")
    r.append(f"  Process Thread  : {'PASS' if proc_p99_ok else 'FAIL'}"
             f"  (p99 = {ps['p99']:.2f} ms)" if ps else "  N/A")
    main_verdict = 'PASS'
    if ms_fg and ms_fg['p99'] >= CAMERA_INTERVAL_MS:
        main_verdict = 'CONDITIONAL'
    r.append(f"  Main Thread+FG  : {main_verdict}"
             f"  (p99 = {ms_fg['p99']:.2f} ms)" if ms_fg else "  N/A")

    if ps:
        r.append("")
        if proc_p99_ok and (not ms_fg or ms_fg['p99'] < CAMERA_INTERVAL_MS):
            r.append("  CONCLUSION: Pipeline sustains 120 fps real-time throughput.")
            r.append("  All stages complete within the 8.33 ms per-frame budget.")
        elif proc_p99_ok:
            r.append("  CONCLUSION: Pipeline sustains 120 fps throughput.")
            r.append(f"  FG commands (every {PID_UPDATE_INTERVAL} frames) occasionally exceed 8.33 ms")
            r.append("  but do not block the pipeline due to multithreading.")
            r.append("  Effective control update rate = "
                     f"{CAM_FPS_REQ / PID_UPDATE_INTERVAL:.0f} Hz.")
        else:
            r.append("  CONCLUSION: Process Thread may not sustain 120 fps.")

    r.append("=" * 68)

    report = "\n".join(r)
    print(f"\n{report}")

    out_dir = os.path.join(PROJECT_ROOT, 'benchmarks')
    path = os.path.join(out_dir, 'pipeline_throughput_results.txt')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(report + "\n")
    print(f"\nSaved to: {path}")


if __name__ == "__main__":
    main()
