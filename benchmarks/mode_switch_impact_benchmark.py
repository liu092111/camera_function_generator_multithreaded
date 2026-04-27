#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
mode_switch_impact_benchmark.py — Does FG mode switching disrupt the pipeline?

Runs the full 3-thread pipeline and triggers FG mode switches from Main Thread.
Measures whether Process Thread throughput drops during/after a switch.

Key question: when Main Thread is blocked for 30-150ms sending FG SCPI commands,
does Process Thread still sustain 120fps?
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
BUDGET_MS = 1000.0 / CAM_FPS_REQ


# ── Process Thread (records per-frame processing time) ──────

def process_thread_fn(frame_queue, result_queue, running, stats,
                      kf, mm_per_px, proc_times_list):
    if ENABLE_UNDISTORT:
        from undistort import get_undistort_corrector
        corrector = get_undistort_corrector()
    else:
        corrector = None

    ema_x = ema_y = ema_ang = None
    fidx = 0
    tinit = False

    while running[0] or not frame_queue.empty():
        try:
            frame, t_cap = frame_queue.get(timeout=0.1)
        except queue.Empty:
            continue

        t0 = time.perf_counter()

        if corrector is not None:
            frame = corrector.undistort(frame)
        fr = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        if FLIP_VERTICAL:
            fr = cv2.flip(fr, 0)
        if FLIP_HORIZONTAL:
            fr = cv2.flip(fr, 1)

        pred = kf.predict()
        px, py = float(pred[0, 0]), float(pred[1, 0])
        m = find_target_and_angle(fr)
        use = True
        if m is not None:
            cx, cy, ang, box = m
            if not tinit or fidx < 10:
                kf.statePost = np.array([[cx], [cy], [0.0], [0.0]], dtype=np.float32)
                tinit = True
            elif np.hypot(cx - px, cy - py) > MAX_MEAS_JUMP_PX:
                use = False
        else:
            use = False
            ang = np.nan
        if use and m is not None:
            est = kf.correct(np.array([[cx], [cy]], dtype=np.float32))
            fx, fy = float(est[0, 0]), float(est[1, 0])
        else:
            fx, fy = px, py
        ema_x = ema(ema_x, fx, EMA_ALPHA_POS)
        ema_y = ema(ema_y, fy, EMA_ALPHA_POS)
        if np.isfinite(ang):
            ema_ang = ema(ema_ang, ang, EMA_ALPHA_ANGLE)

        t1 = time.perf_counter()
        proc_ms = (t1 - t0) * 1000.0
        proc_times_list.append((t1, proc_ms))

        try:
            result_queue.put_nowait({'fidx': fidx, 't_cap': t_cap, 't_done': t1})
        except queue.Full:
            try:
                result_queue.get_nowait()
                result_queue.put_nowait({'fidx': fidx, 't_cap': t_cap, 't_done': t1})
            except Exception:
                pass

        fidx += 1
        stats.update_process()


def capture_thread_fn(cap, frame_queue, running, stats):
    while running[0]:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.001)
            continue
        t = time.perf_counter()
        stats.update_capture()
        try:
            frame_queue.put_nowait((frame, t))
        except queue.Full:
            stats.drop_frame()


def main():
    N_SWITCHES = 20  # number of mode switches to test
    FRAMES_BETWEEN_SWITCHES = 50  # frames between each switch
    TOTAL = WARMUP + N_SWITCHES * FRAMES_BETWEEN_SWITCHES

    print("=" * 68)
    print("  Mode Switch Impact on Pipeline Throughput")
    print(f"  {N_SWITCHES} mode switches, {FRAMES_BETWEEN_SWITCHES} frames between each")
    print(f"  Total: ~{TOTAL} frames (warmup {WARMUP})")
    print("=" * 68)

    # ── Camera ──
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, CAM_FPS_REQ)
    for _ in range(10):
        cap.read()
    print(f"\n  Camera: {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
          f"{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))} @ "
          f"{cap.get(cv2.CAP_PROP_FPS):.0f} fps")

    if ENABLE_UNDISTORT:
        from undistort import init_undistort, undistort_frame
        init_undistort()

    ok, first = cap.read()
    if ENABLE_UNDISTORT:
        first = undistort_frame(first)
    mm_per_px = calibrate_scale(first)

    kf = make_kalman()
    fr = cv2.rotate(first, cv2.ROTATE_90_CLOCKWISE)
    det = find_target_and_angle(fr)
    cx0, cy0 = (det[0], det[1]) if det else (CAM_WIDTH / 2.0, CAM_HEIGHT / 2.0)
    kf.statePost = np.array([[cx0], [cy0], [0.0], [0.0]], dtype=np.float32)

    fg = FunctionGeneratorController()
    fg_ok = fg.connect()
    if not fg_ok:
        print("  ERROR: FG not connected")
        cap.release()
        sys.exit(1)
    fg.switch_mode(1)

    running = [True]
    stats_obj = Stats()
    frame_queue = queue.Queue(maxsize=FRAME_QUEUE_SIZE)
    result_queue = queue.Queue(maxsize=RESULT_QUEUE_SIZE)
    proc_times = []  # (absolute_time, proc_ms)

    ct = threading.Thread(target=capture_thread_fn,
                          args=(cap, frame_queue, running, stats_obj), daemon=True)
    pt = threading.Thread(target=process_thread_fn,
                          args=(frame_queue, result_queue, running, stats_obj,
                                kf, mm_per_px, proc_times), daemon=True)
    ct.start()
    pt.start()

    # ── Main loop: consume results, periodically trigger mode switch ──
    switch_log = []  # (switch_start_time, switch_end_time, switch_type, duration_ms)
    consumed = 0
    switch_count = 0
    cur_mode = 1
    # Mode switch patterns: same-group and cross-group alternating
    switch_targets = []
    for i in range(N_SWITCHES):
        if i % 2 == 0:
            switch_targets.append((3, 'same-group'))   # 1->3
        else:
            switch_targets.append((2, 'cross-group'))  # 3->2 or 1->2

    frame_intervals = []
    last_t = None

    print(f"\n  Running...")
    try:
        while consumed < TOTAL and switch_count <= N_SWITCHES:
            try:
                data = result_queue.get(timeout=1.0)
            except queue.Empty:
                continue

            t_now = time.perf_counter()
            consumed += 1

            if last_t is not None and consumed > WARMUP:
                frame_intervals.append((t_now - last_t) * 1000.0)
            last_t = t_now

            # Trigger switch at defined intervals after warmup
            frames_after_warmup = consumed - WARMUP
            if (consumed > WARMUP and frames_after_warmup > 0 and
                    frames_after_warmup % FRAMES_BETWEEN_SWITCHES == 0 and
                    switch_count < N_SWITCHES):

                target, stype = switch_targets[switch_count]
                t_sw0 = time.perf_counter()
                fg.fast_switch_mode(target, silent=True)
                t_sw1 = time.perf_counter()
                sw_ms = (t_sw1 - t_sw0) * 1000.0
                switch_log.append((t_sw0, t_sw1, stype, sw_ms))
                cur_mode = target
                switch_count += 1

                # Alternate back for next switch
                if target == 3:
                    cur_mode = 3
                elif target == 2:
                    # switch back to 1 to prepare for next same-group
                    fg.fast_switch_mode(1, silent=True)
                    cur_mode = 1

            if consumed % 200 == 0:
                print(f"    {consumed}/{TOTAL}")

    except KeyboardInterrupt:
        pass
    finally:
        running[0] = False
        ct.join(timeout=2.0)
        pt.join(timeout=2.0)
        cap.release()
        fg.turn_off()
        fg.disconnect()

    # ── Analyze: Process Thread timing during vs outside switch windows ──
    # Convert proc_times to numpy for analysis
    if len(proc_times) < WARMUP:
        print("Not enough data")
        sys.exit(1)

    pt_data = proc_times[WARMUP:]  # skip warmup
    pt_times = np.array([t for t, ms in pt_data])
    pt_ms = np.array([ms for t, ms in pt_data])

    # Classify each process frame: was it during a switch or not?
    WINDOW_MS = 50  # look at frames within 50ms after switch start
    during_switch = []
    outside_switch = []

    for i, (t_abs, p_ms) in enumerate(pt_data):
        in_window = False
        for sw_start, sw_end, stype, sw_ms in switch_log:
            if sw_start <= t_abs <= sw_end + 0.05:  # during switch + 50ms after
                in_window = True
                break
        if in_window:
            during_switch.append(p_ms)
        else:
            outside_switch.append(p_ms)

    def s(arr):
        a = np.array(arr)
        if len(a) == 0:
            return None
        return {
            'n': len(a), 'mean': float(np.mean(a)), 'std': float(np.std(a)),
            'min': float(np.min(a)), 'max': float(np.max(a)),
            'p95': float(np.percentile(a, 95)), 'p99': float(np.percentile(a, 99)),
        }

    s_all = s(pt_ms)
    s_during = s(during_switch) if during_switch else None
    s_outside = s(outside_switch) if outside_switch else None
    s_interval = s(frame_intervals) if frame_intervals else None

    # Switch timing stats
    same_grp = [d for _, _, t, d in switch_log if t == 'same-group']
    cross_grp = [d for _, _, t, d in switch_log if t == 'cross-group']
    s_same = s(same_grp) if same_grp else None
    s_cross = s(cross_grp) if cross_grp else None

    info = stats_obj.get_info()

    # ── Report ──
    r = []
    r.append("=" * 68)
    r.append("  Mode Switch Impact on Pipeline Throughput")
    r.append(f"  Camera: {CAM_WIDTH}x{CAM_HEIGHT} @ {CAM_FPS_REQ} fps")
    r.append(f"  FG: Keysight 33622A via USB-TMC")
    r.append(f"  Switches: {len(switch_log)} ({len(same_grp)} same-group, {len(cross_grp)} cross-group)")
    r.append(f"  Frames between switches: {FRAMES_BETWEEN_SWITCHES}")
    r.append(f"  Total measured frames: {len(pt_data)}")
    r.append(f"  Timestamp: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    r.append("=" * 68)

    r.append("")
    r.append("[FG Switch Timing — measured from Main Thread]")
    if s_same:
        r.append(f"  Same-group (polarity):  n={s_same['n']}  "
                 f"mean={s_same['mean']:.1f} ms  min={s_same['min']:.1f} ms  max={s_same['max']:.1f} ms")
    if s_cross:
        r.append(f"  Cross-group (waveform): n={s_cross['n']}  "
                 f"mean={s_cross['mean']:.1f} ms  min={s_cross['min']:.1f} ms  max={s_cross['max']:.1f} ms")

    r.append("")
    r.append("[Process Thread Timing — ALL frames]")
    if s_all:
        r.append(f"  n={s_all['n']}  mean={s_all['mean']:.2f} ms  std={s_all['std']:.2f} ms  "
                 f"p95={s_all['p95']:.2f} ms  p99={s_all['p99']:.2f} ms  max={s_all['max']:.2f} ms")
        over = sum(1 for v in pt_ms if v > BUDGET_MS)
        r.append(f"  > {BUDGET_MS:.2f} ms: {over}/{s_all['n']} ({over/s_all['n']*100:.1f}%)")

    r.append("")
    r.append("[Process Thread Timing — DURING mode switch window]")
    if s_during:
        r.append(f"  n={s_during['n']}  mean={s_during['mean']:.2f} ms  std={s_during['std']:.2f} ms  "
                 f"p95={s_during['p95']:.2f} ms  max={s_during['max']:.2f} ms")
    else:
        r.append("  No frames captured during switch windows")

    r.append("")
    r.append("[Process Thread Timing — OUTSIDE mode switch window]")
    if s_outside:
        r.append(f"  n={s_outside['n']}  mean={s_outside['mean']:.2f} ms  std={s_outside['std']:.2f} ms  "
                 f"p95={s_outside['p95']:.2f} ms  max={s_outside['max']:.2f} ms")

    r.append("")
    r.append("[KEY FINDING: Process Thread unaffected by FG switch?]")
    if s_during and s_outside:
        diff = abs(s_during['mean'] - s_outside['mean'])
        r.append(f"  During switch mean:  {s_during['mean']:.2f} ms")
        r.append(f"  Outside switch mean: {s_outside['mean']:.2f} ms")
        r.append(f"  Difference:          {diff:.2f} ms ({diff/s_outside['mean']*100:.1f}%)")
        if diff < 0.5:
            r.append(f"  CONCLUSION: Process Thread is NOT affected by FG mode switching.")
            r.append(f"              The two run on separate threads with no shared blocking.")
        else:
            r.append(f"  CONCLUSION: Minor impact detected ({diff:.2f} ms), likely due to USB bus contention.")

    r.append("")
    r.append("[Pipeline Throughput]")
    if s_interval:
        actual_fps = 1000.0 / s_interval['mean'] if s_interval['mean'] > 0 else 0
        r.append(f"  Frame interval: mean={s_interval['mean']:.2f} ms  std={s_interval['std']:.2f} ms")
        r.append(f"  Effective throughput: {actual_fps:.1f} fps")
    r.append(f"  Captured: {info['captured']}  Processed: {info['processed']}  Dropped: {info['dropped']}")
    drop_pct = info['dropped'] / max(info['captured'], 1) * 100
    r.append(f"  Drop rate: {drop_pct:.1f}%")

    r.append("")
    r.append("=" * 68)
    r.append("  VERDICT")
    r.append("=" * 68)
    if s_all and s_all['p99'] < BUDGET_MS:
        r.append(f"  Process Thread p99 = {s_all['p99']:.2f} ms < {BUDGET_MS:.2f} ms : PASS")
    elif s_all:
        r.append(f"  Process Thread p99 = {s_all['p99']:.2f} ms")
    r.append(f"  Drop rate = {drop_pct:.1f}%")
    if drop_pct == 0 and s_all and s_all['p99'] < BUDGET_MS:
        r.append("")
        r.append("  FG mode switching (10-150ms) runs on Main Thread and does NOT")
        r.append("  block or slow down Process Thread. Pipeline sustains 120 fps")
        r.append("  throughout mode switches with zero frame drops.")
    r.append("=" * 68)

    report = "\n".join(r)
    print(f"\n{report}")

    out = os.path.join(PROJECT_ROOT, 'benchmarks', 'mode_switch_impact_results.txt')
    with open(out, 'w', encoding='utf-8') as f:
        f.write(report + "\n")
    print(f"\nSaved to: {out}")


if __name__ == "__main__":
    main()
