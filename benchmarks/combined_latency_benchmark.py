#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
combined_latency_benchmark.py — Process Thread + FG Command Combined Timing

Measures the COMBINED time of:
  Process Thread (per-frame image processing)  +  FG command (SCPI writes)

to verify whether the sum fits within the 8.33 ms camera frame budget.

Tests four realistic scenarios:
  Scenario A: Process + set_voltages() (routine PID voltage adjust)
  Scenario B: Process + same-group mode switch (polarity only, e.g. Mode 1->3)
  Scenario C: Process + cross-group mode switch (waveform change, e.g. Mode 1->2)
  Scenario D: Process only, no FG command (non-PID frames)

Runs multiple rounds for proper statistics (mean/std/min/max/p95/p99).
"""

import argparse
import os
import sys
import time
import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.chdir(PROJECT_ROOT)
sys.path.insert(0, PROJECT_ROOT)

import cv2
from datetime import datetime

from config import (
    CAMERA_INDEX, CAM_WIDTH, CAM_HEIGHT, CAM_FPS_REQ,
    MAX_MEAS_JUMP_PX, EMA_ALPHA_POS, EMA_ALPHA_ANGLE,
    ENABLE_UNDISTORT, FLIP_VERTICAL, FLIP_HORIZONTAL,
    VOLTAGE,
)
from signal_processing import make_kalman, ema
from image_processing import find_target_and_angle

WARMUP = 30
FRAMES_PER_ROUND = 200
N_ROUNDS = 5
CAMERA_BUDGET_MS = 1000.0 / CAM_FPS_REQ  # 8.33 ms


def load_corrector():
    if not ENABLE_UNDISTORT:
        return None
    from undistort import UndistortCorrector
    from config import CALIBRATION_DATA_PATH
    c = UndistortCorrector(CALIBRATION_DATA_PATH)
    return c if c.load_calibration() else None


def process_one_frame(frame, kf, state, corrector):
    """Single-frame processing pipeline, returns elapsed ms."""
    t0 = time.perf_counter()

    if corrector is not None:
        frame = corrector.undistort(frame)

    frame_r = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    if FLIP_VERTICAL:
        frame_r = cv2.flip(frame_r, 0)
    if FLIP_HORIZONTAL:
        frame_r = cv2.flip(frame_r, 1)

    pred = kf.predict()
    px, py = float(pred[0, 0]), float(pred[1, 0])

    m = find_target_and_angle(frame_r)
    use = True
    if m is not None:
        cx, cy, ang, box = m
        if not state['init'] or state['idx'] < 10:
            kf.statePost = np.array([[cx], [cy], [0.0], [0.0]], dtype=np.float32)
            state['init'] = True
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

    state['ex'] = ema(state['ex'], fx, EMA_ALPHA_POS)
    state['ey'] = ema(state['ey'], fy, EMA_ALPHA_POS)
    if np.isfinite(ang):
        state['ea'] = ema(state['ea'], ang, EMA_ALPHA_ANGLE)

    state['idx'] += 1
    t1 = time.perf_counter()
    return (t1 - t0) * 1000.0


def new_state():
    return {'ex': None, 'ey': None, 'ea': None, 'init': False, 'idx': 0}


def stats(arr):
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


def fmt_stats(s):
    return (f"mean={s['mean']:6.2f}  std={s['std']:5.2f}  "
            f"min={s['min']:5.2f}  max={s['max']:6.2f}  "
            f"p95={s['p95']:5.2f}  p99={s['p99']:5.2f} ms")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--rounds', type=int, default=N_ROUNDS)
    parser.add_argument('--frames', type=int, default=FRAMES_PER_ROUND)
    args = parser.parse_args()
    n_rounds = args.rounds
    n_frames = args.frames

    print("=" * 70)
    print("  Combined Latency Benchmark: Process Thread + FG Command")
    print(f"  Rounds: {n_rounds}  Frames/round: {n_frames}  Warmup: {WARMUP}")
    print(f"  Camera budget: {CAMERA_BUDGET_MS:.2f} ms (120 fps)")
    print("=" * 70)

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
    corrector = load_corrector()
    print(f"  Undistort: {'enabled' if corrector else 'disabled'}")

    # ── FG ──
    print("\nConnecting to FG...")
    from function_generator import FunctionGeneratorController
    fg = FunctionGeneratorController()
    fg_ok = fg.connect()
    if not fg_ok:
        print("  ERROR: FG not connected. Cannot run combined benchmark.")
        cap.release()
        sys.exit(1)

    # Pre-activate Mode 1
    fg.switch_mode(1)
    print("  FG Mode 1 active")

    # ── Storage for all rounds ──
    all_results = {
        'A_proc': [], 'A_fg': [], 'A_combined': [],
        'B_proc': [], 'B_fg': [], 'B_combined': [],
        'C_proc': [], 'C_fg': [], 'C_combined': [],
        'D_proc': [],
    }
    round_stats = []

    # ── Run rounds ──
    for rnd in range(1, n_rounds + 1):
        print(f"\n--- Round {rnd}/{n_rounds} ---")

        kf = make_kalman()
        kf.statePost = np.array([[CAM_WIDTH/2], [CAM_HEIGHT/2], [0], [0]], dtype=np.float32)
        state = new_state()

        rA_proc, rA_fg, rA_comb = [], [], []
        rB_proc, rB_fg, rB_comb = [], [], []
        rC_proc, rC_fg, rC_comb = [], [], []
        rD_proc = []

        # Warmup
        for _ in range(WARMUP):
            ret, frame = cap.read()
            if ret:
                process_one_frame(frame, kf, state, corrector)

        # ── Scenario A: Process + set_voltages ──
        fg.switch_mode(1)
        for i in range(n_frames):
            ret, frame = cap.read()
            if not ret:
                continue
            proc_ms = process_one_frame(frame, kf, state, corrector)

            t0 = time.perf_counter()
            fg.set_voltages(VOLTAGE, VOLTAGE, silent=True)
            t1 = time.perf_counter()
            fg_ms = (t1 - t0) * 1000.0

            rA_proc.append(proc_ms)
            rA_fg.append(fg_ms)
            rA_comb.append(proc_ms + fg_ms)

        # ── Scenario B: Process + same-group switch (1<->3) ──
        fg.switch_mode(1)
        cur_mode = 1
        for i in range(n_frames):
            ret, frame = cap.read()
            if not ret:
                continue
            proc_ms = process_one_frame(frame, kf, state, corrector)

            target = 3 if cur_mode == 1 else 1
            t0 = time.perf_counter()
            fg.fast_switch_mode(target, silent=True)
            t1 = time.perf_counter()
            fg_ms = (t1 - t0) * 1000.0
            cur_mode = target

            rB_proc.append(proc_ms)
            rB_fg.append(fg_ms)
            rB_comb.append(proc_ms + fg_ms)

        # ── Scenario C: Process + cross-group switch (1<->2) ──
        fg.switch_mode(1)
        cur_mode = 1
        for i in range(n_frames):
            ret, frame = cap.read()
            if not ret:
                continue
            proc_ms = process_one_frame(frame, kf, state, corrector)

            target = 2 if cur_mode == 1 else 1
            t0 = time.perf_counter()
            fg.fast_switch_mode(target, silent=True)
            t1 = time.perf_counter()
            fg_ms = (t1 - t0) * 1000.0
            cur_mode = target

            rC_proc.append(proc_ms)
            rC_fg.append(fg_ms)
            rC_comb.append(proc_ms + fg_ms)

        # ── Scenario D: Process only (no FG) ──
        for i in range(n_frames):
            ret, frame = cap.read()
            if not ret:
                continue
            proc_ms = process_one_frame(frame, kf, state, corrector)
            rD_proc.append(proc_ms)

        # Accumulate
        all_results['A_proc'].extend(rA_proc)
        all_results['A_fg'].extend(rA_fg)
        all_results['A_combined'].extend(rA_comb)
        all_results['B_proc'].extend(rB_proc)
        all_results['B_fg'].extend(rB_fg)
        all_results['B_combined'].extend(rB_comb)
        all_results['C_proc'].extend(rC_proc)
        all_results['C_fg'].extend(rC_fg)
        all_results['C_combined'].extend(rC_comb)
        all_results['D_proc'].extend(rD_proc)

        # Per-round summary
        sA = stats(rA_comb)
        sB = stats(rB_comb)
        sC = stats(rC_comb)
        sD = stats(rD_proc)
        round_stats.append({'A': sA, 'B': sB, 'C': sC, 'D': sD})
        print(f"  A(volt adj)  combined: mean={sA['mean']:.2f} ms  max={sA['max']:.2f} ms")
        print(f"  B(same grp)  combined: mean={sB['mean']:.2f} ms  max={sB['max']:.2f} ms")
        print(f"  C(cross grp) combined: mean={sC['mean']:.2f} ms  max={sC['max']:.2f} ms")
        print(f"  D(no FG)     process:  mean={sD['mean']:.2f} ms  max={sD['max']:.2f} ms")

    # ── Cleanup ──
    fg.turn_off()
    fg.disconnect()
    cap.release()

    # ── Aggregate statistics ──
    sA_p = stats(all_results['A_proc'])
    sA_f = stats(all_results['A_fg'])
    sA_c = stats(all_results['A_combined'])
    sB_p = stats(all_results['B_proc'])
    sB_f = stats(all_results['B_fg'])
    sB_c = stats(all_results['B_combined'])
    sC_p = stats(all_results['C_proc'])
    sC_f = stats(all_results['C_fg'])
    sC_c = stats(all_results['C_combined'])
    sD_p = stats(all_results['D_proc'])

    total_frames = n_rounds * n_frames

    # ── Count budget violations ──
    A_over = sum(1 for v in all_results['A_combined'] if v > CAMERA_BUDGET_MS)
    B_over = sum(1 for v in all_results['B_combined'] if v > CAMERA_BUDGET_MS)
    C_over = sum(1 for v in all_results['C_combined'] if v > CAMERA_BUDGET_MS)
    D_over = sum(1 for v in all_results['D_proc'] if v > CAMERA_BUDGET_MS)

    # ── Report ──
    r = []
    r.append("=" * 74)
    r.append("  Process Thread + FG Command Combined Latency Report")
    r.append(f"  Camera: Arducam B0332 OV9281 {CAM_FPS_REQ}fps {CAM_WIDTH}x{CAM_HEIGHT}")
    r.append(f"  FG: Keysight 33622A via USB-TMC")
    r.append(f"  Rounds: {n_rounds}  Frames/round: {n_frames}  Total: {total_frames} frames/scenario")
    r.append(f"  Warmup: {WARMUP} frames/round")
    r.append(f"  Camera frame budget: {CAMERA_BUDGET_MS:.2f} ms")
    r.append(f"  Timestamp: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    r.append("=" * 74)

    r.append("")
    r.append("-" * 74)
    r.append("  Scenario A: Process + set_voltages()  [routine PID voltage adjust]")
    r.append("-" * 74)
    r.append(f"  Process only : {fmt_stats(sA_p)}")
    r.append(f"  FG only      : {fmt_stats(sA_f)}")
    r.append(f"  Combined     : {fmt_stats(sA_c)}")
    r.append(f"  Budget exceed: {A_over}/{total_frames} frames ({A_over/total_frames*100:.1f}%)")
    r.append(f"  VERDICT      : {'PASS' if sA_c['p99'] < CAMERA_BUDGET_MS else 'CONDITIONAL' if sA_c['p95'] < CAMERA_BUDGET_MS else 'FAIL'}"
             f" (p99={sA_c['p99']:.2f} ms {'<' if sA_c['p99'] < CAMERA_BUDGET_MS else '>'} {CAMERA_BUDGET_MS:.2f} ms)")

    r.append("")
    r.append("-" * 74)
    r.append("  Scenario B: Process + same-group switch  [Mode 1<->3, polarity only]")
    r.append("-" * 74)
    r.append(f"  Process only : {fmt_stats(sB_p)}")
    r.append(f"  FG only      : {fmt_stats(sB_f)}")
    r.append(f"  Combined     : {fmt_stats(sB_c)}")
    r.append(f"  Budget exceed: {B_over}/{total_frames} frames ({B_over/total_frames*100:.1f}%)")
    r.append(f"  VERDICT      : {'PASS' if sB_c['p99'] < CAMERA_BUDGET_MS else 'CONDITIONAL' if sB_c['p95'] < CAMERA_BUDGET_MS else 'FAIL'}"
             f" (p99={sB_c['p99']:.2f} ms {'<' if sB_c['p99'] < CAMERA_BUDGET_MS else '>'} {CAMERA_BUDGET_MS:.2f} ms)")

    r.append("")
    r.append("-" * 74)
    r.append("  Scenario C: Process + cross-group switch  [Mode 1<->2, waveform change]")
    r.append("-" * 74)
    r.append(f"  Process only : {fmt_stats(sC_p)}")
    r.append(f"  FG only      : {fmt_stats(sC_f)}")
    r.append(f"  Combined     : {fmt_stats(sC_c)}")
    r.append(f"  Budget exceed: {C_over}/{total_frames} frames ({C_over/total_frames*100:.1f}%)")
    r.append(f"  VERDICT      : {'PASS' if sC_c['p99'] < CAMERA_BUDGET_MS else 'CONDITIONAL' if sC_c['p95'] < CAMERA_BUDGET_MS else 'FAIL'}"
             f" (p99={sC_c['p99']:.2f} ms {'<' if sC_c['p99'] < CAMERA_BUDGET_MS else '>'} {CAMERA_BUDGET_MS:.2f} ms)")

    r.append("")
    r.append("-" * 74)
    r.append("  Scenario D: Process only  [no FG command, baseline]")
    r.append("-" * 74)
    r.append(f"  Process only : {fmt_stats(sD_p)}")
    r.append(f"  Budget exceed: {D_over}/{total_frames} frames ({D_over/total_frames*100:.1f}%)")
    r.append(f"  VERDICT      : {'PASS' if sD_p['p99'] < CAMERA_BUDGET_MS else 'FAIL'}"
             f" (p99={sD_p['p99']:.2f} ms {'<' if sD_p['p99'] < CAMERA_BUDGET_MS else '>'} {CAMERA_BUDGET_MS:.2f} ms)")

    r.append("")
    r.append("-" * 74)
    r.append("  Per-Round Consistency (combined mean, ms)")
    r.append("-" * 74)
    r.append(f"  {'Round':<8s} {'A(volt adj)':>12s} {'B(same grp)':>12s} {'C(cross grp)':>13s} {'D(no FG)':>10s}")
    for i, rs in enumerate(round_stats):
        r.append(f"  {i+1:<8d} {rs['A']['mean']:12.2f} {rs['B']['mean']:12.2f} {rs['C']['mean']:13.2f} {rs['D']['mean']:10.2f}")
    # Cross-round std
    A_means = [rs['A']['mean'] for rs in round_stats]
    B_means = [rs['B']['mean'] for rs in round_stats]
    C_means = [rs['C']['mean'] for rs in round_stats]
    D_means = [rs['D']['mean'] for rs in round_stats]
    r.append(f"  {'mean':<8s} {np.mean(A_means):12.2f} {np.mean(B_means):12.2f} {np.mean(C_means):13.2f} {np.mean(D_means):10.2f}")
    r.append(f"  {'std':<8s} {np.std(A_means):12.2f} {np.std(B_means):12.2f} {np.std(C_means):13.2f} {np.std(D_means):10.2f}")

    r.append("")
    r.append("=" * 74)
    r.append("  SUMMARY")
    r.append("=" * 74)
    r.append("")
    r.append(f"  Camera frame budget: {CAMERA_BUDGET_MS:.2f} ms (120 fps)")
    r.append("")
    r.append(f"  Scenario A  Process + voltage adjust       "
             f"mean={sA_c['mean']:.2f} ms  p99={sA_c['p99']:.2f} ms  "
             f"{'PASS' if sA_c['p99'] < CAMERA_BUDGET_MS else 'CONDITIONAL' if sA_c['p95'] < CAMERA_BUDGET_MS else 'FAIL'}")
    r.append(f"  Scenario B  Process + same-group switch    "
             f"mean={sB_c['mean']:.2f} ms  p99={sB_c['p99']:.2f} ms  "
             f"{'PASS' if sB_c['p99'] < CAMERA_BUDGET_MS else 'CONDITIONAL' if sB_c['p95'] < CAMERA_BUDGET_MS else 'FAIL'}")
    r.append(f"  Scenario C  Process + cross-group switch   "
             f"mean={sC_c['mean']:.2f} ms  p99={sC_c['p99']:.2f} ms  "
             f"{'PASS' if sC_c['p99'] < CAMERA_BUDGET_MS else 'CONDITIONAL' if sC_c['p95'] < CAMERA_BUDGET_MS else 'FAIL'}")
    r.append(f"  Scenario D  Process only (baseline)        "
             f"mean={sD_p['mean']:.2f} ms  p99={sD_p['p99']:.2f} ms  "
             f"{'PASS' if sD_p['p99'] < CAMERA_BUDGET_MS else 'FAIL'}")

    r.append("")
    r.append("=" * 74)

    report = "\n".join(r)
    print(f"\n{report}")

    out = os.path.join(PROJECT_ROOT, 'benchmarks', 'combined_latency_results.txt')
    with open(out, 'w', encoding='utf-8') as f:
        f.write(report + "\n")
    print(f"\nSaved to: {out}")


if __name__ == "__main__":
    main()
