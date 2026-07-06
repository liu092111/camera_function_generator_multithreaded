# -*- coding: utf-8 -*-
"""
main.py - 旋轉分析統一入口

同時執行：
  1. rotation_trajectory_analysis  → position / angle / angular_velocity 三張圖
  2. angle_comparison_with_target  → 含 target step response 的角度比較圖
  3. Step Response 性能指標        → terminal 輸出 + step_response_metrics.csv

在下方「使用者設定區」修改參數即可，不需要進入個別腳本。
"""

import io
import os
import sys
import shutil
import contextlib
import numpy as np
import pandas as pd

# ── 確保可以 import 同資料夾的模組 ─────────────────────────────────────────
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

import rotation_trajectory_analysis as rta
import angle_comparison_with_target as act


# ============================================================
#  使用者設定區（在這裡修改，不需要動其他檔案）
# ============================================================

CSV_FOLDER    = "file"   # CSV 檔案資料夾（相對於此腳本位置）
OUTPUT_FOLDER = "clockwise_3degree2"      # 輸出資料夾（相對於此腳本位置）

TARGET_ANGLE    = 3    # 目標角度（°）
STEP_START_TIME = None   # Step 起跳時間（秒），None = 自動偵測

# 平滑參數
SMOOTHING_ENABLED = True
SMOOTHING_METHOD  = "savgol"
SAVGOL_WINDOW     = 11
SAVGOL_POLY_ORDER = 3
MOVING_AVG_WINDOW = 5

# Step response 判定標準
RISE_LOW_PCT          = 0.10   # rise time 下限（目標角度的 10%）
RISE_HIGH_PCT         = 0.90   # rise time 上限（目標角度的 90%）
SETTLING_BAND_PCT     = 0.05   # settling band（±5% of TARGET_ANGLE）
STEADY_STATE_LAST_SEC = 1.0    # 用最後幾秒平均作為 steady-state 值


# ============================================================
#  將設定同步到子模組的全域變數
# ============================================================

def _sync_config():
    for mod in (rta, act):
        mod.CSV_FOLDER         = CSV_FOLDER
        mod.OUTPUT_FOLDER      = OUTPUT_FOLDER
        mod.SMOOTHING_ENABLED  = SMOOTHING_ENABLED
        mod.SMOOTHING_METHOD   = SMOOTHING_METHOD
        mod.SAVGOL_WINDOW      = SAVGOL_WINDOW
        mod.SAVGOL_POLY_ORDER  = SAVGOL_POLY_ORDER
        mod.MOVING_AVG_WINDOW  = MOVING_AVG_WINDOW

    act.TARGET_ANGLE    = TARGET_ANGLE
    act.STEP_START_TIME = STEP_START_TIME


# ============================================================
#  Step Response 指標計算
# ============================================================

def _first_cross(t_arr, val_arr, threshold, direction):
    """找第一個越過 threshold 的時間點，direction: 'up' 或 'down'"""
    if direction == "up":
        idx = np.where(val_arr >= threshold)[0]
    else:
        idx = np.where(val_arr <= threshold)[0]
    return t_arr[idx[0]] if len(idx) > 0 else np.nan


def compute_step_response_metrics(data_dict, step_time):
    """
    計算每條軌跡的 step response 性能指標。

    Returns:
        list of dict，每個 dict 包含一條軌跡的指標
    """
    results = []
    direction = "up" if TARGET_ANGLE > 0 else "down"

    for name, df in data_dict.items():
        if "angle_deg_unwrapped" not in df.columns or "t_s" not in df.columns:
            continue

        t     = df["t_s"].to_numpy()
        angle = df["angle_deg_unwrapped"].to_numpy()

        valid   = np.isfinite(t) & np.isfinite(angle)
        t_rel   = t[valid] - t[valid][0]
        ang_rel = angle[valid] - angle[valid][0]

        if len(t_rel) == 0:
            continue

        # 對齊到 step 起跳時間，只取 step 之後的資料
        t_step = t_rel - step_time
        mask   = t_step >= 0
        if mask.sum() < 5:
            continue
        t_after   = t_step[mask]
        ang_after = ang_rel[mask]

        # --- Rise time (10% → 90%) ---
        lo = TARGET_ANGLE * RISE_LOW_PCT
        hi = TARGET_ANGLE * RISE_HIGH_PCT
        t_lo      = _first_cross(t_after, ang_after, lo, direction)
        t_hi      = _first_cross(t_after, ang_after, hi, direction)
        rise_time = round(t_hi - t_lo, 4) if (np.isfinite(t_lo) and np.isfinite(t_hi)) else None

        # --- Overshoot (超過目標的部分，>= 0) ---
        if TARGET_ANGLE > 0:
            peak         = ang_after.max()
            overshoot_deg = max(0.0, peak - TARGET_ANGLE)
        else:
            peak         = ang_after.min()
            overshoot_deg = max(0.0, TARGET_ANGLE - peak)
        overshoot_pct = round(overshoot_deg / abs(TARGET_ANGLE) * 100, 2) if TARGET_ANGLE != 0 else None

        # --- Steady-state error ---
        dt     = np.mean(np.diff(t_after)) if len(t_after) > 1 else 0.01
        last_n = max(int(STEADY_STATE_LAST_SEC / dt), 1)
        steady_val = np.mean(ang_after[-last_n:])
        ss_error   = round(steady_val - TARGET_ANGLE, 3)

        # --- Settling time (5% → 8% → 10% fallback) ---
        def _calc_settling(pct):
            b = abs(TARGET_ANGLE) * pct
            for i in range(len(ang_after)):
                if np.all(np.abs(ang_after[i:] - TARGET_ANGLE) <= b):
                    return round(t_after[i], 4), pct
            return None, pct

        settling_time, settling_band_used = _calc_settling(0.05)
        if settling_time is None:
            settling_time, settling_band_used = _calc_settling(0.08)
        if settling_time is None:
            settling_time, settling_band_used = _calc_settling(0.10)

        settling_label = (f"{settling_time} (±{int(settling_band_used*100)}%)"
                          if settling_time is not None else "N/A")

        results.append({
            "name":                 name,
            "target_angle_deg":     TARGET_ANGLE,
            "rise_time_s":          rise_time      if rise_time      is not None else "N/A",
            "overshoot_deg":        round(overshoot_deg, 3),
            "overshoot_pct":        overshoot_pct  if overshoot_pct  is not None else "N/A",
            "settling_time_s":      settling_label,
            "steady_state_err_deg": ss_error,
        })

    return results


def print_step_response_metrics(metrics):
    """在 terminal 印出 step response 指標"""
    col_defs = [
        ("名稱",           "name",                 28),
        ("Rise time (s)", "rise_time_s",           14),
        ("Overshoot (°)", "overshoot_deg",         14),
        ("Overshoot (%)", "overshoot_pct",         14),
        ("Settling (s)",  "settling_time_s",       13),
        ("SS error (°)",  "steady_state_err_deg",  13),
    ]
    total_w = sum(w for _, _, w in col_defs)

    print("\n" + "=" * total_w)
    print(f"Step Response 性能指標  (Target = {TARGET_ANGLE}°, "
          f"Band = ±{SETTLING_BAND_PCT*100:.0f}%)")
    print("=" * total_w)
    print("".join(label.ljust(w) for label, _, w in col_defs))
    print("-" * total_w)

    for m in metrics:
        row = [str(m[key]).ljust(w) for _, key, w in col_defs]
        print("".join(row))

    print("=" * total_w)


def save_step_response_csv(metrics, output_path):
    """將 step response 指標儲存成 CSV"""
    if not metrics:
        return
    df = pd.DataFrame(metrics)
    df.to_csv(output_path, index=False, encoding="utf-8-sig")


@contextlib.contextmanager
def _quiet():
    """壓掉子模組的 print 輸出"""
    with contextlib.redirect_stdout(io.StringIO()):
        yield


# ============================================================
#  主程式
# ============================================================

def main():
    _sync_config()

    csv_folder    = os.path.join(script_dir, CSV_FOLDER)
    output_folder = os.path.join(script_dir, OUTPUT_FOLDER)
    os.makedirs(output_folder, exist_ok=True)

    print(f"旋轉分析  |  CSV: {CSV_FOLDER}  |  Target: {TARGET_ANGLE}°")
    print("-" * 50)

    # ── 載入資料 ───────────────────────────────────────────────
    with _quiet():
        data_dict = rta.load_csv_files(csv_folder)

    if not data_dict:
        print("找不到 CSV 檔案，程式結束")
        return
    print(f"載入 {len(data_dict)} 個檔案: {', '.join(data_dict.keys())}")

    # ── 生成圖表 ───────────────────────────────────────────────
    with _quiet():
        rta.plot_multi_trajectories(
            data_dict,
            os.path.join(output_folder, "position_comparison.png"),
            title="Position Comparison (Rotation)"
        )
        rta.plot_multi_angle_comparison(
            data_dict,
            os.path.join(output_folder, "angle_comparison.png"),
            title="Angle vs Time Comparison"
        )
        rta.plot_multi_angular_velocity_comparison(
            data_dict,
            os.path.join(output_folder, "angular_velocity_comparison.png"),
            title="Angular Velocity Comparison"
        )
        act.plot_angle_comparison_with_target(
            data_dict,
            os.path.join(output_folder, "angle_comparison_with_target.png"),
            title="Angle vs Time Comparison"
        )
    # ── Step Response 指標 ─────────────────────────────────────
    if STEP_START_TIME is not None:
        step_time = STEP_START_TIME
    else:
        motion_start_times = []
        for _, df in data_dict.items():
            if "angle_deg_unwrapped" not in df.columns or "t_s" not in df.columns:
                continue
            t_arr     = df["t_s"].to_numpy()
            angle_arr = df["angle_deg_unwrapped"].to_numpy()
            valid     = np.isfinite(t_arr) & np.isfinite(angle_arr)
            t_rel     = t_arr[valid] - t_arr[valid][0]
            ang_rel   = angle_arr[valid] - angle_arr[valid][0]
            st = act.detect_motion_start_anchor(ang_rel, t_rel, TARGET_ANGLE)
            motion_start_times.append(st)
        step_time = np.mean(motion_start_times) if motion_start_times else 0.0

    metrics = compute_step_response_metrics(data_dict, step_time)

    metrics_path = os.path.join(output_folder, "step_response_metrics.csv")
    save_step_response_csv(metrics, metrics_path)

    print(f"\n  ✓ step_response_metrics.csv")

    print_step_response_metrics(metrics)

    # ── 複製原始 CSV 到 output 資料夾 ─────────────────────────
    import glob
    for csv_path in glob.glob(os.path.join(csv_folder, "*.csv")):
        shutil.copy2(csv_path, output_folder)
    print(f"\n  ✓ 原始 CSV 已複製至 {OUTPUT_FOLDER}/")

    print("\n完成")


if __name__ == "__main__":
    main()
