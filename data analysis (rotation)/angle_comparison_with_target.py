# -*- coding: utf-8 -*-
"""
角度比較圖（含 Target Step Response）

在原有的 angle comparison 基礎上，額外繪製一條 target step response 線。
Target 是標準的 step response：在起跳時間之前為 0，之後為 TARGET_ANGLE。

使用方法：
1. 修改下方 TARGET_ANGLE 為你的實驗目標角度
2. STEP_START_TIME 設為 None 則自動偵測，或手動設定秒數
3. 執行此腳本：python angle_comparison_with_target.py
4. 輸出圖表：angle_comparison_with_target.png
"""

import os
import glob
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
from scipy.ndimage import uniform_filter1d
from scipy.signal import savgol_filter


# ============================================================
#  使用者可調整的參數
# ============================================================

# Target step response 參數
TARGET_ANGLE = -30   # 目標角度（°），請手動調整為實驗的目標角度
STEP_START_TIME = None  # Step 起跳時間（秒），設為 None 則自動偵測

# CSV 資料夾（相對於此腳本位置）
CSV_FOLDER = "file"
OUTPUT_FOLDER = "."

# 平滑參數
SMOOTHING_ENABLED = True
SMOOTHING_METHOD = "savgol"
SAVGOL_WINDOW = 11
SAVGOL_POLY_ORDER = 3
MOVING_AVG_WINDOW = 5

# 啟動偵測參數
MOTION_START_THRESHOLD = 10.0  # 角速度閾值（°/s），超過此值視為開始運動


# ============================================================
#  工具函式
# ============================================================

def smooth_data(data, method="savgol"):
    """對資料進行平滑處理"""
    if not SMOOTHING_ENABLED or len(data) < SAVGOL_WINDOW:
        return data

    if method == "savgol":
        window = min(SAVGOL_WINDOW, len(data))
        if window % 2 == 0:
            window -= 1
        if window < 3:
            return data
        poly_order = min(SAVGOL_POLY_ORDER, window - 1)
        return savgol_filter(data, window, poly_order)

    elif method == "moving_avg":
        return uniform_filter1d(data, size=MOVING_AVG_WINDOW, mode='nearest')

    return data


def load_csv_files(folder_path):
    """載入資料夾中所有 CSV 檔案"""
    csv_files = glob.glob(os.path.join(folder_path, "*.csv"))

    if not csv_files:
        print(f"警告：在 {folder_path} 中找不到 CSV 檔案")
        return {}

    data_dict = {}
    for csv_file in csv_files:
        try:
            file_name = Path(csv_file).stem
            df = pd.read_csv(csv_file)

            if "x_mm" in df.columns and "y_mm" in df.columns:
                data_dict[file_name] = df
                print(f"[OK] 已載入: {file_name} ({len(df)} 筆資料)")
            else:
                print(f"[SKIP] 跳過 {file_name}: 缺少 x_mm 或 y_mm 欄位")
        except Exception as e:
            print(f"[FAIL] 載入 {csv_file} 失敗: {e}")

    return data_dict


def detect_motion_start(angular_vel, threshold=MOTION_START_THRESHOLD):
    """
    偵測運動開始的時間點索引

    使用滑動窗口，連續幾個點的角速度超過閾值才判定為開始運動
    """
    window_size = 5
    for i in range(len(angular_vel) - window_size):
        window = angular_vel[i:i + window_size]
        if np.mean(np.abs(window)) > threshold:
            return i
    return 0


def detect_motion_start_from_angle(angle_relative, t_relative,
                                    delta_threshold=2.5, window_sec=1.0):
    """
    用滾動窗口內的變化量偵測運動開始時間。

    參數依據已知數據推算：
    - 靜止段雜訊幅度 ±0.5°，1 秒窗口內淨變化量 << 2.5°
    - 運動速度約 3.6°/s，1 秒窗口內變化量 ~3.6° > 2.5°
    因此 window_sec=1.0、delta_threshold=2.5 可乾淨區分兩段。
    """
    dt = np.mean(np.diff(t_relative)) if len(t_relative) > 1 else 0.01
    window_samples = max(int(window_sec / dt), 3)

    consecutive_required = 5

    count = 0
    for i in range(window_samples, len(angle_relative)):
        delta = abs(angle_relative[i] - angle_relative[i - window_samples])
        if delta > delta_threshold:
            count += 1
            if count >= consecutive_required:
                trigger_idx = i - consecutive_required + 1
                # 回推到窗口的起始端，對齊實際運動開始點
                return t_relative[max(trigger_idx - window_samples, 0)]
        else:
            count = 0

    return t_relative[0]

def detect_motion_start_anchor(angle, t, target_angle):
    """
    使用「錨點回溯法」：先找到達到目標 50% 的主運動區，再往回尋找起跳點。
    能完全免疫前期的緩慢漂移與突發雜訊。
    """
    if len(angle) < 10:
        return 0.0
        
    # 1. 抓取初始靜止區間的基準值 (取前 0.5 秒的數據作為 Baseline)
    dt = np.mean(np.diff(t)) if len(t) > 1 else 0.01
    init_samples = max(int(0.5 / dt), 5)
    init_samples = min(init_samples, len(angle) // 4)
    baseline = np.median(angle[:init_samples])
    
    # 2. 尋找「錨點」(Anchor) - 數值確實達到目標角度 50% 的位置
    anchor_val = baseline + target_angle * 0.5
    anchor_idx = 0
    
    for i in range(init_samples, len(angle)):
        # 判斷是否跨越 50% 閾值 (同時兼容正向與負向旋轉)
        if (target_angle > 0 and angle[i] >= anchor_val) or \
           (target_angle < 0 and angle[i] <= anchor_val):
            anchor_idx = i
            break
            
    # 如果沒找到錨點 (例如資料中途截斷)，回傳預設值
    if anchor_idx == 0:
        return 0.0
        
    # 3. 從錨點往「回」找起跳轉折點
    # 設定回推的停止容忍度：取 1.5 度 或 目標角度的 8% (取較大者，用來抵抗前期漂移)
    tolerance = max(abs(target_angle) * 0.08, 1.5)
    
    for i in range(anchor_idx, 0, -1):
        if abs(angle[i] - baseline) <= tolerance:
            # 當往回走到數值落在容忍範圍內，這就是真實的起點
            return t[i]
            
    # Fail-safe
    return t[0]

# ============================================================
#  主要繪圖函式
# ============================================================

def plot_angle_comparison_with_target(data_dict, output_path,
                                      title="Angle vs Time Comparison"):
    """
    繪製多軌跡角度比較圖（角度 vs 時間），並加上 target step response 線

    - Target step response：在起跳時間之前為 0°，之後瞬間跳到 TARGET_ANGLE
    - 起跳時間：若 STEP_START_TIME 為 None，則自動偵測所有軌跡的平均運動開始時間
    """
    if not data_dict:
        print("沒有資料可繪製")
        return

    # 檢查是否有角度資料
    has_angle = any("angle_deg_unwrapped" in df.columns for df in data_dict.values())
    if not has_angle:
        print("CSV 檔案中沒有 angle_deg_unwrapped 欄位，跳過角度比較圖")
        return

    # 創建圖表
    fig, ax = plt.subplots(1, 1, figsize=(11, 8))

    # 設定顏色循環
    colors = plt.cm.tab10.colors

    # 收集所有軌跡的 motion start 時間和最大時間
    motion_start_times = []
    max_time = 0

    # 繪製每條軌跡的角度
    for i, (name, df) in enumerate(data_dict.items()):
        if "angle_deg_unwrapped" not in df.columns or "t_s" not in df.columns:
            continue

        t = df["t_s"].to_numpy()
        angle = df["angle_deg_unwrapped"].to_numpy()

        # 過濾無效值
        valid = np.isfinite(t) & np.isfinite(angle)
        t_valid = t[valid]
        angle_valid = angle[valid]

        if len(t_valid) == 0:
            continue

        # 轉換為相對時間（從 0 開始）
        t_relative = t_valid - t_valid[0]
        # 將角度轉換為相對角度（從 0 開始）
        angle_relative = angle_valid - angle_valid[0]

        # 更新最大時間
        if t_relative[-1] > max_time:
            max_time = t_relative[-1]

        # 使用錨點回溯法來偵測，徹底避開前期漂移
        start_time = detect_motion_start_anchor(angle_relative, t_relative, TARGET_ANGLE)
        motion_start_times.append(start_time)

        color = colors[i % len(colors)]

        # 計算統計資訊
        total_rotation = angle_relative[-1] if len(angle_relative) > 0 else 0
        duration = t_relative[-1] if len(t_relative) > 0 else 1
        avg_angular_vel = total_rotation / duration if duration > 0 else 0

        # 構建 legend 標籤
        label = f"{name} (Δθ={total_rotation:.1f}°, ω̄={avg_angular_vel:.1f}°/s)"

        # 繪製完整資料
        ax.plot(t_relative, angle_relative, lw=3, color=color, label=label, alpha=0.8)

    # 決定 step 起跳時間
    if STEP_START_TIME is not None:
        step_time = STEP_START_TIME
    elif motion_start_times:
        step_time = np.mean(motion_start_times)
    else:
        step_time = 0.0

    print(f"  Step 起跳時間: {step_time:.3f} s"
          f" ({'手動設定' if STEP_START_TIME is not None else '自動偵測'})")

    # 繪製 target step response
    t_target = np.array([0, step_time - 1e-6, step_time, max_time])
    angle_target = np.array([0, 0, TARGET_ANGLE, TARGET_ANGLE])
    ax.plot(t_target, angle_target, lw=2.5, color='red', linestyle='-',
            label=f"Target ({TARGET_ANGLE:.1f}°)", alpha=0.9, zorder=10)

    # 設定圖表樣式
    ax.set_xlabel("Time (s)", fontsize=24, labelpad=15)
    ax.set_ylabel("Relative Angle (deg)", fontsize=24, labelpad=15)
    ax.set_title(title, fontsize=22, pad=20)
    ax.tick_params(axis='both', which='major', labelsize=16)
    ax.legend(loc="best", fontsize=14)

    # 調整佈局
    plt.tight_layout(pad=2.0)

    # 儲存圖表
    fig.savefig(output_path, dpi=1200, bbox_inches='tight', pad_inches=0.3)
    plt.close(fig)
    print(f"[OK] 角度比較圖（含 Target）已儲存: {output_path}")


# ============================================================
#  主程式
# ============================================================

def main():
    print("=" * 60)
    print("角度比較圖（含 Target Step Response）")
    print(f"  TARGET_ANGLE = {TARGET_ANGLE}°")
    print(f"  STEP_START_TIME = {STEP_START_TIME} {'(自動偵測)' if STEP_START_TIME is None else '(手動)'}")
    print("=" * 60)

    # 取得腳本所在目錄
    script_dir = os.path.dirname(os.path.abspath(__file__))
    csv_folder = os.path.join(script_dir, CSV_FOLDER)
    output_folder = os.path.join(script_dir, OUTPUT_FOLDER)

    print(f"\n讀取 CSV 檔案資料夾: {csv_folder}")

    # 載入所有 CSV 檔案
    data_dict = load_csv_files(csv_folder)

    if not data_dict:
        print("\n沒有可用的 CSV 檔案，程式結束")
        return

    print(f"\n共載入 {len(data_dict)} 個檔案")

    # 繪製角度比較圖（含 Target）
    output_path = os.path.join(output_folder, "angle_comparison_with_target.png")
    plot_angle_comparison_with_target(data_dict, output_path)

    print("\n" + "=" * 60)
    print("完成！")
    print("=" * 60)


if __name__ == "__main__":
    main()