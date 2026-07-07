# -*- coding: utf-8 -*-
"""
旋轉軌跡分析工具
讀取 data analysis (rotation)/file/ 資料夾中的所有 CSV 檔案，
將多條旋轉軌跡繪製在同一張圖上，標籤為檔案名稱。

使用方法：
1. 將實驗產出的 CSV 檔案複製到 data analysis (rotation)/file/ 資料夾
2. 執行此腳本：python rotation_trajectory_analysis.py
3. 輸出圖表將儲存在 data analysis (rotation)/ 資料夾

輸出圖表：
- position_comparison.png - 位置軌跡比較
- angle_comparison.png - 角度 vs 時間比較
- angular_velocity_comparison.png - 角速度 vs 時間比較
"""

import os
import glob
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
from scipy.ndimage import uniform_filter1d
from scipy.signal import savgol_filter


# 配置
CSV_FOLDER = "file"  # CSV 檔案資料夾（相對於此腳本的位置）
OUTPUT_FOLDER = "."  # 輸出資料夾（相對於此腳本的位置）
PLOT_RANGE_SCALE = 1.2  # 座標軸範圍縮放比例

# 平滑參數
SMOOTHING_ENABLED = True  # 是否啟用平滑
SMOOTHING_METHOD = "savgol"  # 平滑方法: "savgol" 或 "moving_avg"
SAVGOL_WINDOW = 11  # Savitzky-Golay 窗口大小（必須是奇數）
SAVGOL_POLY_ORDER = 3  # Savitzky-Golay 多項式階數
MOVING_AVG_WINDOW = 5  # 移動平均窗口大小

# 啟動偵測參數
MOTION_START_THRESHOLD = 10.0  # 角速度閾值（°/s），超過此值視為開始運動

# 角速度比較圖專用平滑參數
#   原始 angular_vel_dps 為逐點微分，會被「微小角度變化 ÷ 極短幀距」放大成
#   ±40~68°/s 的假尖峰，把真實 1~7°/s 的組間差距淹沒。故此圖改由「乾淨的
#   unwrapped 角度」重新微分並強化平滑，讓不同電壓的角速度清楚分層。
ANGVEL_SAVGOL_WINDOW = 31   # 角速度重算前後的 savgol 視窗（較大 → 更平滑）
ANGVEL_SAVGOL_POLY   = 2    # savgol 多項式階數
ANGVEL_INTERP_MAX_GAP = 20  # 角度內插補洞的最大缺口點數（超過保留斷線）
ANGVEL_USE_MAGNITUDE = True # True=畫 |ω|（單向旋轉，取絕對值更易分層比較）


def interp_nan(values, max_gap=None):
    """線性內插補內部 NaN 缺口（兩側都有有限值才補），讓序列連續、微分不斷。"""
    v = np.asarray(values, float).copy()
    n = len(v)
    finite = np.isfinite(v)
    if finite.sum() < 2:
        return v
    idx = np.arange(n)
    v_interp = np.interp(idx, idx[finite], v[finite])
    i = 0
    while i < n:
        if finite[i]:
            i += 1
            continue
        j = i
        while j < n and not finite[j]:
            j += 1
        if i - 1 >= 0 and j < n and (max_gap is None or (j - i) <= max_gap):
            v[i:j] = v_interp[i:j]
        i = j
    return v


def diff_with_dt_floor(values, t, dt_floor):
    """一階差分 dv/dt，並對 dt 設下限，抑制掉幀（短 dt）造成的假尖峰。"""
    v = np.full(len(values), np.nan)
    for i in range(1, len(values)):
        if np.isfinite(values[i]) and np.isfinite(values[i - 1]) and t[i] > t[i - 1]:
            dt = max(t[i] - t[i - 1], dt_floor)
            v[i] = (values[i] - values[i - 1]) / dt
    return v


def savgol_smooth(data, window, poly):
    """對有限值序列做 savgol 平滑（視窗自動縮成奇數且不超過資料長度）。"""
    data = np.asarray(data, float)
    n = np.isfinite(data).sum()
    if n < 5:
        return data
    w = min(window, n)
    if w % 2 == 0:
        w -= 1
    if w < 5:
        return data
    p = min(poly, w - 1)
    out = data.copy()
    fin = np.isfinite(data)
    out[fin] = savgol_filter(data[fin], w, p)
    return out


def smooth_data(data, method="savgol"):
    """
    對資料進行平滑處理
    
    Args:
        data: numpy 陣列
        method: 平滑方法 ("savgol" 或 "moving_avg")
    
    Returns:
        平滑後的資料
    """
    if not SMOOTHING_ENABLED or len(data) < SAVGOL_WINDOW:
        return data
    
    if method == "savgol":
        # Savitzky-Golay 濾波器 - 保持邊緣和峰值
        window = min(SAVGOL_WINDOW, len(data))
        if window % 2 == 0:
            window -= 1  # 確保是奇數
        if window < 3:
            return data
        poly_order = min(SAVGOL_POLY_ORDER, window - 1)
        return savgol_filter(data, window, poly_order)
    
    elif method == "moving_avg":
        # 移動平均濾波器
        return uniform_filter1d(data, size=MOVING_AVG_WINDOW, mode='nearest')
    
    return data


def load_csv_files(folder_path):
    """
    載入資料夾中所有 CSV 檔案
    
    Args:
        folder_path: CSV 檔案資料夾路徑
    
    Returns:
        字典 {檔案名稱: DataFrame}
    """
    csv_files = glob.glob(os.path.join(folder_path, "*.csv"))
    
    if not csv_files:
        print(f"警告：在 {folder_path} 中找不到 CSV 檔案")
        return {}
    
    data_dict = {}
    for csv_file in csv_files:
        try:
            # 使用檔案名稱（不含副檔名）作為標籤
            file_name = Path(csv_file).stem
            df = pd.read_csv(csv_file)
            
            # 檢查必要欄位
            if "x_mm" in df.columns and "y_mm" in df.columns:
                data_dict[file_name] = df
                print(f"✓ 已載入: {file_name} ({len(df)} 筆資料)")
            else:
                print(f"⚠ 跳過 {file_name}: 缺少 x_mm 或 y_mm 欄位")
        except Exception as e:
            print(f"✗ 載入 {csv_file} 失敗: {e}")
    
    return data_dict


def plot_multi_trajectories(data_dict, output_path, title="Position Comparison (Rotation)"):
    """
    繪製多軌跡比較圖（位置）
    
    Args:
        data_dict: {檔案名稱: DataFrame} 字典
        output_path: 輸出圖片路徑
        title: 圖表標題
    """
    if not data_dict:
        print("沒有資料可繪製")
        return
    
    # 創建圖表
    fig, ax = plt.subplots(1, 1, figsize=(9, 9))
    
    # 設定顏色循環
    colors = plt.cm.tab10.colors  # 使用 tab10 色盤
    
    # 收集所有資料的範圍
    all_x = []
    all_y = []
    
    # 繪製每條軌跡
    for i, (name, df) in enumerate(data_dict.items()):
        x = df["x_mm"].to_numpy()
        y = df["y_mm"].to_numpy()
        
        # 過濾無效值
        valid = np.isfinite(x) & np.isfinite(y)
        x_valid = x[valid]
        y_valid = y[valid]
        
        if len(x_valid) == 0:
            continue
        
        # 應用平滑濾波
        x_smooth = smooth_data(x_valid, SMOOTHING_METHOD)
        y_smooth = smooth_data(y_valid, SMOOTHING_METHOD)
        
        # 收集範圍（使用平滑後的資料）
        all_x.extend(x_smooth)
        all_y.extend(y_smooth)
        
        # 選擇顏色
        color = colors[i % len(colors)]
        
        # 計算旋轉相關統計資訊
        avg_angular_vel = total_rotation = None
        
        if "angular_vel_dps" in df.columns:
            ang_vel = df["angular_vel_dps"].to_numpy()
            ang_vel_valid = ang_vel[np.isfinite(ang_vel)]
            if len(ang_vel_valid) > 0:
                avg_angular_vel = np.mean(np.abs(ang_vel_valid))
        
        if "angle_deg_unwrapped" in df.columns:
            angle = df["angle_deg_unwrapped"].to_numpy()
            angle_valid = angle[np.isfinite(angle)]
            if len(angle_valid) > 1:
                total_rotation = np.abs(angle_valid[-1] - angle_valid[0])
        
        # 構建 legend 標籤（包含統計資訊）
        stat_parts = []
        if total_rotation is not None:
            stat_parts.append(f"Δθ={total_rotation:.1f}°")
        if avg_angular_vel is not None:
            stat_parts.append(f"ω̄={avg_angular_vel:.1f}°/s")
        
        if stat_parts:
            label = f"{name} ({', '.join(stat_parts)})"
        else:
            label = name
        
        # 繪製平滑後的軌跡
        ax.plot(x_smooth, y_smooth, lw=3, color=color, label=label, alpha=0.8)
        
        # 標記起點（使用原始資料的第一點）
        ax.scatter([x_valid[0]], [y_valid[0]], s=80, color=color, 
                   marker='o', edgecolors='black', linewidths=1, zorder=5)
        
        # 標記終點（使用原始資料的最後一點）
        ax.scatter([x_valid[-1]], [y_valid[-1]], s=80, color=color, 
                   marker='s', edgecolors='black', linewidths=1, zorder=5)
    
    # 設定座標軸範圍
    if all_x and all_y:
        xmin, xmax = min(all_x), max(all_x)
        ymin, ymax = min(all_y), max(all_y)
        xc = 0.5 * (xmin + xmax)
        yc = 0.5 * (ymin + ymax)
        half = 0.5 * max(xmax - xmin, ymax - ymin)
        half = max(half, 1e-6) * PLOT_RANGE_SCALE
        ax.set_xlim(xc - half, xc + half)
        ax.set_ylim(yc - half, yc + half)
    
    # 設定圖表樣式
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x (mm)", fontsize=24, labelpad=15)
    ax.set_ylabel("y (mm)", fontsize=24, labelpad=15)
    ax.set_title(title, fontsize=22, pad=20)
    ax.tick_params(axis='both', which='major', labelsize=16)
    #ax.grid(True, linestyle="--", alpha=0.4)
    ax.legend(loc="best", fontsize=14)
    
    # 添加圖例說明
    smooth_info = f" (Smoothed: {SMOOTHING_METHOD})" if SMOOTHING_ENABLED else ""
    ax.annotate(f"○ = Start, □ = End{smooth_info}", xy=(0.02, 0.02), xycoords='axes fraction',
                fontsize=12, color='gray')
    
    # 調整佈局以增加留白
    plt.tight_layout(pad=2.0)
    
    # 儲存圖表
    fig.savefig(output_path, dpi=1200, bbox_inches='tight', pad_inches=0.3)
    plt.close(fig)
    print(f"\n✓ 位置圖表已儲存: {output_path}")


def detect_motion_start(angular_vel, threshold=MOTION_START_THRESHOLD):
    """
    偵測運動開始的時間點索引
    
    Args:
        angular_vel: 角速度陣列
        threshold: 角速度閾值（°/s）
    
    Returns:
        運動開始的索引位置
    """
    # 使用滑動窗口來避免噪音干擾
    window_size = 5
    for i in range(len(angular_vel) - window_size):
        # 檢查連續幾個點的角速度是否都超過閾值
        window = angular_vel[i:i + window_size]
        if np.mean(np.abs(window)) > threshold:
            return i
    return 0  # 如果沒有找到，返回起始點


def plot_multi_angle_comparison(data_dict, output_path, title="Angle vs Time Comparison"):
    """
    繪製多軌跡角度比較圖（角度 vs 時間）
    排除靜止期間，從運動開始時計算
    
    Args:
        data_dict: {檔案名稱: DataFrame} 字典
        output_path: 輸出圖片路徑
        title: 圖表標題
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
        
        # 轉換為相對時間（從 0 開始）- 顯示完整資料
        t_relative = t_valid - t_valid[0]
        # 將角度轉換為相對角度（從 0 開始）
        angle_relative = angle_valid - angle_valid[0]
        
        # 偵測運動開始時間點（僅用於統計計算）
        motion_start_idx = 0
        if "angular_vel_dps" in df.columns:
            angular_vel = df["angular_vel_dps"].to_numpy()
            angular_vel_valid = angular_vel[valid]
            if len(angular_vel_valid) > 0:
                motion_start_idx = detect_motion_start(angular_vel_valid)
                print(f"  {name}: 偵測到運動開始於索引 {motion_start_idx} (時間 {t_relative[motion_start_idx]:.2f}s)")
        
        color = colors[i % len(colors)]
        
        # 計算統計資訊（僅計算運動期間，不包含靜止時間）
        if motion_start_idx < len(angle_relative):
            # 從運動開始到結束的角度變化
            total_rotation = angle_relative[-1] - angle_relative[motion_start_idx]
            # 運動期間的持續時間
            motion_duration = t_relative[-1] - t_relative[motion_start_idx]
            avg_angular_vel = total_rotation / motion_duration if motion_duration > 0 else 0
        else:
            total_rotation = angle_relative[-1] if len(angle_relative) > 0 else 0
            duration = t_relative[-1] if len(t_relative) > 0 else 1
            avg_angular_vel = total_rotation / duration if duration > 0 else 0
        
        # 構建 legend 標籤（包含統計資訊 - 僅運動期間）
        label = f"{name} (Δθ={total_rotation:.1f}°, ω̄={avg_angular_vel:.1f}°/s)"
        
        # 繪製完整資料（包含靜止部分）
        ax.plot(t_relative, angle_relative, lw=3, color=color, label=label, alpha=0.8)
    
    # 設定圖表樣式
    ax.set_xlabel("Time (s)", fontsize=24, labelpad=15)
    ax.set_ylabel("Relative Angle (deg)", fontsize=24, labelpad=15)
    ax.set_title(title, fontsize=22, pad=20)
    ax.tick_params(axis='both', which='major', labelsize=16)
    #ax.grid(True, linestyle="--", alpha=0.4)
    ax.legend(loc="best", fontsize=14)
    
    # 調整佈局以增加留白
    plt.tight_layout(pad=2.0)
    
    # 儲存圖表
    fig.savefig(output_path, dpi=1200, bbox_inches='tight', pad_inches=0.3)
    plt.close(fig)
    print(f"✓ 角度 vs 時間圖表已儲存: {output_path}")


def plot_multi_angular_velocity_comparison(data_dict, output_path, title="Angular Velocity Comparison"):
    """
    繪製多軌跡角速度比較圖
    
    Args:
        data_dict: {檔案名稱: DataFrame} 字典
        output_path: 輸出圖片路徑
        title: 圖表標題
    """
    if not data_dict:
        print("沒有資料可繪製")
        return
    
    # 檢查是否有角速度資料
    has_angular_vel = any("angular_vel_dps" in df.columns for df in data_dict.values())
    if not has_angular_vel:
        print("CSV 檔案中沒有 angular_vel_dps 欄位，跳過角速度比較圖")
        return
    
    # 創建圖表
    fig, ax = plt.subplots(1, 1, figsize=(11, 7.5))
    
    # 設定顏色循環
    colors = plt.cm.tab10.colors
    
    # 繪製每條軌跡的角速度
    #   不直接畫 CSV 的 angular_vel_dps（逐點微分，含短-dt 假尖峰、正負亂跳），
    #   改由乾淨的 unwrapped 角度重新微分：內插補洞 → savgol 平滑角度 →
    #   dt 下限差分 → 再平滑角速度。單向旋轉時取 |ω| 讓組間差距清楚分層。
    for i, (name, df) in enumerate(data_dict.items()):
        if "t_s" not in df.columns:
            continue

        t_all = df["t_s"].to_numpy(float)

        # 優先用 angle_deg_unwrapped 重算；若無則退回 CSV 既有角速度
        if "angle_deg_unwrapped" in df.columns:
            ang = df["angle_deg_unwrapped"].to_numpy(float)
            fin_t = np.isfinite(t_all)
            t = t_all[fin_t]
            ang = ang[fin_t]
            if len(t) < 5:
                continue
            t_relative = t - t[0]

            # dt 下限 = 中位數幀距，抑制掉幀放大
            dt_arr = np.diff(t)
            dt_arr = dt_arr[np.isfinite(dt_arr) & (dt_arr > 0)]
            dt_floor = float(np.median(dt_arr)) if len(dt_arr) else 0.0

            ang_i = interp_nan(ang, max_gap=ANGVEL_INTERP_MAX_GAP)
            ang_s = savgol_smooth(ang_i, ANGVEL_SAVGOL_WINDOW, ANGVEL_SAVGOL_POLY)
            w = diff_with_dt_floor(ang_s, t, dt_floor)
            w = savgol_smooth(w, ANGVEL_SAVGOL_WINDOW, ANGVEL_SAVGOL_POLY)
            if ANGVEL_USE_MAGNITUDE:
                w = np.abs(w)
        else:
            if "angular_vel_dps" not in df.columns:
                continue
            w_raw = df["angular_vel_dps"].to_numpy(float)
            valid = np.isfinite(t_all) & np.isfinite(w_raw)
            t_relative = t_all[valid] - t_all[valid][0]
            w = np.abs(w_raw[valid]) if ANGVEL_USE_MAGNITUDE else w_raw[valid]

        color = colors[i % len(colors)]

        # 統計：平均角速度用「總轉角 / 時間」（穩健、代表真實轉速），
        # 峰值用平滑後角速度的最大絕對值
        wv = w[np.isfinite(w)]
        if "angle_deg_unwrapped" in df.columns:
            fin_a = np.isfinite(ang_i)
            mean_rate = abs(ang_i[fin_a][-1] - ang_i[fin_a][0]) / (t_relative[-1] - t_relative[0])
        else:
            mean_rate = np.mean(wv) if len(wv) else 0.0
        max_w = np.max(wv) if len(wv) else 0.0

        label = f"{name} (Avg={mean_rate:.1f}, Max={max_w:.1f} °/s)"
        ax.plot(t_relative, w, lw=3, color=color, label=label, alpha=0.85)

    # 設定圖表樣式
    ax.set_xlabel("Time (s)", fontsize=24, labelpad=15)
    ax.set_ylabel("Angular Speed (°/s)", fontsize=24, labelpad=15)
    ax.set_title(title, fontsize=22, pad=20)
    ax.tick_params(axis='both', which='major', labelsize=16)
    #ax.grid(True, linestyle="--", alpha=0.4)
    ax.legend(loc="best", fontsize=14)

    # 取絕對值時不需要零線；保留有號時才畫零線
    if not ANGVEL_USE_MAGNITUDE:
        ax.axhline(y=0, color='gray', linestyle='-', linewidth=0.5, alpha=0.5)
    
    # 調整佈局以增加留白
    plt.tight_layout(pad=2.0)
    
    # 儲存圖表
    fig.savefig(output_path, dpi=1200, bbox_inches='tight', pad_inches=0.3)
    plt.close(fig)
    print(f"✓ 角速度比較圖已儲存: {output_path}")


def plot_mean_angular_speed_bar(data_dict, output_path, title="Mean Angular Speed"):
    """繪製各組平均角速度長條圖（一目了然電壓越高轉越快）。

    平均角速度 = |總轉角| / 持續時間（由 angle_deg_unwrapped 求得），這是最穩健、
    最能代表「整體轉多快」的量，不受瞬時微分雜訊影響。長條顏色與角速度曲線圖一致。

    Args:
        data_dict: {檔案名稱: DataFrame} 字典
        output_path: 輸出圖片路徑
        title: 圖表標題
    """
    if not data_dict:
        print("沒有資料可繪製")
        return

    colors = plt.cm.tab10.colors
    names, rates, bar_colors = [], [], []
    for i, (name, df) in enumerate(data_dict.items()):
        if "angle_deg_unwrapped" not in df.columns or "t_s" not in df.columns:
            continue
        t = df["t_s"].to_numpy(float)
        ang = df["angle_deg_unwrapped"].to_numpy(float)
        m = np.isfinite(t) & np.isfinite(ang)
        if m.sum() < 2:
            continue
        tt, aa = t[m], ang[m]
        dur = tt[-1] - tt[0]
        if dur <= 0:
            continue
        rate = abs(aa[-1] - aa[0]) / dur
        names.append(name)
        rates.append(rate)
        bar_colors.append(colors[i % len(colors)])

    if not names:
        print("沒有可用的角度資料，跳過平均角速度長條圖")
        return

    fig, ax = plt.subplots(1, 1, figsize=(9, 7))
    xpos = np.arange(len(names))
    bars = ax.bar(xpos, rates, color=bar_colors, alpha=0.85, width=0.6,
                  edgecolor='black', linewidth=1)
    # 每根長條頂端標數值
    for b, r in zip(bars, rates):
        ax.text(b.get_x() + b.get_width() / 2, r, f"{r:.1f}",
                ha='center', va='bottom', fontsize=18)

    ax.set_xticks(xpos)
    ax.set_xticklabels(names, fontsize=18)
    ax.set_ylabel("Mean Angular Speed (°/s)", fontsize=22, labelpad=12)
    ax.set_title(title, fontsize=22, pad=15)
    ax.tick_params(axis='y', which='major', labelsize=16)
    ax.set_ylim(0, max(rates) * 1.18)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)

    plt.tight_layout(pad=2.0)
    fig.savefig(output_path, dpi=1200, bbox_inches='tight', pad_inches=0.3)
    plt.close(fig)
    print(f"✓ 平均角速度長條圖已儲存: {output_path}")


def print_statistics(data_dict):
    """
    打印每個軌跡的統計資訊
    
    Args:
        data_dict: {檔案名稱: DataFrame} 字典
    """
    print("\n" + "=" * 60)
    print("旋轉軌跡統計資訊")
    print("=" * 60)
    
    for name, df in data_dict.items():
        print(f"\n【{name}】")
        
        # 計算軌跡長度
        x = df["x_mm"].to_numpy()
        y = df["y_mm"].to_numpy()
        valid = np.isfinite(x) & np.isfinite(y)
        x_valid = x[valid]
        y_valid = y[valid]
        
        if len(x_valid) > 1:
            # 總位移
            total_displacement = np.sqrt((x_valid[-1] - x_valid[0])**2 + 
                                          (y_valid[-1] - y_valid[0])**2)
            print(f"  - 總位移: {total_displacement:.2f} mm")
            print(f"  - 資料點數: {len(x_valid)}")
        
        # 角度統計
        if "angle_deg_unwrapped" in df.columns:
            angle = df["angle_deg_unwrapped"].to_numpy()
            angle_valid = angle[np.isfinite(angle)]
            if len(angle_valid) > 1:
                total_rotation = angle_valid[-1] - angle_valid[0]
                print(f"  - 總旋轉: {total_rotation:.2f}°")
        
        # 角速度統計
        if "angular_vel_dps" in df.columns:
            ang_vel = df["angular_vel_dps"].to_numpy()
            ang_vel_valid = ang_vel[np.isfinite(ang_vel)]
            if len(ang_vel_valid) > 0:
                avg_ang_vel = np.mean(np.abs(ang_vel_valid))
                max_ang_vel = np.max(np.abs(ang_vel_valid))
                print(f"  - 平均角速度: {avg_ang_vel:.2f} °/s")
                print(f"  - 最大角速度: {max_ang_vel:.2f} °/s")
        
        # 時間統計
        if "t_s" in df.columns:
            t = df["t_s"].to_numpy()
            t_valid = t[np.isfinite(t)]
            if len(t_valid) > 1:
                duration = t_valid[-1] - t_valid[0]
                print(f"  - 持續時間: {duration:.2f} s")


def main():
    """主程式"""
    print("=" * 60)
    print("旋轉軌跡分析工具")
    print("=" * 60)
    
    # 取得腳本所在目錄
    script_dir = os.path.dirname(os.path.abspath(__file__))
    csv_folder = os.path.join(script_dir, CSV_FOLDER)
    output_folder = script_dir
    
    print(f"\n讀取 CSV 檔案資料夾: {csv_folder}")
    
    # 載入所有 CSV 檔案
    data_dict = load_csv_files(csv_folder)
    
    if not data_dict:
        print("\n沒有可用的 CSV 檔案，程式結束")
        return
    
    print(f"\n共載入 {len(data_dict)} 個檔案")
    
    # 打印統計資訊
    print_statistics(data_dict)
    
    # 繪製圖表
    print("\n" + "=" * 60)
    print("生成圖表...")
    print("=" * 60)
    
    # 繪製位置軌跡比較圖
    trajectory_output = os.path.join(output_folder, "position_comparison.png")
    plot_multi_trajectories(data_dict, trajectory_output, title="Position Comparison (Rotation)")
    
    # 繪製角度 vs 時間比較圖
    angle_output = os.path.join(output_folder, "angle_comparison.png")
    plot_multi_angle_comparison(data_dict, angle_output, title="Angle vs Time Comparison")
    
    # 繪製角速度比較圖
    angular_vel_output = os.path.join(output_folder, "angular_velocity_comparison.png")
    plot_multi_angular_velocity_comparison(data_dict, angular_vel_output, title="Angular Speed Comparison")

    # 繪製平均角速度長條圖（一目了然的組間比較）
    bar_output = os.path.join(output_folder, "mean_angular_speed_bar.png")
    plot_mean_angular_speed_bar(data_dict, bar_output, title="Mean Angular Speed")

    print("\n" + "=" * 60)
    print("分析完成！")
    print("=" * 60)


if __name__ == "__main__":
    main()
