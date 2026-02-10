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
        ax.plot(x_smooth, y_smooth, lw=2, color=color, label=label, alpha=0.8)
        
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
    ax.set_xlabel("x (mm)", fontsize=20, labelpad=15)
    ax.set_ylabel("y (mm)", fontsize=20, labelpad=15)
    ax.set_title(title, fontsize=22, pad=20)
    ax.tick_params(axis='both', which='major', labelsize=16)
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.legend(loc="best", fontsize=14)
    
    # 添加圖例說明
    smooth_info = f" (Smoothed: {SMOOTHING_METHOD})" if SMOOTHING_ENABLED else ""
    ax.annotate(f"○ = Start, □ = End{smooth_info}", xy=(0.02, 0.02), xycoords='axes fraction',
                fontsize=12, color='gray')
    
    # 調整佈局以增加留白
    plt.tight_layout(pad=2.0)
    
    # 儲存圖表
    fig.savefig(output_path, dpi=220, bbox_inches='tight', pad_inches=0.3)
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
        ax.plot(t_relative, angle_relative, lw=1.5, color=color, label=label, alpha=0.8)
    
    # 設定圖表樣式
    ax.set_xlabel("Time (s)", fontsize=20, labelpad=15)
    ax.set_ylabel("Relative Angle (deg)", fontsize=20, labelpad=15)
    ax.set_title(title, fontsize=22, pad=20)
    ax.tick_params(axis='both', which='major', labelsize=16)
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.legend(loc="best", fontsize=14)
    
    # 調整佈局以增加留白
    plt.tight_layout(pad=2.0)
    
    # 儲存圖表
    fig.savefig(output_path, dpi=220, bbox_inches='tight', pad_inches=0.3)
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
    for i, (name, df) in enumerate(data_dict.items()):
        if "angular_vel_dps" not in df.columns or "t_s" not in df.columns:
            continue
        
        t = df["t_s"].to_numpy()
        angular_vel = df["angular_vel_dps"].to_numpy()
        
        # 轉換為相對時間（從 0 開始）
        valid = np.isfinite(t) & np.isfinite(angular_vel)
        t_valid = t[valid]
        angular_vel_valid = angular_vel[valid]
        
        if len(t_valid) > 0:
            t_relative = t_valid - t_valid[0]
            color = colors[i % len(colors)]
            
            # 計算統計資訊
            avg_angular_vel = np.mean(np.abs(angular_vel_valid))
            max_angular_vel = np.max(np.abs(angular_vel_valid))
            
            # 構建 legend 標籤（包含統計資訊）
            label = f"{name} (Avg={avg_angular_vel:.1f}, Max={max_angular_vel:.1f} °/s)"
            
            ax.plot(t_relative, angular_vel_valid, lw=1.5, color=color, label=label, alpha=0.8)
    
    # 設定圖表樣式
    ax.set_xlabel("Time (s)", fontsize=20, labelpad=15)
    ax.set_ylabel("Angular Velocity (°/s)", fontsize=20, labelpad=15)
    ax.set_title(title, fontsize=22, pad=20)
    ax.tick_params(axis='both', which='major', labelsize=16)
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.legend(loc="best", fontsize=14)
    
    # 添加零線
    ax.axhline(y=0, color='gray', linestyle='-', linewidth=0.5, alpha=0.5)
    
    # 調整佈局以增加留白
    plt.tight_layout(pad=2.0)
    
    # 儲存圖表
    fig.savefig(output_path, dpi=220, bbox_inches='tight', pad_inches=0.3)
    plt.close(fig)
    print(f"✓ 角速度比較圖已儲存: {output_path}")


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
    plot_multi_angular_velocity_comparison(data_dict, angular_vel_output, title="Angular Velocity Comparison")
    
    print("\n" + "=" * 60)
    print("分析完成！")
    print("=" * 60)


if __name__ == "__main__":
    main()
