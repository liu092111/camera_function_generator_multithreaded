# -*- coding: utf-8 -*-
"""
多軌跡分析工具
讀取 data analysis/file/ 資料夾中的所有 CSV 檔案，
將多條軌跡繪製在同一張圖上，標籤為檔案名稱。

使用方法：
1. 將實驗產出的 CSV 檔案複製到 data analysis/file/ 資料夾
2. 執行此腳本：python multi_trajectory_analysis.py
3. 輸出圖表將儲存在 data analysis/ 資料夾

作者：多軌跡分析工具
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

# 啟動檢測參數
STARTUP_SPEED_THRESHOLD = 10.0  # 啟動速度閾值 (mm/s)，速度超過此值視為已啟動


def format_label_name(name):
    """把軌跡名稱中的 "Vpp" 轉成帶下標的 mathtext 形式 "V$_{pp}$"。

    matplotlib 以 $...$ 內的 _{} 表示下標，因此圖例會顯示為 V 加上下標 pp。
    大小寫不敏感（Vpp / VPP / vpp 皆可），其餘文字原樣保留。

    Args:
        name: 原始標籤字串（通常來自 CSV 檔名，例如 "36 Vpp"）

    Returns:
        轉換後的字串，例如 "36 V$_{pp}$"
    """
    import re
    return re.sub(r"[Vv]pp", "V$_{pp}$", name)


def _legend_overlaps_data(ax, legend, renderer):
    """檢查 legend 框是否與任何資料線重疊（皆換算到資料座標後比較）。

    只檢查真正的資料線（label 不以 '_' 開頭者），略過 axvline/axhline
    等輔助線與無 label 的物件。

    Returns:
        True 表示 legend 壓到資料，需要騰出空白；False 表示沒有重疊。
    """
    lbox = legend.get_window_extent(renderer)
    inv = ax.transData.inverted()
    # legend 四角換算到資料座標（取 min/max 以涵蓋座標軸方向反轉的情況）
    (x0, y0) = inv.transform((lbox.x0, lbox.y0))
    (x1, y1) = inv.transform((lbox.x1, lbox.y1))
    lxmin, lxmax = min(x0, x1), max(x0, x1)
    lymin, lymax = min(y0, y1), max(y0, y1)

    for line in ax.get_lines():
        label = line.get_label()
        if not label or label.startswith("_"):
            continue
        xd = np.asarray(line.get_xdata(), dtype=float)
        yd = np.asarray(line.get_ydata(), dtype=float)
        m = np.isfinite(xd) & np.isfinite(yd)
        if np.any((xd[m] >= lxmin) & (xd[m] <= lxmax) &
                  (yd[m] >= lymin) & (yd[m] <= lymax)):
            return True
    return False


def add_legend_with_headroom(fig, ax, loc="upper left", fontsize=14,
                             top_margin=0.05, keep_square=False, **legend_kwargs):
    """在繪圖區內放置 legend，並自動向上延伸 y 軸騰出空白帶，避免 legend 遮住資料。

    作法：先畫出 legend，量測它在顯示座標上佔了 axes 高度的多少比例 f，
    再把 y 軸上界往上拉，使原始資料只佔據下方 (1 - f - top_margin) 的高度，
    legend 則落在上方空白帶中。

    對等比例（aspect='equal'）的圖，延伸 ylim 會改變圖框高度，連帶改變 f，
    因此用數次迭代讓結果收斂。

    Args:
        fig, ax: matplotlib Figure / Axes
        loc: legend 位置（應為上方類，例如 "upper left" / "upper center"）
        fontsize: legend 字體大小
        top_margin: legend 與資料之間額外保留的空白比例
        keep_square: 若為 True，延伸 y 軸的同時把 x 軸設成相同範圍（資料置中），
                     使等比例（aspect='equal'）的圖維持正方形框格。
        legend_kwargs: 其餘傳給 ax.legend 的參數

    Returns:
        legend 物件
    """
    legend = ax.legend(loc=loc, fontsize=fontsize, framealpha=0.9, **legend_kwargs)

    # 先判斷 legend 是否真的壓到資料；沒壓到就保持原本軸範圍，不做任何延伸
    fig.canvas.draw()
    renderer = fig.canvas.get_renderer()
    if not _legend_overlaps_data(ax, legend, renderer):
        return legend

    # 以「目前 y 軸範圍」作為資料實際跨度的基準（ymin 固定，只往上延伸）
    ymin, ymax0 = ax.get_ylim()
    data_range = ymax0 - ymin
    if data_range <= 0:
        return legend

    # 正方形模式：記住目前 x 軸中心，延伸後讓 x 範圍 = y 範圍並置中
    xmin0, xmax0 = ax.get_xlim()
    xc = 0.5 * (xmin0 + xmax0)

    for _ in range(6):
        fig.canvas.draw()  # 觸發 renderer，才能取得 legend 的實際像素尺寸
        renderer = fig.canvas.get_renderer()
        legend_h = legend.get_window_extent(renderer).height
        axes_h = ax.get_window_extent(renderer).height
        if axes_h <= 0:
            break
        f = legend_h / axes_h
        # 資料可用的高度比例（保底 0.15，避免 legend 過大時把資料壓扁）
        avail = max(0.15, 1.0 - f - top_margin)
        new_range = data_range / avail
        ax.set_ylim(ymin, ymin + new_range)
        if keep_square:
            # x 範圍設成與 y 相同並置中，等比例下即為正方形框格
            ax.set_xlim(xc - 0.5 * new_range, xc + 0.5 * new_range)

    return legend


def detect_startup_index(speed_data, threshold=None):
    """
    檢測啟動時間點的索引
    
    Args:
        speed_data: 速度資料陣列
        threshold: 啟動速度閾值，若為 None 則使用全域設定
    
    Returns:
        啟動時間點的索引，若無法檢測則返回 0
    """
    if threshold is None:
        threshold = STARTUP_SPEED_THRESHOLD
    
    # 找到速度首次超過閾值的位置
    startup_indices = np.where(speed_data > threshold)[0]
    
    if len(startup_indices) > 0:
        return startup_indices[0]
    
    return 0


def calculate_avg_speed_after_startup(speed_data, threshold=None):
    """
    計算啟動後的平均速度
    
    Args:
        speed_data: 速度資料陣列
        threshold: 啟動速度閾值
    
    Returns:
        (啟動後的平均速度, 啟動索引)
    """
    startup_idx = detect_startup_index(speed_data, threshold)
    
    # 只計算啟動後的資料
    speed_after_startup = speed_data[startup_idx:]
    
    if len(speed_after_startup) > 0:
        return np.mean(speed_after_startup), startup_idx
    
    return np.mean(speed_data), 0


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


def plot_multi_trajectories(data_dict, output_path, title="Position Comparison",
                            show_stats=True):
    """
    繪製多軌跡比較圖

    Args:
        data_dict: {檔案名稱: DataFrame} 字典
        output_path: 輸出圖片路徑
        title: 圖表標題
        show_stats: 是否在 legend 標籤中附上統計數值（Avg/Max/Offset）。
                    設為 False 時，legend 只顯示材料名稱（簡化版）。
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
        
        # 構建 legend 標籤
        if show_stats:
            # 計算統計資訊
            avg_speed = max_speed = avg_offset = None

            if "speed_mm_s" in df.columns:
                speed = df["speed_mm_s"].to_numpy()
                speed_valid = speed[np.isfinite(speed)]
                if len(speed_valid) > 0:
                    avg_speed = np.mean(speed_valid)
                    max_speed = np.max(speed_valid)

            if "angle_deg_unwrapped" in df.columns:
                angle = df["angle_deg_unwrapped"].to_numpy()
                angle_valid = angle[np.isfinite(angle)]
                if len(angle_valid) > 0:
                    avg_offset = np.mean(np.abs(angle_valid - angle_valid[0]))

            stat_parts = []
            if avg_speed is not None:
                stat_parts.append(f"Avg={avg_speed:.1f}")
            if max_speed is not None:
                stat_parts.append(f"Max={max_speed:.1f} mm/s")
            if avg_offset is not None:
                stat_parts.append(f"Offset={avg_offset:.1f}°")

            disp_name = format_label_name(name)
            label = f"{disp_name} ({', '.join(stat_parts)})" if stat_parts else disp_name
        else:
            # 簡化版：只顯示材料名稱
            label = format_label_name(name)
        
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
    # legend 放在繪圖區內左上角，向上延伸 y 軸騰出空白帶避免遮住軌跡，
    # 並同步調整 x 軸範圍以維持正方形框格（keep_square=True）
    add_legend_with_headroom(fig, ax, loc="upper left", fontsize=14, keep_square=True)

    # 調整佈局以增加留白
    plt.tight_layout(pad=2.0)
    
    # 儲存圖表
    fig.savefig(output_path, dpi=1200, bbox_inches='tight', pad_inches=0.3)
    plt.close(fig)
    print(f"\n✓ 圖表已儲存: {output_path}")


def plot_multi_speed_comparison(data_dict, output_path, title="Speed Comparison"):
    """
    繪製多軌跡速度比較圖
    
    Args:
        data_dict: {檔案名稱: DataFrame} 字典
        output_path: 輸出圖片路徑
        title: 圖表標題
    """
    if not data_dict:
        print("沒有資料可繪製")
        return
    
    # 檢查是否有速度資料
    has_speed = any("speed_mm_s" in df.columns for df in data_dict.values())
    if not has_speed:
        print("CSV 檔案中沒有 speed_mm_s 欄位，跳過速度比較圖")
        return
    
    # 創建圖表
    fig, ax = plt.subplots(1, 1, figsize=(11, 8))
    
    # 設定顏色循環
    colors = plt.cm.tab10.colors
    
    # 繪製每條軌跡的速度
    for i, (name, df) in enumerate(data_dict.items()):
        if "speed_mm_s" not in df.columns or "t_s" not in df.columns:
            continue
        
        t = df["t_s"].to_numpy()
        speed = df["speed_mm_s"].to_numpy()
        
        # 轉換為相對時間（從 0 開始）
        valid = np.isfinite(t) & np.isfinite(speed)
        t_valid = t[valid]
        speed_valid = speed[valid]
        
        if len(t_valid) > 0:
            t_relative = t_valid - t_valid[0]
            color = colors[i % len(colors)]
            
            # 計算統計資訊（使用啟動後的平均速度）
            avg_speed, startup_idx = calculate_avg_speed_after_startup(speed_valid)
            max_speed = np.max(speed_valid)
            
            # 構建 legend 標籤（包含統計資訊）
            label = f"{format_label_name(name)} (Avg={avg_speed:.1f}, Max={max_speed:.1f} mm/s)"
            
            ax.plot(t_relative, speed_valid, lw=3, color=color, label=label, alpha=0.8)
            
            # 標記啟動時間點
            if startup_idx > 0:
                ax.axvline(x=t_relative[startup_idx], color=color, linestyle='--', alpha=0.3, lw=1)
    
    # 設定圖表樣式
    ax.set_xlabel("Time (s)", fontsize=24, labelpad=15)
    ax.set_ylabel("Speed (mm/s)", fontsize=24, labelpad=15)
    ax.set_title(title, fontsize=22, pad=20)
    ax.tick_params(axis='both', which='major', labelsize=16)
    #ax.grid(True, linestyle="--", alpha=0.4)
    # legend 放在繪圖區內左上角，並向上延伸 y 軸騰出空白帶，避免遮住速度曲線
    add_legend_with_headroom(fig, ax, loc="upper left", fontsize=14)

    # 調整佈局以增加留白
    plt.tight_layout(pad=2.0)

    # 儲存圖表
    fig.savefig(output_path, dpi=1200, bbox_inches='tight', pad_inches=0.3)
    plt.close(fig)
    print(f"✓ 速度比較圖已儲存: {output_path}")


def plot_multi_speed_simple(data_dict, output_path, title="Speed Comparison (Simple)"):
    """
    繪製多軌跡速度比較圖（簡化版，正方形，legend 只顯示檔案名稱）
    
    Args:
        data_dict: {檔案名稱: DataFrame} 字典
        output_path: 輸出圖片路徑
        title: 圖表標題
    """
    if not data_dict:
        print("沒有資料可繪製")
        return
    
    # 檢查是否有速度資料
    has_speed = any("speed_mm_s" in df.columns for df in data_dict.values())
    if not has_speed:
        print("CSV 檔案中沒有 speed_mm_s 欄位，跳過簡化速度比較圖")
        return
    
    # 創建長方形圖表
    fig, ax = plt.subplots(1, 1, figsize=(11, 8))
    
    # 設定顏色循環
    colors = plt.cm.tab10.colors
    
    # 繪製每條軌跡的速度
    for i, (name, df) in enumerate(data_dict.items()):
        if "speed_mm_s" not in df.columns or "t_s" not in df.columns:
            continue
        
        t = df["t_s"].to_numpy()
        speed = df["speed_mm_s"].to_numpy()
        
        # 轉換為相對時間（從 0 開始）
        valid = np.isfinite(t) & np.isfinite(speed)
        t_valid = t[valid]
        speed_valid = speed[valid]
        
        if len(t_valid) > 0:
            t_relative = t_valid - t_valid[0]
            color = colors[i % len(colors)]
            
            # Legend 只顯示檔案名稱
            ax.plot(t_relative, speed_valid, lw=3, color=color, label=format_label_name(name), alpha=0.8)
    
    # 設定圖表樣式
    ax.set_xlabel("Time (s)", fontsize=24, labelpad=15)
    ax.set_ylabel("Speed (mm/s)", fontsize=24, labelpad=15)
    ax.set_title(title, fontsize=22, pad=20)
    ax.tick_params(axis='both', which='major', labelsize=16)
    #ax.grid(True, linestyle="--", alpha=0.4)
    # legend 放在繪圖區內左上角，並向上延伸 y 軸騰出空白帶，避免遮住速度曲線
    add_legend_with_headroom(fig, ax, loc="upper left", fontsize=14)
    
    # 調整佈局以增加留白
    plt.tight_layout(pad=2.0)
    
    # 儲存圖表
    fig.savefig(output_path, dpi=1200, bbox_inches='tight', pad_inches=0.3)
    plt.close(fig)
    print(f"✓ 簡化速度比較圖已儲存: {output_path}")


def plot_multi_angle_comparison(data_dict, output_path, title="Orientation Comparison"):
    """
    繪製多軌跡角度比較圖
    
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
    fig, ax = plt.subplots(1, 1, figsize=(11, 7.5))
    
    # 設定顏色循環
    colors = plt.cm.tab10.colors
    
    # 繪製每條軌跡的角度
    for i, (name, df) in enumerate(data_dict.items()):
        if "angle_deg_unwrapped" not in df.columns or "t_s" not in df.columns:
            continue
        
        t = df["t_s"].to_numpy()
        angle = df["angle_deg_unwrapped"].to_numpy()
        
        # 轉換為相對時間（從 0 開始）
        valid = np.isfinite(t) & np.isfinite(angle)
        t_valid = t[valid]
        angle_valid = angle[valid]
        
        if len(t_valid) > 0:
            t_relative = t_valid - t_valid[0]
            # 將角度轉換為相對角度（從 0 開始）
            angle_relative = angle_valid - angle_valid[0]
            color = colors[i % len(colors)]
            
            # 計算平均偏移（相對於起始角度的平均絕對偏差）
            avg_offset = np.mean(np.abs(angle_relative))
            
            # 構建 legend 標籤（包含統計資訊）
            label = f"{format_label_name(name)} (Avg Offset = {avg_offset:.2f}°)"
            
            ax.plot(t_relative, angle_relative, lw=3, color=color, label=label, alpha=0.8)
    
    # 設定圖表樣式
    ax.set_xlabel("Time (s)", fontsize=24, labelpad=15)
    ax.set_ylabel("Relative Angle (deg)", fontsize=24, labelpad=15)
    ax.set_title(title, fontsize=22, pad=20)
    ax.tick_params(axis='both', which='major', labelsize=16)
    # Y 軸刻度自動選取整數間距，控制刻度數量在可清楚標示的範圍（約 8~10 條）
    from matplotlib.ticker import MaxNLocator
    ax.yaxis.set_major_locator(MaxNLocator(nbins=9, steps=[1, 2, 2.5, 5, 10], integer=True))
    #ax.grid(True, linestyle="--", alpha=0.4)
    # legend 放在繪圖區內左上角，並向上延伸 y 軸騰出空白帶，避免遮住角度曲線
    add_legend_with_headroom(fig, ax, loc="upper left", fontsize=14)
    
    # 調整佈局以增加留白
    plt.tight_layout(pad=2.0)
    
    # 儲存圖表
    fig.savefig(output_path, dpi=1200, bbox_inches='tight', pad_inches=0.3)
    plt.close(fig)
    print(f"✓ 角度比較圖已儲存: {output_path}")


def plot_multi_position_error(data_dict, output_path, title="Position Error"):
    """
    繪製多軌跡的「位置誤差」比較圖（單張）。

    定義（依使用者設定）：
    - 理想直線 = 垂直軸（y 方向）。致動器理想上應沿垂直方向直走。
    - 位置誤差 = 每一點相對起點的橫向（x 方向）位移，正值代表偏右、負值偏左。
    - 橫軸 = 沿垂直方向已前進的距離 |Δy|（mm），而非時間。

    與 orientation 圖的差異：這裡量的是「偏掉的實際距離 (mm)」，
    不是行進方向的角度偏差。

    Args:
        data_dict: {檔案名稱: DataFrame} 字典
        output_path: 輸出圖片路徑
        title: 圖表標題
    """
    if not data_dict:
        print("沒有資料可繪製")
        return

    fig, ax = plt.subplots(1, 1, figsize=(11, 8))
    colors = plt.cm.tab10.colors

    for i, (name, df) in enumerate(data_dict.items()):
        if "x_mm" not in df.columns or "y_mm" not in df.columns:
            continue

        x = df["x_mm"].to_numpy()
        y = df["y_mm"].to_numpy()

        valid = np.isfinite(x) & np.isfinite(y)
        x_valid = x[valid]
        y_valid = y[valid]

        if len(x_valid) < 2:
            continue

        # 套用與位置/偏移圖相同的平滑，避免逐點雜訊放大誤差
        x_smooth = smooth_data(x_valid, SMOOTHING_METHOD)
        y_smooth = smooth_data(y_valid, SMOOTHING_METHOD)

        # 橫向誤差（相對起點的 x 位移）與沿垂直方向前進距離
        lateral_error = x_smooth - x_smooth[0]
        vertical_distance = np.abs(y_smooth - y_smooth[0])

        color = colors[i % len(colors)]

        # 統計：最大絕對偏移與 RMS 偏移
        max_abs = np.max(np.abs(lateral_error))
        rms = np.sqrt(np.mean(lateral_error**2))
        label = f"{format_label_name(name)} (Max={max_abs:.2f}, RMS={rms:.2f} mm)"

        ax.plot(vertical_distance, lateral_error, lw=3, color=color,
                label=label, alpha=0.8)

    # 理想直線（零誤差）參考線
    ax.axhline(y=0, color='gray', linestyle='--', alpha=0.5, lw=1)

    ax.set_xlabel("Vertical Distance Traveled (mm)", fontsize=24, labelpad=15)
    ax.set_ylabel("Lateral Position Error (mm)", fontsize=24, labelpad=15)
    ax.set_title(title, fontsize=22, pad=20)
    ax.tick_params(axis='both', which='major', labelsize=16)
    add_legend_with_headroom(fig, ax, loc="upper left", fontsize=14)

    plt.tight_layout(pad=2.0)
    fig.savefig(output_path, dpi=1200, bbox_inches='tight', pad_inches=0.3)
    plt.close(fig)
    print(f"✓ 位置誤差比較圖已儲存: {output_path}")


def plot_multi_trajectory_deviation(data_dict, output_path, title="Trajectory Deviation from Vertical"):
    """
    繪製多軌跡相對於垂直線的偏移比較圖（三個子圖）
    
    假設起始點預設走垂直方向 (y 軸方向)，計算：
    1. 瞬時行進方向 (heading) 相對於垂直軸的偏移角度（每步的速度方向）
    2. 累積偏移角度（從起點到當前位置的連線 vs 垂直軸）
    3. 橫向偏移量 vs 縱向行走距離
    
    Args:
        data_dict: {檔案名稱: DataFrame} 字典
        output_path: 輸出圖片路徑
        title: 圖表標題
    """
    if not data_dict:
        print("沒有資料可繪製")
        return
    
    # 創建三個子圖
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(11, 18), gridspec_kw={'height_ratios': [1, 1, 1]})
    
    # 設定顏色循環
    colors = plt.cm.tab10.colors
    
    for i, (name, df) in enumerate(data_dict.items()):
        if "x_mm" not in df.columns or "y_mm" not in df.columns:
            continue
        
        x = df["x_mm"].to_numpy()
        y = df["y_mm"].to_numpy()
        
        # 過濾無效值
        valid = np.isfinite(x) & np.isfinite(y)
        x_valid = x[valid]
        y_valid = y[valid]
        
        if len(x_valid) < 2:
            continue
        
        # 應用平滑濾波
        x_smooth = smooth_data(x_valid, SMOOTHING_METHOD)
        y_smooth = smooth_data(y_valid, SMOOTHING_METHOD)
        
        # 起始位置
        x0 = x_smooth[0]
        y0 = y_smooth[0]
        
        # ====== 計算瞬時行進方向 (heading) 相對於垂直軸的偏移 ======
        # 每一步的位移差
        step_dx = np.diff(x_smooth)
        step_dy = np.diff(y_smooth)
        step_dist = np.sqrt(step_dx**2 + step_dy**2)
        
        # 瞬時 heading 偏移角度：arctan2(dx, dy)，垂直走為 0°
        instant_heading = np.full_like(step_dx, np.nan)
        significant_step = step_dist > 1e-6  # 避免靜止時的雜訊
        if np.any(significant_step):
            instant_heading[significant_step] = np.degrees(
                np.arctan2(step_dx[significant_step], step_dy[significant_step])
            )
        
        # ====== 計算累積偏移角度（起點到當前位置 vs 垂直線）======
        dx_cumul = x_smooth - x0
        dy_cumul = y_smooth - y0
        y_distance = np.abs(dy_cumul)
        
        cumul_deviation = np.full_like(dx_cumul, np.nan)
        significant_move = y_distance > 0.01
        if np.any(significant_move):
            cumul_deviation[significant_move] = np.degrees(
                np.arctan2(dx_cumul[significant_move], dy_cumul[significant_move])
            )
        
        # 取得時間軸
        if "t_s" in df.columns:
            t = df["t_s"].to_numpy()
            t_valid = t[valid]
            t_relative = t_valid - t_valid[0]
        else:
            t_relative = np.arange(len(x_valid))
        
        # heading 的時間軸（取中點）
        t_heading = 0.5 * (t_relative[:-1] + t_relative[1:])
        
        color = colors[i % len(colors)]
        
        # 計算統計量
        valid_heading = instant_heading[np.isfinite(instant_heading)]
        valid_cumul = cumul_deviation[np.isfinite(cumul_deviation)]
        
        disp_name = format_label_name(name)

        # heading 統計
        if len(valid_heading) > 0:
            avg_heading = np.mean(valid_heading)
            label_heading = f"{disp_name} (Avg={avg_heading:.2f}°)"
        else:
            label_heading = disp_name

        # 累積偏移統計
        if len(valid_cumul) > 0:
            avg_cumul = np.mean(np.abs(valid_cumul))
            label_cumul = f"{disp_name} (Avg |θ|={avg_cumul:.2f}°)"
        else:
            label_cumul = disp_name

        # 橫向偏移統計
        final_lateral_offset = dx_cumul[-1]
        label_lateral = f"{disp_name} (Final offset={final_lateral_offset:.3f} mm)"
        
        # --- 子圖 1: 瞬時 heading 偏移角度 vs 時間 ---
        valid_h = np.isfinite(instant_heading)
        if np.any(valid_h):
            ax1.plot(t_heading[valid_h], instant_heading[valid_h],
                     lw=1.5, color=color, label=label_heading, alpha=0.8)
        
        # --- 子圖 2: 累積偏移角度 vs 時間 ---
        valid_c = np.isfinite(cumul_deviation)
        if np.any(valid_c):
            ax2.plot(t_relative[valid_c], cumul_deviation[valid_c],
                     lw=1.5, color=color, label=label_cumul, alpha=0.8)
        
        # --- 子圖 3: 橫向偏移 (x) vs 縱向位移 (y) ---
        ax3.plot(y_distance, dx_cumul, lw=1.5, color=color, label=label_lateral, alpha=0.8)
    
    # --- 設定子圖 1 樣式 (瞬時 heading) ---
    ax1.axhline(y=0, color='gray', linestyle='--', alpha=0.5, lw=1)
    ax1.set_xlabel("Time (s)", fontsize=20, labelpad=12)
    ax1.set_ylabel("Instantaneous Heading (°)", fontsize=20, labelpad=12)
    ax1.set_title("Instantaneous Heading Deviation from Vertical", fontsize=20, pad=15)
    ax1.tick_params(axis='both', which='major', labelsize=14)
    add_legend_with_headroom(fig, ax1, loc="upper left", fontsize=12)

    # --- 設定子圖 2 樣式 (累積偏移) ---
    ax2.axhline(y=0, color='gray', linestyle='--', alpha=0.5, lw=1)
    ax2.set_xlabel("Time (s)", fontsize=20, labelpad=12)
    ax2.set_ylabel("Cumulative Deviation Angle (°)", fontsize=20, labelpad=12)
    ax2.set_title("Cumulative Direction from Start vs Vertical", fontsize=20, pad=15)
    ax2.tick_params(axis='both', which='major', labelsize=14)
    add_legend_with_headroom(fig, ax2, loc="upper left", fontsize=12)
    
    # --- 設定子圖 3 樣式 (橫向偏移) ---
    ax3.axhline(y=0, color='gray', linestyle='--', alpha=0.5, lw=1)
    ax3.set_xlabel("Vertical Distance Traveled (mm)", fontsize=20, labelpad=12)
    ax3.set_ylabel("Lateral Offset (mm)", fontsize=20, labelpad=12)
    ax3.set_title("Lateral Drift vs Vertical Travel Distance", fontsize=20, pad=15)
    ax3.tick_params(axis='both', which='major', labelsize=14)
    add_legend_with_headroom(fig, ax3, loc="upper left", fontsize=12)
    
    # 調整佈局
    plt.tight_layout(pad=2.5)
    
    # 儲存圖表
    fig.savefig(output_path, dpi=1200, bbox_inches='tight', pad_inches=0.3)
    plt.close(fig)
    print(f"✓ 軌跡偏移比較圖已儲存: {output_path}")


def print_statistics(data_dict):
    """
    打印每個軌跡的統計資訊
    
    Args:
        data_dict: {檔案名稱: DataFrame} 字典
    """
    print("\n" + "=" * 60)
    print("軌跡統計資訊")
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
            
            # 軌跡長度（路徑積分）
            dx = np.diff(x_valid)
            dy = np.diff(y_valid)
            path_length = np.sum(np.sqrt(dx**2 + dy**2))
            
            print(f"  - 總位移: {total_displacement:.2f} mm")
            print(f"  - 軌跡長度: {path_length:.2f} mm")
            print(f"  - 資料點數: {len(x_valid)}")
        
        # 速度統計
        if "speed_mm_s" in df.columns:
            speed = df["speed_mm_s"].to_numpy()
            speed_valid = speed[np.isfinite(speed)]
            if len(speed_valid) > 0:
                print(f"  - 平均速度: {np.mean(speed_valid):.2f} mm/s")
                print(f"  - 最大速度: {np.max(speed_valid):.2f} mm/s")
        
        # 角度統計
        if "angle_deg_unwrapped" in df.columns:
            angle = df["angle_deg_unwrapped"].to_numpy()
            angle_valid = angle[np.isfinite(angle)]
            if len(angle_valid) > 1:
                total_rotation = angle_valid[-1] - angle_valid[0]
                print(f"  - 總旋轉: {total_rotation:.2f}°")
        
        # 軌跡偏移統計（相對於垂直線）
        if len(x_valid) > 1:
            x_smooth = smooth_data(x_valid, SMOOTHING_METHOD)
            y_smooth = smooth_data(y_valid, SMOOTHING_METHOD)
            dx_total = x_smooth[-1] - x_smooth[0]
            dy_total = y_smooth[-1] - y_smooth[0]
            y_dist = np.abs(dy_total)
            if y_dist > 0.01:
                deviation_angle = np.degrees(np.arctan2(dx_total, dy_total))
                print(f"  - 軌跡偏移角度 (相對垂直): {deviation_angle:.2f}°")
                print(f"  - 橫向偏移量: {dx_total:.4f} mm")
                print(f"  - 縱向位移量: {dy_total:.4f} mm")
        
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
    print("多軌跡分析工具")
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
    
    # 繪製軌跡比較圖
    print("\n" + "=" * 60)
    print("生成圖表...")
    print("=" * 60)
    
    trajectory_output = os.path.join(output_folder, "position_comparison.png")
    plot_multi_trajectories(data_dict, trajectory_output, title="Position Comparison")

    # 繪製簡化版軌跡比較圖（legend 只顯示材料名稱，不含 Avg/Max/Offset 數值）
    trajectory_simple_output = os.path.join(output_folder, "position_comparison_simple.png")
    plot_multi_trajectories(data_dict, trajectory_simple_output,
                            title="Position Comparison", show_stats=False)

    # 繪製速度比較圖
    speed_output = os.path.join(output_folder, "speed_comparison.png")
    plot_multi_speed_comparison(data_dict, speed_output, title="Speed Comparison")
    
    # 繪製簡化速度比較圖（正方形，legend 只顯示檔案名稱）
    speed_simple_output = os.path.join(output_folder, "speed_comparison_simple.png")
    plot_multi_speed_simple(data_dict, speed_simple_output, title="Speed Comparison")
    
    # 繪製角度比較圖
    angle_output = os.path.join(output_folder, "orientation_comparison.png")
    plot_multi_angle_comparison(data_dict, angle_output, title="Orientation Comparison")

    # 繪製位置誤差比較圖（橫向偏離理想垂直直線的距離 vs 前進距離）
    error_output = os.path.join(output_folder, "position_error_comparison.png")
    plot_multi_position_error(data_dict, error_output, title="Position Error")

    print("\n" + "=" * 60)
    print("分析完成！")
    print("=" * 60)


if __name__ == "__main__":
    main()
