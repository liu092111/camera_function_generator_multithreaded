# -*- coding: utf-8 -*-
"""
資料匯出與視覺化模組
提供資料處理和圖表生成功能
"""

import os
import cv2
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from config import (
    MODE, INVERT_Y_AXIS, ORIENT_PLOT_WRAPPED, 
    ORIENT_YLIM_DEG, PLOT_RANGE_SCALE
)
from signal_processing import unwrap_angles_deg, unwrap_angles_continuous, wrap_angles_deg, moving_average, finite_diff


def process_and_export_data(rec_data, output_dir, mm_per_px):
    """
    處理並輸出追蹤資料 - 包含圖表生成
    
    Args:
        rec_data: 記錄的資料列表
        output_dir: 輸出目錄
        mm_per_px: mm/pixel 比例
    """
    OUT_PREFIX = f"camera_{MODE}"
    
    # 分離 box 資料
    box_rec = []
    data_without_box = []
    for row in rec_data:
        if len(row) >= 9:  # 包含 box
            frame_idx, t_s, fx, fy, fx_mm_abs, fy_mm_abs, angle, mmpx, box = row
            box_rec.append((frame_idx, t_s, fx, fy, box))
            data_without_box.append((frame_idx, t_s, fx, fy, fx_mm_abs, fy_mm_abs, angle, mmpx))
        else:
            data_without_box.append(row)
    
    # 整理資料
    df = pd.DataFrame(data_without_box, columns=[
        "frame", "t_s", "x_px_filt", "y_px_filt", "x_mm_abs", "y_mm_abs", "angle_deg_raw", "mm_per_px"
    ])
    
    # 計算相對座標
    x_abs = df["x_mm_abs"].to_numpy()
    y_abs = df["y_mm_abs"].to_numpy()
    valid = np.isfinite(x_abs) & np.isfinite(y_abs)
    if valid.any():
        x0, y0 = x_abs[valid][0], y_abs[valid][0]
        df["x_mm"] = x_abs - x0
        df["y_mm"] = -(y_abs - y0)  # 修正Y座標
    else:
        df["x_mm"] = x_abs
        df["y_mm"] = -y_abs
    
    # 速度計算
    t = df["t_s"].to_numpy()
    
    # 角度處理
    # 因為矩形追蹤的角度範圍是 [-90°, 90°]（180° 週期）
    ang_raw = df["angle_deg_raw"].to_numpy()
    mask_ang = np.isfinite(ang_raw)
    ang_unwrap = np.full_like(ang_raw, np.nan, dtype=float)
    if mask_ang.any():
        if MODE.lower() == "rotation":
            # rotation mode 使用連續解包裹，確保角度可以持續累積超過 ±360°
            ang_unwrap[mask_ang] = unwrap_angles_continuous(ang_raw[mask_ang], period=180)
            # 偵測並移除角度跳躍（由於追蹤失敗導致的不合理變化）
            # 計算角度變化率，過濾掉變化太快的點
            valid_idx = np.where(mask_ang)[0]
            if len(valid_idx) > 1:
                for i in range(1, len(valid_idx)):
                    curr_idx = valid_idx[i]
                    prev_idx = valid_idx[i-1]
                    dt = t[curr_idx] - t[prev_idx] if t[curr_idx] > t[prev_idx] else 0.01
                    dtheta = abs(ang_unwrap[curr_idx] - ang_unwrap[prev_idx])
                    # 如果角度變化率超過 500 deg/s，可能是追蹤失敗
                    if dt > 0 and dtheta / dt > 500:
                        ang_unwrap[curr_idx] = np.nan
        else:
            # straight mode 使用標準解包裹
            ang_unwrap[mask_ang] = unwrap_angles_deg(ang_raw[mask_ang], period=180)
        ang_unwrap = moving_average(ang_unwrap, 5)
    df["angle_deg_unwrapped"] = ang_unwrap
    
    # 位移速度計算
    vx = finite_diff(df["x_mm"].to_numpy(), t, smooth_win=5)
    vy = finite_diff(df["y_mm"].to_numpy(), t, smooth_win=5)
    speed = np.sqrt(vx**2 + vy**2)
    df["vx_mm_s"] = vx
    df["vy_mm_s"] = vy
    df["speed_mm_s"] = speed
    
    # 角速度計算（for rotation mode）
    ang_vel = np.full_like(ang_unwrap, np.nan, dtype=float)
    
    if MODE.lower() == "rotation":
        # 使用解包裹後的角度來計算角速度
        # 找出有效的角度索引
        valid_ang_idx = np.where(np.isfinite(ang_unwrap))[0]
        
        if len(valid_ang_idx) >= 2:
            # 計算角速度：使用有限差分
            for i in range(1, len(valid_ang_idx)):
                curr_idx = valid_ang_idx[i]
                prev_idx = valid_ang_idx[i-1]
                dt = t[curr_idx] - t[prev_idx]
                if dt > 0:
                    dtheta = ang_unwrap[curr_idx] - ang_unwrap[prev_idx]
                    ang_vel[curr_idx] = dtheta / dt
            
            # 第一步：移除明顯異常的角速度（基於物理合理性）
            # 假設最大合理角速度不超過 500 deg/s
            MAX_REASONABLE_ANGULAR_VEL = 500.0
            ang_vel[np.abs(ang_vel) > MAX_REASONABLE_ANGULAR_VEL] = np.nan
            
            # 第二步：計算穩態區域的統計資訊（排除開始和結束各 10%）
            valid_ang_vel = ang_vel[np.isfinite(ang_vel)]
            if len(valid_ang_vel) > 20:
                # 找出中間 80% 的數據
                n_trim = int(len(valid_ang_vel) * 0.1)
                if n_trim > 0:
                    trimmed_vel = np.sort(valid_ang_vel)[n_trim:-n_trim]
                else:
                    trimmed_vel = valid_ang_vel
                
                if len(trimmed_vel) > 5:
                    median_vel = np.median(trimmed_vel)
                    # 使用 IQR（四分位距）來估算變異，更穩健
                    q75, q25 = np.percentile(trimmed_vel, [75, 25])
                    iqr = q75 - q25
                    
                    # 過濾掉超出 median ± 3*IQR 的值
                    threshold = max(3.0 * iqr, 30.0)  # 至少 30 deg/s 的閾值
                    outlier_mask = np.abs(ang_vel - median_vel) > threshold
                    ang_vel[outlier_mask] = np.nan
            
            # 第三步：平滑處理（使用較小的窗口以保留細節）
            ang_vel = moving_average(ang_vel, 3)
            
            # 第四步：再次檢查結尾處的異常（最後 5 個點）
            # 如果結尾的角速度與前面的中位數差距過大，設為 NaN
            valid_ang_vel_final = ang_vel[np.isfinite(ang_vel)]
            if len(valid_ang_vel_final) > 10:
                # 用中間 80% 計算參考中位數
                n_ref = int(len(valid_ang_vel_final) * 0.8)
                ref_median = np.median(valid_ang_vel_final[:n_ref])
                ref_std = np.std(valid_ang_vel_final[:n_ref])
                
                # 檢查最後幾個點
                n_check = min(10, len(ang_vel) // 10)
                for i in range(1, n_check + 1):
                    idx = len(ang_vel) - i
                    if np.isfinite(ang_vel[idx]):
                        if np.abs(ang_vel[idx] - ref_median) > 3 * max(ref_std, 20.0):
                            ang_vel[idx] = np.nan
    else:
        # straight mode：使用原有方法
        idx = np.where(mask_ang)[0]
        if len(idx) >= 2:
            for i0, i1 in zip(idx[:-1], idx[1:]):
                dt = t[i1] - t[i0]
                if dt > 0:
                    ang_vel[i1] = (ang_unwrap[i1] - ang_unwrap[i0]) / dt
            ang_vel = moving_average(ang_vel, 5)
    
    df["angular_vel_dps"] = ang_vel
    
    # 輸出 CSV
    csv_path = os.path.join(output_dir, f"{OUT_PREFIX}_pos_angle_speed.csv")
    df.to_csv(csv_path, index=False, encoding="utf-8-sig")
    
    # 生成圖表
    print("生成圖表...")
    plot_paths = generate_plots(df, output_dir, OUT_PREFIX)
    
    # 生成軌跡輪廓圖和組圖
    if len(box_rec) > 0 and valid.any():
        valid_boxes_count = sum(1 for _, _, fx, fy, box in box_rec 
                                if box is not None and np.isfinite(fx) and np.isfinite(fy))
        
        if valid_boxes_count >= 4:
            try:
                additional_plots = generate_trajectory_and_composite(
                    df, box_rec, output_dir, OUT_PREFIX, mm_per_px, x0, y0
                )
                plot_paths.update(additional_plots)
            except Exception as e:
                print(f"生成額外圖表時發生錯誤: {e}")
    
    print(f"\n✓ 資料已輸出到: {output_dir}")
    print(f"  - CSV: {os.path.basename(csv_path)}")
    print(f"  - Position 圖: {os.path.basename(plot_paths['position'])}")
    if MODE.lower() == "straight":
        print(f"  - Speed/Orientation 圖: {os.path.basename(plot_paths['speed_orientation'])}")
    else:
        if 'angle' in plot_paths:
            print(f"  - Angle 圖: {os.path.basename(plot_paths['angle'])}")
        print(f"  - Angular Speed 圖: {os.path.basename(plot_paths['angular_speed'])}")
    if 'trajectory_contour' in plot_paths:
        print(f"  - 軌跡輪廓圖: {os.path.basename(plot_paths['trajectory_contour'])}")
    if 'composite' in plot_paths:
        print(f"  - 8時間點組圖: {os.path.basename(plot_paths['composite'])}")


def generate_plots(df, output_dir, OUT_PREFIX):
    """
    生成追蹤結果圖表
    
    Args:
        df: 資料 DataFrame
        output_dir: 輸出目錄
        OUT_PREFIX: 檔名前綴
    
    Returns:
        圖表路徑字典
    """
    plot_paths = {}
    
    # A) 位置軌跡圖
    figA, ax_pos = plt.subplots(1, 1, figsize=(7, 6), constrained_layout=True)
    x_plot = df["x_mm"].to_numpy()
    y_plot = df["y_mm"].to_numpy()
    ax_pos.plot(x_plot, y_plot, lw=2, label="Trajectory")
    
    # 標記起點和終點
    valid_pos = np.vstack([x_plot, y_plot]).T
    valid_pos = valid_pos[~np.isnan(valid_pos).any(axis=1)]
    if len(valid_pos) > 0:
        ax_pos.scatter([valid_pos[0, 0]], [valid_pos[0, 1]], s=100, c="green", 
                      marker="o", label="Start", zorder=5)
        ax_pos.scatter([valid_pos[-1, 0]], [valid_pos[-1, 1]], s=100, c="red", 
                      marker="o", label="End", zorder=5)
        
        # 設定座標軸範圍
        xmin, xmax = np.nanmin(x_plot), np.nanmax(x_plot)
        ymin, ymax = np.nanmin(y_plot), np.nanmax(y_plot)
        xc = 0.5 * (xmin + xmax)
        yc = 0.5 * (ymin + ymax)
        half = 0.5 * max(xmax - xmin, ymax - ymin)
        half = max(half, 1e-6) * PLOT_RANGE_SCALE
        ax_pos.set_xlim(xc - half, xc + half)
        ax_pos.set_ylim(yc - half, yc + half)
    
    if INVERT_Y_AXIS:
        ax_pos.invert_yaxis()
    ax_pos.set_aspect("equal", adjustable="box")
    ax_pos.set_xlabel("x (mm)", fontsize=24)
    ax_pos.set_ylabel("y (mm)", fontsize=24)
    ax_pos.set_title("Position", fontsize=22)
    ax_pos.legend(loc="best", fontsize=14)
    
    plot_pos_path = os.path.join(output_dir, f"{OUT_PREFIX}_position.png")
    figA.savefig(plot_pos_path, dpi=1200)
    plt.close(figA)
    plot_paths['position'] = plot_pos_path
    
    # B) 速度和角度圖 (straight mode) 或 角速度圖 (rotation mode)
    if MODE.lower() == "straight":
        figB, (ax_s, ax_a) = plt.subplots(1, 2, figsize=(14, 6), constrained_layout=True)
        
        # 左圖：速度
        ax_s.plot(df["t_s"], df["speed_mm_s"], lw=2, label="Speed (mm/s)")
        sp_all = df["speed_mm_s"].to_numpy()
        tt = df["t_s"].to_numpy()
        finite = np.isfinite(sp_all)
        if np.any(finite):
            i_max = np.nanargmax(sp_all)
            ax_s.plot([tt[i_max]], [sp_all[i_max]], marker="o", markersize=8, color="red",
                     label=f"Max: {sp_all[i_max]:.2f} mm/s @ {tt[i_max]:.2f}s")
        ax_s.set_xlabel("Time (s)", fontsize=24)
        ax_s.set_ylabel("Speed (mm/s)", fontsize=24)
        ax_s.set_title("Speed vs Time", fontsize=22)
        ax_s.legend(loc="best", fontsize=14)
        
        # 右圖：角度
        if ORIENT_PLOT_WRAPPED:
            ang_vis = wrap_angles_deg(df["angle_deg_unwrapped"].to_numpy())
        else:
            ang_vis = df["angle_deg_unwrapped"].to_numpy()
        
        if np.isfinite(ang_vis).any():
            first_idx = np.where(np.isfinite(ang_vis))[0][0]
            ang0 = float(ang_vis[first_idx])
            offset_series = wrap_angles_deg(ang_vis - ang0)
            avg_offset_deg = float(np.nanmean(offset_series))
        else:
            avg_offset_deg = float("nan")
        
        label_orient = (f"Orientation (deg)\nAvg offset: {avg_offset_deg:.2f}°" 
                       if np.isfinite(avg_offset_deg) else "Orientation (deg)")
        ax_a.plot(df["t_s"], ang_vis, lw=2, label=label_orient)
        ax_a.set_xlabel("Time (s)", fontsize=24)
        ax_a.set_ylabel("Angle (deg)", fontsize=24)
        ax_a.set_title("Orientation vs Time", fontsize=22)
        if ORIENT_YLIM_DEG is not None and np.isfinite(ORIENT_YLIM_DEG):
            ylim = float(ORIENT_YLIM_DEG)
            ax_a.set_ylim(-ylim, +ylim)
        ax_a.legend(loc="best", fontsize=14)
        
        plot_so_path = os.path.join(output_dir, f"{OUT_PREFIX}_speed_orientation.png")
        figB.savefig(plot_so_path, dpi=1200)
        plt.close(figB)
        plot_paths['speed_orientation'] = plot_so_path
        
    else:  # rotation mode
        tt = df["t_s"].to_numpy()
        
        # 圖1：角度 vs 時間（獨立檔案）
        fig_theta, ax_theta = plt.subplots(1, 1, figsize=(10, 6), constrained_layout=True)
        ang_plot = df["angle_deg_unwrapped"].to_numpy()
        ax_theta.plot(tt, ang_plot, lw=2, color='blue')
        
        finite_ang = np.isfinite(ang_plot)
        if np.any(finite_ang):
            valid_angles = ang_plot[finite_ang]
            total_rotation = valid_angles[-1] - valid_angles[0]
            ax_theta.set_title(f"Angle vs Time (Total rotation: {total_rotation:.1f}°)", fontsize=22)
        else:
            ax_theta.set_title("Angle vs Time", fontsize=22)
        
        ax_theta.set_xlabel("Time (s)", fontsize=24)
        ax_theta.set_ylabel("Angle (deg)", fontsize=24)
        
        plot_theta_path = os.path.join(output_dir, f"{OUT_PREFIX}_angle.png")
        fig_theta.savefig(plot_theta_path, dpi=1200)
        plt.close(fig_theta)
        plot_paths['angle'] = plot_theta_path
        
        # 圖2：角速度 vs 時間（獨立檔案）
        fig_w, ax_w = plt.subplots(1, 1, figsize=(10, 6), constrained_layout=True)
        w_all = df["angular_vel_dps"].to_numpy()
        ax_w.plot(tt, w_all, lw=2, color='blue')
        
        finite = np.isfinite(w_all)
        if np.any(finite):
            mean_vel = np.nanmean(w_all)
            ax_w.set_title(f"Angular Speed vs Time (Mean: {mean_vel:.2f} deg/s)", fontsize=22)
        else:
            ax_w.set_title("Angular Speed vs Time", fontsize=22)
        
        ax_w.set_xlabel("Time (s)", fontsize=24)
        ax_w.set_ylabel("Angular speed (deg/s)", fontsize=24)
        
        plot_w_path = os.path.join(output_dir, f"{OUT_PREFIX}_angular_speed.png")
        fig_w.savefig(plot_w_path, dpi=1200)
        plt.close(fig_w)
        plot_paths['angular_speed'] = plot_w_path
    
    return plot_paths


def generate_trajectory_and_composite(df, box_rec, output_dir, OUT_PREFIX, mm_per_px, x0, y0):
    """
    生成軌跡輪廓圖和8時間點組圖
    
    Args:
        df: 資料 DataFrame
        box_rec: box 記錄列表
        output_dir: 輸出目錄
        OUT_PREFIX: 檔名前綴
        mm_per_px: mm/pixel 比例
        x0, y0: 原點座標
    
    Returns:
        圖表路徑字典
    """
    plot_paths = {}
    
    # 從 box_rec 中提取有效的 boxes
    valid_boxes = [(idx, ts, fx, fy, box) for idx, ts, fx, fy, box in box_rec 
                   if box is not None and np.isfinite(fx) and np.isfinite(fy)]
    
    if len(valid_boxes) < 4:
        print(f"  警告：只有 {len(valid_boxes)} 個有效框，需要至少 4 個")
        return plot_paths
    
    # === 1. 生成軌跡輪廓圖 ===
    figC, ax_contour = plt.subplots(1, 1, figsize=(7, 6), constrained_layout=True)
    
    # 繪製整體路徑
    x_plot = df["x_mm"].to_numpy()
    y_plot = df["y_mm"].to_numpy()
    ax_contour.plot(x_plot, y_plot, lw=2, color='blue', label="Trajectory", alpha=0.7)
    
    # 選擇時間點（最多8個，最少4個）
    n_segments = min(8, len(valid_boxes))
    indices = np.linspace(0, len(valid_boxes) - 1, n_segments, dtype=int)
    
    # 使用統一顏色虛線繪製輪廓
    for i, idx in enumerate(indices):
        frame_idx_val, t_s_val, fx_px_val, fy_px_val, box = valid_boxes[idx]
        
        # 將box的四個頂點轉換為相對mm座標
        box_mm = []
        for vertex in box:
            vx_px, vy_px = vertex[0], vertex[1]
            vx_mm = vx_px * mm_per_px - x0
            vy_mm = -(vy_px * mm_per_px - y0)  # 修正Y座標
            box_mm.append([vx_mm, vy_mm])
        
        box_mm = np.array(box_mm)
        
        # 繪製輪廓（統一淺藍色虛線）
        color = (0.5, 0.7, 1.0)  # 淺藍色
        linestyle = '--'
        linewidth = 1.5
        alpha = 0.7
        
        box_closed = np.vstack([box_mm, box_mm[0:1]])
        ax_contour.plot(box_closed[:, 0], box_closed[:, 1], 
                      linestyle=linestyle, linewidth=linewidth, 
                      color=color, alpha=alpha)
    
    # 標記起點和終點
    valid_pos = np.vstack([x_plot, y_plot]).T
    valid_pos = valid_pos[~np.isnan(valid_pos).any(axis=1)]
    if len(valid_pos) > 0:
        x_start, y_start = valid_pos[0, 0], valid_pos[0, 1]
        x_end, y_end = valid_pos[-1, 0], valid_pos[-1, 1]
        ax_contour.scatter([x_start], [y_start], s=100, c="green", 
                          marker="o", label="Start", zorder=5)
        ax_contour.scatter([x_end], [y_end], s=100, c="red", 
                          marker="o", label="End", zorder=5)
        
        # 設置座標軸範圍
        xmin, xmax = np.nanmin(x_plot), np.nanmax(x_plot)
        ymin, ymax = np.nanmin(y_plot), np.nanmax(y_plot)
        xc = 0.5 * (xmin + xmax)
        yc = 0.5 * (ymin + ymax)
        half_range = 0.5 * max(xmax - xmin, ymax - ymin)
        
        # rotation mode 時使用固定或自適應範圍
        if MODE.lower() == "rotation":
            # 對於 rotation mode，使用固定範圍（例如 ±15mm）來更清楚地顯示軌跡
            # 如果實際範圍大於固定範圍，則使用實際範圍
            fixed_half_range = 15.0  # mm，可根據需要調整
            half_range = max(half_range * PLOT_RANGE_SCALE, fixed_half_range)
        else:
            half_range = max(half_range, 1e-6) * PLOT_RANGE_SCALE
        
        ax_contour.set_xlim(xc - half_range, xc + half_range)
        ax_contour.set_ylim(yc - half_range, yc + half_range)
    
    if INVERT_Y_AXIS:
        ax_contour.invert_yaxis()
    
    ax_contour.set_aspect("equal", adjustable="box")
    ax_contour.set_xlabel("x (mm)", fontsize=24)
    ax_contour.set_ylabel("y (mm)", fontsize=24)
    ax_contour.set_title(f"Position with Time Points", fontsize=22)
    ax_contour.legend(loc="best", fontsize=14)
    
    plot_contour_path = os.path.join(output_dir, f"{OUT_PREFIX}_timepoint_position.png")
    figC.savefig(plot_contour_path, dpi=1200)
    plt.close(figC)
    plot_paths['trajectory_contour'] = plot_contour_path
    
    # === 2. 生成時間點組圖（需要原始影片）===
    raw_video_path = os.path.join(output_dir, f"camera_{MODE}_raw.avi")
    
    if not os.path.exists(raw_video_path):
        raw_video_path = os.path.join(output_dir, f"camera_{MODE}_raw.mp4")
        if not os.path.exists(raw_video_path):
            return plot_paths
    
    try:
        cap_composite = cv2.VideoCapture(raw_video_path)
        
        if not cap_composite.isOpened():
            return plot_paths
        
        total_frames_in_video = int(cap_composite.get(cv2.CAP_PROP_FRAME_COUNT))
        
        # 使用 valid_boxes 在整個 box_rec 中的索引來對應影片幀
        # valid_boxes 的索引對應到 box_rec 的索引，而 box_rec 與錄製的幀一一對應
        # 所以 indices[i] 在 valid_boxes 中的位置就是影片中的實際幀號
        snapshot_video_frames = indices.copy()  # 這些是在 valid_boxes 列表中的索引
        snapshot_times = []
        
        for idx in indices:
            _, t_s_val, _, _, _ = valid_boxes[idx]
            first_valid_time = valid_boxes[0][1]
            relative_time = t_s_val - first_valid_time
            snapshot_times.append(relative_time)
        
        # 建立 DataFrame frame 到行索引的映射
        row_map = {int(r['frame']): i for i, r in df.iterrows()}
        
        # 同時建立 valid_boxes 索引到 DataFrame frame 的映射
        valid_box_to_df_frame = {}
        for i, (frame_idx_val, _, _, _, _) in enumerate(valid_boxes):
            valid_box_to_df_frame[i] = frame_idx_val
        
        extracted_frames = []
        
        for snap_idx, valid_box_idx in enumerate(snapshot_video_frames):
            # valid_box_idx 是在 valid_boxes 中的索引
            # 我們需要找到這個 box 在錄製序列中的位置
            # 由於 valid_boxes 是從 box_rec 篩選出來的，我們需要找到它在原始序列中的位置
            
            # 更簡單的方法：直接用 valid_box_idx 作為在 valid_boxes 中的索引
            # 然後找到對應的原始 box_rec 條目
            target_frame_idx, _, target_fx, target_fy, target_box = valid_boxes[valid_box_idx]
            
            # 在原始影片中，我們需要找到這一幀的位置
            # box_rec 的順序應該與錄製順序一致
            # 找到這個 frame_idx 在 box_rec 中的位置
            video_frame_num = None
            for video_idx, (rec_frame_idx, _, _, _, _) in enumerate(box_rec):
                if rec_frame_idx == target_frame_idx:
                    video_frame_num = video_idx
                    break
            
            if video_frame_num is None or video_frame_num >= total_frames_in_video:
                continue
            
            cap_composite.set(cv2.CAP_PROP_POS_FRAMES, video_frame_num)
            ok, frame = cap_composite.read()
            if not ok:
                continue
            
            # 繪製該時間點之前的所有輪廓（淺藍色虛線）- 累積顯示
            light_blue = (255, 200, 150)  # BGR 淺藍色
            for i in range(snap_idx + 1):  # 包含當前時間點
                box_idx_in_valid = snapshot_video_frames[i]
                _, _, _, _, box = valid_boxes[box_idx_in_valid]
                if box is not None:
                    box_int = np.int32(box)
                    # 繪製虛線輪廓
                    for j in range(4):
                        pt1 = tuple(box_int[j])
                        pt2 = tuple(box_int[(j + 1) % 4])
                        # 簡單虛線實現
                        dist = np.linalg.norm(np.array(pt1) - np.array(pt2))
                        num_segments = int(dist / 10)
                        if num_segments > 0:
                            for k in range(num_segments):
                                if k % 2 == 0:  # 只畫偶數段
                                    start = (
                                        int(pt1[0] + (pt2[0] - pt1[0]) * k / num_segments),
                                        int(pt1[1] + (pt2[1] - pt1[1]) * k / num_segments)
                                    )
                                    end = (
                                        int(pt1[0] + (pt2[0] - pt1[0]) * (k + 1) / num_segments),
                                        int(pt1[1] + (pt2[1] - pt1[1]) * (k + 1) / num_segments)
                                    )
                                    cv2.line(frame, start, end, light_blue, 2)
            
            # 繪製完整的中心路徑（從起點到當前時間點的所有位置）
            center_color_composite = (255, 0, 0)  # BGR 藍色
            current_df_frame = valid_boxes[valid_box_idx][0]  # 獲取 DataFrame 中的 frame 編號
            
            # 收集所有從起點到當前幀的位置
            path_points = []
            for idx in range(len(df)):
                if df.iloc[idx]['frame'] <= current_df_frame:
                    fx_val = df.iloc[idx]['x_px_filt']
                    fy_val = df.iloc[idx]['y_px_filt']
                    if np.isfinite(fx_val) and np.isfinite(fy_val):
                        path_points.append([int(round(fx_val)), int(round(fy_val))])
            
            # 繪製完整路徑（使用 polylines 讓路徑更順滑）
            if len(path_points) > 1:
                pts = np.array(path_points, dtype=np.int32)
                cv2.polylines(frame, [pts], isClosed=False, color=center_color_composite, thickness=2)
            
            # 在所有已經過的時間點上畫圓標記（純藍色）
            for i in range(snap_idx + 1):
                box_idx_for_circle = snapshot_video_frames[i]
                df_frame_for_circle = valid_boxes[box_idx_for_circle][0]
                if df_frame_for_circle in row_map:
                    fx_px = df.iloc[row_map[df_frame_for_circle]]['x_px_filt']
                    fy_px = df.iloc[row_map[df_frame_for_circle]]['y_px_filt']
                    if np.isfinite(fx_px) and np.isfinite(fy_px):
                        cx_i = int(round(fx_px))
                        cy_i = int(round(fy_px))
                        cv2.circle(frame, (cx_i, cy_i), 7, center_color_composite, -1)
            
            # 在右下角添加時間標註
            height, width = frame.shape[:2]
            time_text = f"t={snapshot_times[snap_idx]:.2f}s"
            
            # 設定文字參數
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 1.0
            font_thickness = 2
            text_color = (0, 0, 255)  # 紅色 (BGR)
            
            # 獲取文字大小
            (text_width, text_height), baseline = cv2.getTextSize(
                time_text, font, font_scale, font_thickness
            )
            
            # 計算文字位置（右下角，留一些邊距）
            margin = 10
            text_x = width - text_width - margin
            text_y = height - margin
            
            # 添加文字背景（黑色半透明矩形）
            overlay = frame.copy()
            cv2.rectangle(
                overlay,
                (text_x - 5, text_y - text_height - 5),
                (text_x + text_width + 5, text_y + baseline + 5),
                (0, 0, 0),
                -1
            )
            cv2.addWeighted(overlay, 0.5, frame, 0.5, 0, frame)
            
            # 添加文字
            cv2.putText(
                frame,
                time_text,
                (text_x, text_y),
                font,
                font_scale,
                text_color,
                font_thickness,
                cv2.LINE_AA
            )
            
            extracted_frames.append(frame)
        
        cap_composite.release()
        
        # 將圖水平拼接
        if len(extracted_frames) == n_segments:
            target_height = 300
            resized_frames = []
            for frame in extracted_frames:
                h, w = frame.shape[:2]
                scale = target_height / h
                new_width = int(w * scale)
                resized = cv2.resize(frame, (new_width, target_height))
                resized_frames.append(resized)
            
            combined_frame = np.hstack(resized_frames)
            composite_path = os.path.join(output_dir, f"{OUT_PREFIX}composite.png")
            cv2.imwrite(composite_path, combined_frame)
            plot_paths['composite'] = composite_path
        elif len(extracted_frames) > 0:
            target_height = 300
            resized_frames = []
            for frame in extracted_frames:
                h, w = frame.shape[:2]
                scale = target_height / h
                new_width = int(w * scale)
                resized = cv2.resize(frame, (new_width, target_height))
                resized_frames.append(resized)
            
            combined_frame = np.hstack(resized_frames)
            composite_path = os.path.join(output_dir, f"{OUT_PREFIX}_{len(extracted_frames)}composite.png")
            cv2.imwrite(composite_path, combined_frame)
            plot_paths['composite'] = composite_path
    
    except Exception as e:
        print(f"生成組圖時發生錯誤: {e}")
    
    return plot_paths
