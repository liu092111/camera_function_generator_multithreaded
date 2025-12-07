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
from signal_processing import unwrap_angles_deg, wrap_angles_deg, moving_average, finite_diff


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
    
    # 角度處理
    ang_raw = df["angle_deg_raw"].to_numpy()
    mask_ang = np.isfinite(ang_raw)
    ang_unwrap = np.full_like(ang_raw, np.nan, dtype=float)
    if mask_ang.any():
        ang_unwrap[mask_ang] = unwrap_angles_deg(ang_raw[mask_ang])
        ang_unwrap = moving_average(ang_unwrap, 5)
    df["angle_deg_unwrapped"] = ang_unwrap
    
    # 速度計算
    t = df["t_s"].to_numpy()
    vx = finite_diff(df["x_mm"].to_numpy(), t, smooth_win=5)
    vy = finite_diff(df["y_mm"].to_numpy(), t, smooth_win=5)
    speed = np.sqrt(vx**2 + vy**2)
    df["vx_mm_s"] = vx
    df["vy_mm_s"] = vy
    df["speed_mm_s"] = speed
    
    # 角速度計算（for rotation mode）
    ang_vel = np.full_like(ang_unwrap, np.nan, dtype=float)
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
    ax_pos.set_xlabel("x (mm)")
    ax_pos.set_ylabel("y (mm)")
    ax_pos.set_title("Position")
    ax_pos.grid(True, linestyle="--", alpha=0.4)
    ax_pos.legend(loc="best")
    
    plot_pos_path = os.path.join(output_dir, f"{OUT_PREFIX}_position.png")
    figA.savefig(plot_pos_path, dpi=220)
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
        ax_s.set_xlabel("Time (s)")
        ax_s.set_ylabel("Speed (mm/s)")
        ax_s.set_title("Speed vs Time")
        ax_s.grid(True, linestyle="--", alpha=0.4)
        ax_s.legend(loc="best")
        
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
        ax_a.set_xlabel("Time (s)")
        ax_a.set_ylabel("Angle (deg)")
        ax_a.set_title("Orientation vs Time")
        if ORIENT_YLIM_DEG is not None and np.isfinite(ORIENT_YLIM_DEG):
            ylim = float(ORIENT_YLIM_DEG)
            ax_a.set_ylim(-ylim, +ylim)
        ax_a.grid(True, linestyle="--", alpha=0.4)
        ax_a.legend(loc="best")
        
        plot_so_path = os.path.join(output_dir, f"{OUT_PREFIX}_speed_orientation.png")
        figB.savefig(plot_so_path, dpi=220)
        plt.close(figB)
        plot_paths['speed_orientation'] = plot_so_path
        
    else:  # rotation mode
        figW, ax_w = plt.subplots(1, 1, figsize=(8, 6), constrained_layout=True)
        ax_w.plot(df["t_s"], df["angular_vel_dps"], lw=2, label="Angular speed (deg/s)")
        
        w_all = df["angular_vel_dps"].to_numpy()
        tt = df["t_s"].to_numpy()
        finite = np.isfinite(w_all)
        if np.any(finite):
            idx_rel = int(np.nanargmax(np.abs(w_all[finite])))
            idxs = np.where(finite)[0]
            i_max = idxs[idx_rel]
            ax_w.plot([tt[i_max]], [w_all[i_max]], marker="o", markersize=8, color="red",
                     label=f"Max: {w_all[i_max]:.2f} deg/s @ {tt[i_max]:.2f}s")
        
        ax_w.set_xlabel("Time (s)")
        ax_w.set_ylabel("Angular speed (deg/s)")
        ax_w.set_title("Angular Speed vs Time")
        ax_w.grid(True, linestyle="--", alpha=0.4)
        ax_w.legend(loc="best")
        
        plot_w_path = os.path.join(output_dir, f"{OUT_PREFIX}_angular_speed.png")
        figW.savefig(plot_w_path, dpi=220)
        plt.close(figW)
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
    ax_contour.set_xlabel("x (mm)")
    ax_contour.set_ylabel("y (mm)")
    ax_contour.set_title(f"Trajectory with Time Points")
    ax_contour.grid(True, linestyle="--", alpha=0.4)
    ax_contour.legend(loc="best")
    
    plot_contour_path = os.path.join(output_dir, f"{OUT_PREFIX}_trajectory_center_only.png")
    figC.savefig(plot_contour_path, dpi=220)
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
            composite_path = os.path.join(output_dir, f"{OUT_PREFIX}_{n_segments}_timepoints_composite.png")
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
            composite_path = os.path.join(output_dir, f"{OUT_PREFIX}_{len(extracted_frames)}_timepoints_composite.png")
            cv2.imwrite(composite_path, combined_frame)
            plot_paths['composite'] = composite_path
    
    except Exception as e:
        print(f"生成組圖時發生錯誤: {e}")
    
    return plot_paths
