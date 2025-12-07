# -*- coding: utf-8 -*-
"""
相機執行緒模組
提供多執行緒相機讀取和影像處理功能
"""

import time
import queue
import numpy as np
import cv2
from config import (
    CAM_HEIGHT, CAM_WIDTH, MODE,
    MAX_MEAS_JUMP_PX, EMA_ALPHA_POS, EMA_ALPHA_ANGLE
)
from signal_processing import ema
from image_processing import find_target_and_angle


def capture_thread(cap, frame_queue, running, stats):
    """
    Thread 1: 專門負責快速讀取相機幀
    不做任何處理，只讀取並放入佇列
    
    Args:
        cap: cv2.VideoCapture 物件
        frame_queue: 幀佇列
        running: 執行狀態標誌 (list with single bool)
        stats: Stats 統計物件
    """
    print("[Capture Thread] 啟動")
    consecutive_failures = 0
    MAX_CONSECUTIVE_FAILURES = 10
    
    while running[0]:
        ret, frame = cap.read()
        if not ret:
            consecutive_failures += 1
            if consecutive_failures >= MAX_CONSECUTIVE_FAILURES:
                print(f"[Capture Thread] 連續讀取失敗 {consecutive_failures} 次，停止")
                break
            # 暫時讀取失敗，等待一下再試
            time.sleep(0.001)
            continue
        
        # 讀取成功，重置失敗計數
        consecutive_failures = 0
        timestamp = time.perf_counter()
        stats.update_capture()
        
        # 嘗試放入佇列
        try:
            frame_queue.put_nowait((frame, timestamp))
        except queue.Full:
            # 佇列滿了，丟棄此幀
            stats.drop_frame()
    
    print("[Capture Thread] 結束")


def process_thread(frame_queue, result_queue, running, stats, kf, tracker_state, 
                   mm_per_px, fg_controller):
    """
    Thread 2: 從佇列取出幀並進行所有影像處理
    包括：旋轉、檢測、追蹤、繪圖、計算速度等
    
    Args:
        frame_queue: 輸入幀佇列
        result_queue: 輸出結果佇列
        running: 執行狀態標誌 (list with single bool)
        stats: Stats 統計物件
        kf: Kalman 濾波器
        tracker_state: 追蹤狀態字典
        mm_per_px: mm/pixel 比例
        fg_controller: 函數產生器控制器
    """
    print("[Process Thread] 啟動")
    
    # EMA 狀態
    ema_x = None
    ema_y = None
    ema_ang = None
    
    # 速度計算
    last_x_mm_abs = None
    last_y_mm_abs = None
    last_t = None
    inst_speed = np.nan
    
    # 原點
    origin_x = [None]
    origin_y = [None]
    
    # 幀計數
    frame_idx = [0]
    
    while running[0] or not frame_queue.empty():
        try:
            frame, timestamp = frame_queue.get(timeout=0.1)
        except queue.Empty:
            continue
        
        # 旋轉畫面
        frame_rotated = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        
        # Kalman 預測
        pred = kf.predict()
        px_pred, py_pred = float(pred[0, 0]), float(pred[1, 0])
        
        # 目標檢測
        m = find_target_and_angle(frame_rotated)
        use_meas = True
        if m is not None:
            cx, cy, angle_deg, box = m
            dist = np.hypot(cx - px_pred, cy - py_pred)
            if dist > MAX_MEAS_JUMP_PX:
                use_meas = False
        else:
            use_meas = False
        
        if use_meas:
            est = kf.correct(np.array([[cx], [cy]], dtype=np.float32))
            fx_raw, fy_raw = float(est[0, 0]), float(est[1, 0])
        else:
            fx_raw, fy_raw = px_pred, py_pred
            angle_deg = np.nan
            box = None
        
        # EMA 平滑
        ema_x = ema(ema_x, fx_raw, EMA_ALPHA_POS)
        ema_y = ema(ema_y, fy_raw, EMA_ALPHA_POS)
        ema_ang = ema(ema_ang, angle_deg, EMA_ALPHA_ANGLE) if np.isfinite(angle_deg) else ema_ang
        
        fx, fy = float(ema_x), float(ema_y)
        ang_for_draw = float(ema_ang) if ema_ang is not None else (float(angle_deg) if m is not None else np.nan)
        
        # 繪製追蹤結果
        if box is None:
            sz = 20
            bx = np.array([
                [fx - sz, fy - sz],
                [fx + sz, fy - sz],
                [fx + sz, fy + sz],
                [fx - sz, fy + sz]
            ], dtype=int)
            box_draw = bx
            green_center_x, green_center_y = fx, fy
        else:
            box_draw = box
            green_center_x, green_center_y = cx, cy
        
        display_frame = frame_rotated.copy()
        cv2.polylines(display_frame, [box_draw], isClosed=True, color=(0, 0, 255), thickness=3)
        if np.isfinite(green_center_x) and np.isfinite(green_center_y):
            cv2.circle(display_frame, (int(round(green_center_x)), int(round(green_center_y))), 
                      6, (0, 255, 0), -1)
        
        # 計算座標和速度
        fx_mm_abs = fx * mm_per_px if np.isfinite(fx) else np.nan
        fy_mm_abs = fy * mm_per_px if np.isfinite(fy) else np.nan
        
        # 使用實際時間戳
        if frame_idx[0] == 0:
            tracker_state['start_time'] = timestamp
        t_s = timestamp - tracker_state.get('start_time', timestamp)
        
        if (last_x_mm_abs is not None and last_y_mm_abs is not None and last_t is not None and
            np.isfinite(fx_mm_abs) and np.isfinite(fy_mm_abs) and (t_s > last_t)):
            inst_speed = np.hypot(fx_mm_abs - last_x_mm_abs, fy_mm_abs - last_y_mm_abs) / (t_s - last_t)
        last_x_mm_abs, last_y_mm_abs, last_t = fx_mm_abs, fy_mm_abs, t_s
        
        if origin_x[0] is None and np.isfinite(fx_mm_abs) and np.isfinite(fy_mm_abs):
            origin_x[0], origin_y[0] = fx_mm_abs, fy_mm_abs
        
        # 疊字顯示
        if origin_x[0] is not None and np.isfinite(fx_mm_abs) and np.isfinite(fy_mm_abs):
            rx = fx_mm_abs - origin_x[0]
            ry = -(fy_mm_abs - origin_y[0])  # 修正Y座標正負號
            pos_text = f"Pos(mm): ({rx:.2f}, {ry:.2f})"
        else:
            pos_text = f"Pos(mm): (NaN, NaN)"
        cv2.putText(display_frame, pos_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 
                   0.6, (0, 0, 255), 2)
        
        if np.isfinite(ang_for_draw):
            cv2.putText(display_frame, f"Angle: {ang_for_draw:+.2f} deg", 
                       (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        else:
            cv2.putText(display_frame, "Angle: NaN", (20, 70), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        if np.isfinite(inst_speed):
            cv2.putText(display_frame, f"Speed: {inst_speed:.2f} mm/s", 
                       (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        else:
            cv2.putText(display_frame, "Speed: NaN mm/s", (20, 100), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        # 函數產生器狀態
        fg_status = f"FG: Mode {fg_controller.current_mode}" if fg_controller.current_mode else "FG: OFF"
        cv2.putText(display_frame, fg_status, (20, 130), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        # 錄影狀態
        if tracker_state['recording']:
            cv2.putText(display_frame, "REC", (CAM_HEIGHT - 100, 35), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
            cv2.circle(display_frame, (CAM_HEIGHT - 130, 25), 8, (0, 0, 255), -1)
        
        # 顯示 FPS 資訊
        info = stats.get_info()
        cv2.putText(display_frame, 
                   f"Cap: {info['capture_fps']:.1f} FPS | Proc: {info['process_fps']:.1f} FPS", 
                   (20, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        
        # 放入結果佇列（包含原始幀和 box）
        result_data = {
            'frame': display_frame,
            'raw_frame': frame_rotated.copy(),  # 原始旋轉後的幀（無標註）
            'timestamp': timestamp,
            'frame_idx': frame_idx[0],
            'fx': fx,
            'fy': fy,
            'fx_mm_abs': fx_mm_abs,
            'fy_mm_abs': fy_mm_abs,
            'angle': ang_for_draw,
            'speed': inst_speed,
            't_s': t_s,
            'box': box  # 記錄 box 用於後續繪製
        }
        
        try:
            result_queue.put_nowait(result_data)
        except queue.Full:
            # 如果結果佇列滿了，丟棄舊的結果
            try:
                result_queue.get_nowait()
                result_queue.put_nowait(result_data)
            except:
                pass
        
        frame_idx[0] += 1
        stats.update_process()
    
    print("[Process Thread] 結束")
