# -*- coding: utf-8 -*-
"""
整合式攝影機追蹤與函數產生器控制系統 - 主程式
========================================
多執行緒架構提升 FPS
- Thread 1: 專門快速讀取相機幀
- Thread 2: 影像處理（旋轉、檢測、追蹤、繪圖、計算）
- Main Thread: 顯示和錄影

作者：模組化優化版本
"""

# 抑制警告訊息
import warnings
warnings.filterwarnings('ignore')

import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'  # 抑制 TensorFlow 訊息
os.environ['OPENCV_LOG_LEVEL'] = 'SILENT'  # 抑制 OpenCV 訊息

import cv2
# 抑制 OpenCV C++ 層級的警告
cv2.setLogLevel(0)  # 0 = LOG_LEVEL_SILENT
import time
import threading
import queue
from datetime import datetime
import numpy as np

from config import (
    MODE, CAMERA_INDEX, CAM_WIDTH, CAM_HEIGHT, CAM_FPS_REQ,
    RECORD_OUTPUT, WINDOW_TITLE, FRAME_QUEUE_SIZE, RESULT_QUEUE_SIZE,
    VOLTAGE, ENABLE_UNDISTORT, ENABLE_PID_CONTROL,
    ANGLE_CORRECTION_THRESHOLD, ANGLE_CORRECTION_TOLERANCE, DEFAULT_STRAIGHT_MODE,
    STRAIGHT_PID_KP, STRAIGHT_PID_KI, STRAIGHT_PID_KD, STRAIGHT_DEADBAND, STRAIGHT_CORRECTION_RANGE,
    ROTATION_TARGET_ANGLE, ROTATION_ANGLE_TOLERANCE, ROTATION_DECEL_ANGLE, ROTATION_MIN_VOLTAGE
)
from stats import Stats
from function_generator import FunctionGeneratorController
from image_processing import find_target_and_angle, calibrate_scale
from signal_processing import make_kalman
from camera_threads import capture_thread, process_thread
from data_export import process_and_export_data
from pid_controller import UnifiedPIDController
from thread_safe_state import ThreadSafeState


def show_help(pid_enabled=False):
    """顯示幫助信息"""
    print("\n" + "="*60)
    print("整合式攝影機追蹤與函數產生器控制系統 (多執行緒)")
    print("="*60)
    print("建議操作流程：")
    print("  1. 架設好設備，確認畫面正常")
    print("  2. 按 [Space] 開始錄影")
    print("  3. 按 [1-4] 選擇 FG 模式，驅動 device")
    if pid_enabled:
        print("     （PID 控制已啟用，會自動進行校正）")
    print("")
    print("攝影機控制：")
    print("  [Space]     - 開始/停止錄製")
    print("  [ESC/Q]     - 退出程式")
    print("")
    print("函數產生器控制：")
    print("  [1-4]       - Mode 1-4 (選擇波型，驅動 device)")
    print("  [0]         - 關閉函數產生器輸出")
    print("")
    if pid_enabled:
        print("PID 控制：已在 config.py 中啟用")
        print("  按 1-4 選擇模式後會自動啟用 PID 控制")
        print("")
    print("其他：")
    print("  [H]         - 顯示此幫助")
    print("="*60)


def main():
    """主程式"""
    print("="*60)
    print("整合式攝影機追蹤與函數產生器控制系統 (多執行緒)")
    print("="*60)
    
    # 初始化函數產生器
    fg_controller = FunctionGeneratorController()
    fg_connected = fg_controller.connect()
    
    # 初始化攝影機
    print("\n初始化攝影機...")
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))  # 設定 MJPG
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, CAM_FPS_REQ)
    
    # 暖機
    for _ in range(10):
        cap.read()
    
    # 確認設定
    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    
    print(f"\n相機設定:")
    print(f"  解析度: {actual_width}x{actual_height}")
    print(f"  目標 FPS: {actual_fps:.1f}")
    
    # 初始化畸變校正（如果啟用）
    undistort_enabled = False
    if ENABLE_UNDISTORT:
        print("\n初始化鏡頭畸變校正...")
        from undistort import init_undistort, undistort_frame
        if init_undistort():
            undistort_enabled = True
        else:
            print("⚠ 畸變校正初始化失敗，將使用原始影像")
    else:
        print("\n鏡頭畸變校正：已停用")
    
    # 估算 mm/px - 使用網格或 device 尺寸校準
    ok, first = cap.read()
    if not ok:
        print("無法讀取影像")
        cap.release()
        return
    
    # 對第一幀應用畸變校正（如果啟用）
    if undistort_enabled:
        first = undistort_frame(first)
    
    print("\n正在校準尺度 (mm/pixel)...")
    mm_per_px = calibrate_scale(first)
    print(f"✓ 尺度校準完成: {mm_per_px:.4f} mm/pixel")
    print(f"  驗證：5mm 網格 ≈ {5.0/mm_per_px:.1f} pixels")
    print(f"  驗證：9mm device 寬度 ≈ {9.0/mm_per_px:.1f} pixels")
    print(f"  驗證：6mm device 高度 ≈ {6.0/mm_per_px:.1f} pixels")
    
    # 初始化 Kalman 濾波器
    kf = make_kalman()
    first_rotated = cv2.rotate(first, cv2.ROTATE_90_CLOCKWISE)
    init = find_target_and_angle(first_rotated)
    if init is not None:
        cx0, cy0, ang0, box0 = init
        print(f"✓ 初始位置: ({cx0:.1f}, {cy0:.1f}), 角度: {ang0:.1f}°")
    else:
        cx0, cy0, ang0 = CAM_WIDTH/2.0, CAM_HEIGHT/2.0, 0.0
        print("未檢測到目標，使用預設位置")
    
    kf.statePost = np.array([[cx0], [cy0], [0.0], [0.0]], dtype=np.float32)
    
    # 共享狀態
    running = [True]
    stats = Stats()
    tracker_state = ThreadSafeState({'recording': False, 'pid_active': False})
    
    # 初始化 PID 控制器（如果啟用）
    pid_controller = None
    current_fg_mode = None  # 當前 FG 模式
    if ENABLE_PID_CONTROL:
        print("\n初始化 PID 控制器...")
        pid_controller = UnifiedPIDController(
            mode=MODE, 
            base_voltage=VOLTAGE,
            straight_mode=DEFAULT_STRAIGHT_MODE,
            angle_threshold=ANGLE_CORRECTION_THRESHOLD,
            correction_tolerance=ANGLE_CORRECTION_TOLERANCE
        )
        
        # 配置智能校正控制器的電壓 PID 參數
        pid_controller.smart_straight_ctrl.voltage_pid.set_gains(
            kp=STRAIGHT_PID_KP, 
            ki=STRAIGHT_PID_KI, 
            kd=STRAIGHT_PID_KD
        )
        pid_controller.smart_straight_ctrl.voltage_pid.deadband = STRAIGHT_DEADBAND
        pid_controller.smart_straight_ctrl.voltage_correction_range = STRAIGHT_CORRECTION_RANGE
        
        # 配置 Rotation Mode 參數
        pid_controller.rotation_ctrl.target_angle = ROTATION_TARGET_ANGLE
        pid_controller.rotation_ctrl.angle_tolerance = ROTATION_ANGLE_TOLERANCE
        
        print(f"✓ PID 控制器已初始化 (模式: {MODE})")
        if MODE.lower() == 'straight':
            print(f"  智能姿態校正：角度閾值 ±{ANGLE_CORRECTION_THRESHOLD}°, 校正容差 {ANGLE_CORRECTION_TOLERANCE}°")
            print(f"  電壓調整 PID: Kp={STRAIGHT_PID_KP}, Ki={STRAIGHT_PID_KI}, Kd={STRAIGHT_PID_KD}")
            print(f"  死區: {STRAIGHT_DEADBAND}mm, 電壓校正範圍: ±{STRAIGHT_CORRECTION_RANGE}V")
            print(f"  FG 模式對應: 直走=Mode 1/3, 旋轉校正=Mode 2/4")
        else:
            print(f"  Rotation Mode: 目標角度={ROTATION_TARGET_ANGLE}°, 容差={ROTATION_ANGLE_TOLERANCE}°")
    
    # 建立佇列
    frame_queue = queue.Queue(maxsize=FRAME_QUEUE_SIZE)
    result_queue = queue.Queue(maxsize=RESULT_QUEUE_SIZE)
    
    # 啟動執行緒
    capture_t = threading.Thread(
        target=capture_thread, 
        args=(cap, frame_queue, running, stats), 
        daemon=True
    )
    process_t = threading.Thread(
        target=process_thread, 
        args=(frame_queue, result_queue, running, stats, kf, tracker_state, mm_per_px, fg_controller), 
        daemon=True
    )
    
    capture_t.start()
    process_t.start()
    
    show_help(pid_enabled=ENABLE_PID_CONTROL)
    print(f"\n系統狀態：")
    print(f"✓ 攝影機：已連接 ({actual_width}x{actual_height} @ {actual_fps:.0f} FPS)")
    print(f"{'✓' if undistort_enabled else '✗'} 畸變校正：{'已啟用' if undistort_enabled else '已停用'}")
    print(f"{'✓' if fg_connected else '✗'} 函數產生器：{'已連接' if fg_connected else '未連接'}")
    print(f"{'✓' if ENABLE_PID_CONTROL else '✗'} PID 控制：{'已啟用 (按 1-4 後自動生效)' if ENABLE_PID_CONTROL else '已停用'}")
    print(f"\n系統就緒！按 [H] 查看幫助\n")
    
    # PID 控制相關變數
    pid_update_counter = 0
    PID_UPDATE_INTERVAL = 5  # 每 5 幀更新一次 PID（降低 FG 通訊頻率）
    angle_accumulator = []  # 用於角度累積計算（rotation mode）
    
    # 錄影相關
    writer = None
    writer_raw = None  # 原始影片（無標註）
    rec_data = []
    output_dir = None
    
    try:
        while running[0]:
            try:
                data = result_queue.get(timeout=0.1)
                frame = data['frame']
                
                # 顯示
                cv2.imshow(WINDOW_TITLE, frame)
                
                # 錄影
                if tracker_state['recording'] and RECORD_OUTPUT:
                    if writer is None:
                        run_tag = datetime.now().strftime("%Y%m%d_%H%M%S")
                        # 將電壓格式化為字串
                        voltage_str = str(VOLTAGE)
                        
                        # 建立資料夾名稱（包含 PID 參數，如果啟用）
                        if ENABLE_PID_CONTROL:
                            output_dir = f"{run_tag}_{MODE}_{voltage_str}V_P={STRAIGHT_PID_KP}_I={STRAIGHT_PID_KI}_D={STRAIGHT_PID_KD}"
                        else:
                            output_dir = f"{run_tag}_{MODE}_{voltage_str}V"
                        os.makedirs(output_dir, exist_ok=True)
                        
                        # 使用 MP4 格式（較好壓縮，廣泛支援）
                        out_path = os.path.join(output_dir, f"camera_{MODE}_tracked.mp4")
                        out_path_raw = os.path.join(output_dir, f"camera_{MODE}_raw.mp4")
                        
                        # 從實際 frame 獲取尺寸（避免 undistort 造成尺寸不匹配）
                        frame_height, frame_width = frame.shape[:2]
                        frame_size = (frame_width, frame_height)
                        
                        # 使用 mp4v codec（支援高 FPS，兼容性佳）
                        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                        writer = cv2.VideoWriter(out_path, fourcc, actual_fps, frame_size)
                        writer_raw = cv2.VideoWriter(out_path_raw, fourcc, actual_fps, frame_size)
                        
                        # 檢查 writer 是否成功開啟，嘗試備用 codec
                        if not writer.isOpened():
                            print("警告: mp4v codec 不可用，嘗試 avc1 (H.264)...")
                            fourcc = cv2.VideoWriter_fourcc(*'avc1')
                            writer = cv2.VideoWriter(out_path, fourcc, actual_fps, frame_size)
                            writer_raw = cv2.VideoWriter(out_path_raw, fourcc, actual_fps, frame_size)
                        
                        # 如果 MP4 仍然失敗，回退到 AVI 格式
                        if not writer.isOpened():
                            print("警告: MP4 編碼器不可用，回退到 AVI 格式...")
                            out_path = os.path.join(output_dir, f"camera_{MODE}_tracked.avi")
                            out_path_raw = os.path.join(output_dir, f"camera_{MODE}_raw.avi")
                            fourcc = cv2.VideoWriter_fourcc(*'XVID')
                            writer = cv2.VideoWriter(out_path, fourcc, actual_fps, frame_size)
                            writer_raw = cv2.VideoWriter(out_path_raw, fourcc, actual_fps, frame_size)
                        
                        # 保存 frame_size 以便後續檢查
                        tracker_state['frame_size'] = frame_size
                        tracker_state['actual_fps'] = actual_fps  # 儲存用於 GIF
                        print(f"開始錄影: {out_path} (FPS: {actual_fps:.1f}, Size: {frame_width}x{frame_height})")
                        print(f"原始影片: {out_path_raw}")
                    
                    # 確保 frame 有效且尺寸正確後才寫入
                    if frame is not None and frame.size > 0:
                        expected_size = tracker_state.get('frame_size')
                        current_size = (frame.shape[1], frame.shape[0])
                        if expected_size and current_size == expected_size:
                            writer.write(frame)
                        # 尺寸不匹配時跳過，不寫入
                    
                    # 寫入原始影片（無標註）
                    if 'raw_frame' in data and data['raw_frame'] is not None:
                        raw_frame = data['raw_frame']
                        if raw_frame.size > 0:
                            expected_size = tracker_state.get('frame_size')
                            current_size = (raw_frame.shape[1], raw_frame.shape[0])
                            if expected_size and current_size == expected_size:
                                writer_raw.write(raw_frame)
                    
                    # 記錄資料（包含 box）
                    rec_data.append((
                        data['frame_idx'], data['t_s'],
                        data['fx'], data['fy'],
                        data['fx_mm_abs'], data['fy_mm_abs'],
                        data['angle'], mm_per_px,
                        data.get('box')  # 記錄 box
                    ))
                    
                    # === PID 控制邏輯 ===
                    if pid_controller is not None and tracker_state.get('pid_active', False):
                        pid_update_counter += 1
                        
                        # 每隔一段時間更新 PID
                        if pid_update_counter >= PID_UPDATE_INTERVAL:
                            pid_update_counter = 0
                            
                            # 獲取當前位置和角度
                            x_mm = data['fx_mm_abs']
                            y_mm = data['fy_mm_abs']
                            current_angle = data['angle'] if np.isfinite(data['angle']) else 0.0
                            
                            # 更新 PID 控制器
                            pid_result = pid_controller.update(
                                x_mm, y_mm, current_angle, 
                                current_time=data['timestamp']
                            )
                            
                            # 檢查是否需要切換 FG 模式（智能校正）
                            if pid_result.get('mode_changed', False) and pid_result.get('fg_mode') is not None:
                                new_mode = pid_result['fg_mode']
                                if fg_connected and current_fg_mode != new_mode:
                                    fg_controller.switch_mode(new_mode)
                                    current_fg_mode = new_mode
                            
                            # 應用電壓調整
                            if fg_connected and fg_controller.is_output_active():
                                if pid_result['should_output']:
                                    fg_controller.set_voltages(
                                        pid_result['ch1_voltage'],
                                        pid_result['ch2_voltage'],
                                        silent=True
                                    )
                                else:
                                    # Rotation mode 完成，停止輸出
                                    fg_controller.turn_off()
                                    tracker_state['pid_active'] = False
                                    print(f"[PID] Rotation 完成，已停止輸出")
                            
                            # Rotation mode 完成檢查
                            if pid_result.get('complete', False):
                                print(f"[PID] 已達到目標角度 {ROTATION_TARGET_ANGLE}°")
                
            except queue.Empty:
                pass
            
            # 按鍵處理
            key = cv2.waitKey(1) & 0xFF
            
            if key in (27, ord('q')):  # ESC 或 Q
                print("\n停止中...")
                running[0] = False
                break
            elif key == 32:  # Space
                if not tracker_state['recording']:
                    tracker_state['recording'] = True
                    print("開始錄製...")
                else:
                    tracker_state['recording'] = False
                    print("停止錄製...")
                    if fg_connected:
                        fg_controller.turn_off()
                    break
            elif key == ord('h'):
                show_help()
            elif key == ord('0'):
                if fg_connected:
                    fg_controller.turn_off()
            elif key in [ord('1'), ord('2'), ord('3'), ord('4')]:
                if fg_connected:
                    mode_num = int(chr(key))
                    fg_controller.switch_mode(mode_num)
                    current_fg_mode = mode_num
                    
                    # 如果 PID 控制已在 config 中啟用，自動啟用 PID
                    if pid_controller is not None and ENABLE_PID_CONTROL:
                        # 如果是直走模式 (1 或 3)，更新 PID 控制器的 straight_mode
                        if mode_num in [1, 3]:
                            pid_controller.set_straight_mode(mode_num)
                        
                        # 自動啟用 PID 控制（如果尚未啟用）
                        if not tracker_state.get('pid_active', False):
                            tracker_state['pid_active'] = True
                            pid_controller.reset()
                            pid_controller.enable(True)
                            print(f"[PID] PID 控制已自動啟用 (config.ENABLE_PID_CONTROL = True)")
                else:
                    print("函數產生器未連接")
    
    except KeyboardInterrupt:
        print("\n接收到中斷信號...")
        running[0] = False
    
    finally:
        # 等待執行緒結束
        print("等待執行緒結束...")
        capture_t.join(timeout=2.0)
        process_t.join(timeout=2.0)
        
        # 清理
        cap.release()
        if writer:
            writer.release()
        if writer_raw:
            writer_raw.release()
        cv2.destroyAllWindows()
        
        # 輸出資料
        if len(rec_data) > 0 and output_dir:
            print("\n處理資料並輸出...")
            process_and_export_data(rec_data, output_dir, mm_per_px)
            
        
        # 斷開函數產生器
        fg_controller.disconnect()
        
        # 最終統計
        info = stats.get_info()
        print("\n最終統計:")
        print(f"  讀取幀數: {info['captured']}")
        print(f"  處理幀數: {info['processed']}")
        print(f"  丟棄幀數: {info['dropped']}")
        print(f"  讀取 FPS: {info['capture_fps']:.1f}")
        print(f"  處理 FPS: {info['process_fps']:.1f}")
        print("程式結束")


if __name__ == "__main__":
    main()
