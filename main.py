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

import os
import cv2
import time
import threading
import queue
from datetime import datetime
import numpy as np

from config import (
    MODE, CAMERA_INDEX, CAM_WIDTH, CAM_HEIGHT, CAM_FPS_REQ,
    RECORD_OUTPUT, WINDOW_TITLE, FRAME_QUEUE_SIZE, RESULT_QUEUE_SIZE,
    VOLTAGE
)
from stats import Stats
from function_generator import FunctionGeneratorController
from image_processing import find_target_and_angle, calibrate_scale
from signal_processing import make_kalman
from camera_threads import capture_thread, process_thread
from data_export import process_and_export_data


def show_help():
    """顯示幫助信息"""
    print("\n" + "="*60)
    print("整合式攝影機追蹤與函數產生器控制系統 (多執行緒)")
    print("="*60)
    print("攝影機控制：")
    print("  [Space]     - 開始/停止錄製")
    print("  [ESC/Q]     - 退出程式")
    print("")
    print("函數產生器控制：")
    print("  [1-4]       - Mode 1-4")
    print("  [0]         - 關閉函數產生器輸出")
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
    
    # 估算 mm/px - 使用網格或 device 尺寸校準
    ok, first = cap.read()
    if not ok:
        print("無法讀取影像")
        cap.release()
        return
    
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
    tracker_state = {'recording': False}
    
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
    
    show_help()
    print(f"\n系統狀態：")
    print(f"✓ 攝影機：已連接 ({actual_width}x{actual_height} @ {actual_fps:.0f} FPS)")
    print(f"{'✓' if fg_connected else '✗'} 函數產生器：{'已連接' if fg_connected else '未連接'}")
    print(f"\n系統就緒！按 [H] 查看幫助\n")
    
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
                        output_dir = f"{run_tag}_{MODE}_{voltage_str}V"
                        os.makedirs(output_dir, exist_ok=True)
                        
                        # 使用 AVI 格式以確保兼容性
                        out_path = os.path.join(output_dir, f"camera_{MODE}_tracked.avi")
                        out_path_raw = os.path.join(output_dir, f"camera_{MODE}_raw.avi")
                        
                        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
                        writer = cv2.VideoWriter(out_path, fourcc, actual_fps, (CAM_HEIGHT, CAM_WIDTH))
                        writer_raw = cv2.VideoWriter(out_path_raw, fourcc, actual_fps, (CAM_HEIGHT, CAM_WIDTH))
                        tracker_state['actual_fps'] = actual_fps  # 儲存用於 GIF
                        print(f"開始錄影: {out_path} (FPS: {actual_fps:.1f})")
                        print(f"原始影片: {out_path_raw}")
                    
                    writer.write(frame)
                    
                    # 寫入原始影片（無標註）
                    if 'raw_frame' in data:
                        writer_raw.write(data['raw_frame'])
                    
                    # 記錄資料（包含 box）
                    rec_data.append((
                        data['frame_idx'], data['t_s'],
                        data['fx'], data['fy'],
                        data['fx_mm_abs'], data['fy_mm_abs'],
                        data['angle'], mm_per_px,
                        data.get('box')  # 記錄 box
                    ))
                
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
