# -*- coding: utf-8 -*-
"""
統計資訊模組
提供執行緒安全的效能統計功能
"""

import threading
import time


class Stats:
    """執行緒安全的統計資訊類別"""
    
    def __init__(self):
        self.lock = threading.Lock()
        self.frames_captured = 0
        self.frames_processed = 0
        self.frames_dropped = 0
        self.capture_fps = 0.0
        self.process_fps = 0.0
        self.last_capture_time = time.perf_counter()
        self.last_process_time = time.perf_counter()
    
    def update_capture(self):
        """更新讀取幀數和 FPS"""
        with self.lock:
            self.frames_captured += 1
            now = time.perf_counter()
            if self.frames_captured % 30 == 0:
                elapsed = now - self.last_capture_time
                if elapsed > 0:
                    self.capture_fps = 30.0 / elapsed
                self.last_capture_time = now
    
    def update_process(self):
        """更新處理幀數和 FPS"""
        with self.lock:
            self.frames_processed += 1
            now = time.perf_counter()
            if self.frames_processed % 30 == 0:
                elapsed = now - self.last_process_time
                if elapsed > 0:
                    self.process_fps = 30.0 / elapsed
                self.last_process_time = now
    
    def drop_frame(self):
        """記錄丟棄的幀"""
        with self.lock:
            self.frames_dropped += 1
    
    def get_info(self):
        """獲取統計資訊（執行緒安全）"""
        with self.lock:
            return {
                'captured': self.frames_captured,
                'processed': self.frames_processed,
                'dropped': self.frames_dropped,
                'capture_fps': self.capture_fps,
                'process_fps': self.process_fps
            }
