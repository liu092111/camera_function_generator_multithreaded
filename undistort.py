# -*- coding: utf-8 -*-
"""
鏡頭畸變校正模組
載入校正參數並提供即時影像畸變校正功能
支持兩種校正模式：
1. TPS (Thin Plate Spline) 校正 - 使用 map_x, map_y 映射表
2. 傳統相機校正 - 使用 mtx, dist 參數
"""

import os
import cv2
import numpy as np
from config import CALIBRATION_DATA_PATH, UNDISTORT_CROP


class UndistortCorrector:
    """
    鏡頭畸變校正器
    使用預先計算的相機校正參數來校正影像畸變
    支持 TPS 映射表和傳統相機校正兩種模式
    """
    
    def __init__(self, calibration_path=None):
        """
        初始化校正器
        
        Args:
            calibration_path: 校正數據檔案路徑 (.npz)
                            如果為 None，使用 config 中的預設路徑
        """
        self.calibration_path = calibration_path or CALIBRATION_DATA_PATH
        self.mtx = None           # 相機內參矩陣
        self.dist = None          # 畸變係數
        self.new_mtx = None       # 優化後的相機矩陣
        self.roi = None           # 有效區域
        self.map1 = None          # 校正映射表 x (也可用於 TPS map_x)
        self.map2 = None          # 校正映射表 y (也可用於 TPS map_y)
        self.img_size = None      # 影像尺寸 (width, height)
        self.out_size = None      # TPS 輸出尺寸
        self.reprojection_error = None  # 重投影誤差
        self.is_initialized = False
        self.use_tps = False      # 是否使用 TPS 校正模式
        
    def load_calibration(self):
        """
        載入校正數據
        自動檢測是 TPS 映射表還是傳統相機校正參數
        
        Returns:
            bool: 是否成功載入
        """
        if not os.path.exists(self.calibration_path):
            print(f"✗ 校正數據檔案不存在: {self.calibration_path}")
            return False
        
        try:
            data = np.load(self.calibration_path)
            
            # 優先檢查是否為 TPS 映射表格式
            if 'map_x' in data and 'map_y' in data:
                return self._load_tps_calibration(data)
            # 否則嘗試傳統相機校正格式
            elif 'mtx' in data and 'dist' in data:
                return self._load_traditional_calibration(data)
            else:
                print(f"✗ 無法識別校正數據格式: {self.calibration_path}")
                print(f"  可用的鍵值: {list(data.keys())}")
                return False
            
        except Exception as e:
            print(f"✗ 載入校正數據失敗: {e}")
            return False
    
    def _load_tps_calibration(self, data):
        """
        載入 TPS 校正映射表
        支援裁切版 (map_x_cropped) 和完整版 (map_x)
        
        Args:
            data: npz 數據
            
        Returns:
            bool: 是否成功載入
        """
        self.use_tps = True
        self.use_cropped = False  # 預設使用裁切版
        self.valid_region = None  # 有效區域偏移
        
        # 優先使用裁切版（推薦，無黑邊）
        if 'map_x_cropped' in data and 'map_y_cropped' in data:
            self.map1 = data['map_x_cropped'].astype(np.float32)
            self.map2 = data['map_y_cropped'].astype(np.float32)
            self.use_cropped = True
            
            # 載入有效區域資訊（用於座標轉換）
            if 'valid_region' in data:
                self.valid_region = tuple(data['valid_region'])  # [x0, y0, w, h]
        else:
            # 退回到完整版
            self.map1 = data['map_x'].astype(np.float32)
            self.map2 = data['map_y'].astype(np.float32)
        
        # 載入原始 ROI 資訊
        if 'roi' in data:
            self.roi = tuple(data['roi'])
        else:
            self.roi = (0, 0, self.map1.shape[1], self.map1.shape[0])
        
        # 輸出尺寸
        if 'out_size' in data:
            self.out_size = tuple(data['out_size'])  # 完整版尺寸
        else:
            self.out_size = (self.map1.shape[1], self.map1.shape[0])
        
        # 實際輸出尺寸（根據是否裁切）
        self.img_size = (self.map1.shape[1], self.map1.shape[0])
        self.is_initialized = True
        
        print(f"✓ 已載入 TPS 校正映射表: {self.calibration_path}")
        print(f"  模式: {'裁切版（推薦）' if self.use_cropped else '完整版'}")
        print(f"  輸出尺寸: {self.img_size[0]}x{self.img_size[1]}")
        if self.valid_region:
            print(f"  有效區域偏移: x={self.valid_region[0]}, y={self.valid_region[1]}")
        
        return True
    
    def _load_traditional_calibration(self, data):
        """
        載入傳統相機校正參數
        
        Args:
            data: npz 數據
            
        Returns:
            bool: 是否成功載入
        """
        self.mtx = data['mtx']
        self.dist = data['dist']
        self.use_tps = False
        
        if 'reprojection_error' in data:
            self.reprojection_error = float(data['reprojection_error'])
        
        print(f"✓ 已載入傳統相機校正數據: {self.calibration_path}")
        print(f"  相機矩陣 fx={self.mtx[0,0]:.1f}, fy={self.mtx[1,1]:.1f}")
        print(f"  畸變係數 k1={self.dist[0,0]:.4f}, k2={self.dist[0,1]:.4f}")
        if self.reprojection_error is not None:
            print(f"  重投影誤差: {self.reprojection_error:.4f}")
        
        return True
    
    def initialize_maps(self, img_size):
        """
        初始化校正映射表（預計算以提升效能）
        僅用於傳統相機校正模式，TPS 模式已在載入時初始化
        
        Args:
            img_size: 影像尺寸 (width, height)
        """
        # TPS 模式已經有映射表，直接返回
        if self.use_tps:
            return True
            
        if self.mtx is None or self.dist is None:
            print("✗ 請先載入校正數據")
            return False
        
        self.img_size = img_size
        w, h = img_size
        
        # 獲取優化後的相機矩陣
        # alpha=0: 只保留有效像素（裁剪黑邊）
        # alpha=1: 保留所有像素（可能有黑邊）
        alpha = 0 if UNDISTORT_CROP else 1
        self.new_mtx, self.roi = cv2.getOptimalNewCameraMatrix(
            self.mtx, self.dist, (w, h), alpha, (w, h)
        )
        
        # 預計算校正映射表
        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            self.mtx, self.dist, None, self.new_mtx, (w, h), cv2.CV_16SC2
        )
        
        self.is_initialized = True
        print(f"✓ 校正映射表已初始化 ({w}x{h})")
        
        if self.roi[2] > 0 and self.roi[3] > 0:
            print(f"  有效區域: x={self.roi[0]}, y={self.roi[1]}, "
                  f"w={self.roi[2]}, h={self.roi[3]}")
        
        return True
    
    def undistort(self, frame, crop=None):
        """
        校正單張影像的畸變
        
        Args:
            frame: 輸入影像 (BGR)
            crop: 是否裁剪黑邊（None 使用預設值，True/False 覆蓋）
        
        Returns:
            校正後的影像
        """
        if self.use_tps:
            return self._undistort_tps(frame)
        else:
            return self._undistort_traditional(frame, crop)
    
    def _undistort_tps(self, frame):
        """
        使用 TPS 映射表校正影像
        
        Args:
            frame: 輸入影像 (BGR)
        
        Returns:
            校正後的影像
        """
        if self.map1 is None or self.map2 is None:
            return frame
        
        # 如果有 ROI，先提取 ROI 區域
        x0, y0, w0, h0 = self.roi
        if x0 > 0 or y0 > 0 or w0 < frame.shape[1] or h0 < frame.shape[0]:
            # 確保 ROI 在影像範圍內
            x0 = max(0, min(x0, frame.shape[1]))
            y0 = max(0, min(y0, frame.shape[0]))
            w0 = min(w0, frame.shape[1] - x0)
            h0 = min(h0, frame.shape[0] - y0)
            roi_frame = frame[y0:y0+h0, x0:x0+w0]
        else:
            roi_frame = frame
        
        # 使用 TPS 映射表進行校正
        dst = cv2.remap(roi_frame, self.map1, self.map2, 
                       interpolation=cv2.INTER_LINEAR,
                       borderMode=cv2.BORDER_CONSTANT, 
                       borderValue=(0, 0, 0))
        
        return dst
    
    def _undistort_traditional(self, frame, crop=None):
        """
        使用傳統相機校正參數校正影像
        
        Args:
            frame: 輸入影像 (BGR)
            crop: 是否裁剪黑邊
        
        Returns:
            校正後的影像
        """
        if not self.is_initialized:
            # 如果尚未初始化，先初始化映射表
            h, w = frame.shape[:2]
            if not self.initialize_maps((w, h)):
                return frame  # 初始化失敗，返回原圖
        
        # 使用預計算的映射表進行校正（比 cv2.undistort 快）
        dst = cv2.remap(frame, self.map1, self.map2, cv2.INTER_LINEAR)
        
        # 決定是否裁剪
        do_crop = UNDISTORT_CROP if crop is None else crop
        
        if do_crop and self.roi[2] > 0 and self.roi[3] > 0:
            x, y, w, h = self.roi
            dst = dst[y:y+h, x:x+w]
        
        return dst
    
    def undistort_points(self, points):
        """
        校正點座標的畸變
        
        Args:
            points: 點座標陣列，形狀為 (N, 2) 或 (N, 1, 2)
        
        Returns:
            校正後的點座標
        """
        if self.mtx is None or self.dist is None:
            return points
        
        points = np.array(points, dtype=np.float32)
        if points.ndim == 2:
            points = points.reshape(-1, 1, 2)
        
        undistorted = cv2.undistortPoints(
            points, self.mtx, self.dist, P=self.new_mtx
        )
        
        return undistorted.reshape(-1, 2)
    
    def get_info(self):
        """
        獲取校正器資訊
        
        Returns:
            dict: 包含校正參數的字典
        """
        return {
            'calibration_path': self.calibration_path,
            'is_initialized': self.is_initialized,
            'img_size': self.img_size,
            'reprojection_error': self.reprojection_error,
            'mtx': self.mtx,
            'dist': self.dist,
            'roi': self.roi
        }


# 全域校正器實例（單例模式）
_global_corrector = None


def get_undistort_corrector():
    """
    獲取全域校正器實例
    
    Returns:
        UndistortCorrector: 校正器實例
    """
    global _global_corrector
    if _global_corrector is None:
        _global_corrector = UndistortCorrector()
    return _global_corrector


def init_undistort():
    """
    初始化全域校正器
    
    Returns:
        bool: 是否成功初始化
    """
    corrector = get_undistort_corrector()
    return corrector.load_calibration()


def undistort_frame(frame, crop=None):
    """
    校正單張影像（便捷函數）
    
    Args:
        frame: 輸入影像
        crop: 是否裁剪黑邊
    
    Returns:
        校正後的影像
    """
    corrector = get_undistort_corrector()
    return corrector.undistort(frame, crop)
