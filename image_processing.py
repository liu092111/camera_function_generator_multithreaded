# -*- coding: utf-8 -*-
"""
影像處理模組
提供影像分析和目標檢測相關函數
"""

import cv2
import numpy as np
from config import (
    HSV_YELLOW_LO, HSV_YELLOW_HI,
    HSV_WHITE_LO, HSV_WHITE_HI,
    MIN_CONTOUR_AREA,
    DEVICE_WIDTH_MM, DEVICE_HEIGHT_MM
)


def find_target_and_angle(frame_bgr):
    """
    在影像中尋找目標並計算角度
    
    Args:
        frame_bgr: BGR 格式的影像
    
    Returns:
        (cx, cy, angle_deg, box) 或 None
        - cx, cy: 中心座標
        - angle_deg: 角度（度）
        - box: 矩形的四個頂點
    """
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    mask_y = cv2.inRange(hsv, HSV_YELLOW_LO, HSV_YELLOW_HI)
    mask_w = cv2.inRange(hsv, HSV_WHITE_LO, HSV_WHITE_HI)
    mask = cv2.bitwise_or(mask_y, mask_w)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8), iterations=1)

    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    cnt = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(cnt) < MIN_CONTOUR_AREA:
        return None

    rect = cv2.minAreaRect(cnt)
    (cx, cy), (rw, rh), rect_angle = rect
    
    if rw >= rh:
        angle_deg = rect_angle
    else:
        angle_deg = rect_angle - 90.0
    
    while angle_deg >= 90.0:
        angle_deg -= 180.0
    while angle_deg < -90.0:
        angle_deg += 180.0

    box = cv2.boxPoints(rect).astype(int)
    return (cx, cy, angle_deg, box)


def calibrate_scale(frame):
    """
    校準尺度 (mm/pixel)
    方法 1: 檢測 5mm 網格
    方法 2: 使用 device 尺寸 (9mm × 6mm)
    
    Args:
        frame: 輸入影像
    
    Returns:
        mm/pixel 比例
    """
    GRID_MM = 5.0
    
    # 方法 1: 嘗試檢測網格
    print("  嘗試方法 1: 檢測背景網格...")
    grid_scale = detect_grid_scale(frame, GRID_MM)
    
    if grid_scale is not None and 0.05 < grid_scale < 0.5:  # 合理範圍檢查
        print(f"    ✓ 網格檢測成功")
        return grid_scale
    else:
        print(f"    ✗ 網格檢測失敗或不可靠")
    
    # 方法 2: 使用 device 尺寸
    print("  嘗試方法 2: 使用 device 尺寸 (9mm × 6mm)...")
    device_scale = detect_device_scale(frame, DEVICE_WIDTH_MM, DEVICE_HEIGHT_MM)
    
    if device_scale is not None and 0.05 < device_scale < 0.5:
        print(f"    ✓ Device 檢測成功")
        return device_scale
    else:
        print(f"    ✗ Device 檢測失敗")
    
    # 預設值
    print("  ⚠ 使用預設值: 0.1 mm/pixel")
    return 0.1


def detect_grid_scale(frame, grid_spacing_mm):
    """
    檢測網格間距來估算尺度
    
    Args:
        frame: 輸入影像
        grid_spacing_mm: 網格間距（mm）
    
    Returns:
        mm/pixel 比例或 None
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (3, 3), 0)
    edges = cv2.Canny(gray, 50, 150)
    
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=80, minLineLength=40, maxLineGap=15)
    if lines is None or len(lines) < 6:
        return None
    
    horiz, vert = [], []
    for l in lines:
        x1, y1, x2, y2 = l[0]
        dx, dy = x2 - x1, y2 - y1
        ang = np.degrees(np.arctan2(dy, dx))
        if ang < -90:
            ang += 180
        if ang > 90:
            ang -= 180
        
        if abs(ang) < 8:
            horiz.extend([y1, y2])
        elif abs(abs(ang) - 90) < 8:
            vert.extend([x1, x2])
    
    def get_spacing(positions):
        if len(positions) < 4:
            return None
        positions = sorted(positions)
        unique = [positions[0]]
        for p in positions[1:]:
            if abs(p - unique[-1]) > 1.5:
                unique.append(p)
        
        if len(unique) < 3:
            return None
        
        diffs = np.diff(unique)
        diffs = diffs[(diffs > 2) & (diffs < 300)]
        if len(diffs) == 0:
            return None
        
        return float(np.min(diffs))  # 最小間距應該是 5mm
    
    sp_h = get_spacing(horiz)
    sp_v = get_spacing(vert)
    
    if sp_h and sp_v:
        px_per_cell = (sp_h + sp_v) / 2.0
    elif sp_h:
        px_per_cell = sp_h
    elif sp_v:
        px_per_cell = sp_v
    else:
        return None
    
    if px_per_cell > 0:
        return grid_spacing_mm / px_per_cell
    return None


def detect_device_scale(frame, device_width_mm, device_height_mm):
    """
    使用 device 尺寸 (9mm × 6mm) 來估算尺度
    
    Args:
        frame: 輸入影像
        device_width_mm: Device 寬度（mm）
        device_height_mm: Device 高度（mm）
    
    Returns:
        mm/pixel 比例或 None
    """
    # 先旋轉畫面（因為主程式會旋轉）
    frame_rotated = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    
    # 找到 device
    result = find_target_and_angle(frame_rotated)
    if result is None:
        return None
    
    cx, cy, angle_deg, box = result
    
    # 計算 bounding box 的寬和高（pixels）
    rect = cv2.minAreaRect(box)
    (center_x, center_y), (width_px, height_px), angle = rect
    
    # 確保寬是長邊（9mm），高是短邊（6mm）
    if width_px < height_px:
        width_px, height_px = height_px, width_px
    
    # 使用兩個維度的平均值來估算 mm/pixel
    scale_from_width = device_width_mm / width_px if width_px > 0 else None
    scale_from_height = device_height_mm / height_px if height_px > 0 else None
    
    if scale_from_width and scale_from_height:
        # 使用平均值
        mm_per_px = (scale_from_width + scale_from_height) / 2.0
        print(f"    Device 尺寸: {width_px:.1f} × {height_px:.1f} pixels")
        print(f"    從寬度: {scale_from_width:.4f} mm/px")
        print(f"    從高度: {scale_from_height:.4f} mm/px")
        return mm_per_px
    elif scale_from_width:
        return scale_from_width
    elif scale_from_height:
        return scale_from_height
    
    return None
