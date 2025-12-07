# -*- coding: utf-8 -*-
"""
訊號處理模組
提供數學和訊號處理相關函數
"""

import numpy as np
import cv2
from config import KF_PROCESS_NOISE, KF_MEASURE_NOISE


def unwrap_angles_deg(a_deg):
    """
    解包裹角度（度）
    
    Args:
        a_deg: 角度陣列（度）
    
    Returns:
        解包裹後的角度陣列
    """
    if len(a_deg) == 0:
        return a_deg
    return np.rad2deg(np.unwrap(np.deg2rad(a_deg)))


def wrap_angles_deg(a_deg):
    """
    包裹角度到 [-90, 90] 範圍（度）
    
    Args:
        a_deg: 角度陣列（度）
    
    Returns:
        包裹後的角度陣列
    """
    return ((a_deg + 90) % 180) - 90


def moving_average(a, w):
    """
    移動平均濾波
    
    Args:
        a: 輸入陣列
        w: 視窗大小
    
    Returns:
        平滑後的陣列
    """
    if w is None or w <= 1:
        return a
    a = np.asarray(a, dtype=float)
    if len(a) < w:
        return a
    kernel = np.ones(w) / float(w)
    return np.convolve(a, kernel, mode='same')


def finite_diff(values, t, smooth_win=1):
    """
    有限差分計算速度
    
    Args:
        values: 位置陣列
        t: 時間陣列
        smooth_win: 平滑視窗大小
    
    Returns:
        速度陣列
    """
    v = np.full_like(values, np.nan, dtype=float)
    for i in range(1, len(values)):
        if np.isfinite(values[i]) and np.isfinite(values[i-1]) and (t[i] > t[i-1]):
            v[i] = (values[i] - values[i-1]) / (t[i] - t[i-1])
    if smooth_win and smooth_win > 1:
        v = moving_average(v, smooth_win)
    return v


def make_kalman():
    """
    建立 Kalman 濾波器
    
    Returns:
        cv2.KalmanFilter 物件
    """
    kf = cv2.KalmanFilter(4, 2)
    kf.transitionMatrix = np.array([
        [1, 0, 1, 0],
        [0, 1, 0, 1],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ], dtype=np.float32)
    kf.measurementMatrix = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ], dtype=np.float32)
    kf.processNoiseCov = np.eye(4, dtype=np.float32) * KF_PROCESS_NOISE
    kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * KF_MEASURE_NOISE
    kf.errorCovPost = np.eye(4, dtype=np.float32)
    return kf


def ema(prev, cur, alpha):
    """
    指數移動平均（EMA）
    
    Args:
        prev: 前一個值
        cur: 當前值
        alpha: 平滑係數
    
    Returns:
        平滑後的值
    """
    if prev is None or not np.isfinite(prev):
        return cur
    if not np.isfinite(cur):
        return prev
    return alpha * prev + (1.0 - alpha) * cur
