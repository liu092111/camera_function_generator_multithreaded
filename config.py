# -*- coding: utf-8 -*-
"""
配置管理模組
集中管理所有系統配置參數
"""

import numpy as np

# ========== 攝影機設定 ==========
MODE = "straight"  # "straight" 或 "rotation"
CAMERA_INDEX = 1
CAM_WIDTH = 640    # 攝影機解析度寬度
CAM_HEIGHT = 480   # 攝影機解析度高度
CAM_FPS_REQ = 120  # 目標 FPS
RECORD_OUTPUT = True
WINDOW_TITLE = "Camera & Function Generator Control (Multithreaded)"
DISPLAY_SCALE = 0.6  # 顯示視窗縮放比例（60%）

# ========== 多執行緒設定 ==========
FRAME_QUEUE_SIZE = 120   # 幀佇列大小（可緩存 1 秒的幀）
RESULT_QUEUE_SIZE = 30   # 結果佇列大小

# ========== 尺度校準設定 ==========
GRID_SPACING_MM = 5.0           # 網格間距（mm）
AUTO_GRID_MM_PER_PX = True      # 自動校準尺度
MANUAL_MM_PER_PX = None         # 手動設定 mm/pixel（如果不自動校準）

# Device 尺寸設定（用於校準）
DEVICE_WIDTH_MM = 9.0   # Device 寬度（mm）
DEVICE_HEIGHT_MM = 6.0  # Device 高度（mm）

# ========== 顏色遮罩參數（HSV） ==========
HSV_YELLOW_LO = np.array([15, 60, 120], dtype=np.uint8)
HSV_YELLOW_HI = np.array([35, 255, 255], dtype=np.uint8)
HSV_WHITE_LO = np.array([0, 0, 200], dtype=np.uint8)
HSV_WHITE_HI = np.array([180, 60, 255], dtype=np.uint8)

# ========== 影像處理參數 ==========
MIN_CONTOUR_AREA = 50  # 最小輪廓面積

# ========== 追蹤與濾波參數 ==========
# Kalman 濾波器
KF_PROCESS_NOISE = 1e-5  # 過程噪音（降低以減少抖動）
KF_MEASURE_NOISE = 1e-2  # 測量噪音（增加以更信任預測）

# 指數移動平均（EMA）
EMA_ALPHA_POS = 0.25    # 位置平滑係數（增加以更平滑）
EMA_ALPHA_ANGLE = 0.20  # 角度平滑係數（增加以更平滑）

# 測量跳躍檢測
MAX_MEAS_JUMP_PX = 80   # 最大測量跳躍（pixels）

# 靜止檢測閾值
STATIC_THRESHOLD_PX = 2.0        # 靜止閾值（pixels）
STATIC_SPEED_THRESHOLD = 1.0     # 靜止速度閾值（mm/s）

# ========== 視覺化參數 ==========
INVERT_Y_AXIS = False           # 是否反轉 Y 軸
ORIENT_PLOT_WRAPPED = True      # 角度圖是否使用包裹顯示
ORIENT_YLIM_DEG = 60            # 角度圖 Y 軸範圍（±度）
PLOT_RANGE_SCALE = 1.35         # 圖表範圍縮放因子

# ========== 函數產生器設定 ==========
FG_RESOURCE_STRING = 'USB0::0x0957::0x5707::MY59001615::0::INSTR'

# 模式配置
FG_MODE_CONFIGS = {
    1: {
        'file1': 'modal/ONEPERIOD_A_25k_50k_84p88deg_2000pts.csv',
        'file2': 'modal/ONEPERIOD_B_25k_50k_264p88deg_2000pts.csv',
        'name1': 'WF_25K_84',
        'name2': 'WF_25K_264',
        'ch1_volt': 1.2,  # 可調整電壓（V）
        'ch2_volt': 1.2,  # 可調整電壓（V）
        'ch1_pol': 'NORM',
        'ch2_pol': 'INV',
        'desc': '25k Hz, CH1=NORM, CH2=INV'
    },
    2: {
        'file1': 'modal/ONEPERIOD_C_47k_94k_57p32deg_2000pts.csv',
        'file2': 'modal/ONEPERIOD_D_47k_94k_237p32deg_2000pts.csv',
        'name1': 'WF_47K_57',
        'name2': 'WF_47K_237',
        'ch1_volt': 1.2,  # 可調整電壓（V）
        'ch2_volt': 1.2,  # 可調整電壓（V）
        'ch1_pol': 'NORM',
        'ch2_pol': 'INV',
        'desc': '47k Hz, CH1=NORM, CH2=INV'
    },
    3: {
        'file1': 'modal/ONEPERIOD_A_25k_50k_84p88deg_2000pts.csv',
        'file2': 'modal/ONEPERIOD_B_25k_50k_264p88deg_2000pts.csv',
        'name1': 'WF_25K_84',
        'name2': 'WF_25K_264',
        'ch1_volt': 1.2,  # 可調整電壓（V）
        'ch2_volt': 1.2,  # 可調整電壓（V）
        'ch1_pol': 'INV',
        'ch2_pol': 'NORM',
        'desc': '25k Hz, CH1=INV, CH2=NORM'
    },
    4: {
        'file1': 'modal/ONEPERIOD_C_47k_94k_57p32deg_2000pts.csv',
        'file2': 'modal/ONEPERIOD_D_47k_94k_237p32deg_2000pts.csv',
        'name1': 'WF_47K_57',
        'name2': 'WF_47K_237',
        'ch1_volt': 1.2,  # 可調整電壓（V）
        'ch2_volt': 1.2,  # 可調整電壓（V）
        'ch1_pol': 'INV',
        'ch2_pol': 'NORM',
        'desc': '47k Hz, CH1=INV, CH2=NORM'
    }
}

# ========== 依賴檢查 ==========
# PyVISA 可用性
try:
    import pyvisa as visa
    HAVE_VISA = True
except ImportError:
    HAVE_VISA = False

# Scipy 可用性
try:
    from scipy.signal import savgol_filter
    HAVE_SG = True
except Exception:
    HAVE_SG = False
