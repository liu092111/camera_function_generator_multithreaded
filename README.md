# 整合式攝影機追蹤與函數產生器控制系統

多執行緒架構的高效能攝影機追蹤系統，整合函數產生器控制功能。

## 📋 功能特點

### 核心功能
- 🎥 **高速相機追蹤** - 支援 120 FPS @ 640x480 解析度
- 🧵 **多執行緒架構** - 分離讀取、處理和顯示執行緒以提升效能
- 🎛️ **函數產生器控制** - 支援 4 種波形模式快速切換
- 📊 **即時資料分析** - 位置、速度、角度即時追蹤
- 💾 **自動資料匯出** - CSV 資料和多種視覺化圖表

### 追蹤功能
- Kalman 濾波器平滑追蹤
- 指數移動平均 (EMA) 降低抖動
- 自動尺度校準（支援網格或 device 尺寸）
- 支援 straight 和 rotation 兩種追蹤模式

### 視覺化輸出
- 位置軌跡圖
- 速度/角度時間序列圖
- 軌跡輪廓圖（8 個時間點）
- 8 時間點組合影像

## 📁 專案結構

```
camera_function_generator_multithreaded/
├── main.py                    # 主程式入口
├── config.py                  # 配置管理（含電壓設定）
├── stats.py                   # 執行緒安全的統計模組
├── function_generator.py      # 函數產生器控制
├── image_processing.py        # 影像處理與目標檢測
├── signal_processing.py       # 訊號處理與濾波
├── camera_threads.py          # 多執行緒相機控制
├── data_export.py             # 資料匯出與視覺化
├── modal/                     # 波形檔案目錄
│   ├── ONEPERIOD_A_*.csv
│   ├── ONEPERIOD_B_*.csv
│   ├── ONEPERIOD_C_*.csv
│   └── ONEPERIOD_D_*.csv
├── README.md                  # 本檔案
└── requirements.txt           # Python 依賴套件
```

## 🚀 快速開始

### 環境需求

- Python 3.7+
- Windows 10/11（使用 DirectShow 支援）
- 相機（支援 MJPG 格式）
- （可選）Keysight 33600 系列函數產生器

### 安裝步驟

1. **克隆專案**
```bash
git clone <repository-url>
cd camera_function_generator_multithreaded
```

2. **安裝依賴套件**
```bash
pip install -r requirements.txt
```

3. **配置系統**

編輯 `config.py` 調整以下參數：

```python
# 相機設定
CAMERA_INDEX = 1              # 相機索引
MODE = "straight"             # 追蹤模式："straight" 或 "rotation"

# 函數產生器設定
FG_RESOURCE_STRING = 'USB0::0x0957::0x5707::MY59001615::0::INSTR'

# 電壓設定（可在 FG_MODE_CONFIGS 中調整各模式的電壓）
# 預設：1.2V
```

### 執行程式

```bash
python main.py
```

## 🎮 操作說明

### 鍵盤控制

#### 攝影機控制
- `Space` - 開始/停止錄製
- `ESC` 或 `Q` - 退出程式

#### 函數產生器控制
- `1` - Mode 1 (25k Hz, CH1=NORM, CH2=INV)
- `2` - Mode 2 (47k Hz, CH1=NORM, CH2=INV)
- `3` - Mode 3 (25k Hz, CH1=INV, CH2=NORM)
- `4` - Mode 4 (47k Hz, CH1=INV, CH2=NORM)
- `0` - 關閉函數產生器輸出

#### 其他
- `H` - 顯示幫助訊息

## ⚙️ 配置說明

### 主要配置參數 (config.py)

#### 攝影機參數
```python
MODE = "straight"              # 追蹤模式
CAMERA_INDEX = 1               # 相機索引
CAM_WIDTH = 640                # 解析度寬度
CAM_HEIGHT = 480               # 解析度高度
CAM_FPS_REQ = 120              # 目標 FPS
```

#### 追蹤參數
```python
# Kalman 濾波器
KF_PROCESS_NOISE = 1e-5        # 過程噪音
KF_MEASURE_NOISE = 1e-2        # 測量噪音

# EMA 平滑
EMA_ALPHA_POS = 0.25           # 位置平滑係數
EMA_ALPHA_ANGLE = 0.20         # 角度平滑係數
```

#### 顏色遮罩參數 (HSV)
```python
HSV_YELLOW_LO = [15, 60, 120]  # 黃色下界
HSV_YELLOW_HI = [35, 255, 255] # 黃色上界
HSV_WHITE_LO = [0, 0, 200]     # 白色下界
HSV_WHITE_HI = [180, 60, 255]  # 白色上界
```

#### 函數產生器電壓設定
```python
FG_MODE_CONFIGS = {
    1: {
        'ch1_volt': 1.2,  # Channel 1 電壓 (V)
        'ch2_volt': 1.2,  # Channel 2 電壓 (V)
        # ... 其他設定
    },
    # 模式 2-4 同理
}
```

## 📊 輸出資料

### 檔案結構

每次錄製會產生一個帶時間戳的資料夾：

```
YYYYMMDD_HHMMSS_<mode>_integrated_mt/
├── camera_<mode>_tracked.avi              # 追蹤影片（含標註）
├── camera_<mode>_raw.avi                  # 原始影片（無標註）
├── camera_<mode>_pos_angle_speed.csv      # 追蹤資料 CSV
├── camera_<mode>_position.png             # 位置軌跡圖
├── camera_<mode>_speed_orientation.png    # 速度與角度圖（straight 模式）
├── camera_<mode>_angular_speed.png        # 角速度圖（rotation 模式）
├── camera_<mode>_trajectory_center_only.png  # 軌跡輪廓圖
└── camera_<mode>_8_timepoints_composite.png  # 8 時間點組圖
```

### CSV 資料欄位

- `frame` - 幀編號
- `t_s` - 時間（秒）
- `x_px_filt`, `y_px_filt` - 濾波後位置（pixels）
- `x_mm_abs`, `y_mm_abs` - 絕對位置（mm）
- `x_mm`, `y_mm` - 相對位置（mm）
- `angle_deg_raw` - 原始角度（度）
- `angle_deg_unwrapped` - 解包裹角度（度）
- `vx_mm_s`, `vy_mm_s` - 速度分量（mm/s）
- `speed_mm_s` - 速度大小（mm/s）
- `angular_vel_dps` - 角速度（deg/s）

## 🔧 進階功能

### 尺度校準

系統支援兩種自動校準方法：

1. **網格檢測** - 檢測背景的 5mm 網格
2. **Device 尺寸** - 使用已知的 device 尺寸（9mm × 6mm）

如果自動校準失敗，系統會使用預設值 0.1 mm/pixel。

### 手動校準

在 `config.py` 中設定：

```python
AUTO_GRID_MM_PER_PX = False
MANUAL_MM_PER_PX = 0.08  # 手動設定值
```

## 🐛 常見問題

### Q: 無法連接相機
**A:** 
1. 確認相機索引是否正確（通常為 0 或 1）
2. 檢查相機是否支援 MJPG 格式
3. 確認沒有其他程式佔用相機

### Q: FPS 太低
**A:**
1. 降低解析度（例如改為 320x240）
2. 確認使用 MJPG 格式
3. 檢查 CPU 負載是否過高

### Q: 函數產生器連接失敗
**A:**
1. 確認已安裝 PyVISA 和相關驅動
2. 檢查 `FG_RESOURCE_STRING` 是否正確
3. 確認 USB 連接正常

### Q: 追蹤不穩定
**A:**
1. 調整顏色遮罩參數（HSV 範圍）
2. 增加 EMA 平滑係數
3. 調整 Kalman 濾波器參數
4. 確保光照條件良好

## 📝 開發指南

### 模組說明

- **config.py** - 集中管理所有配置參數
- **stats.py** - 執行緒安全的效能統計
- **function_generator.py** - 完整的函數產生器控制邏輯
- **image_processing.py** - 目標檢測與尺度校準
- **signal_processing.py** - 數學運算和訊號處理
- **camera_threads.py** - 多執行緒相機讀取與處理
- **data_export.py** - 資料處理和圖表生成
- **main.py** - 程式流程控制

### 擴展建議

1. **新增追蹤模式** - 在 `config.py` 修改 `MODE` 並在相應模組新增邏輯
2. **自訂視覺化** - 修改 `data_export.py` 中的圖表生成函數
3. **調整濾波參數** - 在 `config.py` 微調追蹤參數
4. **新增函數產生器模式** - 在 `config.py` 的 `FG_MODE_CONFIGS` 新增模式

## 📄 授權

本專案採用 MIT 授權。

## 🤝 貢獻

歡迎提交 Issue 和 Pull Request！

## 📮 聯絡方式

如有問題或建議，請透過 Issue 聯繫我們。

---

**版本**: 2.0.0 (模組化重構版)  
**最後更新**: 2025-12-07
