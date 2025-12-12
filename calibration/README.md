# 相機鏡頭畸變校正工具

使用棋盤格圖像進行相機標定，校正廣角鏡頭的畸變（distortion）。支援自動優化功能，可剔除高誤差圖片以達到更好的校正精度。

## 功能特點

- ✅ 自動檢測棋盤格尺寸
- ✅ 多種圖像預處理方法，提高棋盤格檢測成功率
- ✅ **自動優化功能** - 自動剔除高誤差圖片
- ✅ **每張圖片誤差分析** - 識別品質不佳的校正圖片
- ✅ 批量校正圖像
- ✅ 生成校正前後對比圖
- ✅ 保存標定參數供後續使用

## 環境需求

- Python 3.6+
- OpenCV (`opencv-python`)
- NumPy

### 安裝依賴

```bash
pip install opencv-python numpy
```

## 快速開始

### 1. 準備棋盤格圖像

將拍攝的棋盤格校正圖像放入 `chest image/` 資料夾中。建議：

- 拍攝 20-40 張不同角度的棋盤格照片
- 確保棋盤格完整出現在圖像中
- 包含不同的角度和距離
- 支援 PNG、JPG、JPEG 格式

### 2. 執行校正程式

```bash
python camera_calibration_optimized.py
```

程式會提供三種模式：

| 模式 | 說明 |
|------|------|
| 1. 自動優化 | 自動剔除高誤差圖片，達到目標誤差 |
| 2. 互動式選擇 | 手動選擇要剔除的圖片 |
| 3. 使用所有圖片 | 不剔除任何圖片 |

### 3. 查看輸出

執行完成後，會生成以下輸出：

| 目錄/文件 | 說明 |
|-----------|------|
| `corner_detection/` | 角點檢測可視化結果（含誤差標註）|
| `undistorted/` | 校正後的圖像 |
| `comparison/comparison.png` | 校正前後對比圖 |
| `calibration_data.npz` | 標定參數文件 |

## 校正結果

### 當前標定精度

- **重投影誤差**: 0.2943（優秀！）
- **使用圖片數**: 18/41
- **棋盤格尺寸**: 10×7 內角點

### 保留的高品質圖片

| 排名 | 圖片 | 誤差 |
|------|------|------|
| 1 | 27.png | 0.0192 |
| 2 | 32.png | 0.0206 |
| 3 | 4.png | 0.0209 |
| 4 | 39.png | 0.0221 |
| 5 | 22.png | 0.0240 |
| ... | ... | ... |

### 剔除的高誤差圖片

以下圖片被自動剔除（誤差較高）：
- 5.png, 9.png, 10.png, 12.png, 14.png
- 16.png, 17.png, 19.png, 20.png, 21.png
- 24.png, 25.png, 26.png, 28.png, 29.png
- 30.png, 31.png, 33.png, 34.png, 35.png
- 36.png, 37.png, 40.png

## 評估標定品質

**重投影誤差（Reprojection Error）**評估標準：

| 重投影誤差 | 品質評估 |
|-----------|---------|
| < 0.3 像素 | ✓ 優秀 |
| 0.3 - 0.5 像素 | 很好 |
| 0.5 - 1.0 像素 | 良好 |
| > 1.0 像素 | 需要改善 |

## 在其他程式中使用標定參數

### 載入並使用標定參數

```python
import cv2
import numpy as np

# 載入標定參數
data = np.load("calibration_data.npz")
mtx = data['mtx']   # 相機內參矩陣
dist = data['dist']  # 畸變係數

# 讀取要校正的圖像
img = cv2.imread("your_image.png")
h, w = img.shape[:2]

# 獲取最優相機矩陣
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

# 校正畸變
undistorted = cv2.undistort(img, mtx, dist, None, newcameramtx)

# 裁剪黑邊（可選）
x, y, w, h = roi
undistorted = undistorted[y:y+h, x:x+w]

# 保存校正後的圖像
cv2.imwrite("corrected_image.png", undistorted)
```

### 即時視訊校正

```python
import cv2
import numpy as np

# 載入參數
data = np.load("calibration_data.npz")
mtx, dist = data['mtx'], data['dist']

# 開啟相機
cap = cv2.VideoCapture(0)
ret, frame = cap.read()
h, w = frame.shape[:2]

# 預計算映射表（效率更高）
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w, h), cv2.CV_32FC1)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    # 即時校正
    undistorted = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)
    
    cv2.imshow('Original', frame)
    cv2.imshow('Undistorted', undistorted)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```

## 標定參數說明

### 相機內參矩陣 (mtx)

```
[[fx,  0, cx],
 [ 0, fy, cy],
 [ 0,  0,  1]]
```

| 參數 | 意義 |
|------|------|
| fx, fy | 焦距（以像素為單位）|
| cx, cy | 主點座標（圖像中心）|

### 畸變係數 (dist)

```
[k1, k2, p1, p2, k3]
```

| 係數 | 類型 | 說明 |
|------|------|------|
| k1, k2, k3 | 徑向畸變 | 控制桶形/枕形畸變 |
| p1, p2 | 切向畸變 | 控制傾斜畸變 |

## 文件結構

```
calibration/
├── camera_calibration_optimized.py  # 主程式（含優化功能）
├── README.md                         # 說明文件
├── TECHNICAL_GUIDE.md               # 技術指南
├── chest image/                      # 棋盤格圖像（輸入）
│   ├── 1.png ~ 41.png
│   └── ...
├── corner_detection/                 # 角點檢測結果（輸出）
├── undistorted/                      # 校正後圖像（輸出）
├── comparison/                       # 對比圖（輸出）
└── calibration_data.npz              # 標定參數（輸出）
```

## 疑難排解

### 棋盤格檢測失敗

1. 確保棋盤格完整出現在圖像中
2. 避免棋盤格被部分遮擋
3. 確保光線充足且均勻
4. 檢查棋盤格是否平整

### 誤差無法降低

1. 檢查被剔除的高誤差圖片，可能是：
   - 圖片模糊
   - 棋盤格角度過於傾斜
   - 光線不均勻
   - 棋盤格不夠平整
2. 參考低誤差圖片的拍攝方式重新拍攝

## License

MIT License
