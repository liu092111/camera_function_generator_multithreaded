# 相機鏡頭畸變校正工具

使用棋盤格圖像進行相機標定，校正廣角鏡頭的畸變（distortion）。

## 功能特點

- 自動檢測棋盤格尺寸
- 多種圖像預處理方法，提高棋盤格檢測成功率
- 批量校正圖像
- 生成校正前後對比圖
- 保存標定參數供後續使用

## 環境需求

- Python 3.6+
- OpenCV (`opencv-python`)
- NumPy

### 安裝依賴

```bash
pip install opencv-python numpy
```

## 使用方法

### 1. 準備棋盤格圖像

將拍攝的棋盤格校正圖像放入 `chest image/` 資料夾中。建議：

- 拍攝 15-25 張不同角度的棋盤格照片
- 確保棋盤格完整出現在圖像中
- 包含不同的角度和距離
- 支援 PNG、JPG、JPEG 格式

### 2. 執行校正程式

```bash
python camera_calibration_final.py
```

### 3. 查看輸出

執行完成後，會生成以下輸出：

| 目錄/文件 | 說明 |
|-----------|------|
| `corner_detection/` | 角點檢測可視化結果 |
| `undistorted/` | 校正後的圖像 |
| `comparison/comparison.png` | 校正前後對比圖 |
| `calibration_data.npz` | 標定參數文件 |

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

### 使用程式內建函數

```python
from camera_calibration_final import load_calibration, undistort_image
import cv2

# 載入標定參數
mtx, dist = load_calibration("calibration_data.npz")

# 校正單張圖像
img = cv2.imread("your_image.png")
corrected = undistort_image(img, mtx, dist, crop=True)
cv2.imwrite("corrected.png", corrected)
```

## 標定參數說明

### 相機內參矩陣 (mtx)

```
[[fx,  0, cx],
 [ 0, fy, cy],
 [ 0,  0,  1]]
```

- `fx`, `fy`: 焦距（以像素為單位）
- `cx`, `cy`: 主點座標（圖像中心）

### 畸變係數 (dist)

```
[k1, k2, p1, p2, k3]
```

- `k1`, `k2`, `k3`: 徑向畸變係數（radial distortion）
- `p1`, `p2`: 切向畸變係數（tangential distortion）

## 棋盤格要求

本程式預設支援以下棋盤格尺寸（會自動檢測）：

- 11×8 方格（10×7 內角點）
- 10×7 方格（9×6 內角點）
- 9×6 方格（8×5 內角點）
- 其他常見尺寸

**注意**：內角點數量 = 方格數 - 1

## 自訂棋盤格尺寸

如果使用非標準尺寸的棋盤格，可以修改程式：

```python
# 直接指定棋盤格尺寸（內角點數量）
mtx, dist, checkerboard_size, error = calibrate_camera(
    image_folder="chest image",
    checkerboard_size=(10, 7)  # 內角點數量 (列, 行)
)
```

## 評估標定品質

- **重投影誤差（Reprojection Error）**：衡量標定精度的指標
  - < 0.5: 優秀
  - 0.5 - 1.0: 良好
  - > 1.0: 可能需要重新拍攝或調整參數

## 疑難排解

### 棋盤格檢測失敗

1. 確保棋盤格完整出現在圖像中
2. 避免棋盤格被部分遮擋
3. 確保光線充足且均勻
4. 檢查棋盤格尺寸是否正確

### 校正效果不佳

1. 增加更多不同角度的棋盤格圖像
2. 確保圖像涵蓋視野的各個區域
3. 檢查重投影誤差是否過高

## 文件結構

```
calibration/
├── camera_calibration_final.py  # 主程式
├── README.md                     # 說明文件
├── chest image/                  # 棋盤格圖像（輸入）
│   ├── 1.png
│   ├── 2.png
│   └── ...
├── corner_detection/             # 角點檢測結果（輸出）
├── undistorted/                  # 校正後圖像（輸出）
├── comparison/                   # 對比圖（輸出）
└── calibration_data.npz          # 標定參數（輸出）
```

## License

MIT License
