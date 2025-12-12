# 相機標定技術指南

本文檔詳細說明相機標定的原理、驗證方法以及輸出參數的使用方式。

---

## 目錄

1. [相機畸變的原理](#1-相機畸變的原理)
2. [相機標定的數學原理](#2-相機標定的數學原理)
3. [標定過程說明](#3-標定過程說明)
4. [如何驗證標定品質](#4-如何驗證標定品質)
5. [輸出參數詳解](#5-輸出參數詳解)
6. [參數的實際應用](#6-參數的實際應用)
7. [優化功能說明](#7-優化功能說明)

---

## 1. 相機畸變的原理

### 1.1 什麼是鏡頭畸變？

鏡頭畸變是由於光學鏡頭的物理特性造成的圖像變形。真實世界中的直線在圖像中可能會呈現為彎曲的線條。

### 1.2 畸變的類型

#### 徑向畸變 (Radial Distortion)

徑向畸變是最常見的畸變類型，沿著鏡頭中心向外輻射方向發生。

```
桶形畸變 (Barrel Distortion)     枕形畸變 (Pincushion Distortion)
    ┌─────────┐                      ┌─────────┐
    │  ╭───╮  │                      │  ╭───╮  │
    │ ╭     ╮ │                      │ ╰     ╯ │
    │ │     │ │         →            │ │     │ │
    │ ╰     ╯ │                      │ ╭     ╮ │
    │  ╰───╯  │                      │  ╰───╯  │
    └─────────┘                      └─────────┘
    (廣角鏡頭常見)                   (長焦鏡頭常見)
```

數學表示：
```
x_distorted = x(1 + k₁r² + k₂r⁴ + k₃r⁶)
y_distorted = y(1 + k₁r² + k₂r⁴ + k₃r⁶)
```

其中 `r² = x² + y²`，`k₁, k₂, k₃` 是徑向畸變係數。

#### 切向畸變 (Tangential Distortion)

切向畸變是由於鏡頭與成像平面不完全平行造成的。

數學表示：
```
x_distorted = x + [2p₁xy + p₂(r² + 2x²)]
y_distorted = y + [p₁(r² + 2y²) + 2p₂xy]
```

其中 `p₁, p₂` 是切向畸變係數。

---

## 2. 相機標定的數學原理

### 2.1 針孔相機模型 (Pinhole Camera Model)

相機標定基於針孔相機模型，將3D世界座標映射到2D圖像座標：

```
世界座標 (X, Y, Z) → 相機座標 → 圖像座標 (u, v)
```

數學表示：

```
s [u]   [fx  0  cx] [r11 r12 r13 t1] [X]
  [v] = [ 0 fy  cy] [r21 r22 r23 t2] [Y]
  [1]   [ 0  0   1] [r31 r32 r33 t3] [Z]
                                     [1]

    ↑       ↑              ↑
    |       |              |
    s    內參矩陣      外參矩陣(R|t)
```

### 2.2 內參矩陣 (Intrinsic Matrix)

內參矩陣 K 描述相機的內部特性：

```
K = [fx   0  cx]
    [ 0  fy  cy]
    [ 0   0   1]
```

| 參數 | 意義 |
|------|------|
| fx, fy | 焦距（以像素為單位）|
| cx, cy | 主點座標（光軸與圖像平面的交點）|

### 2.3 畸變模型

完整的畸變校正公式：

```python
# 1. 歸一化座標
x' = (x - cx) / fx
y' = (y - cy) / fy

# 2. 計算徑向距離
r² = x'² + y'²

# 3. 計算畸變
x_distorted = x'(1 + k₁r² + k₂r⁴ + k₃r⁶) + 2p₁x'y' + p₂(r² + 2x'²)
y_distorted = y'(1 + k₁r² + k₂r⁴ + k₃r⁶) + p₁(r² + 2y'²) + 2p₂x'y'

# 4. 轉回像素座標
u = fx * x_distorted + cx
v = fy * y_distorted + cy
```

---

## 3. 標定過程說明

### 3.1 為什麼使用棋盤格？

1. **易於檢測**：黑白對比明顯，角點特徵清晰
2. **精確定位**：角點可以達到亞像素級精度
3. **已知幾何**：格子尺寸已知，可建立3D-2D對應關係

### 3.2 標定流程

```
┌─────────────────────────────────────────────────────────────┐
│ 1. 角點檢測 (Corner Detection)                               │
│    - 使用 findChessboardCorners() 找到內角點                 │
│    - 使用 cornerSubPix() 精確到亞像素位置                    │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│ 2. 建立對應關係                                              │
│    - 3D 物體點: (0,0,0), (1,0,0), (2,0,0)...               │
│    - 2D 圖像點: 檢測到的角點座標                             │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│ 3. 求解標定參數 (calibrateCamera)                            │
│    - 使用 Zhang 的標定方法                                   │
│    - 最小化重投影誤差                                        │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│ 4. 優化：剔除高誤差圖片（新功能）                             │
│    - 計算每張圖片的重投影誤差                                │
│    - 自動剔除誤差最高的圖片                                  │
│    - 重複直到達到目標誤差                                    │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│ 5. 畸變校正 (undistort)                                      │
│    - 使用標定參數反向映射                                    │
│    - 生成無畸變圖像                                          │
└─────────────────────────────────────────────────────────────┘
```

---

## 4. 如何驗證標定品質

### 4.1 重投影誤差 (Reprojection Error)

**定義**：將3D點使用標定參數投影回2D，與實際檢測到的2D點的距離。

```
誤差 = √[(u_projected - u_detected)² + (v_projected - v_detected)²]
```

**評估標準**：

| 重投影誤差 | 品質評估 |
|-----------|---------|
| < 0.3 像素 | ✓ 優秀 |
| 0.3 - 0.5 像素 | 很好 |
| 0.5 - 1.0 像素 | 良好 |
| > 1.0 像素 | 需要改善 |

**當前標定結果**：0.2943 像素 → **優秀！**

### 4.2 視覺驗證

#### 角點檢測驗證
查看 `corner_detection/` 資料夾中的圖像：
- 角點是否準確落在棋盤格交叉點上
- 每張圖片標註了誤差值或 "EXCLUDED" 標記

#### 校正效果驗證
查看 `comparison/comparison.png`：
- 直線是否變直
- 邊緣畸變是否消除

---

## 5. 輸出參數詳解

### 5.1 calibration_data.npz 內容

```python
import numpy as np
data = np.load("calibration_data.npz")
print(data.files)  # ['mtx', 'dist', 'checkerboard_size', 'reprojection_error']
```

### 5.2 當前標定結果

**內參矩陣 (mtx)**：
```
mtx = [[1227.33    0.00  312.57]
       [   0.00  935.81  160.28]
       [   0.00    0.00    1.00]]
```

| 參數 | 值 | 意義 |
|------|-----|------|
| fx | 1227.33 | X方向焦距（像素）|
| fy | 935.81 | Y方向焦距（像素）|
| cx | 312.57 | 主點X座標 |
| cy | 160.28 | 主點Y座標 |

**畸變係數 (dist)**：
```
dist = [k1, k2, p1, p2, k3]
     = [-0.524, 3.507, -0.0003, 0.0015, -32.89]
```

| 係數 | 值 | 類型 | 說明 |
|------|-----|------|------|
| k1 | -0.524 | 徑向 | 主要畸變項（負值=桶形畸變）|
| k2 | 3.507 | 徑向 | 二次畸變項 |
| k3 | -32.89 | 徑向 | 高階畸變項 |
| p1 | -0.0003 | 切向 | 接近0表示鏡頭安裝良好 |
| p2 | 0.0015 | 切向 | 接近0表示鏡頭安裝良好 |

**畸變分析**：
- k1 為負 → 您的鏡頭呈現**桶形畸變**（典型的廣角鏡頭特徵）
- p1, p2 接近 0 → 切向畸變很小，鏡頭安裝良好

---

## 6. 參數的實際應用

### 6.1 校正單張圖像

```python
import cv2
import numpy as np

# 載入參數
data = np.load("calibration_data.npz")
mtx, dist = data['mtx'], data['dist']

# 讀取圖像
img = cv2.imread("distorted_image.png")
h, w = img.shape[:2]

# 使用最優相機矩陣（保留更多像素）
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
undistorted = cv2.undistort(img, mtx, dist, None, newcameramtx)

# 裁剪有效區域
x, y, w, h = roi
undistorted = undistorted[y:y+h, x:x+w]
```

### 6.2 使用映射表（效率更高）

```python
# 預先計算映射表
mapx, mapy = cv2.initUndistortRectifyMap(
    mtx, dist, None, newcameramtx, (w, h), cv2.CV_32FC1
)

# 使用映射表校正（比 undistort 更快）
undistorted = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)
```

### 6.3 保存為其他格式

```python
# 保存為 JSON（便於其他語言讀取）
import json

data = np.load("calibration_data.npz")
calibration_dict = {
    'camera_matrix': data['mtx'].tolist(),
    'distortion_coefficients': data['dist'].tolist(),
    'reprojection_error': float(data['reprojection_error'])
}

with open('calibration.json', 'w') as f:
    json.dump(calibration_dict, f, indent=2)
```

```python
# 保存為 YAML（OpenCV 格式）
fs = cv2.FileStorage('calibration.yaml', cv2.FILE_STORAGE_WRITE)
fs.write('camera_matrix', data['mtx'])
fs.write('distortion_coefficients', data['dist'])
fs.release()
```

---

## 7. 優化功能說明

### 7.1 每張圖片誤差計算

程式會計算每張圖片的個別重投影誤差：

```python
def calculate_per_image_error(objpoints, imgpoints, rvecs, tvecs, mtx, dist):
    for i in range(len(objpoints)):
        # 投影 3D 點到 2D
        projected, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        # 計算誤差
        error = cv2.norm(imgpoints[i], projected, cv2.NORM_L2) / len(projected)
```

### 7.2 自動優化流程

```
初始狀態: 41 張圖片，誤差 0.8078
    ↓
剔除 29.png (誤差 0.2173)
剔除 37.png (誤差 0.1954)
...
    ↓
經過 24 次迭代
    ↓
最終狀態: 18 張圖片，誤差 0.2943 ✓
```

### 7.3 高誤差圖片的特徵

被剔除的圖片通常有以下問題：

| 問題 | 說明 |
|------|------|
| 模糊 | 角點定位不準確 |
| 角度過大 | 棋盤格過於傾斜 |
| 光線不均 | 部分角點難以檢測 |
| 棋盤格變形 | 紙張不夠平整 |

### 7.4 最佳拍攝建議

根據低誤差圖片的分析：

1. **保持適中角度**：棋盤格傾斜 15-45 度
2. **確保清晰**：避免手震或對焦不準
3. **均勻光線**：避免強烈反光或陰影
4. **棋盤格平整**：使用硬質板材固定

---

## 總結

### 標定成果

| 指標 | 值 |
|------|-----|
| 重投影誤差 | 0.2943（優秀）|
| 使用圖片數 | 18/41 |
| 棋盤格尺寸 | 10×7 內角點 |

### 必要參數

```python
# 最小化使用範例
import cv2
import numpy as np

data = np.load("calibration_data.npz")
mtx, dist = data['mtx'], data['dist']

img = cv2.imread("your_image.png")
corrected = cv2.undistort(img, mtx, dist)
cv2.imwrite("corrected.png", corrected)
