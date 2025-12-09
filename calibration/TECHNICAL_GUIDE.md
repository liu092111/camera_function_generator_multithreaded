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

### 2.3 外參矩陣 (Extrinsic Matrix)

外參矩陣描述相機在世界座標系中的位置和方向：

- **R (3×3)**: 旋轉矩陣
- **t (3×1)**: 平移向量

### 2.4 畸變模型

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
│    - 輸出: 內參矩陣、畸變係數、外參                          │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│ 4. 畸變校正 (undistort)                                      │
│    - 使用標定參數反向映射                                    │
│    - 生成無畸變圖像                                          │
└─────────────────────────────────────────────────────────────┘
```

### 3.3 Zhang 標定法原理

Zhang 標定法是目前最廣泛使用的相機標定方法：

1. **單應性矩陣估計**：計算每張圖像的單應性矩陣 H
2. **內參求解**：從多個 H 矩陣聯立方程求解內參
3. **非線性優化**：使用 Levenberg-Marquardt 算法最小化重投影誤差

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
| < 0.3 像素 | 優秀 |
| 0.3 - 0.5 像素 | 很好 |
| 0.5 - 1.0 像素 | 良好 |
| 1.0 - 2.0 像素 | 可接受 |
| > 2.0 像素 | 需要重新標定 |

**您的標定結果**：0.5456 像素 → **良好**

### 4.2 視覺驗證

#### 角點檢測驗證
查看 `corner_detection/` 資料夾中的圖像：
- 角點是否準確落在棋盤格交叉點上
- 角點連線是否形成規則的網格

#### 校正效果驗證
查看 `comparison/comparison.png`：
- 直線是否變直
- 邊緣畸變是否消除

### 4.3 程式化驗證

```python
import cv2
import numpy as np

# 載入標定參數
data = np.load("calibration_data.npz")
mtx = data['mtx']
dist = data['dist']

# 計算每張圖像的重投影誤差
total_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
    total_error += error

mean_error = total_error / len(objpoints)
print(f"平均重投影誤差: {mean_error}")
```

---

## 5. 輸出參數詳解

### 5.1 calibration_data.npz 內容

```python
import numpy as np
data = np.load("calibration_data.npz")
print(data.files)  # ['mtx', 'dist', 'checkerboard_size', 'reprojection_error']
```

### 5.2 內參矩陣 (mtx)

您的標定結果：
```
mtx = [[1236.59    0.00  355.46]
       [   0.00  944.25  189.43]
       [   0.00    0.00    1.00]]
```

| 參數 | 值 | 意義 |
|------|-----|------|
| fx | 1236.59 | X方向焦距（像素）|
| fy | 944.25 | Y方向焦距（像素）|
| cx | 355.46 | 主點X座標 |
| cy | 189.43 | 主點Y座標 |

**注意**：fx ≠ fy 表示像素不是正方形，這在某些相機中很常見。

### 5.3 畸變係數 (dist)

您的標定結果：
```
dist = [k1, k2, p1, p2, k3]
     = [-0.373, -0.900, -0.004, -0.001, 12.18]
```

| 係數 | 值 | 類型 | 說明 |
|------|-----|------|------|
| k1 | -0.373 | 徑向 | 主要畸變項（負值=桶形畸變）|
| k2 | -0.900 | 徑向 | 二次畸變項 |
| k3 | 12.18 | 徑向 | 高階畸變項 |
| p1 | -0.004 | 切向 | 接近0表示鏡頭與感測器平行良好 |
| p2 | -0.001 | 切向 | 接近0表示鏡頭與感測器平行良好 |

**畸變分析**：
- k1 為負 → 您的鏡頭呈現**桶形畸變**（典型的廣角鏡頭特徵）
- p1, p2 接近 0 → 切向畸變很小，鏡頭安裝良好

### 5.4 其他參數

| 參數 | 說明 |
|------|------|
| checkerboard_size | 棋盤格尺寸 (10, 7) |
| reprojection_error | 重投影誤差 0.5456 |

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

# 方法1: 直接校正
undistorted = cv2.undistort(img, mtx, dist, None, mtx)

# 方法2: 使用最優相機矩陣（保留更多像素）
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
undistorted = cv2.undistort(img, mtx, dist, None, newcameramtx)

# 裁剪有效區域
x, y, w, h = roi
undistorted = undistorted[y:y+h, x:x+w]
```

### 6.2 使用映射表（效率更高）

```python
# 預先計算映射表
mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w, h), cv2.CV_32FC1)

# 使用映射表校正（比 undistort 更快）
undistorted = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)
```

### 6.3 即時視訊校正

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

# 預計算映射表
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

### 6.4 3D 重建與測量

如果需要進行 3D 測量，您可以使用標定參數進行：

```python
# 已知物體上兩點的像素座標
point1 = np.array([100, 200])
point2 = np.array([300, 200])

# 轉換為歸一化座標
fx, fy = mtx[0, 0], mtx[1, 1]
cx, cy = mtx[0, 2], mtx[1, 2]

p1_norm = ((point1[0] - cx) / fx, (point1[1] - cy) / fy)
p2_norm = ((point2[0] - cx) / fx, (point2[1] - cy) / fy)

# 如果知道深度 Z，可以計算實際距離
Z = 1.0  # 假設深度為 1 米
X1, Y1 = p1_norm[0] * Z, p1_norm[1] * Z
X2, Y2 = p2_norm[0] * Z, p2_norm[1] * Z
distance = np.sqrt((X2-X1)**2 + (Y2-Y1)**2)
```

---

## 7. 總結：您可以帶走的參數

### 必要參數

| 參數 | 用途 | 如何取得 |
|------|------|---------|
| **mtx** (內參矩陣) | 校正畸變、3D重建 | `data['mtx']` |
| **dist** (畸變係數) | 校正畸變 | `data['dist']` |

### 使用範例

```python
# 最小化使用範例
import cv2
import numpy as np

data = np.load("calibration_data.npz")
mtx, dist = data['mtx'], data['dist']

img = cv2.imread("your_image.png")
corrected = cv2.undistort(img, mtx, dist)
cv2.imwrite("corrected.png", corrected)
```

### 保存為其他格式

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
