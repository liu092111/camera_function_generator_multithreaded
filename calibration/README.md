# 網格校正工具

使用 TPS（薄板樣條）變換校正網格影像，使彎曲的網格線變成水平垂直的正方格。

## 功能特點

- ✅ 自動偵測網格交點（亞像素精度）
- ✅ TPS 薄板樣條非線性變換
- ✅ 自動裁切有效區域（無黑邊）
- ✅ 校正品質驗證與誤差計算
- ✅ 可重複使用的校正映射表

## 檔案說明

| 檔案 | 說明 |
|------|------|
| `calibration.py` | 主程式 - 執行校正 |
| `calibration_validation.py` | 驗證程式 - 計算校正誤差 |
| `apply_rectification.py` | 工具函式 - 對新影像應用校正 |

## 快速開始

### 1. 準備校正影像

將網格影像命名為 `original.png` 放入 `calibration/` 資料夾。

### 2. 執行校正

```bash
python calibration/calibration.py
```

輸出：
- `rectified.png` - 校正後影像（已裁切）
- `debug_points.png` - 偵測到的控制點
- `tps_rectification_map.npz` - 校正映射表

### 3. 驗證校正品質

```bash
python calibration/calibration_validation.py
```

輸出範例：
```
===== Validation Results =====
Grid size: R=19, C=26
Mean spacing dx=20.940px, dy=20.907px
Spacing consistency: CVx=2.15%, CVy=1.47%
Line straightness: RMS_h=0.139px, RMS_v=0.200px
==> Total error E_total = 2.85%
```

## 校正品質指標

| 指標 | 說明 | 良好標準 |
|------|------|----------|
| CVx, CVy | 格距一致性（變異係數）| < 5% |
| RMS_h | 水平線直線度 | < 0.5 px |
| RMS_v | 垂直線直線度 | < 0.5 px |
| E_total | 總誤差 | < 5% |

## 在其他程式中使用

### 對新影像應用校正

```python
from calibration.apply_rectification import load_rectification_map, apply_rectification

# 載入校正映射表
rect_map = load_rectification_map("calibration/tps_rectification_map.npz")

# 對新影像應用校正
rectified = apply_rectification(your_image, rect_map, use_cropped=True)
```

### 參數說明

| 參數 | 說明 |
|------|------|
| `use_cropped=True` | 輸出裁切版（推薦，無黑邊）|
| `use_cropped=False` | 輸出完整版（含邊緣黑邊）|

## 校正映射表內容

`tps_rectification_map.npz` 包含：

| 欄位 | 說明 |
|------|------|
| `map_x`, `map_y` | 完整版 remap 表 |
| `map_x_cropped`, `map_y_cropped` | 裁切版 remap 表 |
| `roi` | 原始影像 ROI `[x0, y0, w, h]` |
| `valid_region` | 有效區域 `[x0, y0, w, h]` |
| `out_size` | 完整版輸出大小 |

## 技術原理

1. **交點偵測**：使用形態學運算分離水平/垂直線，找出交點
2. **亞像素精度**：使用 `cv2.cornerSubPix()` 提升定位精度
3. **TPS 變換**：薄板樣條插值建立平滑的非線性映射
4. **有效區域**：根據網格範圍自動計算並裁切

詳細技術說明請參考 [TECHNICAL_GUIDE.md](TECHNICAL_GUIDE.md)

## 疑難排解

### 交點偵測失敗
- 調整 `bin_img` 的閾值參數
- 嘗試反相（取消註解 `bin_img = 255 - bin_img`）
- 調整形態學 kernel 大小

### 校正後仍有畸變
- 增加網格密度
- 檢查原始影像是否清晰
- 調整 TPS 的 `SMOOTH` 參數

## License

MIT License
