"""
網格校正程式 - TPS 薄板樣條變換
自動偵測網格交點並校正為水平垂直的正方格
"""
import cv2
import numpy as np
from scipy.interpolate import Rbf
import os

# =========================
# 0) 讀圖 & ROI 設定
# =========================
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
IMG_PATH = os.path.join(SCRIPT_DIR, "original.png")
img = cv2.imread(IMG_PATH)
if img is None:
    raise FileNotFoundError(f"Cannot read {IMG_PATH}")

ROI = None  # 可設定 ROI=(x, y, w, h)，None 表示整張影像

if ROI is None:
    x0, y0, w0, h0 = 0, 0, img.shape[1], img.shape[0]
else:
    x0, y0, w0, h0 = ROI

roi = img[y0:y0+h0, x0:x0+w0].copy()
OUT_W, OUT_H = roi.shape[1], roi.shape[0]

# 保存原始圖像副本（用於最終輸出比較）
original_roi = roi.copy()

# =========================
# 1) 交點偵測 - 亞像素精度
# =========================
gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
gray_blur = cv2.GaussianBlur(gray, (3, 3), 0)

bin_img = cv2.adaptiveThreshold(
    gray_blur, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 31, -5
)

# 形態學分離水平線/垂直線
h_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (40, 1))
v_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 40))

h_lines = cv2.morphologyEx(bin_img, cv2.MORPH_OPEN, h_kernel, iterations=1)
v_lines = cv2.morphologyEx(bin_img, cv2.MORPH_OPEN, v_kernel, iterations=1)

# 增加線寬以獲得更好的交點
h_lines = cv2.dilate(h_lines, cv2.getStructuringElement(cv2.MORPH_RECT, (1, 3)), 1)
v_lines = cv2.dilate(v_lines, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 1)), 1)

intersections = cv2.bitwise_and(h_lines, v_lines)

# 找交點連通元件
num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(intersections)

pts = []
for i in range(1, num_labels):
    area = stats[i, cv2.CC_STAT_AREA]
    if 3 <= area <= 800:
        cx, cy = centroids[i]
        pts.append([cx, cy])

pts = np.array(pts, dtype=np.float32)

# 亞像素精度優化
if len(pts) > 0:
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
    pts = cv2.cornerSubPix(gray, pts, (5, 5), (-1, -1), criteria)

print(f"[INFO] 偵測到 {len(pts)} 個交點（亞像素精度）")

if len(pts) < 30:
    print(f"[WARN] 交點太少：{len(pts)}")

# =========================
# 2) 網格排序
# =========================
def cluster_1d(vals, tol):
    """把 1D 數值分群：相近的歸一群"""
    vals = np.array(vals)
    order = np.argsort(vals)
    groups = []
    cur = [order[0]]
    for idx in order[1:]:
        if abs(vals[idx] - vals[cur[-1]]) <= tol:
            cur.append(idx)
        else:
            groups.append(cur)
            cur = [idx]
    groups.append(cur)
    return groups

# 估計格距
ys = np.sort(pts[:, 1])
dy = np.diff(ys)
dy = dy[(dy > 2) & (dy < 60)]
step_y = np.median(dy) if len(dy) > 0 else 20

xs = np.sort(pts[:, 0])
dx = np.diff(xs)
dx = dx[(dx > 2) & (dx < 60)]
step_x = np.median(dx) if len(dx) > 0 else 20

print(f"[INFO] 估計格距: step_x={step_x:.2f}, step_y={step_y:.2f}")

# Y 方向分群
tol_y = max(5.0, step_y * 0.4)
y_groups = cluster_1d(pts[:, 1], tol=tol_y)

# 建立列
rows = []
for g in y_groups:
    row_pts = pts[g]
    row_pts = row_pts[np.argsort(row_pts[:, 0])]
    rows.append(row_pts)

rows = sorted(rows, key=lambda r: np.mean(r[:, 1]))

# 統計各列的點數
row_lens = np.array([len(r) for r in rows])
target_cols = int(np.median(row_lens))

# 過濾太短的列
rows2 = [r for r in rows if len(r) >= max(6, int(target_cols * 0.8))]
if len(rows2) < 5:
    rows2 = [r for r in rows if len(r) >= max(4, int(target_cols * 0.6))]
target_cols = int(np.median([len(r) for r in rows2]))

# 每列裁切成同樣欄數
grid = []
for r in rows2:
    if len(r) > target_cols:
        start = (len(r) - target_cols) // 2
        r = r[start:start + target_cols]
    elif len(r) < target_cols:
        continue
    grid.append(r)

grid = np.stack(grid, axis=0)
R, C = grid.shape[0], grid.shape[1]
print(f"[INFO] grid size: R={R}, C={C}, total points={R*C}")

# =========================
# 3) 建立「理想正方格」目標點
# =========================
step = float(np.mean([step_x, step_y]))

dest_w = OUT_W
dest_h = OUT_H

grid_w = (C - 1) * step
grid_h = (R - 1) * step
off_x = (dest_w - grid_w) / 2.0
off_y = (dest_h - grid_h) / 2.0

dest = np.zeros_like(grid, dtype=np.float32)
for i in range(R):
    for j in range(C):
        dest[i, j, 0] = off_x + j * step
        dest[i, j, 1] = off_y + i * step

src_pts = grid.reshape(-1, 2)
dst_pts = dest.reshape(-1, 2)

# =========================
# 4) 增加邊界錨點
# =========================
margin = 10
edge_pts_src = []
edge_pts_dst = []

corners = [
    (margin, margin),
    (OUT_W - margin, margin),
    (margin, OUT_H - margin),
    (OUT_W - margin, OUT_H - margin)
]
for cx, cy in corners:
    edge_pts_dst.append([cx, cy])
    edge_pts_src.append([cx, cy])

num_edge = 5
for i in range(num_edge):
    t = (i + 1) / (num_edge + 1)
    edge_pts_dst.append([margin + t * (OUT_W - 2*margin), margin])
    edge_pts_src.append([margin + t * (OUT_W - 2*margin), margin])
    edge_pts_dst.append([margin + t * (OUT_W - 2*margin), OUT_H - margin])
    edge_pts_src.append([margin + t * (OUT_W - 2*margin), OUT_H - margin])
    edge_pts_dst.append([margin, margin + t * (OUT_H - 2*margin)])
    edge_pts_src.append([margin, margin + t * (OUT_H - 2*margin)])
    edge_pts_dst.append([OUT_W - margin, margin + t * (OUT_H - 2*margin)])
    edge_pts_src.append([OUT_W - margin, margin + t * (OUT_H - 2*margin)])

edge_pts_src = np.array(edge_pts_src, dtype=np.float32)
edge_pts_dst = np.array(edge_pts_dst, dtype=np.float32)

all_src = np.vstack([src_pts, edge_pts_src])
all_dst = np.vstack([dst_pts, edge_pts_dst])

print(f"[INFO] 總控制點數: {len(all_src)}")

# =========================
# 5) TPS 建立逆映射
# =========================
X = all_dst[:, 0]
Y = all_dst[:, 1]
x = all_src[:, 0]
y = all_src[:, 1]

# TPS smooth 參數：較小的值 = 更精確但可能過擬合，較大的值 = 更平滑但可能欠擬合
# 對於精確的網格校正，使用較小的 smooth 值
SMOOTH = 0.1
print(f"[INFO] 使用 TPS smooth={SMOOTH}...")

rbf_x = Rbf(X, Y, x, function="thin_plate", smooth=SMOOTH)
rbf_y = Rbf(X, Y, y, function="thin_plate", smooth=SMOOTH)

grid_X, grid_Y = np.meshgrid(np.arange(dest_w, dtype=np.float32),
                             np.arange(dest_h, dtype=np.float32))
map_x = rbf_x(grid_X, grid_Y).astype(np.float32)
map_y = rbf_y(grid_X, grid_Y).astype(np.float32)

rectified = cv2.remap(roi, map_x, map_y, interpolation=cv2.INTER_LINEAR,
                      borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))

# =========================
# 6) 計算有效區域並裁切
# =========================
dbg = roi.copy()
for (px, py) in src_pts.astype(int):
    cv2.circle(dbg, (px, py), 2, (0, 255, 0), -1)

cv2.imwrite(os.path.join(SCRIPT_DIR, "debug_points.png"), dbg)

# 計算有效區域
grid_min_x = np.min(dst_pts[:, 0])
grid_max_x = np.max(dst_pts[:, 0])
grid_min_y = np.min(dst_pts[:, 1])
grid_max_y = np.max(dst_pts[:, 1])

crop_margin = 5
valid_x0 = int(np.ceil(grid_min_x)) + crop_margin
valid_y0 = int(np.ceil(grid_min_y)) + crop_margin
valid_x1 = int(np.floor(grid_max_x)) - crop_margin
valid_y1 = int(np.floor(grid_max_y)) - crop_margin

valid_w = valid_x1 - valid_x0
valid_h = valid_y1 - valid_y0

print(f"[INFO] 有效區域: ({valid_x0}, {valid_y0}), size={valid_w}x{valid_h}")

# 裁切
rectified_cropped = rectified[valid_y0:valid_y1, valid_x0:valid_x1].copy()
map_x_cropped = map_x[valid_y0:valid_y1, valid_x0:valid_x1].copy()
map_y_cropped = map_y[valid_y0:valid_y1, valid_x0:valid_x1].copy()

# 保存結果
cv2.imwrite(os.path.join(SCRIPT_DIR, "rectified.png"), rectified_cropped)

np.savez(
    os.path.join(SCRIPT_DIR, "tps_rectification_map.npz"),
    map_x=map_x,
    map_y=map_y,
    map_x_cropped=map_x_cropped,
    map_y_cropped=map_y_cropped,
    roi=np.array([x0, y0, w0, h0]),
    out_size=np.array([OUT_W, OUT_H]),
    valid_region=np.array([valid_x0, valid_y0, valid_w, valid_h]),
    crop_margin=crop_margin
)

# =========================
# 7) 生成驗證比較圖
# =========================
# 在原圖上繪製偵測到的網格點
original_with_pts = original_roi.copy()
for (px, py) in src_pts.astype(int):
    cv2.circle(original_with_pts, (px, py), 3, (0, 255, 0), -1)

# 在校正圖上繪製目標網格點位置
rectified_with_grid = rectified_cropped.copy()
# 計算裁切後的目標點座標
cropped_dst_pts = dst_pts.copy()
cropped_dst_pts[:, 0] -= valid_x0
cropped_dst_pts[:, 1] -= valid_y0

for (px, py) in cropped_dst_pts.astype(int):
    if 0 <= px < valid_w and 0 <= py < valid_h:
        cv2.circle(rectified_with_grid, (px, py), 3, (0, 0, 255), -1)

# 保存帶標記的圖像
cv2.imwrite(os.path.join(SCRIPT_DIR, "original_with_points.png"), original_with_pts)
cv2.imwrite(os.path.join(SCRIPT_DIR, "rectified_with_grid.png"), rectified_with_grid)

print("[OK] Saved debug_points.png")
print("[OK] Saved rectified.png")
print("[OK] Saved original_with_points.png")
print("[OK] Saved rectified_with_grid.png")
print("[OK] Saved tps_rectification_map.npz")

# =========================
# 8) 顯示校正統計
# =========================
print("\n===== 校正統計 =====")
print(f"原始影像尺寸: {OUT_W} x {OUT_H}")
print(f"校正後影像尺寸: {valid_w} x {valid_h}")
print(f"網格尺寸: {R} 行 x {C} 列")
print(f"平均格距: {step:.2f} px")
