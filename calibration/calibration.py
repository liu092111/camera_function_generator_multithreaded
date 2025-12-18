import cv2
import numpy as np
from scipy.interpolate import Rbf

# =========================
# 0) 讀圖 & ROI 設定
# =========================
IMG_PATH = "original.png"  # <-- 改成你的檔名
img = cv2.imread(IMG_PATH)
if img is None:
    raise FileNotFoundError(f"Cannot read {IMG_PATH}")

# ROI: (x, y, w, h) 你可以先用整張，或只切你實驗區
# 建議你先用「實驗會用的那塊」：效果更穩、更快
ROI = None  # 例如 ROI=(60, 40, 1050, 760)

if ROI is None:
    x0, y0, w0, h0 = 0, 0, img.shape[1], img.shape[0]
else:
    x0, y0, w0, h0 = ROI

roi = img[y0:y0+h0, x0:x0+w0].copy()

# 你希望輸出的校正ROI大小（可等於ROI大小）
OUT_W, OUT_H = roi.shape[1], roi.shape[0]

# =========================
# 1) 偵測網格交點（控制點）
#    策略：二值化 -> 取交點（水平線與垂直線的交會）
# =========================
gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

# 對你這種黑底白線網格，通常用 adaptive threshold + 反相會比較好
bin_img = cv2.adaptiveThreshold(
    gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 31, -5
)

# 讓線變白、背景變黑（依你的影像可能要反相；若效果不對就切換）
# bin_img = 255 - bin_img

# 用形態學分離水平線/垂直線
h_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (35, 1))  # 可調
v_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 35))  # 可調

h_lines = cv2.morphologyEx(bin_img, cv2.MORPH_OPEN, h_kernel, iterations=1)
v_lines = cv2.morphologyEx(bin_img, cv2.MORPH_OPEN, v_kernel, iterations=1)

# 交點 = 水平線 & 垂直線 同時為線的位置
intersections = cv2.bitwise_and(h_lines, v_lines)

# 把交點變得更像「點」
dot_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
intersections = cv2.dilate(intersections, dot_kernel, iterations=1)

# 找交點的連通元件中心
num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(intersections)

pts = []
for i in range(1, num_labels):
    area = stats[i, cv2.CC_STAT_AREA]
    # 面積過小多半是雜訊，過大可能是模糊連成塊（可調）
    if 5 <= area <= 500:
        cx, cy = centroids[i]
        pts.append([cx, cy])

pts = np.array(pts, dtype=np.float32)

if len(pts) < 30:
    print(f"[WARN] 交點太少：{len(pts)}，你可能需要調 kernel 或反相 bin_img。")

# =========================
# 2) 把交點排序成「網格」(行列結構)
#    做法：先依 y 分群成多列，再每列依 x 排序
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

# 估計格距（用 y 差的中位數）
# 先粗略依 y 排序，算相鄰差的中位數當作 tol 的參考
ys = np.sort(pts[:, 1])
dy = np.diff(ys)
dy = dy[(dy > 1) & (dy < 80)]  # 篩掉極端值（可調）
if len(dy) == 0:
    raise RuntimeError("無法估計格距，請調整前處理/ROI。")
step_y = np.median(dy)

tol_y = max(6.0, step_y * 0.45)  # 可調：越大越容易合併成同一列
y_groups = cluster_1d(pts[:, 1], tol=tol_y)

# 每一列取 y 平均，列內依 x 排序
rows = []
for g in y_groups:
    row_pts = pts[g]
    row_pts = row_pts[np.argsort(row_pts[:, 0])]
    rows.append(row_pts)

# 列依 y 由上到下排序
rows = sorted(rows, key=lambda r: np.mean(r[:, 1]))

# 讓每一列取「共同的欄數」：用眾數/中位數（避免某些列缺點）
row_lens = np.array([len(r) for r in rows])
target_cols = int(np.median(row_lens))

# 過濾太短的列
rows2 = [r for r in rows if len(r) >= max(5, int(target_cols * 0.7))]
target_cols = int(np.median([len(r) for r in rows2]))

# 每列裁切成同樣欄數（取中間段，避免邊緣偵測不穩）
grid = []
for r in rows2:
    if len(r) > target_cols:
        start = (len(r) - target_cols) // 2
        r = r[start:start + target_cols]
    grid.append(r)

grid = np.stack(grid, axis=0)  # (R, C, 2)
R, C = grid.shape[0], grid.shape[1]
print(f"[INFO] grid size detected: R={R}, C={C}, points={R*C}")

if R < 6 or C < 6:
    print("[WARN] 網格列/欄太少，非線性校正可能不夠穩。建議放大ROI或調偵測參數。")

# =========================
# 3) 建立「理想正方格」目標點（dest）
#    你可以指定每格像素距離（用平均格距估）
# =========================
# 估計平均格距（x方向）
xs_sorted = np.sort(grid.reshape(-1, 2)[:, 0])
dx = np.diff(xs_sorted)
dx = dx[(dx > 1) & (dx < 80)]
step_x = np.median(dx) if len(dx) else step_y
step = float(np.mean([step_x, step_y]))

# 目標點：置中排版，保持格距一致
margin = 20
dest_w = OUT_W
dest_h = OUT_H

# 讓理想格網鋪在輸出ROI中（以 step 決定密度）
grid_w = (C - 1) * step
grid_h = (R - 1) * step
off_x = (dest_w - grid_w) / 2.0
off_y = (dest_h - grid_h) / 2.0

dest = np.zeros_like(grid, dtype=np.float32)
for i in range(R):
    for j in range(C):
        dest[i, j, 0] = off_x + j * step
        dest[i, j, 1] = off_y + i * step

src_pts = grid.reshape(-1, 2)         # 原圖交點 (x,y)
dst_pts = dest.reshape(-1, 2)         # 理想交點 (X,Y)

# =========================
# 4) TPS：建立「目標 -> 原圖」的逆映射
#    用 Rbf(thin_plate) 擬合 (X,Y)->x 與 (X,Y)->y
# =========================
X = dst_pts[:, 0]
Y = dst_pts[:, 1]
x = src_pts[:, 0]
y = src_pts[:, 1]

rbf_x = Rbf(X, Y, x, function="thin_plate")
rbf_y = Rbf(X, Y, y, function="thin_plate")

# 生成 remap 表
grid_X, grid_Y = np.meshgrid(np.arange(dest_w, dtype=np.float32),
                             np.arange(dest_h, dtype=np.float32))
map_x = rbf_x(grid_X, grid_Y).astype(np.float32)
map_y = rbf_y(grid_X, grid_Y).astype(np.float32)

# remap：把「理想影像」每個像素去原圖找對應位置取樣
rectified = cv2.remap(roi, map_x, map_y, interpolation=cv2.INTER_LINEAR,
                      borderMode=cv2.BORDER_CONSTANT, borderValue=(0,0,0))

# =========================
# 5) Debug輸出：看控制點落點是否合理
# =========================
dbg = roi.copy()
for (px, py) in src_pts.astype(int):
    cv2.circle(dbg, (px, py), 2, (0, 255, 0), -1)

cv2.imwrite("debug_points.png", dbg)
cv2.imwrite("rectified_tps.png", rectified)
print("[OK] Saved debug_points.png")
print("[OK] Saved rectified_tps.png")

np.savez(
    "tps_rectification_map.npz",
    map_x=map_x,
    map_y=map_y,
    roi=np.array([x0, y0, w0, h0]),
    out_size=np.array([OUT_W, OUT_H])
)

print("[OK] Saved tps_rectification_map.npz")
