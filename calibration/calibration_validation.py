"""
校正驗證程式 - 計算校正後影像的誤差指標
"""
import cv2
import numpy as np
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
IMG_PATH = os.path.join(SCRIPT_DIR, "rectified.png")

img = cv2.imread(IMG_PATH)
if img is None:
    raise FileNotFoundError(IMG_PATH)

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# 二值化
bin_img = cv2.adaptiveThreshold(
    gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 31, -5
)

# 抽水平線 / 垂直線
h_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (35, 1))
v_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 35))

h_lines = cv2.morphologyEx(bin_img, cv2.MORPH_OPEN, h_kernel, iterations=1)
v_lines = cv2.morphologyEx(bin_img, cv2.MORPH_OPEN, v_kernel, iterations=1)

intersections = cv2.bitwise_and(h_lines, v_lines)
intersections = cv2.dilate(intersections, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)), 1)

num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(intersections)

pts = []
for i in range(1, num_labels):
    area = stats[i, cv2.CC_STAT_AREA]
    if 5 <= area <= 500:
        pts.append(centroids[i])
pts = np.array(pts, dtype=np.float32)

if len(pts) < 50:
    raise RuntimeError(f"Too few intersections detected: {len(pts)}")

# 把點排序成網格
def cluster_1d(vals, tol):
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

ys = np.sort(pts[:, 1])
dy = np.diff(ys)
dy = dy[(dy > 1) & (dy < 80)]
step_y = np.median(dy)
tol_y = max(6.0, step_y * 0.45)

y_groups = cluster_1d(pts[:, 1], tol_y)

rows = []
for g in y_groups:
    r = pts[g]
    r = r[np.argsort(r[:, 0])]
    rows.append(r)
rows = sorted(rows, key=lambda r: np.mean(r[:, 1]))

row_lens = np.array([len(r) for r in rows])
C = int(np.median(row_lens))
rows = [r for r in rows if len(r) >= max(6, int(C * 0.7))]
C = int(np.median([len(r) for r in rows]))

grid = []
for r in rows:
    if len(r) > C:
        s = (len(r) - C) // 2
        r = r[s:s+C]
    grid.append(r)

grid = np.stack(grid, axis=0)
R, C = grid.shape[0], grid.shape[1]

# 指標 1：格距一致性
dx = grid[:, 1:, 0] - grid[:, :-1, 0]
dy = grid[1:, :, 1] - grid[:-1, :, 1]

mean_dx = float(np.mean(dx))
mean_dy = float(np.mean(dy))
cvx = float(np.std(dx) / mean_dx)
cvy = float(np.std(dy) / mean_dy)

# 指標 2：直線度（RMS）
def line_rms(points):
    x = points[:, 0]
    y = points[:, 1]
    A = np.vstack([x, np.ones_like(x)]).T
    a, b = np.linalg.lstsq(A, y, rcond=None)[0]
    y_fit = a * x + b
    resid = y - y_fit
    return float(np.sqrt(np.mean(resid**2)))

rms_h = np.mean([line_rms(grid[i, :, :]) for i in range(R)])

def vline_rms(points):
    x = points[:, 0]
    y = points[:, 1]
    A = np.vstack([y, np.ones_like(y)]).T
    c, d = np.linalg.lstsq(A, x, rcond=None)[0]
    x_fit = c * y + d
    resid = x - x_fit
    return float(np.sqrt(np.mean(resid**2)))

rms_v = np.mean([vline_rms(grid[:, j, :]) for j in range(C)])

# 指標 3：中心 vs 邊緣尺度差
r0 = int(R * 0.35); r1 = int(R * 0.65)
c0 = int(C * 0.35); c1 = int(C * 0.65)

dx_center = dx[r0:r1, c0:c1-1]
dy_center = dy[r0:r1-1, c0:c1]

k = 2
dx_edge = np.concatenate([
    dx[:k, :].reshape(-1),
    dx[-k:, :].reshape(-1),
    dx[:, :k].reshape(-1),
    dx[:, -k:].reshape(-1),
])
dy_edge = np.concatenate([
    dy[:k, :].reshape(-1),
    dy[-k:, :].reshape(-1),
    dy[:, :k].reshape(-1),
    dy[:, -k:].reshape(-1),
])

mean_dx_center = float(np.mean(dx_center))
mean_dx_edge   = float(np.mean(dx_edge))
mean_dy_center = float(np.mean(dy_center))
mean_dy_edge   = float(np.mean(dy_edge))

escale_x = abs(mean_dx_edge - mean_dx_center) / mean_dx_center
escale_y = abs(mean_dy_edge - mean_dy_center) / mean_dy_center

# 總誤差
dbar = float((mean_dx + mean_dy) / 2.0)
E_total = 100.0 * np.sqrt(
    cvx**2 + cvy**2 + (rms_h / dbar)**2 + (rms_v / dbar)**2
)

print("===== Validation Results =====")
print(f"Grid size: R={R}, C={C}")
print(f"Mean spacing dx={mean_dx:.3f}px, dy={mean_dy:.3f}px")
print(f"Spacing consistency: CVx={cvx*100:.2f}%, CVy={cvy*100:.2f}%")
print(f"Line straightness: RMS_h={rms_h:.3f}px, RMS_v={rms_v:.3f}px")
print(f"Center-Edge scale diff: Ex={escale_x*100:.2f}%, Ey={escale_y*100:.2f}%")
print(f"==> Total error E_total = {E_total:.2f}%")
