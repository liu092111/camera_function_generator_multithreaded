# -*- coding: utf-8 -*-
"""
對 experiment_backward 底下新錄的 camera_straight_fixed.mp4 重跑完整離線分析。
============================================================================
背景（2026-07-19 查證）：
  - camera_straight_fixed.mp4 是「即時追蹤器已烙上紅框(device 外框)+ 綠點(中心)」
    的渲染影片（with control 286x438、without control 264x424），非原始生影片。
  - 兩資料夾既有的 CSV / 四張圖，是先前用「舊 tracked.mp4（720x1280）座標系的 CSV」
    畫的，與這支新影片不對應。故此腳本從新影片逐幀重新追蹤，重建整套分析。

追蹤方式（純視覺，忠實於影片本身編碼的追蹤結果）：
  - 中心：綠點 HSV 遮罩質心（每幀 100% 偵得，已驗證）。
  - device 外框：紅框 HSV 遮罩 → minAreaRect（頂點 + 長軸朝向）。

比例尺（mm/px）決策——與 film_success/analyze_film_success.py 對「灰底網格」影片的
既定方法論一致：灰底 device 白≈灰格，格距與 device 量差達 ~40%，且要「框精確貼合
device」→ 一律用 device 長邊定標：mm/px = 9mm / (紅框長邊像素中位數)。
  同時印出網格週期定標值與既有 CSV 值做交叉驗證（誠實標註不確定性，不覆寫判斷）。

出圖對齊 `backward with control` 既有樣式：
  - <prefix>_position.png            Position (Trajectory)，有標題、主方向對齊 +Y
  - <prefix>_speed_orientation.png   左：Speed vs Time（標紅點最大值）；右：Orientation vs Time
  - <prefix>_trajectory_center_only.png  軌跡 + 8 個等間隔時間點的 device 外框（無標題）
  - <prefix>_8_timepoints_composite.png  相機疊圖：歷史淺藍虛線框 + 藍中心路徑 + 紅時間戳
  - <prefix>_pos_angle_speed_FROMFIXED.csv  逐幀數據

事實忠實原則：
  - 位置/速度/朝向由影片逐幀偵測結果重算；不沿用不對應的舊 CSV 數值。
  - 去雜訊沿用 paper_figures.py 同一套（Hampel → 移動平均）。
  - 首版輸出到各資料夾的子夾 fixed_analysis/，不覆蓋任何既有檔（可回退）。

用法：
  python3 analyze_fixed_video.py "experiment_backward/backward with control"
  python3 analyze_fixed_video.py "experiment_backward/without control backward"
"""

import os
import sys
import numpy as np
import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))

# --- HSV 遮罩（針對渲染影片的鮮綠中心點與鮮紅外框）---
G_LO = np.array([40, 80, 80], np.uint8)
G_HI = np.array([90, 255, 255], np.uint8)
R1_LO, R1_HI = np.array([0, 120, 80], np.uint8), np.array([10, 255, 255], np.uint8)
R2_LO, R2_HI = np.array([170, 120, 80], np.uint8), np.array([180, 255, 255], np.uint8)

DEVICE_LONG_MM = 9.0        # device 長邊實體尺寸（mm）→ 定標基準
DEVICE_SHORT_MM = 6.0

# --- 去雜訊（與 paper_figures.py 一致）---
POS_HAMPEL_WIN = 7
POS_MAD_K = 5.0
POS_FLOOR_PX = 6.0
POS_SMOOTH_WIN = 5
SMOOTH_WIN = 5

# --- 出圖 ---
BOX_COLOR = (0.5, 0.7, 1.0)
N_TP = 8
PLOT_PAD = 1.35
DPI = 600
# composite
TARGET_H = 300
LIGHT_BLUE = (255, 200, 150)   # BGR 淺藍
PATH_BLUE = (255, 0, 0)        # BGR 藍


# ----------------------------------------------------------------------
# 去雜訊工具
# ----------------------------------------------------------------------
def hampel_1d(v, win=POS_HAMPEL_WIN, k=POS_MAD_K, floor=POS_FLOOR_PX):
    v = np.asarray(v, float).copy()
    n = len(v); half = win // 2
    for i in range(n):
        if not np.isfinite(v[i]):
            continue
        w = v[max(0, i - half):min(n, i + half + 1)]
        w = w[np.isfinite(w)]
        if len(w) < 3:
            continue
        med = np.median(w)
        mad = 1.4826 * np.median(np.abs(w - med))
        if abs(v[i] - med) > max(k * mad, floor):
            v[i] = med
    return v


def moving_average(a, w=POS_SMOOTH_WIN):
    if w is None or w <= 1:
        return np.asarray(a, float)
    a = np.asarray(a, float)
    out = np.full_like(a, np.nan)
    half = w // 2
    for i in range(len(a)):
        win = a[max(0, i - half):min(len(a), i + half + 1)]
        win = win[np.isfinite(win)]
        if len(win):
            out[i] = win.mean()
    return out


def finite_diff(values, t):
    v = np.full(len(values), np.nan)
    for i in range(1, len(values)):
        if np.isfinite(values[i]) and np.isfinite(values[i - 1]) and t[i] > t[i - 1]:
            v[i] = (values[i] - values[i - 1]) / (t[i] - t[i - 1])
    return v


def principal_direction_xy(x, y):
    pts = np.vstack([x, y]).T
    pts = pts[~np.isnan(pts).any(axis=1)]
    if len(pts) < 2:
        return 0.0
    pts = pts - pts.mean(0, keepdims=True)
    eva, eve = np.linalg.eig(np.cov(pts.T))
    v = eve[:, int(np.argmax(eva))]
    return (np.pi / 2) - np.arctan2(v[1], v[0])


def rotate_xy(x, y, phi):
    c, s = np.cos(phi), np.sin(phi)
    return c * x - s * y, s * x + c * y


def grid_period_px(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    g = cv2.GaussianBlur(gray, (5, 5), 0).astype(float)
    gx = np.abs(cv2.Sobel(g, cv2.CV_64F, 1, 0, ksize=3)).mean(0)
    gy = np.abs(cv2.Sobel(g, cv2.CV_64F, 0, 1, ksize=3)).mean(1)

    def acf(sig):
        s = sig - sig.mean()
        ac = np.correlate(s, s, "full")[len(s) - 1:]
        if ac[0] == 0:
            return None
        ac /= ac[0]
        d = np.diff(ac)
        for i in range(8, 120):
            if d[i - 1] > 0 and d[i] <= 0 and ac[i] > 0.1:
                return float(i)
        return None
    cands = [p for p in (acf(gx), acf(gy)) if p]
    return float(np.median(cands)) if cands else None


# ----------------------------------------------------------------------
# 逐幀追蹤
# ----------------------------------------------------------------------
def track_video(video):
    """回傳 dict：frames, cx[], cy[](綠點中心 px), long_px[], box_pts[](紅框 4 頂點 px),
    long_axis_deg[](長軸朝向，度)。偵測失敗填 NaN / None。"""
    cap = cv2.VideoCapture(video)
    cx, cy, long_px, boxes, axis_deg = [], [], [], [], []
    while True:
        ok, img = cap.read()
        if not ok:
            break
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        gmask = cv2.inRange(hsv, G_LO, G_HI)
        rmask = cv2.bitwise_or(cv2.inRange(hsv, R1_LO, R1_HI),
                               cv2.inRange(hsv, R2_LO, R2_HI))
        # 綠點中心質心
        gc = cv2.findContours(gmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
        gxv = gyv = np.nan
        if gc:
            m = max(gc, key=cv2.contourArea)
            M = cv2.moments(m)
            if M["m00"] > 0:
                gxv, gyv = M["m10"] / M["m00"], M["m01"] / M["m00"]
        # 紅框 minAreaRect
        rc = cv2.findContours(rmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
        lp = np.nan; bp = None; adeg = np.nan
        if rc:
            m = max(rc, key=cv2.contourArea)
            if cv2.contourArea(m) >= 3:
                rect = cv2.minAreaRect(m)
                (rcx, rcy), (w, h), ang = rect
                lp = max(w, h)
                bp = cv2.boxPoints(rect)
                # 長軸方向角（度）：長邊向量方向
                if w >= h:
                    adeg = ang
                else:
                    adeg = ang + 90.0
        cx.append(gxv); cy.append(gyv); long_px.append(lp)
        boxes.append(bp); axis_deg.append(adeg)
    cap.release()
    fps = None
    cap = cv2.VideoCapture(video)
    fps = cap.get(cv2.CAP_PROP_FPS)
    cap.release()
    return dict(cx=np.array(cx), cy=np.array(cy), long_px=np.array(long_px),
                boxes=boxes, axis_deg=np.array(axis_deg), fps=fps,
                n=len(cx))


def unwrap_axis_deg(a):
    """長軸角有 180° 週期歧義：逐幀選離前值最近的等效角，unwrap 成連續。"""
    a = np.asarray(a, float).copy()
    out = np.full(len(a), np.nan)
    prev = None
    for i in range(len(a)):
        if not np.isfinite(a[i]):
            continue
        if prev is None:
            out[i] = a[i]; prev = a[i]; continue
        best = a[i]; bestd = abs(a[i] - prev)
        for k in (-180, 180, -360, 360):
            cand = a[i] + k
            if abs(cand - prev) < bestd:
                best = cand; bestd = abs(cand - prev)
        out[i] = best; prev = best
    return out


# ----------------------------------------------------------------------
# 主流程
# ----------------------------------------------------------------------
def analyze(folder_rel):
    folder = os.path.join(HERE, folder_rel)
    video = os.path.join(folder, "camera_straight_fixed.mp4")
    out_dir = os.path.join(folder, "fixed_analysis")
    os.makedirs(out_dir, exist_ok=True)
    prefix = "camera_straight"

    print(f"\n===== {folder_rel} =====")
    tr = track_video(video)
    n, fps = tr["n"], tr["fps"]
    print(f"  frames={n}  fps={fps:.3f}  綠點偵得={np.isfinite(tr['cx']).sum()}/{n}")

    # --- 比例尺：device 長邊定標，並交叉驗證 ---
    long_med = float(np.nanmedian(tr["long_px"]))
    mmpp_device = DEVICE_LONG_MM / long_med
    cap = cv2.VideoCapture(video); ok, f0 = cap.read(); cap.release()
    gp = grid_period_px(f0)
    print(f"  [比例尺] device 長邊中位數={long_med:.1f}px -> mm/px={mmpp_device:.4f}  (採用)")
    if gp:
        print(f"           交叉驗證: 網格週期={gp:.1f}px -> {5/gp:.4f}(格5mm) / {10/gp:.4f}(格10mm)")
    mmpp = mmpp_device

    # --- 時間軸：frame / fps ---
    t = np.arange(n) / fps

    # --- 位置去雜訊（像素域）→ mm（相對起點）→ 主方向對齊 +Y ---
    xpx = moving_average(hampel_1d(tr["cx"]))
    ypx = moving_average(hampel_1d(tr["cy"]))
    valid = np.isfinite(xpx) & np.isfinite(ypx)
    x0, y0 = xpx[valid][0], ypx[valid][0]
    x_mm = (xpx - x0) * mmpp
    y_mm = (ypx - y0) * mmpp

    # 忠實對齊影片座標：不做 PCA 主方向旋轉、不翻正（那會使 S 形軌跡左右鏡像，
    # 且藍框偏離影片中紅框原位）。直接用影片真實 mm 座標（x 向右、y 向下），
    # 出圖時以 invert_yaxis() 讓「影片往下走 = 圖上往下」，與 composite 完全一致。
    xr, yr = x_mm, y_mm

    # --- 速度（mm/s）---
    vx = moving_average(finite_diff(xr, t), SMOOTH_WIN)
    vy = moving_average(finite_diff(yr, t), SMOOTH_WIN)
    speed = np.sqrt(vx**2 + vy**2)

    # --- 朝向：長軸角 unwrap，相對首幀（度）。與座標翻轉無關，不受影響 ---
    axis_unw = unwrap_axis_deg(tr["axis_deg"])
    axis_unw = moving_average(hampel_1d(axis_unw, floor=2.0), SMOOTH_WIN)
    orient_offset = axis_unw - axis_unw[np.isfinite(axis_unw)][0]

    # --- 寫 CSV ---
    csv_path = os.path.join(out_dir, f"{prefix}_pos_angle_speed_FROMFIXED.csv")
    header = "frame,t_s,x_px,y_px,mm_per_px,x_mm,y_mm,orient_offset_deg,vx_mm_s,vy_mm_s,speed_mm_s"
    with open(csv_path, "w", encoding="utf-8") as fh:
        fh.write(header + "\n")
        for i in range(n):
            fh.write(f"{i},{t[i]:.5f},{xpx[i]:.3f},{ypx[i]:.3f},{mmpp:.6f},"
                     f"{xr[i]:.4f},{yr[i]:.4f},{orient_offset[i]:.3f},"
                     f"{vx[i]:.4f},{vy[i]:.4f},{speed[i]:.4f}\n")
    print(f"  [CSV] {os.path.basename(csv_path)}  "
          f"(dur={t[-1]:.2f}s, 位移={np.hypot(xr[valid][-1],yr[valid][-1]):.1f}mm, "
          f"v_max={np.nanmax(speed):.1f}mm/s)")

    # ================= 出圖 =================
    # (1) Position
    fig, ax = plt.subplots(figsize=(7, 6), constrained_layout=True)
    ax.plot(xr, yr, lw=2, color="#1f77b4", label="Trajectory")
    vp = np.vstack([xr, yr]).T; vp = vp[~np.isnan(vp).any(axis=1)]
    ax.scatter([vp[0, 0]], [vp[0, 1]], s=100, c="green", zorder=5, label="Start")
    ax.scatter([vp[-1, 0]], [vp[-1, 1]], s=100, c="red", zorder=5, label="End")
    _square(ax, xr, yr)
    ax.invert_yaxis()   # 影片 y 向下 → 圖上往下，與 composite 方向一致
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x (mm)", fontsize=22); ax.set_ylabel("y (mm)", fontsize=22)
    ax.set_title("Position (Trajectory)", fontsize=20)
    ax.tick_params(labelsize=14); ax.grid(False); ax.legend(loc="best", fontsize=13)
    _save(fig, out_dir, f"{prefix}_position.png")

    # (2) Speed + Orientation (雙 panel，對齊 with control 既有樣式)
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(14, 6), constrained_layout=True)
    a1.plot(t, speed, lw=2, color="#1f77b4", label="Speed (mm/s)")
    fin = np.isfinite(speed)
    if fin.any():
        i = np.where(fin)[0][int(np.nanargmax(speed[fin]))]
        a1.plot([t[i]], [speed[i]], "o", ms=8, color="red",
                label=f"Max: {speed[i]:.2f} mm/s @ {t[i]:.2f}s")
    a1.set_xlabel("Time (s)", fontsize=20); a1.set_ylabel("Speed (mm/s)", fontsize=20)
    a1.set_title("Speed vs Time", fontsize=18); a1.tick_params(labelsize=13)
    a1.legend(loc="best", fontsize=12)
    a2.plot(t, orient_offset, lw=2, color="#1f77b4",
            label=f"Orientation offset (deg)\nAvg: {np.nanmean(orient_offset):.2f}°")
    a2.set_ylim(-60, 60)
    a2.set_xlabel("Time (s)", fontsize=20)
    a2.set_ylabel("Angle offset from start (deg)", fontsize=20)
    a2.set_title("Orientation vs Time", fontsize=18); a2.tick_params(labelsize=13)
    a2.legend(loc="best", fontsize=12)
    _save(fig, out_dir, f"{prefix}_speed_orientation.png")

    # (3) Trajectory + 8 device boxes（直接用逐幀偵測到的紅框頂點換算 mm，不旋轉/翻正，
    #     使藍框落在影片中紅框原位）
    fig, ax = plt.subplots(figsize=(7, 6), constrained_layout=True)
    ax.plot(xr, yr, lw=2, color="blue", alpha=0.7, label="Trajectory")
    box_frames = [i for i in range(n) if tr["boxes"][i] is not None]
    box_pts = []
    if len(box_frames) >= N_TP:
        sel = np.linspace(0, len(box_frames) - 1, N_TP, dtype=int)
        for s in sel:
            i = box_frames[s]
            bxr = (tr["boxes"][i][:, 0] - x0) * mmpp
            byr = (tr["boxes"][i][:, 1] - y0) * mmpp
            bxr = np.append(bxr, bxr[0]); byr = np.append(byr, byr[0])
            ax.plot(bxr, byr, "--", lw=1.5, color=BOX_COLOR, alpha=0.8)
            box_pts.extend(np.vstack([bxr, byr]).T.tolist())
    ax.scatter([vp[0, 0]], [vp[0, 1]], s=100, c="green", marker="o", label="Start", zorder=5)
    ax.scatter([vp[-1, 0]], [vp[-1, 1]], s=100, c="red", marker="o", label="End", zorder=5)
    _square(ax, xr, yr, extra=box_pts)
    ax.invert_yaxis()   # 與 position / composite 方向一致
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x (mm)", fontsize=24); ax.set_ylabel("y (mm)", fontsize=24)
    ax.legend(loc="best", fontsize=14)
    _save(fig, out_dir, f"{prefix}_trajectory_center_only.png")

    # (4) 8-timepoints composite（相機疊圖）
    make_composite(video, tr, out_dir, prefix)
    print(f"  [完成] 輸出於 {os.path.relpath(out_dir, HERE)}")


def _square(ax, xs, ys, extra=None):
    xall = [xs[np.isfinite(xs)]]; yall = [ys[np.isfinite(ys)]]
    if extra:
        ep = np.asarray(extra); xall.append(ep[:, 0]); yall.append(ep[:, 1])
    xa = np.concatenate(xall); ya = np.concatenate(yall)
    xc, yc = 0.5 * (xa.min() + xa.max()), 0.5 * (ya.min() + ya.max())
    half = max(0.5 * max(np.ptp(xa), np.ptp(ya)) * PLOT_PAD, 1e-6)
    ax.set_xlim(xc - half, xc + half); ax.set_ylim(yc - half, yc + half)


def _save(fig, out_dir, name):
    p = os.path.join(out_dir, name)
    fig.savefig(p, dpi=DPI, bbox_inches="tight", pad_inches=0.3)
    plt.close(fig)
    print(f"  [圖] {name}")


def make_composite(video, tr, out_dir, prefix):
    cap = cv2.VideoCapture(video)
    frames = []
    while True:
        ok, img = cap.read()
        if not ok:
            break
        frames.append(img)
    cap.release()
    n = len(frames)
    valid = [i for i in range(n) if np.isfinite(tr["cx"][i])]
    sel = [valid[k] for k in np.linspace(0, len(valid) - 1, N_TP, dtype=int)]
    panels = []
    for si, fi in enumerate(sel):
        img = frames[fi].copy()
        # 疊：僅「過去」snapshot 的淺藍虛線框（不含當前幀），才不會罩住當前的紅框。
        # 淺藍框本就是從紅框偵測的 boxPoints，形狀與紅框一模一樣。
        for prev in sel[:si]:
            bp = tr["boxes"][prev]
            if bp is not None:
                pts = bp.astype(int)
                for e in range(4):
                    p1, p2 = tuple(pts[e]), tuple(pts[(e + 1) % 4])
                    cv2.line(img, p1, p2, LIGHT_BLUE, 1)
        # 起點到當下的藍色中心路徑
        path = [(int(tr["cx"][j]), int(tr["cy"][j])) for j in range(fi + 1)
                if np.isfinite(tr["cx"][j])]
        for e in range(1, len(path)):
            cv2.line(img, path[e - 1], path[e], PATH_BLUE, 2)
        if path:
            cv2.circle(img, path[-1], 3, PATH_BLUE, -1)
        # 時間戳（字體放大加粗，縮放拼接後仍清楚）
        ts = fi / tr["fps"]
        h, w = img.shape[:2]
        label = f"t={ts:.2f}s"
        fs = max(1.2, w / 160.0)
        thick = 3
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, fs, thick)
        cv2.rectangle(img, (5, h - th - 16), (14 + tw, h - 4), (0, 0, 0), -1)
        cv2.putText(img, label, (8, h - 10), cv2.FONT_HERSHEY_SIMPLEX, fs,
                    (0, 0, 255), thick, cv2.LINE_AA)
        # 等高縮放
        sc = TARGET_H / h
        panels.append(cv2.resize(img, (int(w * sc), TARGET_H)))
    comp = cv2.hconcat(panels)
    p = os.path.join(out_dir, f"{prefix}_8_timepoints_composite.png")
    cv2.imwrite(p, comp)
    print(f"  [圖] {os.path.basename(p)}")


if __name__ == "__main__":
    if len(sys.argv) > 1:
        analyze(sys.argv[1])
    else:
        analyze("experiment_backward/backward with control")
