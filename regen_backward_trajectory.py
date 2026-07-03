# -*- coding: utf-8 -*-
"""
只重繪 experiment_backward/backward with control 的
  camera_straight_trajectory_center_only.png

重點：8 個時間點的 device 外框「從影片逐幀實際偵測」（黃+白 HSV → 最大輪廓 →
minAreaRect → boxPoints，與即時追蹤器 find_target_and_angle 同一套），
框精準貼合 device 白色本體（已驗證偵測中心與烙印綠點 <1px 吻合），
而非固定 9×6mm 假框。

樣式對齊範本 filming_straight_and_speed.py 的 trajectory 圖：
  正方形、無網格、主方向旋轉貼齊 +Y、無標題、淺藍虛線框、Start/End 標記。

事實忠實：軌跡線與座標由既有 CSV 的像素座標換算（× mm_per_px、相對起點），
方框頂點由影片偵測得到再換算 mm；不重跑數值分析、不改任何 CSV。
"""

import os
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
FOLDER = os.path.join(HERE, "experiment_backward", "backward with control")
CSV = os.path.join(FOLDER, "camera_straight_pos_angle_speed.csv")
VIDEO = os.path.join(FOLDER, "camera_straight_tracked.mp4")
PREFIX = "camera_straight"

# 與即時追蹤器 (config.py / integrated_camera_fg_control_multithreaded.py) 一致
HSV_Y_LO = np.array([15, 60, 120], np.uint8)
HSV_Y_HI = np.array([35, 255, 255], np.uint8)
HSV_W_LO = np.array([0, 0, 200], np.uint8)
HSV_W_HI = np.array([180, 60, 255], np.uint8)
MIN_CONTOUR_AREA = 50

# 出圖樣式（對齊範本）
ALIGN_TRAJ_TO_Y = True
INVERT_Y_AXIS = False
PLOT_PAD = 1.35
BOX_COLOR = (0.5, 0.7, 1.0)
N_TP = 8


def detect_box(frame):
    """回傳 device 的 minAreaRect 4 頂點 (px)，偵測失敗回 None。"""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    m = cv2.bitwise_or(cv2.inRange(hsv, HSV_Y_LO, HSV_Y_HI),
                       cv2.inRange(hsv, HSV_W_LO, HSV_W_HI))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations=1)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8), iterations=1)
    cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(c) < MIN_CONTOUR_AREA:
        return None
    return cv2.boxPoints(cv2.minAreaRect(c))


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


def main():
    df = pd.read_csv(CSV)
    mmpp = float(df["mm_per_px"].dropna().iloc[0])

    # 軌跡（相對起點 mm）：與即時管線一致 x_mm = x_px*mmpp - x0
    xpx = df["x_px_filt"].to_numpy(float)
    ypx = df["y_px_filt"].to_numpy(float)
    frames = df["frame"].to_numpy().astype(int)
    valid_px = np.isfinite(xpx) & np.isfinite(ypx)
    x0 = xpx[valid_px][0] * mmpp
    y0 = ypx[valid_px][0] * mmpp
    x_mm = xpx * mmpp - x0
    y_mm = ypx * mmpp - y0

    # 逐幀偵測 device 框（影片幀號 = CSV frame - 第一個 CSV frame）
    f0 = frames[0]
    cap = cv2.VideoCapture(VIDEO)
    nvid = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    frame_to_box = {}
    for fr in frames:
        vf = int(fr - f0)
        if vf < 0 or vf >= nvid:
            continue
        cap.set(cv2.CAP_PROP_POS_FRAMES, vf)
        ok, img = cap.read()
        if not ok:
            continue
        box = detect_box(img)
        if box is not None:
            frame_to_box[int(fr)] = box
    cap.release()
    print(f"[偵測] {len(frame_to_box)}/{len(frames)} 幀成功偵測到 device 框")

    # 主方向旋轉角（與範本 ALIGN_TRAJ_TO_Y 一致）
    phi = principal_direction_xy(x_mm, y_mm) if ALIGN_TRAJ_TO_Y else 0.0
    xr, yr = rotate_xy(x_mm, y_mm, phi)

    fig, ax = plt.subplots(figsize=(7, 6), constrained_layout=True)
    ax.plot(xr, yr, lw=2, color="blue", alpha=0.7, label="Trajectory")

    # 8 個等間隔（在有框的幀中取）
    box_frames = [int(f) for f in frames if int(f) in frame_to_box]
    box_pts = []
    if len(box_frames) >= N_TP:
        sel = np.linspace(0, len(box_frames) - 1, N_TP, dtype=int)
        for s in sel:
            fr = box_frames[s]
            box = frame_to_box[fr]
            # 框頂點 px → mm（相對起點）→ 主方向旋轉
            bx_mm = box[:, 0] * mmpp - x0
            by_mm = box[:, 1] * mmpp - y0
            bxr, byr = rotate_xy(bx_mm, by_mm, phi)
            bxr = np.append(bxr, bxr[0])
            byr = np.append(byr, byr[0])
            ax.plot(bxr, byr, "--", lw=1.5, color=BOX_COLOR, alpha=0.7)
            box_pts.extend(np.vstack([bxr, byr]).T.tolist())

    vp = np.vstack([xr, yr]).T
    vp = vp[~np.isnan(vp).any(axis=1)]
    if len(vp):
        ax.scatter([vp[0, 0]], [vp[0, 1]], s=100, c="green", marker="o", label="Start", zorder=5)
        ax.scatter([vp[-1, 0]], [vp[-1, 1]], s=100, c="red", marker="o", label="End", zorder=5)

    allx = np.concatenate([xr[np.isfinite(xr)]] + ([np.array(box_pts)[:, 0]] if box_pts else []))
    ally = np.concatenate([yr[np.isfinite(yr)]] + ([np.array(box_pts)[:, 1]] if box_pts else []))
    xc, yc = 0.5 * (allx.min() + allx.max()), 0.5 * (ally.min() + ally.max())
    half = max(0.5 * max(np.ptp(allx), np.ptp(ally)), 1e-6) * PLOT_PAD
    ax.set_xlim(xc - half, xc + half)
    ax.set_ylim(yc - half, yc + half)
    if INVERT_Y_AXIS:
        ax.invert_yaxis()
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x (mm)", fontsize=24)
    ax.set_ylabel("y (mm)", fontsize=24)
    ax.legend(loc="best", fontsize=14)   # 無標題（同範本）
    p = os.path.join(FOLDER, f"{PREFIX}_trajectory_center_only.png")
    fig.savefig(p, dpi=1200, bbox_inches="tight", pad_inches=0.3)
    plt.close(fig)
    print(f"[圖] {os.path.basename(p)}")


if __name__ == "__main__":
    main()
