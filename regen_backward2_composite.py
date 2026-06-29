# -*- coding: utf-8 -*-
"""
重新產生 backward2 (IMG_7129) 的 8-timepoints 相機疊圖 composite。

方向修正（2026-06-29）：
  - 真實畫面為資料夾內的 IMG_7129.MOV（428×640），device 實際「往下走」。
  - film-tracking 的 IMG_7129_tracked.mp4（720×790）是「被旋轉過」的版本，
    device 在其中變成往左走 → 先前用它導致 composite 方向錯誤。
  - 改為「全部幾何（方框、中心路徑、snapshot 圓點）都從原始 MOV 逐幀重新偵測」，
    不再依賴解析度/朝向不符的 CSV 像素座標，方向即與真實一致（往下）。

時間標籤放大：
  - 參考 clockwise composite（每 panel ~203px 寬、字明顯偏大），
    font_scale 依 panel 寬度自適應放大，並加粗，使縮放拼接後字夠大。

繪製邏輯沿用 filming_straight_and_speed.py 的 composite 段落：
  8 等間隔有效幀為 snapshot；每幀疊「至今所有 snapshot 的淺藍虛線方框」
  +「起點到當下的藍色中心路徑」+ snapshot 點藍色實心圓
  + 右下角 t=..s 紅字（黑底半透明）；8 張等高縮放後水平拼接。

數值忠實：方框/中心由影片逐幀偵測（minAreaRect），純視覺；不改任何 CSV 數值。
"""

import os
import importlib.util
import cv2
import numpy as np

FT = "/mnt/c/Users/liuuhua/Desktop/Git Repository/film-tracking"
OUT_DIR = ("/mnt/c/Users/liuuhua/Desktop/Git Repository/"
           "camera_function_generator_multithreaded/experiment_backward/"
           "without control backward2")
VIDEO = os.path.join(OUT_DIR, "IMG_7129.MOV")   # 真實畫面（device 往下走）
PREFIX = "IMG_7129"
N_SEG = 8
TARGET_H = 300                 # 拼接後每張高度（與參考一致）
LIGHT_BLUE = (255, 200, 150)   # BGR 淺藍（虛線方框）
PATH_BLUE = (255, 0, 0)        # BGR 藍（中心路徑/圓點）


def _load(mod_name, filename):
    spec = importlib.util.spec_from_file_location(mod_name, os.path.join(FT, filename))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def detect_all(video_path, detector):
    """逐幀偵測，回傳 list[(idx, center(x,y) or None, box(4,2) or None)]。"""
    cap = cv2.VideoCapture(video_path)
    out, idx, prev = [], 0, None
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        meas = detector(frame, prev_angle=prev)
        if meas is not None:
            out.append((idx, (meas[0], meas[1]), meas[8]))
            if meas[6] is not None and np.isfinite(meas[6]):
                prev = meas[6]
        else:
            out.append((idx, None, None))
        idx += 1
    cap.release()
    return out


def dashed_box(frame, box, color):
    """以 ~10px 為一節、隔節繪製的虛線方框（與原 composite 一致）。"""
    b = np.int32(box)
    for j in range(4):
        p1, p2 = b[j], b[(j + 1) % 4]
        dist = np.linalg.norm(p1 - p2)
        nseg = int(dist / 10)
        for k in range(nseg):
            if k % 2 == 0:
                s = (int(p1[0] + (p2[0]-p1[0])*k/nseg),     int(p1[1] + (p2[1]-p1[1])*k/nseg))
                e = (int(p1[0] + (p2[0]-p1[0])*(k+1)/nseg), int(p1[1] + (p2[1]-p1[1])*(k+1)/nseg))
                cv2.line(frame, s, e, color, 2)


def main():
    S = _load("filming_straight_and_speed", "filming_straight_and_speed.py")
    cap0 = cv2.VideoCapture(VIDEO)
    fps = cap0.get(cv2.CAP_PROP_FPS) or 30.0
    W = int(cap0.get(3)); H = int(cap0.get(4))
    cap0.release()

    det = detect_all(VIDEO, S.find_target_and_angle)
    valid = [d for d in det if d[1] is not None and d[2] is not None]
    if len(valid) < N_SEG:
        print(f"[錯誤] 有效幀僅 {len(valid)} < {N_SEG}")
        return

    sel = [valid[k] for k in np.linspace(0, len(valid)-1, N_SEG, dtype=int)]
    f0 = valid[0][0]               # 第一個有效幀的索引（時間 0 基準）
    cents = {d[0]: d[1] for d in valid}   # idx -> center
    boxes = {d[0]: d[2] for d in valid}

    # 時間標籤字體：依「縮放後 panel 寬度」自適應，使最終字夠大（參考圖風格）
    panel_w_after = int(W * TARGET_H / H)
    font = cv2.FONT_HERSHEY_SIMPLEX
    fs = max(1.6, panel_w_after / 130.0)   # 縮放前較大，縮小後仍清楚
    ft = max(3, int(fs * 2))

    cap = cv2.VideoCapture(VIDEO)
    frames_out = []
    for si, (idx, _, _) in enumerate(sel):
        cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
        ok, frame = cap.read()
        if not ok:
            continue
        # 至今所有 snapshot 的淺藍虛線方框
        for j in range(si + 1):
            dashed_box(frame, boxes[sel[j][0]], LIGHT_BLUE)
        # 起點到當下的藍色中心路徑（所有有效幀 idx<=當前）
        pts = [(int(round(cents[i][0])), int(round(cents[i][1])))
               for i in sorted(cents) if i <= idx]
        for i in range(1, len(pts)):
            cv2.line(frame, pts[i-1], pts[i], PATH_BLUE, 2)
        # snapshot 點藍色實心圓
        for j in range(si + 1):
            c = cents[sel[j][0]]
            cv2.circle(frame, (int(round(c[0])), int(round(c[1]))), 7, PATH_BLUE, -1)
        # 右下角時間標註（黑底半透明 + 紅字，字體放大）
        txt = f"t={(idx - f0)/fps:.2f}s"
        (tw, th), bl = cv2.getTextSize(txt, font, fs, ft)
        tx, ty = W - tw - 12, H - 14
        ov = frame.copy()
        cv2.rectangle(ov, (tx-6, ty-th-6), (tx+tw+6, ty+bl+6), (0, 0, 0), -1)
        cv2.addWeighted(ov, 0.5, frame, 0.5, 0, frame)
        cv2.putText(frame, txt, (tx, ty), font, fs, (0, 0, 255), ft, cv2.LINE_AA)
        frames_out.append(frame)
    cap.release()

    if len(frames_out) != N_SEG:
        print(f"[錯誤] 僅取得 {len(frames_out)} 張畫面")
        return

    resized = [cv2.resize(f, (int(f.shape[1] * TARGET_H / f.shape[0]), TARGET_H))
               for f in frames_out]
    combined = np.hstack(resized)
    out = os.path.join(OUT_DIR, f"{PREFIX}_8_timepoints_composite.png")
    cv2.imwrite(out, combined)
    print(f"[輸出] {out}  ({combined.shape[1]}x{combined.shape[0]})  "
          f"font_scale={fs:.2f} thickness={ft}")


if __name__ == "__main__":
    main()
