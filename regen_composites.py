# -*- coding: utf-8 -*-
"""
重新產生 backward / counterclockwise 兩組的 8-timepoints 相機疊圖 composite。

方向：兩組各自的影片即為正確朝向，不旋轉。
  - backward：experiment_backward/20251112_173936_straight_integrated/
              camera_straight_tracked.mp4（720×1280，device 往下走，與 CSV 像素吻合 <1px）
  - ccw     ：film-tracking/IMG_7110_analysis_output/IMG_7110_tracked.mp4
              （948×720，原地旋轉，與 CSV 像素吻合 <1px）

字體：與 backward2 重產版本一致 —— 右下角 t=..s 紅字（黑底半透明），
      font_scale 依 panel 寬度自適應放大並加粗。

繪製邏輯沿用 filming_*_and_speed.py 的 composite 段落：
  8 等間隔有效幀為 snapshot；每幀疊「至今所有 snapshot 的淺藍虛線方框」
  +「起點到當下的藍色中心路徑」+ snapshot 點藍色實心圓；8 張等高縮放後水平拼接。

數值忠實：方框/中心由影片逐幀偵測，純視覺；不改任何 CSV 數值。
"""

import os
import importlib.util
import cv2
import numpy as np

ROOT = "/mnt/c/Users/liuuhua/Desktop/Git Repository"
FT = os.path.join(ROOT, "film-tracking")
CFG = os.path.join(ROOT, "camera_function_generator_multithreaded")

N_SEG = 8
TARGET_H = 300
LIGHT_BLUE = (255, 200, 150)   # BGR
PATH_BLUE = (255, 0, 0)        # BGR

DATASETS = [
    dict(key="backward",
         video=os.path.join(CFG, "experiment_backward",
                            "20251112_173936_straight_integrated",
                            "camera_straight_tracked.mp4"),
         out_dir=os.path.join(CFG, "experiment_backward", "without control backward"),
         prefix="camera_straight",
         module="filming_straight_and_speed.py"),
    dict(key="counterclockwise",
         video=os.path.join(FT, "IMG_7110_analysis_output", "IMG_7110_tracked.mp4"),
         out_dir=os.path.join(CFG, "experiment_counterclockwise",
                              "without control counterclockwise"),
         prefix="IMG_7110",
         module="filming_rotation_and_speed.py"),
]


def _load(filename):
    spec = importlib.util.spec_from_file_location(filename.split(".")[0],
                                                  os.path.join(FT, filename))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def detect_all(video_path, detector):
    """逐幀偵測，回傳 list[(idx, center(x,y) or None, box(4,2) or None)]。
    兼容直走偵測器（吃 prev_angle）與旋轉偵測器（不吃）。"""
    use_prev = "prev_angle" in detector.__code__.co_varnames
    cap = cv2.VideoCapture(video_path)
    out, idx, prev = [], 0, None
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        meas = detector(frame, prev_angle=prev) if use_prev else detector(frame)
        if meas is not None:
            out.append((idx, (meas[0], meas[1]), meas[8]))
            if use_prev and meas[6] is not None and np.isfinite(meas[6]):
                prev = meas[6]
        else:
            out.append((idx, None, None))
        idx += 1
    cap.release()
    return out


def dashed_box(frame, box, color):
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


def make_composite(ds):
    S = _load(ds["module"])
    cap0 = cv2.VideoCapture(ds["video"])
    fps = cap0.get(cv2.CAP_PROP_FPS) or 30.0
    W = int(cap0.get(3)); H = int(cap0.get(4))
    cap0.release()

    det = detect_all(ds["video"], S.find_target_and_angle)
    valid = [d for d in det if d[1] is not None and d[2] is not None]
    if len(valid) < N_SEG:
        print(f"[{ds['key']}] [錯誤] 有效幀僅 {len(valid)} < {N_SEG}")
        return

    sel = [valid[k] for k in np.linspace(0, len(valid)-1, N_SEG, dtype=int)]
    f0 = valid[0][0]
    cents = {d[0]: d[1] for d in valid}
    boxes = {d[0]: d[2] for d in valid}

    # 與 backward2 一致的自適應大字體
    panel_w_after = int(W * TARGET_H / H)
    font = cv2.FONT_HERSHEY_SIMPLEX
    fs = max(1.6, panel_w_after / 130.0)
    ft = max(3, int(fs * 2))

    cap = cv2.VideoCapture(ds["video"])
    frames_out = []
    for si, (idx, _, _) in enumerate(sel):
        cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
        ok, frame = cap.read()
        if not ok:
            continue
        for j in range(si + 1):
            dashed_box(frame, boxes[sel[j][0]], LIGHT_BLUE)
        pts = [(int(round(cents[i][0])), int(round(cents[i][1])))
               for i in sorted(cents) if i <= idx]
        for i in range(1, len(pts)):
            cv2.line(frame, pts[i-1], pts[i], PATH_BLUE, 2)
        for j in range(si + 1):
            c = cents[sel[j][0]]
            cv2.circle(frame, (int(round(c[0])), int(round(c[1]))), 7, PATH_BLUE, -1)
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
        print(f"[{ds['key']}] [錯誤] 僅取得 {len(frames_out)} 張")
        return

    resized = [cv2.resize(f, (int(f.shape[1] * TARGET_H / f.shape[0]), TARGET_H))
               for f in frames_out]
    combined = np.hstack(resized)
    out = os.path.join(ds["out_dir"], f"{ds['prefix']}_8_timepoints_composite.png")
    cv2.imwrite(out, combined)
    print(f"[{ds['key']}] [輸出] {out}  ({combined.shape[1]}x{combined.shape[0]})  "
          f"font_scale={fs:.2f} thickness={ft}")


def main():
    for ds in DATASETS:
        make_composite(ds)


if __name__ == "__main__":
    main()
