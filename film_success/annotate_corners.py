# -*- coding: utf-8 -*-
"""
人工標註 device 4 角工具
========================
搭配 analyze_film_success.py 的混合式校正流程：

  1. 先匯出標註圖：
        python3 analyze_film_success.py --export-annotate
     （或單支：python3 analyze_film_success.py --export-annotate IMG_6915）
     會在各 <stem>_analysis_output/annotate/ 產生 8 張全解析度原圖 + manifest.json

  2. 執行本工具標註（單支或全部）：
        python3 annotate_corners.py                 # 全部有 annotate/ 的影片
        python3 annotate_corners.py straight_1       # 只標 straight_1

     每張圖：左鍵點 device 的 4 個角（順序不限，程式會自動排成矩形角序）。
       - 點滿 4 點自動進下一張；按 n 跳下一張、p 上一張、u 復原上一點、
         r 清除本張、q 存檔離開。
     標註存到 annotate/corners.json。

  3. 回到分析，讀回校正：
        python3 analyze_film_success.py             # 偵測到 corners.json 即自動採用

設計：本工具只負責「收集 4 角像素座標」，幾何換算（中心、長短軸、角度）
由分析腳本統一處理，確保與 9×6mm 框繪製一致。
"""

import os
import sys
import glob
import json

import numpy as np
import cv2
import matplotlib
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))


def order_corners(pts):
    """把 4 個點排成 [左上, 右上, 右下, 左下]（影像座標，y 向下）。"""
    pts = np.array(pts, dtype=float)
    c = pts.mean(axis=0)
    ang = np.arctan2(pts[:, 1] - c[1], pts[:, 0] - c[0])
    order = np.argsort(ang)            # 逆時針排序
    pts = pts[order]
    # 旋到讓第一點為最靠左上者
    start = np.argmin(pts[:, 0] + pts[:, 1])
    pts = np.roll(pts, -start, axis=0)
    return pts.tolist()


def annotate_video(stem):
    ann_dir = os.path.join(HERE, f"{stem}_analysis_output", "annotate")
    manifest_path = os.path.join(ann_dir, "manifest.json")
    if not os.path.exists(manifest_path):
        print(f"  [skip] {stem}: 找不到 {manifest_path}（請先 --export-annotate）")
        return
    with open(manifest_path, encoding="utf-8") as f:
        manifest = json.load(f)
    frames = manifest["frames"]

    out_path = os.path.join(ann_dir, "corners.json")
    saved = {}
    if os.path.exists(out_path):
        with open(out_path, encoding="utf-8") as f:
            saved = {int(k): v for k, v in json.load(f).get("corners", {}).items()}

    state = {"idx": 0, "pts": list(saved.get(0, []))}

    fig, ax = plt.subplots(figsize=(10, 8))
    plt.subplots_adjust(bottom=0.08)

    def load(i):
        img = cv2.imread(os.path.join(ann_dir, frames[i]["file"]))
        return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    def redraw():
        ax.clear()
        i = state["idx"]
        ax.imshow(load(i))
        pts = state["pts"]
        if pts:
            P = np.array(pts)
            ax.plot(P[:, 0], P[:, 1], "o", color="yellow", ms=10, mec="black")
            for j, (px, py) in enumerate(pts):
                ax.text(px + 6, py, str(j + 1), color="yellow", fontsize=14,
                        weight="bold")
            if len(pts) == 4:
                op = np.array(order_corners(pts) + [order_corners(pts)[0]])
                ax.plot(op[:, 0], op[:, 1], "-", color="lime", lw=2)
        done = sum(1 for k in range(len(frames)) if len(saved.get(k, [])) == 4)
        ax.set_title(f"{stem}  [{i+1}/{len(frames)}]  t={frames[i]['t_s']:.2f}s  "
                     f"已標完 {done}/{len(frames)}\n"
                     f"左鍵點4角 | n下一張 p上一張 u復原 r清除 q存檔離開",
                     fontsize=12)
        ax.set_xticks([]); ax.set_yticks([])
        fig.canvas.draw_idle()

    def commit():
        state_pts = state["pts"]
        if len(state_pts) == 4:
            saved[state["idx"]] = order_corners(state_pts)
        elif len(state_pts) == 0:
            saved.pop(state["idx"], None)

    def goto(i):
        commit()
        state["idx"] = max(0, min(len(frames) - 1, i))
        state["pts"] = list(saved.get(state["idx"], []))
        redraw()

    def on_click(event):
        if event.inaxes != ax or event.button != 1:
            return
        if event.xdata is None:
            return
        if len(state["pts"]) >= 4:
            return
        state["pts"].append([float(event.xdata), float(event.ydata)])
        if len(state["pts"]) == 4:
            saved[state["idx"]] = order_corners(state["pts"])
            redraw()
            if state["idx"] < len(frames) - 1:
                goto(state["idx"] + 1)
                return
        redraw()

    def on_key(event):
        if event.key == "n":
            goto(state["idx"] + 1)
        elif event.key == "p":
            goto(state["idx"] - 1)
        elif event.key == "u":
            if state["pts"]:
                state["pts"].pop()
                saved.pop(state["idx"], None)
                redraw()
        elif event.key == "r":
            state["pts"] = []
            saved.pop(state["idx"], None)
            redraw()
        elif event.key == "q":
            commit()
            save_and_close()

    def save_and_close():
        out = {"video": manifest["video"],
               "device_long_mm": manifest["device_long_mm"],
               "device_short_mm": manifest["device_short_mm"],
               "corners": {str(k): v for k, v in saved.items() if len(v) == 4},
               "frame_meta": {str(fr["i"]): {"frame": fr["frame"], "t_s": fr["t_s"]}
                              for fr in frames}}
        with open(out_path, "w", encoding="utf-8") as f:
            json.dump(out, f, ensure_ascii=False, indent=2)
        print(f"  {stem}: 存檔 {len(out['corners'])}/{len(frames)} 張 → {out_path}")
        plt.close(fig)

    fig.canvas.mpl_connect("button_press_event", on_click)
    fig.canvas.mpl_connect("key_press_event", on_key)
    fig.canvas.mpl_connect("close_event", lambda e: save_and_close())
    redraw()
    plt.show()


def main():
    sel = sys.argv[1:]
    ann = sorted(glob.glob(os.path.join(HERE, "*_analysis_output", "annotate",
                                         "manifest.json")))
    stems = [os.path.basename(os.path.dirname(os.path.dirname(p)))
             .replace("_analysis_output", "") for p in ann]
    if sel:
        stems = [s for s in stems if any(k in s for k in sel)]
    if not stems:
        print("找不到標註圖。請先：python3 analyze_film_success.py --export-annotate")
        return
    print(f"待標註：{', '.join(stems)}")
    for s in stems:
        annotate_video(s)
    print("全部完成。回到分析：python3 analyze_film_success.py")


if __name__ == "__main__":
    main()
