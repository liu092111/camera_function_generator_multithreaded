# -*- coding: utf-8 -*-
"""
橫向位置偏移圖（Lateral Deviation）
====================================
量的是「假設 device 應沿直線走，實際偏離理想直線多少距離（mm）」。
與 orientation 圖（device 本體自己偏轉的角度）不同——這裡量的是「走歪的距離」。

理想直線的定義（對任意走向都正確）：
  - 以軌跡的 PCA 主軸方向為「理想前進方向」（與 position 圖對齊 +Y 的邏輯一致）。
  - 理想直線 = 通過起點、沿主軸方向的射線。
  - 橫向偏移 = 每一點到這條理想直線的「垂直距離」（帶正負號）。
      * 沿主軸投影量 = 已前進距離（圖的橫軸）
      * 垂直於主軸的分量 = 橫向偏移（圖的縱軸），右偏為 +、左偏為 −

輸出：<prefix>_lateral_deviation.png
統計：Max（最大絕對偏移）、RMS（均方根偏移）、Final（終點偏移）。

事實忠實：數值一律取自既有 CSV 的 x_mm/y_mm（相對起點的 mm 座標），
不重跑追蹤；僅做與其他圖一致的輕度平滑以抑制逐點雜訊。
"""

import os
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

BLUE = "#1f77b4"

# 各資料夾的資料來源（prefix 用於輸出檔名；csv 為相對資料夾的檔名）
DATASETS = [
    dict(folder="experiment_forward/with control",
         csv="with control.csv", prefix="1.7V straight"),
    dict(folder="experiment_forward/without control forward",
         csv="camera_straight_pos_angle_speed_filtered.csv", prefix="camera_straight"),
]


def moving_average(a, w=5):
    a = np.asarray(a, float)
    if w is None or w <= 1:
        return a
    out = np.full_like(a, np.nan)
    half = w // 2
    for i in range(len(a)):
        win = a[max(0, i - half):min(len(a), i + half + 1)]
        win = win[np.isfinite(win)]
        if len(win):
            out[i] = win.mean()
    return out


def principal_axis(x, y):
    """回傳軌跡 PCA 主軸單位向量（沿最大變異方向）。"""
    pts = np.vstack([x, y]).T
    pts = pts[~np.isnan(pts).any(axis=1)]
    if len(pts) < 2:
        return np.array([0.0, 1.0])
    c = pts - pts.mean(0, keepdims=True)
    eva, eve = np.linalg.eig(np.cov(c.T))
    v = eve[:, int(np.argmax(eva))]
    return v / (np.linalg.norm(v) + 1e-12)


def lateral_deviation(x, y):
    """回傳 (沿主軸前進距離 s, 橫向偏移 d)。
    d>0 = 相對前進方向偏右、d<0 = 偏左。皆相對起點與主軸。"""
    valid = np.isfinite(x) & np.isfinite(y)
    x0, y0 = x[valid][0], y[valid][0]
    dx, dy = x - x0, y - y0
    u = principal_axis(x, y)              # 主軸（前進方向）
    n = np.array([-u[1], u[0]])           # 主軸的法向量（橫向）
    # 讓主軸方向與實際位移總方向一致（避免主軸指反）
    disp = np.array([np.nanmean(dx), np.nanmean(dy)])
    if np.dot(disp, u) < 0:
        u = -u
        n = np.array([-u[1], u[0]])
    s = dx * u[0] + dy * u[1]             # 沿主軸投影 = 前進距離
    d = dx * n[0] + dy * n[1]             # 垂直分量 = 橫向偏移
    return s, d


def plot_one(ds, root):
    path = os.path.join(root, ds["folder"], ds["csv"])
    df = pd.read_csv(path)
    x = moving_average(df["x_mm"].to_numpy(float), 5)
    y = moving_average(df["y_mm"].to_numpy(float), 5)
    s, d = lateral_deviation(x, y)

    fin = np.isfinite(s) & np.isfinite(d)
    s, d = s[fin], d[fin]
    max_abs = float(np.max(np.abs(d)))
    rms = float(np.sqrt(np.mean(d ** 2)))
    final = float(d[-1])

    fig, ax = plt.subplots(figsize=(8, 6), constrained_layout=True)
    ax.axhline(0, color="gray", ls="--", lw=1, alpha=0.6)   # 理想直線（零偏移）
    ax.plot(s, d, lw=2, color=BLUE,
            label=f"Lateral deviation\nMax {max_abs:.2f} mm, RMS {rms:.2f} mm")
    # 標最大偏移點
    i = int(np.argmax(np.abs(d)))
    ax.plot([s[i]], [d[i]], "o", ms=8, color="red",
            label=f"Max |dev|: {d[i]:.2f} mm @ {s[i]:.1f} mm")
    ax.set_xlabel("Distance traveled (mm)", fontsize=22)
    ax.set_ylabel("Lateral deviation (mm)", fontsize=22)
    ax.set_title("Lateral Deviation from Straight Line", fontsize=20)
    # y 軸對稱置中，讓「偏離 0」的程度一目了然
    ylim = max(max_abs * 1.3, 1.0)
    ax.set_ylim(-ylim, ylim)
    ax.legend(loc="best", fontsize=13)
    ax.annotate("0 = on ideal line,  + = right,  − = left",
                xy=(0.02, 0.02), xycoords="axes fraction", fontsize=11, color="gray")
    out = os.path.join(root, ds["folder"], f"{ds['prefix']}_lateral_deviation.png")
    fig.savefig(out, dpi=600, bbox_inches="tight", pad_inches=0.3)
    plt.close(fig)
    print(f"[圖] {ds['folder']}/{os.path.basename(out)}  "
          f"Max={max_abs:.2f} RMS={rms:.2f} Final={final:.2f} mm")


def main():
    root = os.path.dirname(os.path.abspath(__file__))
    for ds in DATASETS:
        plot_one(ds, root)
    print("完成。")


if __name__ == "__main__":
    main()
