# -*- coding: utf-8 -*-
"""
論文用「without control」四組數據統一出圖 + CSV 去雜訊腳本。

目標（依使用者需求）：
  1. 四組（backward / forward / clockwise / counterclockwise）各自產出一致的圖表：
       - <prefix>_position.png            位置軌跡
       - <prefix>_trajectory_center_only.png  軌跡 + 8 個等間隔時間點的 device 外框
       - 直走組：<prefix>_speed.png            速度 vs 時間
       - 旋轉組：<prefix>_angular_speed.png + <prefix>_theta_vs_time.png
     8 個時間點外框格式/取點方式完全一致（固定 9×6 mm 虛線框，linspace 等間隔），
     所有圖一律 grid off。
  2. 對每組 CSV 去雜訊：Hampel 去尖刺 → 移動平均，重算 x_mm/y_mm 與速度/角速度，
     輸出 <prefix>_pos_angle_speed_filtered.csv（不覆蓋原檔）。
  3. backward 特別修正（已與使用者確認）：
       - 比例尺改用 0.123 mm/px（沿用 _fixed，原 0.556 使位移虛增約 4.5 倍）
       - 時間軸改用實際影片幀率 10.522 fps 重建（原 CSV 誤用 120 fps 間隔，
         使時長被壓成 0.29 s、速度高估約 12 倍）
     圖檔加 "(low-fps 36-frame clip)" 註記，標明此組品質低於其他三組。

事實忠實原則：
  - 數值曲線一律由各組既有的 *_pos_angle_speed.csv 去雜訊後重算，不重跑影片追蹤。
  - 8 個方框純為視覺呈現（固定 device 尺寸、CSV 角度），不參與任何數值統計。
  - 相機疊圖 composite 需要原始影片；forward / clockwise 找不到來源影片，沿用既有 PNG。
"""

import os
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ---------------------------------------------------------------------------
# 全域參數（與 film_success/analyze_film_success.py、film-tracking 一致）
# ---------------------------------------------------------------------------
POS_HAMPEL_WIN = 7      # Hampel 視窗（幀）
POS_MAD_K      = 5.0    # 位置尖刺門檻（× 視窗 MAD）
POS_FLOOR_PX   = 6.0    # 位置離群絕對地板（px）
POS_SMOOTH_WIN = 5      # 位置移動平均視窗（幀）
SMOOTH_WIN     = 5      # 角度/速度移動平均視窗（幀）
ANG_INTERP_MAX_GAP = 15  # 角度內插補洞的最大缺口點數（超過則保留斷線，避免亂補長缺口）

DEVICE_LONG_MM  = 9.0   # device 長邊（mm）
DEVICE_SHORT_MM = 6.0   # device 短邊（mm）
BOX_COLOR       = (0.5, 0.7, 1.0)   # 虛線方框顏色（淺藍）
N_TIMEPOINTS    = 8     # 等間隔時間點數
PLOT_PAD        = 1.35  # 座標軸留白倍率
DPI             = 600   # 論文輸出解析度

HERE = os.path.dirname(os.path.abspath(__file__))

# ---------------------------------------------------------------------------
# 四組資料定義
#   mmpp_override : None=用 CSV 內的 mm_per_px；數值=強制覆蓋（backward 用）
#   fps_override  : None=用 CSV 的 t_s；數值=用 frame 索引 + 此 fps 重建時間軸
# ---------------------------------------------------------------------------
DATASETS = [
    dict(key="backward", mode="straight",
         folder="experiment_backward/without control backward",
         csv="camera_straight_pos_angle_speed.csv",
         prefix="camera_straight",
         mmpp_override=0.12304817675552832,   # 沿用 _fixed
         fps_override=10.522,                  # 實際影片幀率
         note="low-fps 36-frame clip"),
    dict(key="backward2", mode="straight",
         folder="experiment_backward/without control backward2",
         csv="IMG_7129_pos_angle_speed.csv",
         prefix="IMG_7129",
         mmpp_override=None, fps_override=None, note=None,
         # 原始 CSV 為水平移動，主方向旋轉對齊成垂直，與其他直走組一致
         align_vertical=True),
    dict(key="forward", mode="straight",
         folder="experiment_forward/without control forward",
         csv="camera_straight_pos_angle_speed.csv",
         prefix="camera_straight",
         mmpp_override=None, fps_override=None, note=None),
    dict(key="clockwise", mode="rotation",
         folder="experiment_clockwise/without control clockwise",
         csv="camera_rotation_pos_angle_speed.csv",
         prefix="camera_rotation",
         mmpp_override=None, fps_override=None, note=None),
    dict(key="counterclockwise", mode="rotation",
         folder="experiment_counterclockwise/without control counterclockwise",
         csv="IMG_7110_pos_angle_speed.csv",
         prefix="IMG_7110",
         mmpp_override=None, fps_override=None, note=None),
]


# ---------------------------------------------------------------------------
# 去雜訊工具
# ---------------------------------------------------------------------------
def hampel_1d(v, win=POS_HAMPEL_WIN, k=POS_MAD_K, floor=POS_FLOOR_PX):
    """Hampel 濾波：偏離局部視窗中位數 > max(k×MAD, floor) 的孤立尖刺替補成中位數。"""
    v = np.asarray(v, float).copy()
    n = len(v)
    half = win // 2
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
    """置中移動平均，邊界以可用窗縮短，保留 NaN 安全。"""
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


def finite_diff(values, t, dt_floor=None):
    """逐點一階差分（dv/dt），首點為 NaN。

    dt_floor：時間間隔下限。追蹤掉幀時相鄰時間戳可能極短（例如 0.002s，
    遠小於中位數 0.012s），若直接相除會把微小角度變化放大成數百 deg/s 的
    假尖峰。設下限後 dt 至少為 dt_floor，抑制此類假訊號。對等間隔資料
    （dt 均等於中位數）此下限不會觸發，結果與原本完全相同。
    """
    v = np.full(len(values), np.nan)
    for i in range(1, len(values)):
        if np.isfinite(values[i]) and np.isfinite(values[i - 1]) and t[i] > t[i - 1]:
            dt = t[i] - t[i - 1]
            if dt_floor is not None:
                dt = max(dt, dt_floor)
            v[i] = (values[i] - values[i - 1]) / dt
    return v


def interp_nan(values, max_gap=None):
    """線性內插補內部 NaN 缺口，讓序列連續（供微分與畫實體線用）。

    僅補「兩側都有有限值」的內部缺口；序列開頭/結尾的 NaN 不外插。
    追蹤暫時失敗留下的 NaN 缺口補起來後，畫線即為實體不斷線。

    Args:
        values: 一維序列
        max_gap: 缺口點數上限；超過此長度的缺口保留 NaN 不補。None=全部內部缺口都補。
    """
    v = np.asarray(values, float).copy()
    n = len(v)
    finite = np.isfinite(v)
    if finite.sum() < 2:
        return v
    idx = np.arange(n)
    v_interp = np.interp(idx, idx[finite], v[finite])
    i = 0
    while i < n:
        if finite[i]:
            i += 1
            continue
        j = i
        while j < n and not finite[j]:
            j += 1
        # 內部缺口 [i, j)：左右都有有限值才補，且不超過 max_gap
        if i - 1 >= 0 and j < n and (max_gap is None or (j - i) <= max_gap):
            v[i:j] = v_interp[i:j]
        i = j
    return v


def principal_direction_xy(x, y):
    """以 PCA 主軸求旋轉角 phi，使主要運動方向對齊 +Y（與 film-tracking 一致）。"""
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


# ---------------------------------------------------------------------------
# 核心：載入 + 去雜訊 + 重算
# ---------------------------------------------------------------------------
def process_csv(ds):
    """回傳去雜訊後的 DataFrame（欄位與原檔相容），並寫出 *_filtered.csv。"""
    path = os.path.join(HERE, ds["folder"], ds["csv"])
    df = pd.read_csv(path)

    # 1) 比例尺
    mmpp = ds["mmpp_override"] if ds["mmpp_override"] is not None \
        else float(df["mm_per_px"].dropna().iloc[0])

    # 2) 時間軸：fps_override 時用 frame 索引重建（修 backward 錯誤時間戳）
    frame = df["frame"].to_numpy(float)
    if ds["fps_override"] is not None:
        t = (frame - frame[0]) / ds["fps_override"]
    else:
        t = df["t_s"].to_numpy(float)

    # 3) 位置去雜訊：Hampel 去尖刺 → 移動平均（在像素域做，再換算 mm）
    xpx = hampel_1d(df["x_px_filt"].to_numpy(float))
    ypx = hampel_1d(df["y_px_filt"].to_numpy(float))
    xpx = moving_average(xpx)
    ypx = moving_average(ypx)

    valid = np.isfinite(xpx) & np.isfinite(ypx)
    x0 = xpx[valid][0]
    y0 = ypx[valid][0]
    x_mm = (xpx - x0) * mmpp
    y_mm = (ypx - y0) * mmpp

    # 3b) 主方向對齊：部分組原始為水平移動，旋轉到 +Y 使所有直走組方向一致。
    #     僅幾何旋轉、不縮放，故速度大小、總位移等數值不受影響。
    phi = 0.0
    flipped = False
    if ds.get("align_vertical"):
        phi = principal_direction_xy(x_mm, y_mm)
        x_mm, y_mm = rotate_xy(x_mm, y_mm, phi)
        # 對齊後若主方向朝 +Y（start 在下），翻正使 start 在上、end 在下，與其他組一致
        if y_mm[valid][-1] > y_mm[valid][0]:
            y_mm = -y_mm
            x_mm = -x_mm
            flipped = True

    # 4) 速度（mm/s）：由去雜訊後位置重算，再平滑
    vx = moving_average(finite_diff(x_mm, t), SMOOTH_WIN)
    vy = moving_average(finite_diff(y_mm, t), SMOOTH_WIN)
    speed = np.sqrt(vx**2 + vy**2)

    # 5) 角度：
    #    - angle_deg_raw 僅供軌跡圖的 device 方框朝向用，去尖刺 + 平滑即可。
    #    - angle_deg_unwrapped 已由原始追蹤管線正確 unwrap（minAreaRect raw 角為
    #      180° 週期，naive np.unwrap 會產生鋸齒假象），故「直接對既有 unwrapped
    #      欄位去雜訊」，不從 raw 重新 unwrap，以忠實保留總旋轉量。
    ang_raw = df["angle_deg_raw"].to_numpy(float)
    ang_raw_s = moving_average(hampel_1d(ang_raw, floor=2.0), SMOOTH_WIN)
    # 方框朝向需與軌跡套用相同的主方向旋轉（+ 翻正 180°），確保方框跟著對齊
    if ds.get("align_vertical"):
        ang_raw_s = ang_raw_s + np.degrees(phi) + (180.0 if flipped else 0.0)
    ang_unwrap = moving_average(hampel_1d(df["angle_deg_unwrapped"].to_numpy(float),
                                          floor=2.0), SMOOTH_WIN)
    # 角速度：先把追蹤失敗留下的內部 NaN 缺口內插補起來（畫線才連續、微分才不斷），
    # 再以「dt 下限 = 中位數 dt」的差分抑制掉幀造成的短-dt 假尖峰，最後平滑。
    dt_arr = np.diff(t)
    dt_arr = dt_arr[np.isfinite(dt_arr) & (dt_arr > 0)]
    dt_floor = float(np.median(dt_arr)) if len(dt_arr) else None
    ang_for_vel = interp_nan(ang_unwrap, max_gap=ANG_INTERP_MAX_GAP)
    ang_vel = moving_average(finite_diff(ang_for_vel, t, dt_floor=dt_floor), SMOOTH_WIN)

    out = pd.DataFrame({
        "frame": df["frame"].to_numpy(),
        "t_s": t,
        "x_px_filt": xpx,
        "y_px_filt": ypx,
        "mm_per_px": mmpp,
        "x_mm": x_mm,
        "y_mm": y_mm,
        "angle_deg_raw": ang_raw_s,
        "angle_deg_unwrapped": ang_unwrap,
        "angular_vel_dps": ang_vel,
        "vx_mm_s": vx,
        "vy_mm_s": vy,
        "speed_mm_s": speed,
    })
    out_path = os.path.join(HERE, ds["folder"],
                            ds["prefix"] + "_pos_angle_speed_filtered.csv")
    out.to_csv(out_path, index=False)
    print(f"  [CSV] 去雜訊輸出 -> {os.path.basename(out_path)}  "
          f"(mm/px={mmpp:.4f}, dur={t[-1]-t[0]:.2f}s, "
          f"v_max={np.nanmax(speed):.1f} mm/s)")
    return out, mmpp


# ---------------------------------------------------------------------------
# 繪圖工具
# ---------------------------------------------------------------------------
def device_box(cx, cy, angle_deg, mmpp=None):
    """回傳以 (cx,cy) 為中心、依 angle 朝向的 9×6 mm 矩形 5 個閉合頂點（mm 座標）。"""
    hl, hs = DEVICE_LONG_MM / 2.0, DEVICE_SHORT_MM / 2.0
    corners = np.array([[-hl, -hs], [hl, -hs], [hl, hs], [-hl, hs]])
    a = np.radians(angle_deg if np.isfinite(angle_deg) else 0.0)
    R = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
    pts = corners @ R.T + np.array([cx, cy])
    return np.vstack([pts, pts[0]])


def square_limits(ax, xs, ys, extra=None):
    xall = [xs[np.isfinite(xs)]]
    yall = [ys[np.isfinite(ys)]]
    if extra is not None and len(extra):
        ep = np.asarray(extra)
        xall.append(ep[:, 0]); yall.append(ep[:, 1])
    xa = np.concatenate(xall); ya = np.concatenate(yall)
    xc, yc = 0.5 * (xa.min() + xa.max()), 0.5 * (ya.min() + ya.max())
    half = max(0.5 * max(np.ptp(xa), np.ptp(ya)) * PLOT_PAD, 1e-6)
    ax.set_xlim(xc - half, xc + half)
    ax.set_ylim(yc - half, yc + half)


def title_note(base, ds):
    return f"{base}  ({ds['note']})" if ds.get("note") else base


def plot_position(df, ds, out_dir):
    x, y = df["x_mm"].to_numpy(), df["y_mm"].to_numpy()
    fig, ax = plt.subplots(figsize=(7, 6), constrained_layout=True)
    ax.plot(x, y, lw=2, color="#1f77b4", label="Trajectory")
    vp = np.vstack([x, y]).T
    vp = vp[~np.isnan(vp).any(axis=1)]
    ax.scatter([vp[0, 0]], [vp[0, 1]], s=100, c="green", zorder=5, label="Start")
    ax.scatter([vp[-1, 0]], [vp[-1, 1]], s=100, c="red", zorder=5, label="End")
    square_limits(ax, x, y)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x (mm)", fontsize=22)
    ax.set_ylabel("y (mm)", fontsize=22)
    ax.set_title(title_note("Position", ds), fontsize=20)
    ax.tick_params(labelsize=14)
    ax.grid(False)
    ax.legend(loc="best", fontsize=13)
    p = os.path.join(out_dir, f"{ds['prefix']}_position.png")
    fig.savefig(p, dpi=DPI, bbox_inches="tight", pad_inches=0.3)
    plt.close(fig)
    print(f"  [圖] {os.path.basename(p)}")


def plot_trajectory_center_only(df, ds, out_dir):
    x, y = df["x_mm"].to_numpy(), df["y_mm"].to_numpy()
    ang = df["angle_deg_raw"].to_numpy()
    fig, ax = plt.subplots(figsize=(7, 6), constrained_layout=True)
    ax.plot(x, y, lw=2, color="#1f77b4", alpha=0.75, label="Trajectory")

    valid = np.where(np.isfinite(x) & np.isfinite(y))[0]
    box_pts = []
    if len(valid) >= N_TIMEPOINTS:
        sel = valid[np.linspace(0, len(valid) - 1, N_TIMEPOINTS, dtype=int)]
        for k in sel:
            box = device_box(x[k], y[k], ang[k])
            ax.plot(box[:, 0], box[:, 1], "--", lw=1.5, color=BOX_COLOR, alpha=0.85)
            box_pts.extend(box.tolist())
    vp = np.vstack([x, y]).T
    vp = vp[~np.isnan(vp).any(axis=1)]
    ax.scatter([vp[0, 0]], [vp[0, 1]], s=100, c="green", zorder=5, label="Start")
    ax.scatter([vp[-1, 0]], [vp[-1, 1]], s=100, c="red", zorder=5, label="End")
    square_limits(ax, x, y, extra=box_pts)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x (mm)", fontsize=22)
    ax.set_ylabel("y (mm)", fontsize=22)
    ax.set_title(title_note("Trajectory with 8 Time Points", ds), fontsize=18)
    ax.tick_params(labelsize=14)
    ax.grid(False)
    ax.legend(loc="best", fontsize=13)
    p = os.path.join(out_dir, f"{ds['prefix']}_trajectory_center_only.png")
    fig.savefig(p, dpi=DPI, bbox_inches="tight", pad_inches=0.3)
    plt.close(fig)
    print(f"  [圖] {os.path.basename(p)}")


def plot_speed(df, ds, out_dir):
    t, sp = df["t_s"].to_numpy(), df["speed_mm_s"].to_numpy()
    fig, ax = plt.subplots(figsize=(8, 6), constrained_layout=True)
    ax.plot(t, sp, lw=2, color="#1f77b4", label="Speed (mm/s)")
    fin = np.isfinite(sp)
    if fin.any():
        i = np.where(fin)[0][int(np.nanargmax(sp[fin]))]
        ax.plot([t[i]], [sp[i]], "o", ms=8, color="red",
                label=f"Max: {sp[i]:.2f} mm/s @ {t[i]:.2f}s")
    ax.set_xlabel("Time (s)", fontsize=22)
    ax.set_ylabel("Speed (mm/s)", fontsize=22)
    ax.set_title(title_note("Speed vs Time", ds), fontsize=20)
    ax.tick_params(labelsize=14)
    ax.grid(False)
    ax.legend(loc="best", fontsize=13)
    p = os.path.join(out_dir, f"{ds['prefix']}_speed.png")
    fig.savefig(p, dpi=DPI, bbox_inches="tight", pad_inches=0.3)
    plt.close(fig)
    print(f"  [圖] {os.path.basename(p)}")


def plot_angular_speed(df, ds, out_dir):
    t, w = df["t_s"].to_numpy(), df["angular_vel_dps"].to_numpy()
    t = t - t[np.isfinite(t)][0]  # 時間軸歸零，各組一致從 0 開始
    fig, ax = plt.subplots(figsize=(8, 6), constrained_layout=True)
    ax.plot(t, w, lw=2, color="#1f77b4", label="Angular speed (deg/s)")
    fin = np.isfinite(w)
    if fin.any():
        i = np.where(fin)[0][int(np.nanargmax(np.abs(w[fin])))]
        ax.plot([t[i]], [w[i]], "o", ms=8, color="red",
                label=f"Max: {w[i]:.2f} deg/s @ {t[i]:.2f}s")
    ax.set_xlabel("Time (s)", fontsize=22)
    ax.set_ylabel("Angular speed (deg/s)", fontsize=22)
    ax.set_title(title_note("Angular Speed vs Time", ds), fontsize=20)
    ax.tick_params(labelsize=14)
    ax.grid(False)
    ax.legend(loc="best", fontsize=13)
    p = os.path.join(out_dir, f"{ds['prefix']}_angular_speed.png")
    fig.savefig(p, dpi=DPI, bbox_inches="tight", pad_inches=0.3)
    plt.close(fig)
    print(f"  [圖] {os.path.basename(p)}")


def plot_theta(df, ds, out_dir):
    t, th = df["t_s"].to_numpy(), df["angle_deg_unwrapped"].to_numpy()
    t = t - t[np.isfinite(t)][0]  # 時間軸歸零，各組一致從 0 開始
    th = interp_nan(th, max_gap=ANG_INTERP_MAX_GAP)  # 補內部缺口，畫實體線
    fig, ax = plt.subplots(figsize=(8, 6), constrained_layout=True)
    ax.plot(t, th, lw=2, color="#1f77b4", label="θ (deg)")
    fin = np.isfinite(th)
    if fin.any():
        tv, thv = t[fin], th[fin]
        ax.scatter([tv[0]], [thv[0]], s=80, c="green", zorder=5,
                   label=f"Start: {thv[0]:.2f}°")
        ax.scatter([tv[-1]], [thv[-1]], s=80, c="red", zorder=5,
                   label=f"End: {thv[-1]:.2f}°")
        base = f"Theta vs Time (Total rotation: {thv[-1]-thv[0]:.2f}°)"
    else:
        base = "Theta vs Time"
    ax.set_xlabel("Time (s)", fontsize=22)
    ax.set_ylabel("θ (deg)", fontsize=22)
    ax.set_title(title_note(base, ds), fontsize=18)
    ax.tick_params(labelsize=14)
    ax.grid(False)
    ax.legend(loc="best", fontsize=13)
    p = os.path.join(out_dir, f"{ds['prefix']}_theta_vs_time.png")
    fig.savefig(p, dpi=DPI, bbox_inches="tight", pad_inches=0.3)
    plt.close(fig)
    print(f"  [圖] {os.path.basename(p)}")


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------
def main():
    for ds in DATASETS:
        out_dir = os.path.join(HERE, ds["folder"])
        print(f"\n===== {ds['key']} ({ds['mode']}) =====")
        df, mmpp = process_csv(ds)
        plot_position(df, ds, out_dir)
        plot_trajectory_center_only(df, ds, out_dir)
        if ds["mode"] == "straight":
            plot_speed(df, ds, out_dir)
        else:
            plot_angular_speed(df, ds, out_dir)
            plot_theta(df, ds, out_dir)
    print("\n完成。所有圖 grid off；CSV 去雜訊輸出為 *_filtered.csv（原檔保留）。")
    print("composite 相機疊圖：backward/ccw 有來源影片可另行重產；"
          "forward/clockwise 無影片，沿用既有 PNG。")


if __name__ == "__main__":
    main()
