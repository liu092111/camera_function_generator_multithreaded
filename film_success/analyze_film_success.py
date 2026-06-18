# -*- coding: utf-8 -*-
"""
film_success 影片離線分析（自動判斷 直走 / 旋轉 + 雜訊過濾）
============================================================
改寫自 film-tracking/filming_straight_and_speed.py 與 filming_rotation_and_speed.py，
針對 film_success/*.MOV 的實際情況加上四項強化：

1. 黃膠錨點優先：先用黃色 HSV 遮罩定位「黃膠」，再只取「離黃膠最近、且涵蓋
   黃膠」的白色團塊作為本體。畫面中的圓形水平儀、負重鐵片邊緣、雜物因為
   沒有黃膠或離黃膠遠，會被自動排除。

2. 雜訊過濾（重點）：
   - 位置 MAD 穩健離群偵測：單幀位移超過穩健門檻者標記為離群，用前值替補。
   - 結尾速度大跳截斷：序列尾端若出現速度暴增（如參考 CSV 結尾 66~87 mm/s），
     自動裁掉尾端異常段。
   - 角度連續性校正：沿用參考腳本的 ±180° 候選 + 單幀變化上限。

3. 自動判斷模式：依「淨位移 vs 淨旋轉角」比例，將每支影片分類為
   TRANSLATION（直走/平移）或 ROTATION（旋轉），套用對應的圖。

4. 輸出沿用參考格式，存到各自的 <stem>_analysis_output/：
   - <stem>_position.png            軌跡圖
   - <stem>_speed_orientation.png   直走：速度+朝向；旋轉：角速度+θ
   - <stem>_trajectory_center_only.png  軌跡 + 8 個時間點外框
   - <stem>_pos_angle_speed.csv     逐幀數據（含過濾標記）

執行：
    python3 analyze_film_success.py            # 全部 8 支
    python3 analyze_film_success.py IMG_6750   # 單支（用檔名片段）
"""

import os
import sys
import glob

import cv2
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ========= 使用者設定 =========
HERE = os.path.dirname(os.path.abspath(__file__))

# 顏色遮罩。黃色範圍放寬（原 [15,60,120] 太嚴，暗處/小黃膠常漏偵；放寬後 0 漏偵）。
HSV_YELLOW_LO = np.array([12, 40,  80], dtype=np.uint8)
HSV_YELLOW_HI = np.array([40, 255, 255], dtype=np.uint8)
YELLOW_MIN_AREA = 4           # 黃膠最小面積（px）
HSV_WHITE_LO  = np.array([0,   0, 200], dtype=np.uint8)
HSV_WHITE_HI  = np.array([180, 60, 255], dtype=np.uint8)
# 偏暗白色本體（V 偏低、低飽和）。只在黃膠錨點周邊 ROI 內用於估角度的連通塊。
HSV_BODY_LO   = np.array([0,   0, 110], dtype=np.uint8)
HSV_BODY_HI   = np.array([180, 90, 255], dtype=np.uint8)
# 灰色（負重鐵片）：低飽和 + 中等亮度。只在黃膠錨點周邊 ROI 內套用，
# 避免把灰色網格背景一起抓進來。
HSV_GRAY_LO   = np.array([0,   0,  60], dtype=np.uint8)
HSV_GRAY_HI   = np.array([180, 60, 200], dtype=np.uint8)
GRAY_ROI_HALF = 38            # 本體/灰色遮罩 ROI 半徑（px，繞黃膠錨點；略大於 device 半長即可，太大會連到網格）

# --- 旋轉角度：像素樣板比對（template matching, NCC）---
TEMPLATE_HALF   = 45          # 樣板/比對 patch 半邊長（px，需涵蓋整個 device）
COARSE_STEP_DEG = 2.0         # 角度搜尋步長（度）
SEARCH_SPAN_DEG = 30.0        # 以前一幀角度為中心的搜尋半幅（度）；首幀全 360 掃
# （payload 旗標已移入 CFG per-video 設定）

MIN_CONTOUR_AREA   = 50
ANCHOR_FALLBACK_BOX = 18      # 本體太小時，以黃膠錨點為中心造此半邊長的方框續追
SMOOTH_WIN_ANGLE   = 5
BOX_ANGLE_MED_WIN  = 21        # TRANSLATION 框朝向的滑動中位數窗（幀）：跟得上 device 邊走邊微旋，濾掉 NCC 逐幀尖刺
SMOOTH_WIN_SPEED   = 5
SPEED_THRESH_ACTIVE = 0.1      # mm/s，低於視為靜止（算平均速度時排除）

# --- 雜訊過濾參數 ---
POS_HAMPEL_WIN     = 7         # Hampel 濾波視窗（幀）
POS_MAD_K          = 5.0       # 位置尖刺門檻（× 視窗 MAD）
POS_FLOOR_PX       = 6.0       # 位置離群的絕對地板（px）：近靜止影片避免誤標抖動
POS_SMOOTH_WIN     = 5         # 位置移動平均視窗（幀）：抑制次像素抖動被放大成假速度
STATIC_STEP_MM     = 0.08      # 單幀位移 < 此值（mm）視為靜止 → 速度歸零，去結尾抖動
STATIC_MIN_RUN     = 5         # 連續靜止幀數門檻
TAIL_SPEED_MAD_K   = 5.0       # 結尾速度大跳偵測門檻（× MAD）
MAX_ANGLE_CHANGE_PER_FRAME = 15.0  # 度/幀，角度跳變上限

# --- 模式自動判斷門檻（per-video cfg["mode"]="auto" 時生效）---
ROT_NET_DEG_MIN    = 12.0      # 淨旋轉角 ≥ 此值且位移小 → 旋轉
ROT_DISP_MM_MAX    = 4.0       # 旋轉判定的位移上限（mm）
# （框偏移 off_long/off_short、運動模式 mode 已移入 CFG per-video 設定）

# --- 出圖：統一正方形 ---
SQUARE_FIG_IN      = 7.0       # 軌跡/位置圖正方形畫布邊長（吋）
ALIGN_TRAJ_TO_Y    = False     # position 圖是否把主要移動方向旋轉對齊（預設關，顯示真實軌跡方向）
PLOT_PAD_FACTOR    = 1.25      # 座標範圍 = 數據半幅 × 此倍率（含 device 外框）
PLOT_MIN_HALF_MM   = 6.0       # 座標最小半幅（mm），避免近靜止影片過度放大

# --- Device 尺寸校正（無網格背景時的比例尺來源）---
# device 實體長寬（mm）。對沒有網格可參照的影片（如牛皮紙背景），
# 改用追蹤到的 minAreaRect 像素長寬對應此實體尺寸來換算 mm/px。
DEVICE_LONG_MM   = 9.0
DEVICE_SHORT_MM  = 6.0

# ============================================================
# Per-video 校正表（每支獨立設定，取代舊的散落全域值）
# ------------------------------------------------------------
# 每支影片手機架設距離不同，比例尺必須逐支量；device/貼法也分三類，
# 故錨點偏移、運動模式也逐支設定。各欄位：
#   name        : 輸出描述名（資料夾/檔名前綴）
#   scale_pref  : 比例尺主來源。"device"=用 dev_mmpp（框長邊=device長邊，保證貼合）；
#                 "grid"=用 grid_cell_mm/ACF格距。另一個來源自動作交叉驗證印出。
#   grid_cell_mm: 網格一格實體 mm。淺灰格=10、深黑格=5、None=無網格
#   dev_mmpp    : device 量到的 mm/px（= 9mm / device 長邊 px，2026-06-15 逐支看刻度尺量）
#   off_long    : 框中心相對黃膠錨點沿「長軸」偏移比例（× 半長）
#   off_short   : 沿「短軸」偏移比例（× 半短）；影像 y 向下，負 = 黃膠在 device 下方→框往上
#   mode        : "auto" / "rotation" / "translation"（手動覆寫自動判斷）
#   payload     : 是否為負重鐵塊（黃膠被鐵塊半遮，需放寬偵測）
#
# 比例尺決策（誠實標註，2026-06-15 逐支看刻度尺放大圖量得）：
#   - 灰底 straight_1/2、slope_1：device 白≈灰格，格距與 device 量差達 ~40%，
#     且「框要精確貼合 device」→ 一律用 device 長邊定 mm/px（scale_pref="device"）。
#     量得長邊 px：6915=59、6943=49、6750=48。
#   - straight_3（黑底 5mm）：device 0.15 與格距 5/35=0.143 吻合（高對比），用 grid。
#   - payload_1/2：灰鐵塊蓋住 device 看不到 9×6 邊界，黃膠在中央 → 只能用 grid。
#   - slope_2：整片黃膠的不同 device，9×6 邊界不明顯 → 用 grid。
#   - kraft：牛皮紙無網格，device 長邊 63px → 9/63≈0.143。
#   off_long/off_short 2026-06-15 經第一幀亮度剖面量測：黃膠錨點實際大致落在
#   device 中心（各支 frac≈0.4~0.5），非原假設的左下角 → 偏移一律設 0。
#
# 選填欄位（未填則套 _CFG_DEFAULTS 預設）：
#   dev_long_mm / dev_short_mm: 該支 device 框的長/短邊實體 mm（預設 9/6）。
#       payload_2 因 device 放反，長短軸對調 → 6/9。
_CFG_DEFAULTS = dict(scale_pref="grid", grid_cell_mm=5, dev_mmpp=None,
                     off_long=0.0, off_short=0.0, mode="auto", payload=False,
                     dev_long_mm=DEVICE_LONG_MM, dev_short_mm=DEVICE_SHORT_MM)
CFG = {
    "IMG_6750": dict(name="slope_level_1", scale_pref="device", grid_cell_mm=10, dev_mmpp=9.0/48,
                     off_long=0.0,  off_short=0.0,  mode="auto",        payload=False),
    "IMG_6756": dict(name="slope_level_2", scale_pref="grid",   grid_cell_mm=10, dev_mmpp=None,
                     off_long=0.0,  off_short=0.0,  mode="rotation",    payload=False),
    "IMG_6762": dict(name="payload_1",     scale_pref="grid",   grid_cell_mm=10, dev_mmpp=None,
                     off_long=0.0,  off_short=0.0,  mode="auto",        payload=True),
    # payload_2：device 放反，框尺寸為 6mm(長軸方向)×9mm(短軸方向)
    "IMG_6764": dict(name="payload_2",     scale_pref="grid",   grid_cell_mm=10, dev_mmpp=None,
                     off_long=0.0,  off_short=0.0,  mode="rotation",    payload=True,
                     dev_long_mm=6.0, dev_short_mm=9.0),
    # straight_1：實際是先逆時針再順時針的 back-and-forth 旋轉，非直線運動
    "IMG_6915": dict(name="straight_1",    scale_pref="device", grid_cell_mm=10, dev_mmpp=9.0/53,
                     off_long=0.0,  off_short=0.0,  mode="rotation",    payload=False),
    "IMG_6943": dict(name="straight_2",    scale_pref="device", grid_cell_mm=10, dev_mmpp=9.0/49,
                     off_long=0.0,  off_short=0.0,  mode="translation", payload=False),
    "IMG_7129": dict(name="straight_3",    scale_pref="grid",   grid_cell_mm=5,  dev_mmpp=9.0/60,
                     off_long=0.0,  off_short=0.0,  mode="translation", payload=False),
    "IMG_6919": dict(name="kraft_straight", scale_pref="device", grid_cell_mm=None, dev_mmpp=9.0/63,
                     off_long=0.0,  off_short=0.0,  mode="translation", payload=False),
}


def _cfg(img_stem):
    """取得該影片的 per-video 設定，套上 _CFG_DEFAULTS 補齊選填欄位；
    未列入 CFG 的影片給安全預設（grid 5mm、auto、9x6）。"""
    base = dict(_CFG_DEFAULTS)
    base["name"] = img_stem
    base.update(CFG.get(img_stem, {}))
    return base


COPY_VIDEO_INTO_OUTPUT = True      # 複製原始 MOV 進輸出資料夾並改描述名

# --- 時間門控（temporal gating）：拒絕離預期位置太遠的團塊 ---
# 半徑 = device 對角線像素 × 此倍率（初始用畫面比例估）。擋掉水平儀/鐵片/雜物。
GATE_RADIUS_FACTOR = 4.0
GATE_RADIUS_MIN_PX = 60.0
# ============================


# ----------------------------------------------------------------------
# 1. 比例尺：格距偵測 + device 9mm 雙重認證
# ----------------------------------------------------------------------
def estimate_grid_period_px(frame):
    """回傳網格一格的像素週期（不乘 mm）。用邊緣投影自相關（ACF），
    經 8 支實測比 HoughLinesP 穩定（灰白/黑底格距 35~55px 皆乾淨）。
    取 x、y 兩方向 ACF 首峰的中位數；失敗回 None。"""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    g = cv2.GaussianBlur(gray, (5, 5), 0).astype(float)
    gx = np.abs(cv2.Sobel(g, cv2.CV_64F, 1, 0, ksize=3)).mean(0)
    gy = np.abs(cv2.Sobel(g, cv2.CV_64F, 0, 1, ksize=3)).mean(1)

    def acf_period(sig):
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

    cands = [p for p in (acf_period(gx), acf_period(gy)) if p]
    return float(np.median(cands)) if cands else None


def measure_device_long_px(frame):
    """粗量 device 長邊像素（供 9mm 交叉驗證）。以黃膠錨點為中心取 ROI，
    用 Canny 邊緣能量閉合成團塊，取含錨點、長寬比最接近 1.5 者的 minAreaRect 長邊。

    誠實標註：device 白色 ≈ 灰格背景時，此量測在灰底影片不可靠（邊界易貼到
    ROI 邊界），僅作為交叉驗證印出與偏差警告，不直接當灰底的比例尺來源。
    回傳 (long_px, aspect) 或 None。"""
    a = find_yellow_anchor(frame)
    if a is None:
        return None
    ax, ay = a
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    R = 60
    H, W = gray.shape
    y0, y1 = int(max(0, ay - R)), int(min(H, ay + R))
    x0, x1 = int(max(0, ax - R)), int(min(W, ax + R))
    sub = gray[y0:y1, x0:x1]
    aax, aay = ax - x0, ay - y0
    e = cv2.Canny(sub, 40, 120)
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    blob = cv2.morphologyEx(e, cv2.MORPH_CLOSE, k, iterations=2)
    blob = cv2.morphologyEx(blob, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    cnts = cv2.findContours(blob, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
    cnts = [c for c in cnts if cv2.contourArea(c) >= 200]
    if not cnts:
        return None

    def score(c):
        (cx, cy), (w, h), _ = cv2.minAreaRect(c)
        lo, sh = max(w, h), max(min(w, h), 1)
        contains = cv2.pointPolygonTest(c, (float(aax), float(aay)), False) >= 0
        return (0 if contains else 1,
                abs(lo / sh - 1.5) + np.hypot(cx - aax, cy - aay) / 40.0)

    (cx, cy), (w, h), _ = cv2.minAreaRect(min(cnts, key=score))
    lo, sh = max(w, h), max(min(w, h), 1)
    return float(lo), float(lo / sh)


def resolve_mm_per_px(frame, cfg):
    """雙重認證比例尺。主來源由 cfg['scale_pref'] 決定，另一來源作交叉驗證：
      - scale_pref="device"：mm/px = cfg['dev_mmpp']（= 9mm / device長邊px，看圖量得）。
        框長邊 = device 長邊，保證貼合。grid（grid_cell_mm/ACF格距）作交叉印出。
      - scale_pref="grid"  ：mm/px = grid_cell_mm / ACF格距。dev_mmpp（若有）作交叉印出。
    回傳 (mm_per_px, scale_src, warn)。warn 為 None 或交叉驗證偏差過大的字串。"""
    warn = None
    period = estimate_grid_period_px(frame)
    grid_mmpp = (cfg["grid_cell_mm"] / period
                 if (cfg["grid_cell_mm"] and period) else None)
    dev_mmpp = cfg["dev_mmpp"]

    if cfg["scale_pref"] == "device":
        if dev_mmpp is None:
            # 設定缺漏的保險：退回 grid
            if grid_mmpp:
                return grid_mmpp, f"grid {cfg['grid_cell_mm']}mm/{period:.0f}px (dev_mmpp 缺)", \
                       "scale_pref=device 但 dev_mmpp=None，已退回 grid"
            return 0.1, "fallback 0.1", "device 與 grid 皆無"
        src = f"device 9mm/長邊 → {dev_mmpp:.4f}"
        if grid_mmpp:
            if abs(grid_mmpp - dev_mmpp) / dev_mmpp > 0.25:
                warn = (f"交叉驗證：grid={grid_mmpp:.4f}(格{cfg['grid_cell_mm']}mm/"
                        f"{period:.0f}px) vs device={dev_mmpp:.4f} 差>25%（灰底預期，"
                        f"已採 device 確保框貼合）")
        return dev_mmpp, src, warn

    # scale_pref == "grid"
    if grid_mmpp is None:
        if dev_mmpp is not None:
            return dev_mmpp, f"device 9mm/長邊 → {dev_mmpp:.4f} (grid 失敗)", \
                   "grid 偵測失敗，已退回 device"
        return 0.1, "fallback 0.1", "grid ACF + device 皆失敗"
    src = f"grid {cfg['grid_cell_mm']}mm / {period:.0f}px = {grid_mmpp:.4f}"
    if dev_mmpp is not None and abs(dev_mmpp - grid_mmpp) / grid_mmpp > 0.25:
        warn = (f"交叉驗證：device={dev_mmpp:.4f} vs grid={grid_mmpp:.4f} 差>25%；"
                f"請人工確認格子={cfg['grid_cell_mm']}mm 正確")
    return grid_mmpp, src, warn


# ----------------------------------------------------------------------
# 1b. 人工標註讀回（混合式校正）：4 角 → 中心(px)、角度(deg)、長短軸(px)
# ----------------------------------------------------------------------
def load_corner_annotations(out_dir):
    """讀 <out_dir>/annotate/corners.json，回傳 list[dict]，每筆含
       frame, t_s, cx, cy（中心 px）, angle_deg（長軸相對 +x，與追蹤角同慣例）,
       long_px, short_px。無檔回 None。

    角度定義：取 4 角排序後（左上,右上,右下,左下），長軸向量 = 右上-左上 與
    左下-右下 的平均；angle = atan2(dy, dx)。device 長短軸亦由 4 角邊長平均得。
    """
    path = os.path.join(out_dir, "annotate", "corners.json")
    if not os.path.exists(path):
        return None
    import json
    with open(path, encoding="utf-8") as f:
        data = json.load(f)
    corners = data.get("corners", {})
    meta = data.get("frame_meta", {})
    out = []
    for k, pts in corners.items():
        P = np.array(pts, dtype=float)        # [左上,右上,右下,左下]
        if P.shape != (4, 2):
            continue
        cx, cy = P.mean(axis=0)
        # 兩條長軸邊向量（上緣、下緣）平均 → 角度
        top = P[1] - P[0]
        bot = P[2] - P[3]
        long_vec = (top + bot) / 2.0
        angle = float(np.degrees(np.arctan2(long_vec[1], long_vec[0])))
        # 長邊 = 上下緣長度平均；短邊 = 左右緣長度平均
        long_px = float((np.linalg.norm(top) + np.linalg.norm(bot)) / 2.0)
        short_px = float((np.linalg.norm(P[3] - P[0]) +
                          np.linalg.norm(P[2] - P[1])) / 2.0)
        fm = meta.get(k, {})
        out.append(dict(i=int(k), frame=int(fm.get("frame", -1)),
                        t_s=float(fm.get("t_s", np.nan)),
                        cx=float(cx), cy=float(cy), angle_deg=angle,
                        long_px=long_px, short_px=short_px))
    out.sort(key=lambda d: d["frame"])
    return out if out else None


def anchor_correct_angle(frames, angle_unwrapped, anno):
    """用人工標註的 8 個角度當 ground-truth 錨點，校正密集逐幀角度的漂移。

    作法：在每個標註幀算「人工角 − 自動角」的殘差，對整段做線性插值（端點外推
    用最近殘差），再加回自動角。如此密集曲線的形狀（細節）保留，但被拉回人工真值，
    消除模板比對的累積漂移與假尖峰造成的偏置。
    """
    fr = np.asarray(frames, dtype=float)
    a = np.array(angle_unwrapped, dtype=float)
    anno_fr, resid = [], []
    for d in anno:
        if d["frame"] < 0:
            continue
        j = int(np.argmin(np.abs(fr - d["frame"])))
        if not np.isfinite(a[j]):
            continue
        # 人工角與自動角可能差 180°（長軸方向二義性）→ 取最接近的等價角
        cand = d["angle_deg"]
        while cand - a[j] > 90:
            cand -= 180
        while cand - a[j] < -90:
            cand += 180
        anno_fr.append(fr[j])
        resid.append(cand - a[j])
    if len(anno_fr) < 2:
        return a
    corr = np.interp(fr, anno_fr, resid, left=resid[0], right=resid[-1])
    return a + corr


# ----------------------------------------------------------------------
# 2. 偵測：黃膠錨點優先 → 鎖定涵蓋黃膠的本體（排除水平儀/雜物）
# ----------------------------------------------------------------------
def _centroid(c):
    M = cv2.moments(c)
    if M["m00"] == 0:
        return None
    return M["m10"] / M["m00"], M["m01"] / M["m00"]


def find_yellow_anchor(frame_bgr):
    """回傳最大黃膠團塊的質心 (x, y)；無黃膠回 None。
    供比例尺 device 量測使用（與 find_target 的錨點偵測同一套 HSV）。"""
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, HSV_YELLOW_LO, HSV_YELLOW_HI)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    cs = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
    cs = [c for c in cs if cv2.contourArea(c) >= YELLOW_MIN_AREA]
    if not cs:
        return None
    return _centroid(max(cs, key=cv2.contourArea))


def _rotate_patch(patch, deg):
    h, w = patch.shape[:2]
    M = cv2.getRotationMatrix2D((w / 2.0, h / 2.0), deg, 1.0)
    return cv2.warpAffine(patch, M, (w, h), flags=cv2.INTER_LINEAR,
                          borderMode=cv2.BORDER_REFLECT)


def template_angle(gray_patch, template, prev_angle, coarse=COARSE_STEP_DEG,
                   span=SEARCH_SPAN_DEG):
    """像素樣板比對求旋轉角：把 template 旋轉不同角度與 gray_patch 比對（NCC），
    取最佳匹配角。以 prev_angle 為中心 ±span 搜尋；prev_angle 為 None 時全 360 掃。
    回傳最佳角（度）。"""
    if template is None or gray_patch.shape != template.shape:
        return np.nan
    if prev_angle is None or not np.isfinite(prev_angle):
        candidates = np.arange(-180, 180, coarse)
    else:
        candidates = np.arange(prev_angle - span, prev_angle + span + 1, coarse)
    best_a, best_s = np.nan, -2.0
    g = gray_patch.astype(np.float32)
    g = (g - g.mean()) / (g.std() + 1e-6)
    for a in candidates:
        rot = _rotate_patch(template, a).astype(np.float32)
        rot = (rot - rot.mean()) / (rot.std() + 1e-6)
        score = float((g * rot).mean())          # 正規化交叉相關
        if score > best_s:
            best_s, best_a = score, a
    return best_a


def find_target(frame_bgr, prev_angle=None, gate=None, gate_r=None, payload=False,
                template=None):
    """回傳 (ax, ay, angle_deg, gray_patch, anchor_ok)；偵測失敗回 None。

    策略（因 device 白色本體與白色網格背景亮度相近、HSV 難分）：
      1. 追蹤點 = 黃膠錨點質心（黃色 HSV 放寬後最穩，只有 device 有黃膠）。
      2. 角度 = 以黃膠錨點為中心取灰階 patch，與第一幀樣板做旋轉樣板比對（NCC）。
         不依賴分割，對「device 與背景亮度接近」最穩。
      3. 輪廓框在繪圖時用固定 9x6mm（不分割完整輪廓）。
    回傳的 gray_patch 供主程式在第一幀建立樣板。

    gate / gate_r：時間門控，黃膠候選離 gate 太遠者拒絕（擋水平儀的黃綠氣泡液等）。
    """
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    mask_y = cv2.inRange(hsv, HSV_YELLOW_LO, HSV_YELLOW_HI)
    mask_y = cv2.morphologyEx(mask_y, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    ys = cv2.findContours(mask_y, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
    y_cands = [(c, _centroid(c)) for c in ys if cv2.contourArea(c) >= YELLOW_MIN_AREA]
    y_cands = [(c, ct) for c, ct in y_cands if ct is not None]
    if not y_cands:
        return None

    # 選黃膠錨點：有 gate 時取門控內離 gate 最近者，否則取最大黃塊
    if gate is not None:
        y_cands.sort(key=lambda p: np.hypot(p[1][0] - gate[0], p[1][1] - gate[1]))
        within = [p for p in y_cands
                  if np.hypot(p[1][0] - gate[0], p[1][1] - gate[1]) <= (gate_r or 1e9)]
        cnt_y, anchor = (within[0] if within else y_cands[0])
    else:
        cnt_y, anchor = max(y_cands, key=lambda p: cv2.contourArea(p[0]))
    ax, ay = anchor

    # 取以錨點為中心的灰階 patch（固定大小，供樣板比對）
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    h, w = gray.shape
    rh = TEMPLATE_HALF
    x0r, y0r = int(ax) - rh, int(ay) - rh
    patch = cv2.getRectSubPix(gray, (2 * rh, 2 * rh), (float(ax), float(ay)))

    # 角度：樣板比對（template=None 表第一幀，角度定義為 0；以 patch 回傳建樣板）。
    # 角度相對第一幀樣板，可累積超過 ±90°（真實旋轉），故不做 ±90 包裹。
    if template is None:
        angle_deg = 0.0
    else:
        angle_deg = template_angle(patch, template, prev_angle)
        if not np.isfinite(angle_deg):
            angle_deg = prev_angle if prev_angle is not None else 0.0

    return (float(ax), float(ay), float(angle_deg), cnt_y, patch)


# ----------------------------------------------------------------------
# 輔助
# ----------------------------------------------------------------------
def make_kalman(x0=0.0, y0=0.0):
    kf = cv2.KalmanFilter(4, 2)
    kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1],
                                    [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
    kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
    kf.processNoiseCov = np.eye(4, dtype=np.float32) * 1e-3
    kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 1e-2
    kf.errorCovPost = np.eye(4, dtype=np.float32)
    kf.statePost = np.array([[x0], [y0], [0.0], [0.0]], np.float32)
    return kf


def moving_average(a, w):
    if w is None or w <= 1:
        return a
    a = np.asarray(a, float)
    if len(a) < w:
        return a
    out = np.full_like(a, np.nan)
    half = w // 2
    for i in range(len(a)):
        win = a[max(0, i - half):min(len(a), i + half + 1)]
        win = win[np.isfinite(win)]
        if len(win):
            out[i] = win.mean()
    return out


def moving_median(a, w):
    """大窗滑動中位數：跟得上真實緩慢變化（如 device 邊走邊微旋），同時剔除
    孤立尖刺（NCC 比對的逐幀雜訊跳動）。用於 TRANSLATION 框朝向平滑。"""
    if w is None or w <= 1:
        return np.asarray(a, float)
    a = np.asarray(a, float)
    out = np.full_like(a, np.nan)
    half = w // 2
    for i in range(len(a)):
        win = a[max(0, i - half):min(len(a), i + half + 1)]
        win = win[np.isfinite(win)]
        if len(win):
            out[i] = np.median(win)
    return out


def finite_diff(values, t, smooth_win=1):
    v = np.full_like(values, np.nan, dtype=float)
    for i in range(1, len(values)):
        if np.isfinite(values[i]) and np.isfinite(values[i - 1]) and t[i] > t[i - 1]:
            v[i] = (values[i] - values[i - 1]) / (t[i] - t[i - 1])
    return moving_average(v, smooth_win) if smooth_win and smooth_win > 1 else v


def unwrap_deg(a):
    a = np.asarray(a, float)
    if len(a) == 0:
        return a
    return np.degrees(np.unwrap(np.radians(a)))


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


# ----------------------------------------------------------------------
# 3. 雜訊過濾
# ----------------------------------------------------------------------
def mad(a):
    a = a[np.isfinite(a)]
    if len(a) == 0:
        return 0.0
    med = np.median(a)
    return 1.4826 * np.median(np.abs(a - med))


def longest_valid_run(valid):
    """回傳最長連續 True 段的 (start, end_exclusive)。用於裁掉頭尾偵測不穩段
    （影片開頭/結尾元件未進場或被遮擋造成的 NaN gap 及其邊緣跳變）。"""
    best_s = best_e = 0
    i = 0
    n = len(valid)
    while i < n:
        if valid[i]:
            j = i
            while j < n and valid[j]:
                j += 1
            if j - i > best_e - best_s:
                best_s, best_e = i, j
            i = j
        else:
            i += 1
    return best_s, best_e


def _hampel_1d(v, win, k, floor):
    """Hampel 濾波：以局部視窗中位數為基準，偏離 > k×MAD(窗) 且 > floor 的點
    視為尖刺，替補成視窗中位數。回傳 (濾波後序列, 離群旗標)。

    與「替補成前值」不同，視窗中位數會跟著真實運動移動，因此持續運動不會被
    誤殺，只有孤立尖刺（雜訊跳變）會被修掉，避免雪崩式誤標。
    """
    v = v.astype(float).copy()
    n = len(v)
    flag = np.zeros(n, dtype=bool)
    half = win // 2
    for i in range(n):
        if not np.isfinite(v[i]):
            continue
        lo, hi = max(0, i - half), min(n, i + half + 1)
        w = v[lo:hi]
        w = w[np.isfinite(w)]
        if len(w) < 3:
            continue
        med = np.median(w)
        m = 1.4826 * np.median(np.abs(w - med))
        thr = max(k * m, floor)
        if abs(v[i] - med) > thr:
            v[i] = med
            flag[i] = True
    return v, flag


def filter_position(x_px, y_px):
    """對 x、y 各做 Hampel 濾波去尖刺；回傳過濾後座標 + 離群旗標（任一軸觸發）。"""
    xf, fx = _hampel_1d(x_px, POS_HAMPEL_WIN, POS_MAD_K, POS_FLOOR_PX)
    yf, fy = _hampel_1d(y_px, POS_HAMPEL_WIN, POS_MAD_K, POS_FLOOR_PX)
    return xf, yf, (fx | fy)


def trim_tail_speed_jump(speed):
    """偵測結尾的速度暴增段（如參考 CSV 結尾跳到 66~87 mm/s），回傳有效長度。

    作法：以序列前 80% 的速度建立穩健門檻，從尾端往前找連續超標段，裁掉它。
    """
    n = len(speed)
    s = np.asarray(speed, float)
    finite = np.isfinite(s)
    if finite.sum() < 10:
        return n
    base = s[finite][:int(0.8 * finite.sum())]
    med, m = np.median(base), mad(base)
    thr = med + TAIL_SPEED_MAD_K * max(m, 1e-6)
    # 從尾端往前，連續超過門檻的就裁掉
    cut = n
    for i in range(n - 1, max(0, n - 1 - n // 3), -1):
        if np.isfinite(s[i]) and s[i] > thr:
            cut = i
        else:
            break
    return cut


# ----------------------------------------------------------------------
# 主流程：追蹤 + 過濾 + 分析 + 出圖
# ----------------------------------------------------------------------
def process(video_path):
    img_stem = os.path.splitext(os.path.basename(video_path))[0]  # 原始 IMG 編號
    cfg = _cfg(img_stem)                                          # per-video 校正
    stem = cfg["name"]                                           # 描述名（輸出用）
    out_dir = os.path.join(HERE, f"{stem}_analysis_output")
    os.makedirs(out_dir, exist_ok=True)

    cap = cv2.VideoCapture(video_path)
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    ok, first = cap.read()
    if not ok:
        print(f"  [skip] {stem}: 讀取失敗")
        return

    is_payload = cfg["payload"]

    # 複製原始影片進輸出資料夾並改描述名（根目錄仍保留原始 MOV 備份）
    if COPY_VIDEO_INTO_OUTPUT:
        import shutil
        dst = os.path.join(out_dir, f"{stem}{os.path.splitext(video_path)[1]}")
        if not os.path.exists(dst):
            shutil.copy2(video_path, dst)

    init = find_target(first, payload=is_payload)
    kf = make_kalman(*(init[:2] if init else (first.shape[1] / 2, first.shape[0] / 2)))
    # 門控半徑：追蹤點是黃膠錨點，device 不會瞬移，用固定半徑擋掉遠處雜物即可
    gate_r = GATE_RADIUS_MIN_PX

    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
    rec, box_rec = [], []
    prev_angle = None
    gate = init[:2] if init is not None else None
    template = None         # 第一幀成功偵測時建立的灰階樣板（旋轉比對用）
    fidx = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        pred = kf.predict()
        gate_pred = (float(pred[0, 0]), float(pred[1, 0])) if gate is not None else None
        meas = find_target(frame, prev_angle=prev_angle,
                           gate=gate_pred or gate, gate_r=gate_r,
                           payload=is_payload, template=template)
        if meas is not None:
            cx, cy, ang, cnt, patch = meas
            est = kf.correct(np.array([[cx], [cy]], np.float32))
            fx, fy = float(est[0, 0]), float(est[1, 0])
            gate = (fx, fy)
            if np.isfinite(ang):
                prev_angle = ang
            if template is None and patch is not None:
                template = patch        # 以第一幀 patch 當旋轉樣板（角度基準 0°）
        else:
            fx = fy = ang = np.nan
        rec.append((fidx, fidx / fps, fx, fy, ang, np.nan))
        box_rec.append((fidx, fidx / fps, fx, fy, None))
        fidx += 1
    cap.release()

    # ---- 決定比例尺 mm/px（雙重認證：格距 grid_cell_mm/ACF + device 9mm 交叉）----
    mm_per_px, scale_src, scale_warn = resolve_mm_per_px(first, cfg)
    auto_scale = scale_src
    if scale_warn:
        print(f"  [scale-warn] {stem}: {scale_warn}")

    df = pd.DataFrame(rec, columns=["frame", "t_s", "x_px_raw", "y_px_raw",
                                    "angle_deg_raw", "_mmpp_unused"])
    df["mm_per_px"] = mm_per_px      # 全片統一比例尺（迴圈後才決定）
    df.drop(columns=["_mmpp_unused"], inplace=True)

    # --- 裁掉頭尾偵測不穩段：只保留最長連續偵測主段 ---
    # 僅當主段夠長（≥ 全片 30% 且 ≥ 20 幀）才裁，避免對偵測稀疏的影片過度截斷。
    valid_raw = (np.isfinite(df["x_px_raw"].to_numpy())
                 & np.isfinite(df["y_px_raw"].to_numpy()))
    rs, re = longest_valid_run(valid_raw)
    run_len = re - rs
    if run_len >= 20 and run_len >= 0.30 * len(df):
        df = df.iloc[rs:re].reset_index(drop=True)

    # --- 位置雜訊過濾（Hampel 去尖刺）---
    xf, yf, outlier = filter_position(df["x_px_raw"].to_numpy(),
                                      df["y_px_raw"].to_numpy())
    df["x_px_filt"] = xf
    df["y_px_filt"] = yf
    df["pos_outlier"] = outlier.astype(int)

    # 位置移動平均（抑制次像素抖動，避免結尾靜止段被放大成假速度）
    xf = moving_average(xf, POS_SMOOTH_WIN)
    yf = moving_average(yf, POS_SMOOTH_WIN)
    df["x_px_filt"] = xf
    df["y_px_filt"] = yf

    # 絕對 mm → 相對起點
    x_abs = xf * mm_per_px
    y_abs = yf * mm_per_px
    valid = np.isfinite(x_abs) & np.isfinite(y_abs)
    x0 = x_abs[valid][0] if valid.any() else 0.0
    y0 = y_abs[valid][0] if valid.any() else 0.0
    df["x_mm"] = x_abs - x0
    df["y_mm"] = y_abs - y0

    # 角度 unwrap + 平滑
    araw = df["angle_deg_raw"].to_numpy()
    ma = np.isfinite(araw)
    aun = np.full_like(araw, np.nan)
    if ma.any():
        aun[ma] = unwrap_deg(araw[ma])
        aun = moving_average(aun, SMOOTH_WIN_ANGLE)
    df["angle_deg_unwrapped"] = aun

    # 速度
    t = df["t_s"].to_numpy()
    vx = finite_diff(df["x_mm"].to_numpy(), t, SMOOTH_WIN_SPEED)
    vy = finite_diff(df["y_mm"].to_numpy(), t, SMOOTH_WIN_SPEED)
    speed = np.hypot(vx, vy)

    # 靜止段速度歸零：連續多幀單幀位移 < 門檻時，視為 device 停下，把速度歸零，
    # 消除停下後的次像素抖動被放大成的假速度尖刺。
    # 門檻 = 絕對地板「且」遠低於該片運動規模（p90 step），避免把整支慢速
    # 爬行運動（如某些位移僅 ~2mm 的影片）誤殺成靜止。
    xmm, ymm = df["x_mm"].to_numpy(), df["y_mm"].to_numpy()
    stepmm = np.concatenate([[0.0], np.hypot(np.diff(xmm), np.diff(ymm))])
    p90 = np.nanpercentile(stepmm, 90) if np.isfinite(stepmm).any() else 0.0
    if p90 > 2 * STATIC_STEP_MM:           # 該片確有明顯運動，才啟用靜止歸零
        is_static = stepmm < STATIC_STEP_MM
        i, n = 0, len(is_static)
        while i < n:
            if is_static[i]:
                j = i
                while j < n and is_static[j]:
                    j += 1
                if j - i >= STATIC_MIN_RUN:
                    vx[i:j] = vy[i:j] = speed[i:j] = 0.0
                i = j
            else:
                i += 1
    df["vx_mm_s"], df["vy_mm_s"], df["speed_mm_s"] = vx, vy, speed

    # 角速度
    angvel = finite_diff(aun, t, SMOOTH_WIN_ANGLE)
    df["angular_vel_dps"] = angvel

    # --- 結尾速度大跳截斷 ---
    cut = trim_tail_speed_jump(speed)
    df["tail_trimmed"] = 0
    if cut < len(df):
        df.loc[cut:, "tail_trimmed"] = 1
    df_valid = df.iloc[:cut].copy()    # 用於繪圖/統計的乾淨段

    # --- 模式自動判斷 ---
    xx = df_valid["x_mm"].to_numpy()
    yy = df_valid["y_mm"].to_numpy()
    finite_xy = np.isfinite(xx) & np.isfinite(yy)
    if finite_xy.sum() >= 2:
        net_disp = float(np.hypot(xx[finite_xy][-1] - xx[finite_xy][0],
                                  yy[finite_xy][-1] - yy[finite_xy][0]))
        max_disp = float(np.nanmax(np.hypot(xx - xx[finite_xy][0],
                                            yy - yy[finite_xy][0])))
    else:
        net_disp = max_disp = 0.0
    au = df_valid["angle_deg_unwrapped"].to_numpy()
    au = au[np.isfinite(au)]
    net_rot = float(au[-1] - au[0]) if len(au) > 1 else 0.0

    is_rotation = (abs(net_rot) >= ROT_NET_DEG_MIN and max_disp <= ROT_DISP_MM_MAX)
    # 手動覆寫（per-video cfg["mode"]）：rotation / translation / auto
    if cfg["mode"] == "rotation":
        is_rotation = True
    elif cfg["mode"] == "translation":
        is_rotation = False
    mode = "ROTATION" if is_rotation else "TRANSLATION"

    # --- 人工標註讀回（混合式校正）---
    # 若有 annotate/corners.json：用 8 個人工角度錨定校正密集角度漂移；
    # df_valid["angle_deg_unwrapped"] 被拉回人工真值（保留形狀細節）。
    anno = load_corner_annotations(out_dir)
    if anno is not None:
        fr_all = df_valid["frame"].to_numpy()
        df_valid["angle_deg_unwrapped"] = anchor_correct_angle(
            fr_all, df_valid["angle_deg_unwrapped"].to_numpy(), anno)
        # 角速度也用校正後角度重算
        df_valid["angular_vel_dps"] = finite_diff(
            df_valid["angle_deg_unwrapped"].to_numpy(),
            df_valid["t_s"].to_numpy(), SMOOTH_WIN_ANGLE)
        print(f"  [annotate] {stem}: 採用人工標註 {len(anno)} 點校正角度")

    # --- 框朝向 ---
    # device 與背景亮度接近時，NCC 角度逐幀抖動會讓 8 個外框各自亂轉。但直走時
    # device 本身也可能邊走邊微旋（非走偏，是元件自身傾斜），不能一刀切成單一固定角。
    # 故 TRANSLATION 框朝向改用「大窗滑動中位數」：跟得上真實緩慢旋轉，又濾掉逐幀尖刺。
    # ROTATION 影片保留逐幀角度（本來就是要看旋轉）。有人工標註時直接用校正後角度。
    if anno is not None:
        df_valid["angle_for_box"] = df_valid["angle_deg_unwrapped"]
    elif not is_rotation:
        df_valid["angle_for_box"] = moving_median(
            df_valid["angle_deg_unwrapped"].to_numpy(), BOX_ANGLE_MED_WIN)
    else:
        df_valid["angle_for_box"] = df_valid["angle_deg_unwrapped"]

    # CSV
    csv_path = os.path.join(out_dir, f"{stem}_pos_angle_speed.csv")
    df.to_csv(csv_path, index=False, encoding="utf-8-sig")

    # === 出圖 ===
    _plot_position(df_valid, out_dir, stem, mode)
    if is_rotation:
        _plot_rotation(df_valid, out_dir, stem)
    else:
        _plot_speed_orientation(df_valid, out_dir, stem)
    _plot_trajectory_boxes(df_valid, box_rec, mm_per_px, x0, y0, out_dir, stem,
                           cut, cfg)
    _make_camera_composite(video_path, df_valid, mm_per_px, out_dir, stem, fps,
                           cfg, anno=anno)

    scale_str = f"{mm_per_px:.4f} mm/px ({auto_scale})"
    trimmed = len(df) - cut
    print(f"  {stem}: {mode}  | disp={max_disp:.1f}mm net_rot={net_rot:.0f}° "
          f"| {scale_str} | tail trimmed={trimmed} frames | "
          f"outliers={int(df['pos_outlier'].sum())}")


def _square_limits(x, y, extra_pts=None):
    """回傳以資料為中心、等半幅的正方形座標範圍 (xc-h, xc+h, yc-h, yc+h)。"""
    xs = [x[np.isfinite(x)]]
    ys = [y[np.isfinite(y)]]
    if extra_pts is not None and len(extra_pts):
        ep = np.asarray(extra_pts)
        xs.append(ep[:, 0]); ys.append(ep[:, 1])
    xall = np.concatenate(xs); yall = np.concatenate(ys)
    if len(xall) == 0:
        return -PLOT_MIN_HALF_MM, PLOT_MIN_HALF_MM, -PLOT_MIN_HALF_MM, PLOT_MIN_HALF_MM
    xc = 0.5 * (xall.min() + xall.max())
    yc = 0.5 * (yall.min() + yall.max())
    half = 0.5 * max(xall.max() - xall.min(), yall.max() - yall.min())
    half = max(half * PLOT_PAD_FACTOR, PLOT_MIN_HALF_MM)
    return xc - half, xc + half, yc - half, yc + half


# ---- 圖 A：Position 軌跡（統一正方形）----
def _plot_position(df, out_dir, stem, mode):
    fig, ax = plt.subplots(figsize=(SQUARE_FIG_IN, SQUARE_FIG_IN),
                           constrained_layout=True)
    x = df["x_mm"].to_numpy()
    y = df["y_mm"].to_numpy()
    if mode == "TRANSLATION" and ALIGN_TRAJ_TO_Y:
        phi = principal_direction_xy(x, y)
        x, y = rotate_xy(x, y, phi)

    ax.plot(x, y, lw=2, color="#3b6ea5", label="Trajectory")
    vp = np.vstack([x, y]).T
    vp = vp[~np.isnan(vp).any(axis=1)]
    if len(vp):
        ax.scatter([vp[0, 0]], [vp[0, 1]], s=110, c="green", zorder=5, label="Start")
        ax.scatter([vp[-1, 0]], [vp[-1, 1]], s=110, c="red", zorder=5, label="End")
    xlo, xhi, ylo, yhi = _square_limits(x, y)
    ax.set_xlim(xlo, xhi); ax.set_ylim(ylo, yhi)
    ax.invert_yaxis()    # 影像 y 向下；反向 y 軸讓圖面方向與相機視角一致（修上下顛倒/視覺左右相反）
    ax.set_aspect("equal", adjustable="box")
    ax.set_box_aspect(1)
    ax.set_xlabel("x (mm)", fontsize=22)
    ax.set_ylabel("y (mm)", fontsize=22)
    ax.set_title("Position (Trajectory)", fontsize=20)
    ax.legend(loc="best", fontsize=13)
    fig.savefig(os.path.join(out_dir, f"{stem}_position.png"),
                dpi=300, bbox_inches="tight", pad_inches=0.3)
    plt.close(fig)


# ---- 圖 B（直走）：Speed + Orientation ----
def _plot_speed_orientation(df, out_dir, stem):
    fig, (axs, axa) = plt.subplots(1, 2, figsize=(14, 6), constrained_layout=True)
    t = df["t_s"].to_numpy()

    axs.plot(t, df["speed_mm_s"], lw=2, color="#c0392b")
    sp = df["speed_mm_s"].to_numpy()
    fin = np.isfinite(sp)
    if fin.any():
        i = np.where(fin)[0][np.nanargmax(sp[fin])]
        axs.plot([t[i]], [sp[i]], "o", ms=8, color="red",
                 label=f"Max {sp[i]:.2f} mm/s @ {t[i]:.2f}s")
        axs.legend(loc="best", fontsize=13)
    axs.set_xlabel("Time (s)", fontsize=22)
    axs.set_ylabel("Speed (mm/s)", fontsize=22)
    axs.set_title("Speed vs Time", fontsize=20)

    araw = df["angle_deg_raw"].to_numpy()
    fa = np.isfinite(araw)
    if fa.any():
        off = moving_average(araw - araw[fa][0], SMOOTH_WIN_ANGLE)
        avg = np.nanmean(off[np.isfinite(off)])
        axa.plot(t, off, lw=2, color="#8a6fae",
                 label=f"Orientation offset\nAvg {avg:.2f}°")
        axa.legend(loc="best", fontsize=13)
    axa.set_xlabel("Time (s)", fontsize=22)
    axa.set_ylabel("Angle offset (deg)", fontsize=22)
    axa.set_title("Orientation vs Time", fontsize=20)
    fig.savefig(os.path.join(out_dir, f"{stem}_speed_orientation.png"),
                dpi=300, bbox_inches="tight", pad_inches=0.3)
    plt.close(fig)


# ---- 圖 B（旋轉）：Angular speed + Theta ----
def _plot_rotation(df, out_dir, stem):
    fig, (axw, axt) = plt.subplots(1, 2, figsize=(14, 6), constrained_layout=True)
    t = df["t_s"].to_numpy()

    axw.plot(t, df["angular_vel_dps"], lw=2, color="#5a9367")
    w = df["angular_vel_dps"].to_numpy()
    fin = np.isfinite(w)
    if fin.any():
        i = np.where(fin)[0][int(np.nanargmax(np.abs(w[fin])))]
        axw.plot([t[i]], [w[i]], "o", ms=8, color="red",
                 label=f"Max {w[i]:.2f} °/s @ {t[i]:.2f}s")
        axw.legend(loc="best", fontsize=13)
    axw.set_xlabel("Time (s)", fontsize=22)
    axw.set_ylabel("Angular speed (deg/s)", fontsize=22)
    axw.set_title("Angular Speed vs Time", fontsize=20)

    th = df["angle_deg_unwrapped"].to_numpy()
    ft = np.isfinite(th)
    if ft.any():
        axt.plot(t, th, lw=2, color="#3b6ea5")
        tv, thv = t[ft], th[ft]
        axt.scatter([tv[0]], [thv[0]], s=80, c="green", zorder=5,
                    label=f"Start {thv[0]:.1f}°")
        axt.scatter([tv[-1]], [thv[-1]], s=80, c="red", zorder=5,
                    label=f"End {thv[-1]:.1f}°")
        axt.set_title(f"θ vs Time (total {thv[-1]-thv[0]:.1f}°)", fontsize=20)
        axt.legend(loc="best", fontsize=13)
    else:
        axt.set_title("θ vs Time", fontsize=20)
    axt.set_xlabel("Time (s)", fontsize=22)
    axt.set_ylabel("θ (deg)", fontsize=22)
    fig.savefig(os.path.join(out_dir, f"{stem}_speed_orientation.png"),
                dpi=300, bbox_inches="tight", pad_inches=0.3)
    plt.close(fig)


def _device_rect_mm(ax_mm, ay_mm, angle_deg, off_long, off_short,
                    long_mm=DEVICE_LONG_MM, short_mm=DEVICE_SHORT_MM):
    """輸入黃膠錨點座標 (ax,ay,mm) 與本體朝向 angle_deg，回傳 long×short mm 矩形的
    4 頂點（mm，閉合 5 點）。框中心由錨點沿本體朝向偏移 off_long/off_short
    （device 局部座標：長軸 +x、短軸 +y；short 負值=影像上方）。"""
    hl, hs = long_mm / 2.0, short_mm / 2.0
    corners = np.array([[-hl, -hs], [hl, -hs], [hl, hs], [-hl, hs], [-hl, -hs]])
    a = np.radians(angle_deg if np.isfinite(angle_deg) else 0.0)
    c, s = np.cos(a), np.sin(a)
    R = np.array([[c, -s], [s, c]])
    # 黃膠在本體局部座標約 (off_long*hl, off_short*hs) → 框中心 = 錨點 - 此偏移
    off_local = np.array([hl * off_long, hs * off_short])
    center = np.array([ax_mm, ay_mm]) - R @ off_local
    rot = corners @ R.T + center
    return rot


# ---- 圖 C：軌跡 + 8 個時間點外框（固定 device 9x6mm 大小）----
def _plot_trajectory_boxes(df, box_rec, mm_per_px, x0, y0, out_dir, stem, cut,
                           cfg):
    fig, ax = plt.subplots(figsize=(SQUARE_FIG_IN, SQUARE_FIG_IN),
                           constrained_layout=True)
    x = df["x_mm"].to_numpy()
    y = df["y_mm"].to_numpy()
    # 框朝向：TRANSLATION 用固定角度（angle_for_box），ROTATION 用逐幀角度
    ang = df["angle_for_box"].to_numpy()
    olg, osh = cfg["off_long"], cfg["off_short"]
    lmm, smm = cfg["dev_long_mm"], cfg["dev_short_mm"]
    ax.plot(x, y, lw=2, color="#3b6ea5", alpha=0.75, label="Trajectory")

    # 8 個等間隔時間點，畫固定 device 尺寸的外框（中心=追蹤質心，朝向=追蹤角度）
    valid = np.where(np.isfinite(x) & np.isfinite(y))[0]
    if len(valid) >= 8:
        idxs = valid[np.linspace(0, len(valid) - 1, 8, dtype=int)]
        for k in idxs:
            rect = _device_rect_mm(x[k], y[k], ang[k], olg, osh, lmm, smm)
            ax.plot(rect[:, 0], rect[:, 1], "--", lw=1.5,
                    color=(0.5, 0.7, 1.0), alpha=0.85)
    vp = np.vstack([x, y]).T
    vp = vp[~np.isnan(vp).any(axis=1)]
    if len(vp):
        ax.scatter([vp[0, 0]], [vp[0, 1]], s=110, c="green", zorder=5, label="Start")
        ax.scatter([vp[-1, 0]], [vp[-1, 1]], s=110, c="red", zorder=5, label="End")
    # 座標範圍只用軌跡點算（與 position 圖一致），不納入外框頂點，避免被撐大
    xlo, xhi, ylo, yhi = _square_limits(x, y)
    ax.set_xlim(xlo, xhi); ax.set_ylim(ylo, yhi)
    ax.invert_yaxis()    # 與 position 圖一致：反向 y 軸符合相機視角
    ax.set_aspect("equal", adjustable="box")
    ax.set_box_aspect(1)
    ax.set_xlabel("x (mm)", fontsize=22)
    ax.set_ylabel("y (mm)", fontsize=22)
    ax.set_title("Trajectory with 8 Time Points", fontsize=18)
    ax.legend(loc="best", fontsize=13)
    fig.savefig(os.path.join(out_dir, f"{stem}_trajectory_center_only.png"),
                dpi=300, bbox_inches="tight", pad_inches=0.3)
    plt.close(fig)


# ---- 圖 D：camera composite（8 個影片畫面 + 累積軌跡 + 固定 device 外框）----
def _make_camera_composite(video_path, df, mm_per_px, out_dir, stem, fps, cfg,
                           anno=None):
    """有 anno（人工標註 4 角）時：snapshot 直接用標註幀、外框直接畫你標的四邊形
    （100% 你的真值，不再用黃膠錨點+尺寸重建，避免 2~3px 中心差與角度誤差）。
    無 anno 時：沿用自動偵測（8 等間隔幀、9×6mm 框繞錨點繪製）。"""
    frames_col = df["frame"].to_numpy().astype(int)
    xpx = df["x_px_filt"].to_numpy()
    ypx = df["y_px_filt"].to_numpy()
    # 框朝向：TRANSLATION 用固定角度（angle_for_box），ROTATION 用逐幀角度
    ang = df["angle_for_box"].to_numpy()
    olg, osh = cfg["off_long"], cfg["off_short"]
    valid = np.where(np.isfinite(xpx) & np.isfinite(ypx))[0]
    if len(valid) < 8:
        return

    # 有標註：用標註幀當 snapshot，並建立 frame→4角 對照
    anno_quad = {}
    if anno:
        snap_frames = np.array([d["frame"] for d in anno if d["frame"] >= 0],
                               dtype=int)
        import json
        cj = os.path.join(out_dir, "annotate", "corners.json")
        with open(cj, encoding="utf-8") as f:
            cdata = json.load(f)
        meta = cdata.get("frame_meta", {})
        for k, pts in cdata.get("corners", {}).items():
            fr = int(meta.get(k, {}).get("frame", -1))
            if fr >= 0:
                anno_quad[fr] = np.array(pts, dtype=np.int32)
        snap_frames = np.sort(snap_frames)
        # snapshot 中心用標註角中心（最準），對應到最近的 df 列以畫軌跡
        sel = np.array([int(np.argmin(np.abs(frames_col - fr)))
                        for fr in snap_frames])
    else:
        sel = valid[np.linspace(0, len(valid) - 1, 8, dtype=int)]
        snap_frames = frames_col[sel]
    t0 = df["t_s"].to_numpy()[valid[0]]

    # device 外框半尺寸（px）；長短軸實體 mm 由 per-video cfg 決定（payload_2 為 6×9）
    hl = (cfg["dev_long_mm"] / 2.0) / mm_per_px
    hs = (cfg["dev_short_mm"] / 2.0) / mm_per_px

    def device_box_px(ax_px, ay_px, a_deg):
        # 黃膠錨點 → 框中心偏移（與 _device_rect_mm 一致），影像 y 向下故短軸取負
        c = np.array([[-hl, -hs], [hl, -hs], [hl, hs], [-hl, hs]])
        a = np.radians(a_deg if np.isfinite(a_deg) else 0.0)
        co, si = np.cos(a), np.sin(a)
        R = np.array([[co, -si], [si, co]])
        off_local = np.array([hl * olg, -hs * osh])
        center = np.array([ax_px, ay_px]) - R @ off_local
        r = c @ R.T + center
        return np.int32(r)

    cap = cv2.VideoCapture(video_path)
    extracted = []
    for si, fnum in enumerate(snap_frames):
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(fnum))
        ok, frame = cap.read()
        if not ok:
            continue
        k0 = sel[si]              # 此 snapshot 對應的 df 列索引（當下時間點）
        # 累積軌跡（藍線）：從起點到目前 snapshot（保留，看路徑）
        pts = []
        for k in valid:
            if frames_col[k] <= fnum:
                pts.append((int(round(xpx[k])), int(round(ypx[k]))))
        for i in range(1, len(pts)):
            cv2.line(frame, pts[i - 1], pts[i], (255, 0, 0), 2)
        # 過去 snapshot 點：小灰點（淡化，不畫框，避免一堆殘留框看起來像選錯）
        # 有標註時用標註中心（與當前框同一基準），避免歷史點與框基準不一致看起來像選錯
        for j in range(si):
            pf = int(snap_frames[j])
            if pf in anno_quad:
                q = anno_quad[pf]
                gx, gy = int(q[:, 0].mean()), int(q[:, 1].mean())
            else:
                kk = sel[j]
                gx, gy = int(round(xpx[kk])), int(round(ypx[kk]))
            cv2.circle(frame, (gx, gy), 4, (180, 180, 180), -1)
        # 只畫「當下時間點」的 device 外框 + 中心點（清楚顯示此刻框住 device）
        if int(fnum) in anno_quad:
            # 直接畫你標的 4 角四邊形（你的真值）；中心 = 4 角質心
            quad = anno_quad[int(fnum)]
            cx, cy = int(round(quad[:, 0].mean())), int(round(quad[:, 1].mean()))
            cv2.polylines(frame, [quad], True, (0, 200, 255), 2)
        else:
            cx, cy = int(round(xpx[k0])), int(round(ypx[k0]))
            box = device_box_px(xpx[k0], ypx[k0], ang[k0])
            cv2.polylines(frame, [box], True, (0, 200, 255), 2)
        cv2.circle(frame, (cx, cy), 6, (255, 0, 0), -1)
        # 標註（右下，黑底）：時間 + device 框實體尺寸
        h, w = frame.shape[:2]
        font = cv2.FONT_HERSHEY_SIMPLEX
        txt = f"t={df['t_s'].to_numpy()[k0] - t0:.2f}s"
        sztxt = f"box {cfg['dev_long_mm']:.0f}x{cfg['dev_short_mm']:.0f}mm"
        (tw, th), bl = cv2.getTextSize(txt, font, 1.0, 2)
        (sw, sh), sbl = cv2.getTextSize(sztxt, font, 0.6, 2)
        bw = max(tw, sw)
        tx, ty = w - bw - 12, h - 12
        ov = frame.copy()
        cv2.rectangle(ov, (tx - 6, ty - th - sh - 16),
                      (tx + bw + 6, ty + bl + 6), (0, 0, 0), -1)
        cv2.addWeighted(ov, 0.5, frame, 0.5, 0, frame)
        cv2.putText(frame, sztxt, (tx, ty - th - 8), font, 0.6,
                    (0, 200, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, txt, (tx, ty), font, 1.0, (0, 0, 255), 2, cv2.LINE_AA)
        extracted.append(frame)
    cap.release()

    if len(extracted) == 8:
        target_h = 320
        resized = []
        for fr in extracted:
            h, w = fr.shape[:2]
            resized.append(cv2.resize(fr, (int(w * target_h / h), target_h)))
        combined = np.hstack(resized)
        cv2.imwrite(os.path.join(out_dir, f"{stem}_8_timepoints_composite.png"),
                    combined)


def export_annotation_frames(video_path, n_points=8):
    """匯出供人工標註的 8 個時間點全解析度乾淨原圖（無框）。
    存到 <stem>_analysis_output/annotate/：
       frame_{i}_t{time}.png   全解析度原幀
       manifest.json           各圖對應的 frame index 與 t(s)（讀回校正用）
    人工標註流程：對每張圖點 device 的 4 個角（順序不限），用 annotate_corners.py。
    """
    import json
    img_stem = os.path.splitext(os.path.basename(video_path))[0]
    cfg = _cfg(img_stem)
    stem = cfg["name"]
    out_dir = os.path.join(HERE, f"{stem}_analysis_output", "annotate")
    os.makedirs(out_dir, exist_ok=True)

    cap = cv2.VideoCapture(video_path)
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    if total <= 0:
        print(f"  [skip] {stem}: 讀取失敗")
        cap.release()
        return
    idxs = np.linspace(0, total - 1, n_points, dtype=int)
    manifest = {"video": os.path.basename(video_path), "fps": fps,
                "device_long_mm": cfg["dev_long_mm"],
                "device_short_mm": cfg["dev_short_mm"], "frames": []}
    for i, fidx in enumerate(idxs):
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(fidx))
        ok, frame = cap.read()
        if not ok:
            continue
        t = fidx / fps
        fname = f"frame_{i}_t{t:.2f}.png"
        cv2.imwrite(os.path.join(out_dir, fname), frame)
        manifest["frames"].append({"i": i, "frame": int(fidx), "t_s": float(t),
                                   "file": fname})
    cap.release()
    with open(os.path.join(out_dir, "manifest.json"), "w", encoding="utf-8") as f:
        json.dump(manifest, f, ensure_ascii=False, indent=2)
    print(f"  {stem}: 匯出 {len(manifest['frames'])} 張標註圖 → {out_dir}")


def main():
    args = sys.argv[1:]
    do_export = "--export-annotate" in args
    sel = [a for a in args if not a.startswith("--")]
    vids = sorted(glob.glob(os.path.join(HERE, "*.MOV")))
    if sel:
        vids = [v for v in vids if any(s in v for s in sel)]

    if do_export:
        print(f"匯出 {len(vids)} 支影片的 8 時間點標註圖（全解析度乾淨原圖）...")
        for v in vids:
            export_annotation_frames(v)
        print("完成。請用 annotate_corners.py 標註 device 4 角，再執行一般分析讀回校正。")
        return

    print(f"分析 {len(vids)} 支影片（自動判斷 直走/旋轉 + 雜訊過濾）...")
    for v in vids:
        process(v)
    print("完成。")


if __name__ == "__main__":
    main()
