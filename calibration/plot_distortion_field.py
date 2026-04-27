"""
畸變向量場圖 — 從 TPS remap map 產生
輸出：fig_4_tps_distortion_vector_field.png（論文用高解析度）
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import os

# ── 路徑設定 ────────────────────────────────────────────────
NPZ_PATH = r"C:\Users\User\Desktop\Git Repository\camera_function_generator_multithreaded\calibration\tps_rectification_map.npz"
OUT_PATH  = r"C:\Users\User\Desktop\Git Repository\camera_function_generator_multithreaded\calibration\fig_4_tps_distortion_vector_field.png"
# ────────────────────────────────────────────────────────────

# ── 參數（可調整） ───────────────────────────────────────────
STEP        = 20    # 每隔幾個像素畫一個箭頭（越小越密）
SCALE       = 0.4   # 箭頭放大倍率（越大箭頭越長，視覺上更明顯）
ARROW_COLOR = "#d62728"   # 箭頭顏色（論文常用紅色）
BG_COLOR    = "#f7f7f7"   # 背景色
# ────────────────────────────────────────────────────────────

# 讀取 remap map
data  = np.load(NPZ_PATH)
map_x = data["map_x"]   # shape (480, 640)
map_y = data["map_y"]   # shape (480, 640)
H, W  = map_x.shape     # 480, 640

print(f"map_x shape: {map_x.shape}")
print(f"map_y shape: {map_y.shape}")

# 建立像素格點
xs = np.arange(0, W, STEP)
ys = np.arange(0, H, STEP)
gx, gy = np.meshgrid(xs, ys)   # 採樣點的「校正後座標」

# 取得對應的 map 值（原始影像座標）
src_x = map_x[gy, gx]   # 原始 x
src_y = map_y[gy, gx]   # 原始 y

# 畸變向量 = 原始座標 - 校正後座標
#  正值 → 原始點在校正點右邊（校正時把像素向左移）
dx = src_x - gx
dy = src_y - gy

# 向量長度（用於統計）
magnitude = np.sqrt(dx**2 + dy**2)
print(f"\n畸變向量統計（像素）：")
print(f"  平均幅度：{magnitude.mean():.3f} px")
print(f"  最大幅度：{magnitude.max():.3f} px")
print(f"  最小幅度：{magnitude.min():.3f} px")

# ── 繪圖 ─────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(9, 6.5), facecolor="white")
ax.set_facecolor(BG_COLOR)

# 淡灰色格線（模擬影像座標背景）
for x in range(0, W + 1, 80):
    ax.axvline(x, color="#cccccc", linewidth=0.4, zorder=0)
for y in range(0, H + 1, 60):
    ax.axhline(y, color="#cccccc", linewidth=0.4, zorder=0)

# 畫箭頭（quiver）
Q = ax.quiver(
    gx, gy,
    dx * SCALE, -dy * SCALE,   # y 軸在影像座標中向下，matplotlib 向上，需取負
    magnitude,                  # 用幅度決定顏色
    cmap="Reds",
    clim=[0, magnitude.max()],
    angles="xy",
    scale_units="xy",
    scale=1,
    width=0.003,
    headwidth=4,
    headlength=5,
    headaxislength=4,
    zorder=3
)

# colorbar
cbar = fig.colorbar(Q, ax=ax, fraction=0.03, pad=0.02)
cbar.set_label("Distortion magnitude (pixels)", fontsize=11)
cbar.ax.tick_params(labelsize=9)

# 邊框標示影像範圍
ax.set_xlim(0, W)
ax.set_ylim(H, 0)   # 影像座標 y 向下
ax.set_xlabel("u  (pixels)", fontsize=12)
ax.set_ylabel("v  (pixels)", fontsize=12)
ax.set_title(
    "TPS Lens Distortion Vector Field\n"
    "(arrows show displacement from corrected to original image coordinates)",
    fontsize=11, pad=10
)
ax.tick_params(labelsize=9)

# 說明文字框
info_text = (
    f"Image size: {W}×{H} px\n"
    f"Sampling: every {STEP} px\n"
    f"Mean distortion: {magnitude.mean():.2f} px\n"
    f"Max distortion:  {magnitude.max():.2f} px"
)
ax.text(
    0.02, 0.97, info_text,
    transform=ax.transAxes,
    fontsize=8.5,
    verticalalignment="top",
    bbox=dict(boxstyle="round,pad=0.4", facecolor="white", alpha=0.85, edgecolor="#aaaaaa")
)

plt.tight_layout()
plt.savefig(OUT_PATH, dpi=300, bbox_inches="tight")
print(f"\n圖已儲存：{OUT_PATH}")
plt.show()