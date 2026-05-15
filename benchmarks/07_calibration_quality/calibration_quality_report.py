#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
calibration_quality_report.py — TPS Grid Rectification Quality Report

This system uses TPS (Thin Plate Spline) rectification with a printed grid
pattern, NOT traditional chessboard/OpenCV camera calibration.

Quality metrics:
  1. TPS fit residual: how accurately detected grid points map to ideal positions
  2. Post-rectification grid uniformity: spacing consistency (CV%)
  3. Post-rectification line straightness: RMS deviation from fitted lines
  4. Center-edge scale consistency
  5. Visual comparison: before/after rectification with annotated points
"""

import os
import sys
import csv
import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.chdir(PROJECT_ROOT)
sys.path.insert(0, PROJECT_ROOT)

import cv2
from datetime import datetime
from scipy.interpolate import Rbf

CALIB_DIR = os.path.join(PROJECT_ROOT, 'calibration')
NPZ_PATH = os.path.join(CALIB_DIR, 'tps_rectification_map.npz')
ORIG_PATH = os.path.join(CALIB_DIR, 'original.png')
RECT_PATH = os.path.join(CALIB_DIR, 'rectified.png')
VIZ_DIR = os.path.join(PROJECT_ROOT, 'benchmarks', 'calibration_viz')
GRID_SPACING_MM = 5.0


# ── Grid Detection (same as calibration.py) ──────────────────

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


def detect_grid_intersections(gray):
    """Detect grid intersections with sub-pixel accuracy."""
    gray_blur = cv2.GaussianBlur(gray, (3, 3), 0)
    bin_img = cv2.adaptiveThreshold(
        gray_blur, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 31, -5
    )

    h_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (40, 1))
    v_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 40))
    h_lines = cv2.morphologyEx(bin_img, cv2.MORPH_OPEN, h_kernel, iterations=1)
    v_lines = cv2.morphologyEx(bin_img, cv2.MORPH_OPEN, v_kernel, iterations=1)
    h_lines = cv2.dilate(h_lines, cv2.getStructuringElement(cv2.MORPH_RECT, (1, 3)), 1)
    v_lines = cv2.dilate(v_lines, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 1)), 1)

    intersections = cv2.bitwise_and(h_lines, v_lines)
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(intersections)

    pts = []
    for i in range(1, num_labels):
        area = stats[i, cv2.CC_STAT_AREA]
        if 3 <= area <= 800:
            pts.append(centroids[i])
    pts = np.array(pts, dtype=np.float32)

    if len(pts) > 0:
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
        pts = cv2.cornerSubPix(gray, pts, (5, 5), (-1, -1), criteria)

    return pts


def sort_grid(pts):
    """Sort detected points into a rectangular grid."""
    ys = np.sort(pts[:, 1])
    dy = np.diff(ys)
    dy = dy[(dy > 2) & (dy < 60)]
    step_y = np.median(dy) if len(dy) > 0 else 20

    xs = np.sort(pts[:, 0])
    dx = np.diff(xs)
    dx = dx[(dx > 2) & (dx < 60)]
    step_x = np.median(dx) if len(dx) > 0 else 20

    tol_y = max(5.0, step_y * 0.4)
    y_groups = cluster_1d(pts[:, 1], tol=tol_y)

    rows = []
    for g in y_groups:
        row_pts = pts[g]
        row_pts = row_pts[np.argsort(row_pts[:, 0])]
        rows.append(row_pts)
    rows = sorted(rows, key=lambda r: np.mean(r[:, 1]))

    row_lens = np.array([len(r) for r in rows])
    target_cols = int(np.median(row_lens))

    rows2 = [r for r in rows if len(r) >= max(6, int(target_cols * 0.8))]
    if len(rows2) < 5:
        rows2 = [r for r in rows if len(r) >= max(4, int(target_cols * 0.6))]
    target_cols = int(np.median([len(r) for r in rows2]))

    grid = []
    for r in rows2:
        if len(r) > target_cols:
            start = (len(r) - target_cols) // 2
            r = r[start:start + target_cols]
        elif len(r) < target_cols:
            continue
        grid.append(r)

    grid = np.stack(grid, axis=0)
    return grid, step_x, step_y


def build_ideal_grid(grid, step, out_w, out_h):
    """Build ideal rectangular grid target points."""
    R, C = grid.shape[0], grid.shape[1]
    grid_w = (C - 1) * step
    grid_h = (R - 1) * step
    off_x = (out_w - grid_w) / 2.0
    off_y = (out_h - grid_h) / 2.0

    dest = np.zeros_like(grid, dtype=np.float32)
    for i in range(R):
        for j in range(C):
            dest[i, j, 0] = off_x + j * step
            dest[i, j, 1] = off_y + i * step
    return dest


# ── Quality Metrics ──────────────────────────────────────────

def compute_tps_residual(src_pts, dst_pts, smooth=0.1):
    """
    Build TPS from dst->src, then evaluate at dst points.
    Residual = distance between TPS(dst) and actual src.
    """
    X, Y = dst_pts[:, 0], dst_pts[:, 1]
    x, y = src_pts[:, 0], src_pts[:, 1]

    rbf_x = Rbf(X, Y, x, function="thin_plate", smooth=smooth)
    rbf_y = Rbf(X, Y, y, function="thin_plate", smooth=smooth)

    pred_x = rbf_x(X, Y)
    pred_y = rbf_y(X, Y)

    residuals = np.sqrt((pred_x - x)**2 + (pred_y - y)**2)
    return residuals, rbf_x, rbf_y


def compute_post_rect_metrics(rect_img):
    """Compute post-rectification quality metrics on the rectified image."""
    if len(rect_img.shape) == 3:
        gray = cv2.cvtColor(rect_img, cv2.COLOR_BGR2GRAY)
    else:
        gray = rect_img

    pts = detect_grid_intersections(gray)
    if len(pts) < 30:
        return None

    grid, step_x, step_y = sort_grid(pts)
    R, C = grid.shape[0], grid.shape[1]

    # Spacing consistency
    dx = grid[:, 1:, 0] - grid[:, :-1, 0]
    dy_grid = grid[1:, :, 1] - grid[:-1, :, 1]
    mean_dx = float(np.mean(dx))
    mean_dy = float(np.mean(dy_grid))
    cv_x = float(np.std(dx) / mean_dx) if mean_dx > 0 else 0
    cv_y = float(np.std(dy_grid) / mean_dy) if mean_dy > 0 else 0

    # Line straightness
    def line_rms(points):
        x, y = points[:, 0], points[:, 1]
        A = np.vstack([x, np.ones_like(x)]).T
        a, b = np.linalg.lstsq(A, y, rcond=None)[0]
        return float(np.sqrt(np.mean((y - (a * x + b))**2)))

    def vline_rms(points):
        x, y = points[:, 0], points[:, 1]
        A = np.vstack([y, np.ones_like(y)]).T
        c, d = np.linalg.lstsq(A, x, rcond=None)[0]
        return float(np.sqrt(np.mean((x - (c * y + d))**2)))

    rms_h_list = [line_rms(grid[i, :, :]) for i in range(R)]
    rms_v_list = [vline_rms(grid[:, j, :]) for j in range(C)]
    rms_h = float(np.mean(rms_h_list))
    rms_v = float(np.mean(rms_v_list))

    # Center-edge scale
    r0, r1 = int(R * 0.35), int(R * 0.65)
    c0, c1 = int(C * 0.35), int(C * 0.65)
    dx_center = dx[r0:r1, c0:c1-1] if c1 > c0+1 else dx[r0:r1, :]
    k = 2
    dx_edge = np.concatenate([dx[:k, :].ravel(), dx[-k:, :].ravel(),
                              dx[:, :k].ravel(), dx[:, -k:].ravel()])
    escale_x = abs(float(np.mean(dx_edge)) - float(np.mean(dx_center))) / float(np.mean(dx_center)) if float(np.mean(dx_center)) > 0 else 0

    dbar = (mean_dx + mean_dy) / 2.0
    E_total = 100.0 * np.sqrt(cv_x**2 + cv_y**2 + (rms_h / dbar)**2 + (rms_v / dbar)**2)

    return {
        'R': R, 'C': C, 'n_points': R * C,
        'mean_dx': mean_dx, 'mean_dy': mean_dy,
        'cv_x': cv_x, 'cv_y': cv_y,
        'rms_h': rms_h, 'rms_v': rms_v,
        'rms_h_list': rms_h_list, 'rms_v_list': rms_v_list,
        'escale_x': escale_x,
        'E_total': E_total,
        'grid': grid,
    }


# ── Visualization ────────────────────────────────────────────

def save_viz(orig_img, src_pts, dst_pts, residuals, rect_img, post_metrics, viz_dir):
    os.makedirs(viz_dir, exist_ok=True)

    # 1. Original + detected points
    vis1 = orig_img.copy()
    if len(vis1.shape) == 2:
        vis1 = cv2.cvtColor(vis1, cv2.COLOR_GRAY2BGR)
    for pt in src_pts.astype(int):
        cv2.circle(vis1, tuple(pt), 3, (0, 255, 0), -1)
    cv2.imwrite(os.path.join(viz_dir, 'original_detected_points.png'), vis1)

    # 2. Original + residual vectors (src->ideal via TPS)
    vis2 = orig_img.copy()
    if len(vis2.shape) == 2:
        vis2 = cv2.cvtColor(vis2, cv2.COLOR_GRAY2BGR)
    max_res = max(np.max(residuals), 0.01)
    for i in range(len(src_pts)):
        pt = tuple(src_pts[i].astype(int))
        r = residuals[i]
        color_val = min(255, int(r / max_res * 255))
        color = (0, 255 - color_val, color_val)  # green=low, red=high
        cv2.circle(vis2, pt, 4, color, -1)
    cv2.imwrite(os.path.join(viz_dir, 'tps_residual_heatmap.png'), vis2)

    # 3. Rectified + detected grid
    if rect_img is not None and post_metrics is not None:
        vis3 = rect_img.copy()
        if len(vis3.shape) == 2:
            vis3 = cv2.cvtColor(vis3, cv2.COLOR_GRAY2BGR)
        grid = post_metrics['grid']
        R, C = grid.shape[0], grid.shape[1]
        for i in range(R):
            for j in range(C):
                pt = tuple(grid[i, j].astype(int))
                cv2.circle(vis3, pt, 2, (0, 255, 0), -1)
            pts_row = grid[i, :, :].astype(int)
            for k in range(C - 1):
                cv2.line(vis3, tuple(pts_row[k]), tuple(pts_row[k+1]), (255, 0, 0), 1)
        for j in range(C):
            pts_col = grid[:, j, :].astype(int)
            for k in range(R - 1):
                cv2.line(vis3, tuple(pts_col[k]), tuple(pts_col[k+1]), (255, 0, 0), 1)
        cv2.imwrite(os.path.join(viz_dir, 'rectified_grid_overlay.png'), vis3)

    # 4. Side-by-side comparison
    h1 = orig_img.shape[0]
    h2 = rect_img.shape[0] if rect_img is not None else h1
    max_h = max(h1, h2)

    left = orig_img.copy()
    if len(left.shape) == 2:
        left = cv2.cvtColor(left, cv2.COLOR_GRAY2BGR)
    pad_left = np.zeros((max_h, left.shape[1], 3), dtype=np.uint8)
    pad_left[:h1, :, :] = left

    if rect_img is not None:
        right = rect_img.copy()
        if len(right.shape) == 2:
            right = cv2.cvtColor(right, cv2.COLOR_GRAY2BGR)
        pad_right = np.zeros((max_h, right.shape[1], 3), dtype=np.uint8)
        pad_right[:h2, :, :] = right
        sep = np.ones((max_h, 4, 3), dtype=np.uint8) * 128
        combined = np.hstack([pad_left, sep, pad_right])
        cv2.putText(combined, "Original", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(combined, "Rectified", (left.shape[1] + 14, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imwrite(os.path.join(viz_dir, 'before_after_comparison.png'), combined)


# ── Main ─────────────────────────────────────────────────────

def main():
    print("=" * 64)
    print("  Camera Calibration Quality Report")
    print("  Method: TPS (Thin Plate Spline) Grid Rectification")
    print("=" * 64)

    # ── Load original image ──
    orig_img = cv2.imread(ORIG_PATH)
    if orig_img is None:
        print(f"ERROR: Cannot read {ORIG_PATH}")
        sys.exit(1)
    h, w = orig_img.shape[:2]
    print(f"\nOriginal image: {ORIG_PATH}")
    print(f"  Size: {w}x{h}")

    gray = cv2.cvtColor(orig_img, cv2.COLOR_BGR2GRAY)

    # ── Step 1: Detect grid intersections ──
    print("\n[Step 1] Detecting grid intersections...")
    pts = detect_grid_intersections(gray)
    print(f"  Detected: {len(pts)} intersections (sub-pixel refined)")

    if len(pts) < 30:
        print("  ERROR: Too few intersections. Cannot proceed.")
        sys.exit(1)

    # ── Step 2: Sort into grid ──
    print("\n[Step 2] Sorting into grid...")
    grid, step_x, step_y = sort_grid(pts)
    R, C = grid.shape[0], grid.shape[1]
    step = float(np.mean([step_x, step_y]))
    print(f"  Grid: {R} rows x {C} cols = {R*C} points")
    print(f"  Estimated spacing: step_x={step_x:.2f} px, step_y={step_y:.2f} px")

    mm_per_px = GRID_SPACING_MM / step
    print(f"  Derived mm/px: {mm_per_px:.4f} ({GRID_SPACING_MM} mm / {step:.2f} px)")

    # ── Step 3: Build ideal grid and compute TPS residual ──
    print("\n[Step 3] Computing TPS fit residual...")
    ideal = build_ideal_grid(grid, step, w, h)

    src_pts = grid.reshape(-1, 2)
    dst_pts = ideal.reshape(-1, 2)

    # Add edge anchor points (same as calibration.py)
    margin = 10
    edge_src, edge_dst = [], []
    corners = [(margin, margin), (w - margin, margin),
               (margin, h - margin), (w - margin, h - margin)]
    for cx, cy in corners:
        edge_src.append([cx, cy])
        edge_dst.append([cx, cy])
    num_edge = 5
    for i in range(num_edge):
        t = (i + 1) / (num_edge + 1)
        for ex, ey in [(margin + t * (w - 2*margin), margin),
                       (margin + t * (w - 2*margin), h - margin),
                       (margin, margin + t * (h - 2*margin)),
                       (w - margin, margin + t * (h - 2*margin))]:
            edge_src.append([ex, ey])
            edge_dst.append([ex, ey])

    edge_src = np.array(edge_src, dtype=np.float32)
    edge_dst = np.array(edge_dst, dtype=np.float32)

    all_src = np.vstack([src_pts, edge_src])
    all_dst = np.vstack([dst_pts, edge_dst])

    n_grid = len(src_pts)
    n_edge = len(edge_src)
    n_total = len(all_src)
    print(f"  Control points: {n_grid} grid + {n_edge} edge anchors = {n_total} total")

    residuals_all, rbf_x, rbf_y = compute_tps_residual(all_src, all_dst, smooth=0.1)
    residuals_grid = residuals_all[:n_grid]

    rms_residual = float(np.sqrt(np.mean(residuals_grid**2)))
    mean_residual = float(np.mean(residuals_grid))
    max_residual = float(np.max(residuals_grid))
    min_residual = float(np.min(residuals_grid))
    std_residual = float(np.std(residuals_grid))

    print(f"  TPS fit RMS residual (grid points only): {rms_residual:.4f} px")
    print(f"  Mean: {mean_residual:.4f} px, Max: {max_residual:.4f} px")

    # ── Step 4: Post-rectification quality ──
    print("\n[Step 4] Analyzing post-rectification quality...")
    rect_img = cv2.imread(RECT_PATH)
    post = None
    if rect_img is not None:
        post = compute_post_rect_metrics(rect_img)
        if post:
            print(f"  Post-rect grid: {post['R']}x{post['C']} = {post['n_points']} points")
            print(f"  Spacing consistency: CV_x={post['cv_x']*100:.2f}%, CV_y={post['cv_y']*100:.2f}%")
            print(f"  Line straightness: RMS_h={post['rms_h']:.3f} px, RMS_v={post['rms_v']:.3f} px")
            print(f"  Center-edge scale diff: {post['escale_x']*100:.2f}%")
            print(f"  Total error index: {post['E_total']:.2f}%")
        else:
            print("  WARNING: Could not detect grid in rectified image")
    else:
        print(f"  WARNING: Rectified image not found at {RECT_PATH}")

    # ── Step 5: Load NPZ and verify ──
    print("\n[Step 5] Verifying TPS map file...")
    npz_data = np.load(NPZ_PATH)
    map_x = npz_data['map_x']
    map_y = npz_data['map_y']
    roi = npz_data['roi']
    out_size = npz_data['out_size']
    valid_region = npz_data['valid_region']
    crop_margin = int(npz_data['crop_margin'])

    # Distortion magnitude stats
    gx, gy = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))
    dist_mag = np.sqrt((map_x - gx)**2 + (map_y - gy)**2)

    print(f"  Map shape: {map_x.shape}")
    print(f"  ROI: {roi}")
    print(f"  Output size: {out_size}")
    print(f"  Valid region: {valid_region}")
    print(f"  Distortion magnitude: mean={dist_mag.mean():.2f} px, max={dist_mag.max():.2f} px")

    # ── Step 6: Per-point CSV data ──
    per_point_data = []
    for i in range(n_grid):
        r_idx = i // C
        c_idx = i % C
        per_point_data.append({
            'point_id': i,
            'row': r_idx, 'col': c_idx,
            'src_x': float(src_pts[i, 0]), 'src_y': float(src_pts[i, 1]),
            'dst_x': float(dst_pts[i, 0]), 'dst_y': float(dst_pts[i, 1]),
            'residual_px': float(residuals_grid[i]),
        })

    # ── Step 7: Visualizations ──
    print("\n[Step 6] Generating visualizations...")
    save_viz(orig_img, src_pts, dst_pts, residuals_grid, rect_img, post, VIZ_DIR)
    print(f"  Saved to: {VIZ_DIR}/")

    # ── Quality verdict ──
    if rms_residual < 0.3:
        fit_grade = "Excellent (< 0.3 px)"
    elif rms_residual < 0.5:
        fit_grade = "Good (< 0.5 px)"
    elif rms_residual < 1.0:
        fit_grade = "Acceptable (< 1.0 px)"
    else:
        fit_grade = "Needs improvement (> 1.0 px)"

    if post and post['E_total'] < 2.0:
        rect_grade = "Excellent (< 2%)"
    elif post and post['E_total'] < 5.0:
        rect_grade = "Good (< 5%)"
    elif post:
        rect_grade = "Needs improvement (> 5%)"
    else:
        rect_grade = "N/A"

    # ── Report ──
    r = []
    r.append("=" * 68)
    r.append("  Camera Calibration Quality Report")
    r.append("  Camera: Arducam B0332 OV9281 Monochrome 640x480")
    r.append("  Method: TPS (Thin Plate Spline) Grid Rectification")
    r.append(f"  Grid: {GRID_SPACING_MM} mm spacing")
    r.append(f"  Timestamp: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    r.append("=" * 68)

    r.append("")
    r.append("[Source Image]")
    r.append(f"  File: {ORIG_PATH}")
    r.append(f"  Size: {w}x{h} px")

    r.append("")
    r.append("[Grid Detection]")
    r.append(f"  Intersections detected: {len(pts)} (sub-pixel refined)")
    r.append(f"  Grid structure: {R} rows x {C} cols = {R*C} points")
    r.append(f"  Estimated spacing: x={step_x:.2f} px, y={step_y:.2f} px, avg={step:.2f} px")
    r.append(f"  Derived scale: {mm_per_px:.4f} mm/px ({GRID_SPACING_MM} mm grid)")

    r.append("")
    r.append("[TPS Fit Quality]")
    r.append(f"  Control points: {n_grid} grid + {n_edge} edge anchors = {n_total} total")
    r.append(f"  TPS smooth parameter: 0.1")
    r.append(f"  RMS residual (grid points): {rms_residual:.4f} px ({rms_residual * mm_per_px:.4f} mm)")
    r.append(f"  Mean residual:              {mean_residual:.4f} px ({mean_residual * mm_per_px:.4f} mm)")
    r.append(f"  Std residual:               {std_residual:.4f} px ({std_residual * mm_per_px:.4f} mm)")
    r.append(f"  Min residual:               {min_residual:.4f} px")
    r.append(f"  Max residual:               {max_residual:.4f} px ({max_residual * mm_per_px:.4f} mm)")
    r.append(f"  Grade: {fit_grade}")

    high_res = [(i, float(residuals_grid[i])) for i in range(n_grid) if residuals_grid[i] > 1.0]
    if high_res:
        r.append(f"  Points with residual > 1.0 px: {len(high_res)}")
        for idx, val in sorted(high_res, key=lambda x: -x[1])[:10]:
            ri, ci = idx // C, idx % C
            r.append(f"    Point ({ri},{ci}): {val:.4f} px")
    else:
        r.append(f"  Points with residual > 1.0 px: 0")

    if post:
        r.append("")
        r.append("[Post-Rectification Grid Quality]")
        r.append(f"  Grid detected: {post['R']}x{post['C']} = {post['n_points']} points")
        r.append(f"  Mean spacing: dx={post['mean_dx']:.3f} px, dy={post['mean_dy']:.3f} px")
        r.append(f"  Spacing consistency: CV_x={post['cv_x']*100:.2f}%, CV_y={post['cv_y']*100:.2f}%")
        r.append(f"  Line straightness:")
        r.append(f"    Horizontal RMS: {post['rms_h']:.4f} px ({post['rms_h'] * mm_per_px:.4f} mm)")
        r.append(f"    Vertical RMS:   {post['rms_v']:.4f} px ({post['rms_v'] * mm_per_px:.4f} mm)")
        r.append(f"  Center-edge scale difference: {post['escale_x']*100:.2f}%")
        r.append(f"  Total error index: {post['E_total']:.2f}%")
        r.append(f"  Grade: {rect_grade}")

    r.append("")
    r.append("[TPS Distortion Map Statistics]")
    r.append(f"  Map shape: {map_x.shape[1]}x{map_x.shape[0]} px")
    r.append(f"  Cropped output: {valid_region[2]}x{valid_region[3]} px")
    r.append(f"  Crop margin: {crop_margin} px")
    r.append(f"  Distortion magnitude:")
    r.append(f"    Mean: {dist_mag.mean():.2f} px ({dist_mag.mean() * mm_per_px:.3f} mm)")
    r.append(f"    Max:  {dist_mag.max():.2f} px ({dist_mag.max() * mm_per_px:.3f} mm)")
    r.append(f"    Std:  {dist_mag.std():.2f} px")

    r.append("")
    r.append("[Visualizations]")
    r.append(f"  {VIZ_DIR}/original_detected_points.png")
    r.append(f"  {VIZ_DIR}/tps_residual_heatmap.png")
    r.append(f"  {VIZ_DIR}/rectified_grid_overlay.png")
    r.append(f"  {VIZ_DIR}/before_after_comparison.png")

    r.append("")
    r.append("=" * 68)
    r.append("  CONCLUSION")
    r.append("=" * 68)
    r.append(f"  TPS fit RMS residual: {rms_residual:.4f} px ({fit_grade})")
    if post:
        r.append(f"  Post-rectification error: {post['E_total']:.2f}% ({rect_grade})")
        r.append(f"  Straightness after correction: H={post['rms_h']:.4f} px, V={post['rms_v']:.4f} px")
    thesis_ok = rms_residual < 0.5 and (post is None or post['E_total'] < 5.0)
    r.append(f"  Meets thesis standard (RMS < 0.5 px): {'Yes' if rms_residual < 0.5 else 'No'}")
    r.append("=" * 68)

    report = "\n".join(r)
    print(f"\n{report}")

    # Save report
    out_dir = os.path.join(PROJECT_ROOT, 'benchmarks')
    txt_path = os.path.join(out_dir, 'calibration_report.txt')
    csv_path = os.path.join(out_dir, 'calibration_raw.csv')

    with open(txt_path, 'w', encoding='utf-8') as f:
        f.write(report + "\n")

    with open(csv_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=['point_id', 'row', 'col',
                                               'src_x', 'src_y', 'dst_x', 'dst_y',
                                               'residual_px'])
        writer.writeheader()
        writer.writerows(per_point_data)

    print(f"\nReport: {txt_path}")
    print(f"Raw CSV: {csv_path}")
    print(f"Viz: {VIZ_DIR}/")


if __name__ == "__main__":
    main()
