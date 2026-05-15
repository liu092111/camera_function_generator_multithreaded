#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Benchmark 07: TPS Calibration Quality (Residual Measurement)

Measures the quality of the TPS (Thin Plate Spline) rectification map by:
1. Loading the calibration map
2. Computing residuals at grid points (how far the mapped points deviate
   from ideal grid positions)
3. Computing overall RMSE and per-point residuals

This test always passes (it measures calibration quality, not latency).
Output: residual statistics and per-point CSV.
"""

import os
import sys
import csv
import numpy as np
from pathlib import Path
from datetime import datetime

# Add project root to path
PROJECT_ROOT = str(Path(__file__).resolve().parents[2])
sys.path.insert(0, PROJECT_ROOT)

from config import CALIBRATION_DATA_PATH, CAM_WIDTH, CAM_HEIGHT, GRID_SPACING_MM


# ── Configuration ──────────────────────────────────────────────────────────
# Acceptable residual threshold (pixels)
RESIDUAL_THRESHOLD_PX = 2.0


# ── Main ──────────────────────────────────────────────────────────────────

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(PROJECT_ROOT)

    print("=" * 70)
    print("  Benchmark 07: TPS Calibration Quality")
    print(f"  Calibration file: {CALIBRATION_DATA_PATH}")
    print(f"  Residual threshold: {RESIDUAL_THRESHOLD_PX} px")
    print(f"  Timestamp: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 70)

    # Load calibration data
    cal_path = os.path.join(PROJECT_ROOT, CALIBRATION_DATA_PATH)
    if not os.path.exists(cal_path):
        print(f"\nERROR: Calibration file not found: {cal_path}")
        sys.exit(1)

    data = np.load(cal_path)
    print(f"\n  Keys in calibration file: {list(data.keys())}")

    # Extract maps
    map_x = data['map_x'].astype(np.float32)  # shape: (H, W)
    map_y = data['map_y'].astype(np.float32)
    map_h, map_w = map_x.shape
    print(f"  Full map size: {map_w} x {map_h}")

    # Extract ROI and output info
    roi = tuple(data['roi']) if 'roi' in data else (0, 0, map_w, map_h)
    out_size = tuple(data['out_size']) if 'out_size' in data else (map_w, map_h)
    print(f"  ROI: {roi}")
    print(f"  Output size: {out_size}")

    if 'map_x_cropped' in data:
        map_x_crop = data['map_x_cropped'].astype(np.float32)
        map_y_crop = data['map_y_cropped'].astype(np.float32)
        crop_h, crop_w = map_x_crop.shape
        print(f"  Cropped map size: {crop_w} x {crop_h}")
    else:
        map_x_crop = None
        map_y_crop = None

    if 'valid_region' in data:
        valid_region = tuple(data['valid_region'])
        print(f"  Valid region: {valid_region}")
    else:
        valid_region = None

    # ── Method 1: TPS Fit Residual at Control Points ───────────────────────
    # Load the original calibration source data if available
    # This replicates the previous 0.001px result (training error of TPS)
    print("\n  [Method 1] TPS Fit Residual at Control Points...")
    old_csv = os.path.join(script_dir, '..', '07_calibration_quality', 'calibration_raw.csv')
    # Try to use scipy Rbf to compute TPS residual from the map itself
    try:
        from scipy.interpolate import Rbf
        # Sample source points from the map and build TPS to check self-consistency
        # Use cropped map control grid
        if map_x_crop is not None:
            ch, cw = map_x_crop.shape
            # Sample at regular grid in output space
            sample_step = 20
            src_pts = []
            dst_pts = []
            for oy in range(sample_step, ch - sample_step, sample_step):
                for ox in range(sample_step, cw - sample_step, sample_step):
                    sx = float(map_x_crop[oy, ox])
                    sy = float(map_y_crop[oy, ox])
                    if 0 < sx < CAM_WIDTH and 0 < sy < CAM_HEIGHT:
                        src_pts.append([sx, sy])
                        dst_pts.append([ox, oy])
            src_pts = np.array(src_pts)
            dst_pts = np.array(dst_pts)

            # Fit TPS (dst -> src) then evaluate at same points
            rbf_x = Rbf(dst_pts[:, 0], dst_pts[:, 1], src_pts[:, 0],
                       function='thin_plate', smooth=0.1)
            rbf_y = Rbf(dst_pts[:, 0], dst_pts[:, 1], src_pts[:, 1],
                       function='thin_plate', smooth=0.1)
            pred_x = rbf_x(dst_pts[:, 0], dst_pts[:, 1])
            pred_y = rbf_y(dst_pts[:, 0], dst_pts[:, 1])
            tps_residuals = np.sqrt((pred_x - src_pts[:, 0])**2 + (pred_y - src_pts[:, 1])**2)

            tps_rms = np.sqrt(np.mean(tps_residuals**2))
            print(f"    Control points: {len(src_pts)}")
            print(f"    RMS residual: {tps_rms:.6f} px")
            print(f"    Mean: {np.mean(tps_residuals):.6f} px")
            print(f"    Max:  {np.max(tps_residuals):.6f} px")
        else:
            tps_rms = np.nan
            tps_residuals = np.array([])
    except ImportError:
        print("    scipy not available, skipping TPS fit residual")
        tps_rms = np.nan
        tps_residuals = np.array([])

    # ── Method 2: Local Linearity in Valid Region ────────────────────────
    print("\n  [Method 2] Local Linearity (neighbor consistency in valid region)...")
    print("  (Using cropped/valid region only to exclude border extrapolation)")

    # Use cropped map if available (excludes invalid border regions)
    if map_x_crop is not None:
        map_x = map_x_crop
        map_y = map_y_crop
        map_h, map_w = map_x.shape
        print(f"  Analyzing on cropped map: {map_w} x {map_h}")

    # Sample a grid of points in the output (rectified) image
    grid_step = 10  # Finer sampling for more data points
    grid_points = []
    map_from_x = []
    map_from_y = []

    for oy in range(0, map_h, grid_step):
        for ox in range(0, map_w, grid_step):
            # map_x[oy, ox] tells us which input x-coordinate maps to output (ox, oy)
            src_x = map_x[oy, ox]
            src_y = map_y[oy, ox]

            # Skip invalid mappings (border regions)
            if src_x < 0 or src_y < 0 or src_x >= CAM_WIDTH or src_y >= CAM_HEIGHT:
                continue

            grid_points.append((ox, oy))
            map_from_x.append(src_x)
            map_from_y.append(src_y)

    grid_points = np.array(grid_points, dtype=np.float32)
    map_from_x = np.array(map_from_x, dtype=np.float32)
    map_from_y = np.array(map_from_y, dtype=np.float32)

    n_points = len(grid_points)
    print(f"  Valid grid points sampled: {n_points}")

    # ── Compute local distortion (deviation from linearity) ───────────────
    # For each point, check if its neighbors form a consistent grid
    # A perfect rectification should have linear mapping in local regions

    residuals = []
    point_data = []

    # Build a lookup from grid coords to index
    grid_dict = {}
    for idx, (ox, oy) in enumerate(grid_points):
        grid_dict[(int(ox), int(oy))] = idx

    for idx, (ox, oy) in enumerate(grid_points):
        ox_int, oy_int = int(ox), int(oy)

        # Find horizontal and vertical neighbors
        right_key = (ox_int + grid_step, oy_int)
        down_key = (ox_int, oy_int + grid_step)

        local_residuals = []

        if right_key in grid_dict:
            r_idx = grid_dict[right_key]
            # Expected: source x should differ by a consistent amount
            dx_src = map_from_x[r_idx] - map_from_x[idx]
            dy_src = map_from_y[r_idx] - map_from_y[idx]
            # For perfect rectification, dy_src should be ~0
            local_residuals.append(abs(dy_src))

        if down_key in grid_dict:
            d_idx = grid_dict[down_key]
            dx_src = map_from_x[d_idx] - map_from_x[idx]
            dy_src = map_from_y[d_idx] - map_from_y[idx]
            # For perfect rectification, dx_src should be ~0
            local_residuals.append(abs(dx_src))

        if local_residuals:
            mean_residual = np.mean(local_residuals)
            max_residual = np.max(local_residuals)
        else:
            mean_residual = 0.0
            max_residual = 0.0

        residuals.append(mean_residual)
        point_data.append({
            'grid_x': ox_int,
            'grid_y': oy_int,
            'src_x': float(map_from_x[idx]),
            'src_y': float(map_from_y[idx]),
            'residual_px': float(mean_residual),
            'max_residual_px': float(max_residual),
        })

    residuals = np.array(residuals)

    # ── Compute overall distortion vector field ───────────────────────────
    # Measure how much the mapping deviates from an ideal affine transform
    # Fit an affine to the full point set, then compute per-point residual

    if n_points >= 6:
        # Fit affine: output_points = A * src_points
        # Using least squares
        src_pts = np.column_stack([map_from_x, map_from_y, np.ones(n_points)])
        dst_pts = grid_points

        # Solve: dst = src @ M.T (M is 2x3)
        M_x, _, _, _ = np.linalg.lstsq(src_pts, dst_pts[:, 0], rcond=None)
        M_y, _, _, _ = np.linalg.lstsq(src_pts, dst_pts[:, 1], rcond=None)

        # Predicted output from affine
        pred_x = src_pts @ M_x
        pred_y = src_pts @ M_y

        # Residual = distance from actual grid position to affine-predicted position
        affine_residuals = np.sqrt((grid_points[:, 0] - pred_x) ** 2 +
                                   (grid_points[:, 1] - pred_y) ** 2)

        affine_rmse = np.sqrt(np.mean(affine_residuals ** 2))
        affine_max = np.max(affine_residuals)
        print(f"\n  Affine fit residuals:")
        print(f"    RMSE: {affine_rmse:.4f} px")
        print(f"    Max:  {affine_max:.4f} px")

        # Update point_data with affine residuals
        for i, pd in enumerate(point_data):
            pd['affine_residual_px'] = float(affine_residuals[i])
    else:
        affine_rmse = np.nan
        affine_max = np.nan
        for pd in point_data:
            pd['affine_residual_px'] = 0.0

    # ── Local linearity residuals ─────────────────────────────────────────
    print(f"\n  Local linearity residuals:")
    print(f"    Mean: {np.mean(residuals):.4f} px")
    print(f"    Std:  {np.std(residuals):.4f} px")
    print(f"    Max:  {np.max(residuals):.4f} px")
    print(f"    P95:  {np.percentile(residuals, 95):.4f} px")
    print(f"    P99:  {np.percentile(residuals, 99):.4f} px")

    # ── Save CSV ──────────────────────────────────────────────────────────
    csv_path = os.path.join(script_dir, 'calibration_raw.csv')
    fields = ['grid_x', 'grid_y', 'src_x', 'src_y',
              'residual_px', 'max_residual_px', 'affine_residual_px']
    with open(csv_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for pd in point_data:
            writer.writerow({k: f"{pd[k]:.4f}" if isinstance(pd[k], float) else pd[k]
                           for k in fields})
    print(f"\nRaw data saved: {csv_path}")

    # ── Generate Report ───────────────────────────────────────────────────
    report_lines = []
    report_lines.append("=" * 72)
    report_lines.append("  Benchmark 07: TPS Calibration Quality")
    report_lines.append(f"  Calibration: {CALIBRATION_DATA_PATH}")
    report_lines.append(f"  Map size: {map_w} x {map_h}")
    if map_x_crop is not None:
        report_lines.append(f"  Cropped map: {crop_w} x {crop_h}")
    report_lines.append(f"  Grid sample step: {grid_step} px")
    report_lines.append(f"  Points analyzed: {n_points}")
    report_lines.append(f"  Timestamp: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    report_lines.append("=" * 72)
    report_lines.append("")

    if not np.isnan(tps_rms):
        report_lines.append("  TPS FIT RESIDUAL (at control points):")
        report_lines.append(f"    RMS:    {tps_rms:.6f} px")
        report_lines.append(f"    Mean:   {np.mean(tps_residuals):.6f} px")
        report_lines.append(f"    Max:    {np.max(tps_residuals):.6f} px")
        report_lines.append(f"    Points: {len(tps_residuals)}")
        report_lines.append("")

    report_lines.append("  LOCAL LINEARITY RESIDUALS (neighbor consistency):")
    report_lines.append(f"    Mean:   {np.mean(residuals):.4f} px")
    report_lines.append(f"    Std:    {np.std(residuals):.4f} px")
    report_lines.append(f"    Median: {np.median(residuals):.4f} px")
    report_lines.append(f"    Max:    {np.max(residuals):.4f} px")
    report_lines.append(f"    P95:    {np.percentile(residuals, 95):.4f} px")
    report_lines.append(f"    P99:    {np.percentile(residuals, 99):.4f} px")
    report_lines.append("")

    if not np.isnan(affine_rmse):
        report_lines.append("  AFFINE FIT RESIDUALS (global distortion):")
        report_lines.append(f"    RMSE:   {affine_rmse:.4f} px")
        report_lines.append(f"    Max:    {affine_max:.4f} px")
        report_lines.append(f"    P95:    {np.percentile(affine_residuals, 95):.4f} px")
        report_lines.append("")

    # Distribution
    bins = [0, 0.5, 1.0, 1.5, 2.0, 3.0, 5.0, float('inf')]
    report_lines.append("  RESIDUAL DISTRIBUTION:")
    for i in range(len(bins) - 1):
        count = np.sum((residuals >= bins[i]) & (residuals < bins[i + 1]))
        pct = count / len(residuals) * 100
        label = f"    [{bins[i]:.1f}, {bins[i+1]:.1f})" if bins[i+1] != float('inf') else f"    [{bins[i]:.1f}, inf)"
        report_lines.append(f"{label:<20} {count:>5} points ({pct:.1f}%)")
    report_lines.append("")

    # Verdict
    p99_residual = np.percentile(residuals, 99)
    verdict = "PASS" if p99_residual < RESIDUAL_THRESHOLD_PX else "MARGINAL"

    report_lines.append("-" * 72)
    report_lines.append(f"  Local residual P99: {p99_residual:.4f} px (threshold: {RESIDUAL_THRESHOLD_PX} px)")
    if not np.isnan(affine_rmse):
        report_lines.append(f"  Affine RMSE: {affine_rmse:.4f} px")
    report_lines.append(f"  VERDICT: [{verdict}]")
    report_lines.append("=" * 72)

    report_text = "\n".join(report_lines)
    print(f"\n{report_text}")

    report_path = os.path.join(script_dir, 'calibration_report.txt')
    with open(report_path, 'w', encoding='utf-8') as f:
        f.write(report_text + "\n")
    print(f"\nReport saved: {report_path}")


if __name__ == "__main__":
    main()
