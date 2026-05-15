#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
capture_calibration_images.py — Interactive grid pattern image capture

Opens the Arducam B0332, detects grid intersections in real-time,
and lets the user save frames by pressing SPACE.
"""

import os
import sys
import cv2
import numpy as np
from datetime import datetime

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PROJECT_ROOT)

from config import CAMERA_INDEX, CAM_WIDTH, CAM_HEIGHT, CAM_FPS_REQ

SAVE_DIR = os.path.join(PROJECT_ROOT, 'calibration', 'calib_images')


def detect_grid_fast(gray):
    """Fast grid detection for live preview."""
    blur = cv2.GaussianBlur(gray, (3, 3), 0)
    bw = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                               cv2.THRESH_BINARY, 31, -5)
    hk = cv2.getStructuringElement(cv2.MORPH_RECT, (40, 1))
    vk = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 40))
    hl = cv2.morphologyEx(bw, cv2.MORPH_OPEN, hk)
    vl = cv2.morphologyEx(bw, cv2.MORPH_OPEN, vk)
    hl = cv2.dilate(hl, cv2.getStructuringElement(cv2.MORPH_RECT, (1, 3)))
    vl = cv2.dilate(vl, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 1)))
    ix = cv2.bitwise_and(hl, vl)
    n, _, stats, centroids = cv2.connectedComponentsWithStats(ix)
    pts = []
    for i in range(1, n):
        if 3 <= stats[i, cv2.CC_STAT_AREA] <= 800:
            pts.append(centroids[i])
    return np.array(pts, dtype=np.float32) if pts else np.array([], dtype=np.float32).reshape(0, 2)


def main():
    os.makedirs(SAVE_DIR, exist_ok=True)
    existing = [f for f in os.listdir(SAVE_DIR) if f.endswith('.png')]

    print("=" * 56)
    print("  Grid Pattern Image Capture")
    print(f"  Save to: {SAVE_DIR}")
    print(f"  Existing images: {len(existing)}")
    print("=" * 56)
    print("\n  SPACE = save frame")
    print("  Q     = quit")
    print("\n  Tips:")
    print("  - Fill at least 60% of the frame with the grid")
    print("  - Capture from different angles and positions")
    print("  - Avoid motion blur (hold still when pressing SPACE)")
    print("  - Aim for 20+ images for good calibration")

    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, CAM_FPS_REQ)

    saved_count = len(existing)
    next_idx = saved_count + 1

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if len(frame.shape) == 3 else frame
        pts = detect_grid_fast(gray)

        display = frame.copy() if len(frame.shape) == 3 else cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        if len(pts) > 0:
            for pt in pts.astype(int):
                cv2.circle(display, tuple(pt), 2, (0, 255, 0), -1)

        n_pts = len(pts)
        color = (0, 255, 0) if n_pts > 100 else (0, 255, 255) if n_pts > 50 else (0, 0, 255)
        cv2.putText(display, f"Points: {n_pts}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        cv2.putText(display, f"Saved: {saved_count}", (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(display, "SPACE=save  Q=quit", (10, CAM_HEIGHT - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        cv2.imshow("Calibration Capture", display)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break
        elif key == 32:  # SPACE
            fname = f"calib_{next_idx:03d}.png"
            path = os.path.join(SAVE_DIR, fname)
            cv2.imwrite(path, frame)
            saved_count += 1
            next_idx += 1
            print(f"  Saved: {fname} ({n_pts} points detected)")

    cap.release()
    cv2.destroyAllWindows()
    print(f"\nTotal saved: {saved_count} images in {SAVE_DIR}")


if __name__ == "__main__":
    main()
