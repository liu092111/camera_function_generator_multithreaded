"""
Automatic Camera Distortion Calibration
自動相機畸變校正腳本

用於校正廣角鏡頭的桶形畸變（Barrel Distortion）
"""

import cv2
import numpy as np
import glob
import os
import json
from datetime import datetime


def load_calibration(filename="camera_calibration.json"):
    """
    載入校正參數
    
    Args:
        filename: 校正參數文件路徑
        
    Returns:
        camera_matrix: 相機內參矩陣
        dist_coeffs: 畸變係數
    """
    if not os.path.exists(filename):
        raise FileNotFoundError(f"Calibration file not found: {filename}")
    
    with open(filename, 'r') as f:
        data = json.load(f)
    
    camera_matrix = np.array(data["camera_matrix"])
    dist_coeffs = np.array(data["dist_coeffs"])
    
    return camera_matrix, dist_coeffs


def undistort_image(image, camera_matrix, dist_coeffs):
    """
    對單張圖像進行畸變校正
    
    Args:
        image: 輸入圖像 (BGR)
        camera_matrix: 相機內參矩陣
        dist_coeffs: 畸變係數
        
    Returns:
        校正後的圖像
    """
    h, w = image.shape[:2]
    
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
        camera_matrix, dist_coeffs, (w, h), 1, (w, h)
    )
    
    undistorted = cv2.undistort(
        image, camera_matrix, dist_coeffs, None, new_camera_matrix
    )
    
    return undistorted


def apply_calibration(k1=-0.30, k2=0.08, input_folder="img distortion", output_folder="img_corrected"):
    """
    應用畸變校正到所有圖像
    
    Args:
        k1: 主要徑向畸變係數（負值校正桶形畸變）
        k2: 次要徑向畸變係數
        input_folder: 原始圖像資料夾
        output_folder: 校正後圖像輸出資料夾
    """
    print(f"Applying calibration: k1={k1}, k2={k2}")
    
    # 獲取所有圖像
    image_paths = sorted(glob.glob(os.path.join(input_folder, "*.png")))
    if not image_paths:
        print(f"No images found in {input_folder}")
        return
    
    # 讀取第一張圖獲取尺寸
    sample_image = cv2.imread(image_paths[0])
    h, w = sample_image.shape[:2]
    
    # 建立相機矩陣
    fx = fy = 0.9 * max(w, h)  # 廣角鏡頭焦距估計
    cx, cy = w / 2, h / 2      # 主點在圖像中心
    
    camera_matrix = np.array([
        [fx, 0, cx],
        [0, fy, cy],
        [0, 0, 1]
    ], dtype=np.float64)
    
    # 畸變係數: [k1, k2, p1, p2, k3]
    dist_coeffs = np.array([k1, k2, 0, 0, 0], dtype=np.float64)
    
    # 保存校正參數
    calibration_data = {
        "camera_matrix": camera_matrix.tolist(),
        "dist_coeffs": dist_coeffs.flatten().tolist(),
        "image_size": [w, h],
        "calibration_date": datetime.now().isoformat(),
        "parameters": {"k1": k1, "k2": k2}
    }
    
    with open("camera_calibration.json", 'w') as f:
        json.dump(calibration_data, f, indent=2)
    print("Calibration saved to camera_calibration.json")
    
    # 創建輸出資料夾
    os.makedirs(output_folder, exist_ok=True)
    
    # 創建 undistortion maps（提高效率）
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
        camera_matrix, dist_coeffs, (w, h), 1, (w, h)
    )
    
    map1, map2 = cv2.initUndistortRectifyMap(
        camera_matrix, dist_coeffs, None,
        new_camera_matrix, (w, h), cv2.CV_32FC1
    )
    
    # 處理所有圖像
    print(f"\nProcessing {len(image_paths)} images...")
    
    for img_path in image_paths:
        image = cv2.imread(img_path)
        if image is None:
            continue
        
        # 應用校正
        undistorted = cv2.remap(image, map1, map2, cv2.INTER_LINEAR)
        
        # 保存
        output_path = os.path.join(output_folder, os.path.basename(img_path))
        cv2.imwrite(output_path, undistorted)
        print(f"  Saved: {output_path}")
    
    # 創建比較圖
    create_comparison(image_paths[0], output_folder, camera_matrix, dist_coeffs)
    
    print(f"\nDone! Corrected images saved to {output_folder}/")


def create_comparison(original_path, output_folder, camera_matrix, dist_coeffs):
    """
    創建原始與校正後的對比圖
    """
    original = cv2.imread(original_path)
    h, w = original.shape[:2]
    
    new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(
        camera_matrix, dist_coeffs, (w, h), 1, (w, h)
    )
    
    corrected = cv2.undistort(original, camera_matrix, dist_coeffs, None, new_camera_matrix)
    
    # 並排比較
    comparison = np.hstack([original, corrected])
    
    # 添加標籤
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(comparison, 'Original (Distorted)', (10, 40), font, 1.0, (0, 255, 0), 2)
    cv2.putText(comparison, 'Corrected', (w + 10, 40), font, 1.0, (0, 255, 0), 2)
    
    output_path = os.path.join(output_folder, "comparison.png")
    cv2.imwrite(output_path, comparison)
    print(f"Comparison saved to {output_path}")


if __name__ == "__main__":
    # 使用優化後的參數進行校正
    apply_calibration(k1=-0.30, k2=0.08)
