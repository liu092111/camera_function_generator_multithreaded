"""
相機鏡頭畸變校正程式 - 最終版
使用多種圖像增強技術提高棋盤格檢測率
"""

import cv2
import numpy as np
import glob
import os

# 終止條件
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


def gamma_correction(img, gamma=1.5):
    """伽馬校正"""
    inv_gamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** inv_gamma) * 255
                      for i in np.arange(0, 256)]).astype("uint8")
    return cv2.LUT(img, table)


def sharpen_image(img):
    """銳化圖像"""
    kernel = np.array([[-1, -1, -1],
                       [-1,  9, -1],
                       [-1, -1, -1]])
    return cv2.filter2D(img, -1, kernel)


def enhance_contrast_strong(gray):
    """強力對比度增強"""
    # 先歸一化
    normalized = cv2.normalize(gray, None, 0, 255, cv2.NORM_MINMAX)
    
    # CLAHE 強力增強
    clahe = cv2.createCLAHE(clipLimit=5.0, tileGridSize=(4, 4))
    enhanced = clahe.apply(normalized)
    
    # 伽馬校正提亮暗部
    enhanced = gamma_correction(enhanced, 1.2)
    
    return enhanced


def preprocess_for_detection(gray):
    """
    生成多種預處理版本以提高檢測成功率
    """
    results = []
    
    # 1. 原始灰度圖
    results.append(("原始", gray))
    
    # 2. 歸一化
    normalized = cv2.normalize(gray, None, 0, 255, cv2.NORM_MINMAX)
    results.append(("歸一化", normalized))
    
    # 3. 直方圖均衡化
    equalized = cv2.equalizeHist(gray)
    results.append(("均衡化", equalized))
    
    # 4. CLAHE 輕度
    clahe1 = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    clahe_light = clahe1.apply(gray)
    results.append(("CLAHE輕度", clahe_light))
    
    # 5. CLAHE 強力
    clahe2 = cv2.createCLAHE(clipLimit=5.0, tileGridSize=(4, 4))
    clahe_strong = clahe2.apply(normalized)
    results.append(("CLAHE強力", clahe_strong))
    
    # 6. 強力對比度增強
    strong_enhanced = enhance_contrast_strong(gray)
    results.append(("強力增強", strong_enhanced))
    
    # 7. 伽馬校正 (提亮)
    gamma_bright = gamma_correction(gray, 1.5)
    results.append(("伽馬提亮", gamma_bright))
    
    # 8. 伽馬校正 (壓暗) + CLAHE
    gamma_dark = gamma_correction(gray, 0.7)
    gamma_dark_clahe = clahe2.apply(gamma_dark)
    results.append(("伽馬壓暗+CLAHE", gamma_dark_clahe))
    
    # 9. 銳化
    sharpened = sharpen_image(gray)
    results.append(("銳化", sharpened))
    
    # 10. 歸一化 + 銳化
    norm_sharp = sharpen_image(normalized)
    results.append(("歸一化+銳化", norm_sharp))
    
    # 11. CLAHE + 銳化
    clahe_sharp = sharpen_image(clahe_light)
    results.append(("CLAHE+銳化", clahe_sharp))
    
    # 12. 高斯模糊後的CLAHE（去噪）
    blurred = cv2.GaussianBlur(gray, (3, 3), 0)
    blurred_clahe = clahe2.apply(blurred)
    results.append(("模糊+CLAHE", blurred_clahe))
    
    # 13. 雙邊濾波（保邊去噪）
    bilateral = cv2.bilateralFilter(gray, 9, 75, 75)
    bilateral_clahe = clahe2.apply(bilateral)
    results.append(("雙邊濾波+CLAHE", bilateral_clahe))
    
    return results


def find_checkerboard(gray, checkerboard_size, method_name=""):
    """
    嘗試使用多種方法檢測棋盤格
    """
    # 檢測標誌組合
    flag_combinations = [
        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FILTER_QUADS,
        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE,
        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FILTER_QUADS,
        cv2.CALIB_CB_ADAPTIVE_THRESH,
        cv2.CALIB_CB_NORMALIZE_IMAGE,
        cv2.CALIB_CB_FILTER_QUADS,
        cv2.CALIB_CB_FAST_CHECK,
        0,  # 無標誌
    ]
    
    for flags in flag_combinations:
        ret, corners = cv2.findChessboardCorners(gray, checkerboard_size, flags)
        if ret:
            return True, corners
    
    # 嘗試 findChessboardCornersSB (更新的方法，如果可用)
    try:
        ret, corners = cv2.findChessboardCornersSB(gray, checkerboard_size)
        if ret:
            return True, corners
    except:
        pass
    
    return False, None


def detect_checkerboard_robust(img, checkerboard_size):
    """
    使用所有預處理方法嘗試檢測棋盤格
    """
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # 獲取所有預處理版本
    preprocessed_list = preprocess_for_detection(gray)
    
    for method_name, processed in preprocessed_list:
        ret, corners = find_checkerboard(processed, checkerboard_size, method_name)
        if ret:
            # 在原始灰度圖上精確化角點
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            return True, corners_refined, method_name
    
    # 如果標準方法都失敗，嘗試反轉圖像（有時棋盤格是反向的）
    for method_name, processed in preprocessed_list:
        inverted = 255 - processed
        ret, corners = find_checkerboard(inverted, checkerboard_size, f"{method_name}(反轉)")
        if ret:
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            return True, corners_refined, f"{method_name}(反轉)"
    
    return False, None, None


def auto_detect_checkerboard_size(images, possible_sizes=None):
    """
    自動檢測棋盤格尺寸
    """
    if possible_sizes is None:
        # 嘗試多種可能的尺寸
        possible_sizes = [
            (10, 7), (7, 10),
            (9, 6), (6, 9),
            (8, 5), (5, 8),
            (8, 6), (6, 8),
            (7, 6), (6, 7),
            (9, 7), (7, 9),
            (8, 7), (7, 8),
            (11, 8), (8, 11),
            (9, 8), (8, 9),
        ]
    
    print("自動檢測棋盤格尺寸...")
    
    for size in possible_sizes:
        for fname in images[:10]:  # 只測試前10張
            img = cv2.imread(fname)
            if img is None:
                continue
            
            ret, corners, method = detect_checkerboard_robust(img, size)
            if ret:
                print(f"檢測到棋盤格尺寸: {size[0]}x{size[1]} (使用 {method})")
                return size
    
    return None


def calibrate_camera(image_folder, checkerboard_size=None):
    """
    相機標定主函數
    """
    # 獲取所有圖像
    images = glob.glob(os.path.join(image_folder, "*.png"))
    images.extend(glob.glob(os.path.join(image_folder, "*.jpg")))
    images.extend(glob.glob(os.path.join(image_folder, "*.jpeg")))
    
    if len(images) == 0:
        print(f"錯誤：在 {image_folder} 中找不到圖像")
        return None, None, None, None
    
    print(f"找到 {len(images)} 張圖像")
    
    # 自動檢測或使用指定的棋盤格尺寸
    if checkerboard_size is None:
        checkerboard_size = auto_detect_checkerboard_size(images)
        if checkerboard_size is None:
            print("無法自動檢測棋盤格尺寸")
            return None, None, None, None
    
    # 準備物體點
    objp = np.zeros((checkerboard_size[0] * checkerboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1, 2)
    
    objpoints = []
    imgpoints = []
    img_shape = None
    successful_images = 0
    
    # 創建輸出目錄
    os.makedirs("corner_detection", exist_ok=True)
    
    print(f"\n使用棋盤格尺寸: {checkerboard_size[0]}x{checkerboard_size[1]}")
    print("開始處理圖像...\n")
    
    for fname in images:
        img = cv2.imread(fname)
        if img is None:
            continue
        
        if img_shape is None:
            img_shape = img.shape[1::-1]  # (width, height)
        
        ret, corners, method = detect_checkerboard_robust(img, checkerboard_size)
        
        if ret:
            objpoints.append(objp)
            imgpoints.append(corners)
            successful_images += 1
            print(f"✓ {os.path.basename(fname)} (使用 {method})")
            
            # 繪製並儲存角點
            img_with_corners = cv2.drawChessboardCorners(img.copy(), checkerboard_size, corners, ret)
            output_path = os.path.join("corner_detection", os.path.basename(fname))
            cv2.imwrite(output_path, img_with_corners)
        else:
            print(f"✗ {os.path.basename(fname)}")
    
    print(f"\n成功檢測 {successful_images}/{len(images)} 張圖像")
    
    if successful_images < 3:
        print("錯誤：成功檢測的圖像數量太少（至少需要3張）")
        return None, None, None, None
    
    # 進行相機標定
    print("\n正在進行相機標定...")
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img_shape, None, None)
    
    print(f"\n標定完成！")
    print(f"重投影誤差: {ret:.4f}")
    print(f"\n相機內參矩陣:\n{mtx}")
    print(f"\n畸變係數: {dist.flatten()}")
    
    return mtx, dist, checkerboard_size, ret


def undistort_image(img, mtx, dist, crop=True):
    """校正單張圖像"""
    h, w = img.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
    
    if crop and roi[2] > 0 and roi[3] > 0:
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]
    
    return dst


def undistort_all_images(image_folder, mtx, dist, output_folder="undistorted"):
    """批量校正圖像"""
    os.makedirs(output_folder, exist_ok=True)
    
    images = glob.glob(os.path.join(image_folder, "*.png"))
    images.extend(glob.glob(os.path.join(image_folder, "*.jpg")))
    
    print(f"\n正在校正 {len(images)} 張圖像...")
    
    for fname in images:
        img = cv2.imread(fname)
        if img is None:
            continue
        
        undistorted = undistort_image(img, mtx, dist)
        output_path = os.path.join(output_folder, os.path.basename(fname))
        cv2.imwrite(output_path, undistorted)
        print(f"已校正: {os.path.basename(fname)}")
    
    print(f"所有校正後的圖像已儲存至 {output_folder}/")


def save_calibration(mtx, dist, checkerboard_size, error, filename="calibration_data.npz"):
    """儲存標定參數"""
    np.savez(filename, 
             mtx=mtx, 
             dist=dist, 
             checkerboard_size=checkerboard_size,
             reprojection_error=error)
    print(f"\n標定參數已儲存至 {filename}")


def load_calibration(filename="calibration_data.npz"):
    """載入標定參數"""
    data = np.load(filename)
    return data['mtx'], data['dist']


def create_comparison(image_folder, output_folder="comparison"):
    """創建對比圖"""
    os.makedirs(output_folder, exist_ok=True)
    
    images = glob.glob(os.path.join(image_folder, "*.png"))
    if not images:
        return
    
    # 選擇第一張圖像創建對比
    original = cv2.imread(images[0])
    undistorted_path = os.path.join("undistorted", os.path.basename(images[0]))
    
    if not os.path.exists(undistorted_path):
        return
    
    undistorted = cv2.imread(undistorted_path)
    
    # 調整尺寸
    h1, w1 = original.shape[:2]
    h2, w2 = undistorted.shape[:2]
    max_h = max(h1, h2)
    
    if h1 != max_h:
        scale = max_h / h1
        original = cv2.resize(original, (int(w1 * scale), max_h))
    if h2 != max_h:
        scale = max_h / h2
        undistorted = cv2.resize(undistorted, (int(w2 * scale), max_h))
    
    # 添加標籤
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(original, "Original", (10, 30), font, 1, (0, 255, 0), 2)
    cv2.putText(undistorted, "Undistorted", (10, 30), font, 1, (0, 255, 0), 2)
    
    # 拼接
    comparison = np.hstack([original, undistorted])
    comparison_path = os.path.join(output_folder, "comparison.png")
    cv2.imwrite(comparison_path, comparison)
    print(f"對比圖已儲存至 {comparison_path}")


def main():
    image_folder = "chest image"
    
    print("=" * 60)
    print("相機鏡頭畸變校正程式")
    print("=" * 60)
    print(f"圖像資料夾: {image_folder}")
    print("=" * 60 + "\n")
    
    # 執行標定
    mtx, dist, checkerboard_size, error = calibrate_camera(image_folder)
    
    if mtx is None:
        print("\n" + "=" * 60)
        print("標定失敗！")
        print("=" * 60)
        return
    
    # 儲存標定參數
    save_calibration(mtx, dist, checkerboard_size, error)
    
    # 校正所有圖像
    undistort_all_images(image_folder, mtx, dist)
    
    # 創建對比圖
    print("\n正在創建對比圖...")
    create_comparison(image_folder)
    
    print("\n" + "=" * 60)
    print("校正完成！")
    print("=" * 60)
    print(f"棋盤格尺寸: {checkerboard_size[0]}x{checkerboard_size[1]}")
    print(f"重投影誤差: {error:.4f}")
    print("-" * 60)
    print("輸出文件:")
    print("  - corner_detection/ : 角點檢測可視化")
    print("  - undistorted/      : 校正後圖像")
    print("  - comparison/       : 校正前後對比")
    print("  - calibration_data.npz : 標定參數")


if __name__ == "__main__":
    main()
