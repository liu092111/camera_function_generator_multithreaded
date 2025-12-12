"""
相機鏡頭畸變校正程式 - 優化版
支援計算每張圖片的重投影誤差，並可剔除高誤差圖片以優化校正結果
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
    normalized = cv2.normalize(gray, None, 0, 255, cv2.NORM_MINMAX)
    clahe = cv2.createCLAHE(clipLimit=5.0, tileGridSize=(4, 4))
    enhanced = clahe.apply(normalized)
    enhanced = gamma_correction(enhanced, 1.2)
    return enhanced


def preprocess_for_detection(gray):
    """生成多種預處理版本以提高檢測成功率"""
    results = []
    
    results.append(("原始", gray))
    
    normalized = cv2.normalize(gray, None, 0, 255, cv2.NORM_MINMAX)
    results.append(("歸一化", normalized))
    
    equalized = cv2.equalizeHist(gray)
    results.append(("均衡化", equalized))
    
    clahe1 = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    clahe_light = clahe1.apply(gray)
    results.append(("CLAHE輕度", clahe_light))
    
    clahe2 = cv2.createCLAHE(clipLimit=5.0, tileGridSize=(4, 4))
    clahe_strong = clahe2.apply(normalized)
    results.append(("CLAHE強力", clahe_strong))
    
    strong_enhanced = enhance_contrast_strong(gray)
    results.append(("強力增強", strong_enhanced))
    
    gamma_bright = gamma_correction(gray, 1.5)
    results.append(("伽馬提亮", gamma_bright))
    
    gamma_dark = gamma_correction(gray, 0.7)
    gamma_dark_clahe = clahe2.apply(gamma_dark)
    results.append(("伽馬壓暗+CLAHE", gamma_dark_clahe))
    
    sharpened = sharpen_image(gray)
    results.append(("銳化", sharpened))
    
    norm_sharp = sharpen_image(normalized)
    results.append(("歸一化+銳化", norm_sharp))
    
    clahe_sharp = sharpen_image(clahe_light)
    results.append(("CLAHE+銳化", clahe_sharp))
    
    blurred = cv2.GaussianBlur(gray, (3, 3), 0)
    blurred_clahe = clahe2.apply(blurred)
    results.append(("模糊+CLAHE", blurred_clahe))
    
    bilateral = cv2.bilateralFilter(gray, 9, 75, 75)
    bilateral_clahe = clahe2.apply(bilateral)
    results.append(("雙邊濾波+CLAHE", bilateral_clahe))
    
    return results


def find_checkerboard(gray, checkerboard_size, method_name=""):
    """嘗試使用多種方法檢測棋盤格"""
    flag_combinations = [
        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FILTER_QUADS,
        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE,
        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FILTER_QUADS,
        cv2.CALIB_CB_ADAPTIVE_THRESH,
        cv2.CALIB_CB_NORMALIZE_IMAGE,
        cv2.CALIB_CB_FILTER_QUADS,
        cv2.CALIB_CB_FAST_CHECK,
        0,
    ]
    
    for flags in flag_combinations:
        ret, corners = cv2.findChessboardCorners(gray, checkerboard_size, flags)
        if ret:
            return True, corners
    
    try:
        ret, corners = cv2.findChessboardCornersSB(gray, checkerboard_size)
        if ret:
            return True, corners
    except:
        pass
    
    return False, None


def detect_checkerboard_robust(img, checkerboard_size):
    """使用所有預處理方法嘗試檢測棋盤格"""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    preprocessed_list = preprocess_for_detection(gray)
    
    for method_name, processed in preprocessed_list:
        ret, corners = find_checkerboard(processed, checkerboard_size, method_name)
        if ret:
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            return True, corners_refined, method_name
    
    for method_name, processed in preprocessed_list:
        inverted = 255 - processed
        ret, corners = find_checkerboard(inverted, checkerboard_size, f"{method_name}(反轉)")
        if ret:
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            return True, corners_refined, f"{method_name}(反轉)"
    
    return False, None, None


def auto_detect_checkerboard_size(images, possible_sizes=None):
    """自動檢測棋盤格尺寸"""
    if possible_sizes is None:
        possible_sizes = [
            (10, 7), (7, 10), (9, 6), (6, 9), (8, 5), (5, 8),
            (8, 6), (6, 8), (7, 6), (6, 7), (9, 7), (7, 9),
            (8, 7), (7, 8), (11, 8), (8, 11), (9, 8), (8, 9),
        ]
    
    print("自動檢測棋盤格尺寸...")
    
    for size in possible_sizes:
        for fname in images[:10]:
            img = cv2.imread(fname)
            if img is None:
                continue
            
            ret, corners, method = detect_checkerboard_robust(img, size)
            if ret:
                print(f"檢測到棋盤格尺寸: {size[0]}x{size[1]} (使用 {method})")
                return size
    
    return None


def calculate_per_image_error(objpoints, imgpoints, rvecs, tvecs, mtx, dist, image_names):
    """
    計算每張圖片的重投影誤差
    
    返回:
        errors: 每張圖片的誤差列表 [(誤差, 圖片名稱, 索引), ...]
        mean_error: 平均誤差
    """
    errors = []
    total_error = 0
    total_points = 0
    
    for i, (objp, imgp) in enumerate(zip(objpoints, imgpoints)):
        # 投影3D點到2D
        projected, _ = cv2.projectPoints(objp, rvecs[i], tvecs[i], mtx, dist)
        
        # 計算誤差
        error = cv2.norm(imgp, projected, cv2.NORM_L2) / len(projected)
        errors.append((error, image_names[i], i))
        
        total_error += error * len(projected)
        total_points += len(projected)
    
    mean_error = total_error / total_points if total_points > 0 else 0
    
    return errors, mean_error


def collect_calibration_data(image_folder, checkerboard_size=None):
    """
    收集校正數據，返回所有成功檢測的圖片資訊
    """
    images = glob.glob(os.path.join(image_folder, "*.png"))
    images.extend(glob.glob(os.path.join(image_folder, "*.jpg")))
    images.extend(glob.glob(os.path.join(image_folder, "*.jpeg")))
    
    if len(images) == 0:
        print(f"錯誤：在 {image_folder} 中找不到圖像")
        return None
    
    print(f"找到 {len(images)} 張圖像")
    
    if checkerboard_size is None:
        checkerboard_size = auto_detect_checkerboard_size(images)
        if checkerboard_size is None:
            print("無法自動檢測棋盤格尺寸")
            return None
    
    # 準備物體點
    objp = np.zeros((checkerboard_size[0] * checkerboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1, 2)
    
    data = {
        'objpoints': [],
        'imgpoints': [],
        'image_names': [],
        'image_paths': [],
        'checkerboard_size': checkerboard_size,
        'img_shape': None
    }
    
    print(f"\n使用棋盤格尺寸: {checkerboard_size[0]}x{checkerboard_size[1]}")
    print("開始處理圖像...\n")
    
    for fname in images:
        img = cv2.imread(fname)
        if img is None:
            continue
        
        if data['img_shape'] is None:
            data['img_shape'] = img.shape[1::-1]
        
        ret, corners, method = detect_checkerboard_robust(img, checkerboard_size)
        
        if ret:
            data['objpoints'].append(objp)
            data['imgpoints'].append(corners)
            data['image_names'].append(os.path.basename(fname))
            data['image_paths'].append(fname)
            print(f"✓ {os.path.basename(fname)} (使用 {method})")
        else:
            print(f"✗ {os.path.basename(fname)}")
    
    print(f"\n成功檢測 {len(data['image_names'])}/{len(images)} 張圖像")
    
    return data


def calibrate_with_data(data, excluded_indices=None):
    """
    使用給定的數據進行校正，可以排除特定圖片
    
    參數:
        data: 校正數據字典
        excluded_indices: 要排除的圖片索引列表
    
    返回:
        mtx, dist, rvecs, tvecs, error, used_indices
    """
    if excluded_indices is None:
        excluded_indices = []
    
    # 過濾數據
    objpoints = []
    imgpoints = []
    used_indices = []
    
    for i in range(len(data['objpoints'])):
        if i not in excluded_indices:
            objpoints.append(data['objpoints'][i])
            imgpoints.append(data['imgpoints'][i])
            used_indices.append(i)
    
    if len(objpoints) < 3:
        print("錯誤：可用圖片數量太少（至少需要3張）")
        return None, None, None, None, None, None
    
    # 進行相機標定
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, data['img_shape'], None, None
    )
    
    return mtx, dist, rvecs, tvecs, ret, used_indices


def optimize_calibration(data, target_error=0.3, max_iterations=15, max_exclude_ratio=0.5):
    """
    迭代優化校正結果，通過剔除高誤差圖片來降低總誤差
    
    參數:
        data: 校正數據
        target_error: 目標誤差閾值
        max_iterations: 最大迭代次數
        max_exclude_ratio: 最大可剔除比例
    
    返回:
        最佳校正結果
    """
    print("\n" + "=" * 60)
    print("開始優化校正...")
    print(f"目標誤差: < {target_error}")
    print("=" * 60)
    
    total_images = len(data['image_names'])
    max_exclude = int(total_images * max_exclude_ratio)
    excluded_indices = []
    
    best_result = None
    best_error = float('inf')
    
    for iteration in range(max_iterations):
        print(f"\n--- 迭代 {iteration + 1} ---")
        print(f"使用圖片數: {total_images - len(excluded_indices)}/{total_images}")
        
        # 進行校正
        mtx, dist, rvecs, tvecs, error, used_indices = calibrate_with_data(data, excluded_indices)
        
        if mtx is None:
            print("校正失敗，停止優化")
            break
        
        # 計算每張圖片的誤差
        used_names = [data['image_names'][i] for i in used_indices]
        used_objpoints = [data['objpoints'][i] for i in used_indices]
        used_imgpoints = [data['imgpoints'][i] for i in used_indices]
        
        per_image_errors, mean_error = calculate_per_image_error(
            used_objpoints, used_imgpoints, rvecs, tvecs, mtx, dist, used_names
        )
        
        print(f"當前重投影誤差: {error:.4f}")
        
        # 記錄最佳結果
        if error < best_error:
            best_error = error
            best_result = {
                'mtx': mtx,
                'dist': dist,
                'rvecs': rvecs,
                'tvecs': tvecs,
                'error': error,
                'used_indices': used_indices,
                'excluded_indices': excluded_indices.copy(),
                'per_image_errors': per_image_errors
            }
        
        # 檢查是否達到目標
        if error <= target_error:
            print(f"\n✓ 達到目標誤差！({error:.4f} <= {target_error})")
            break
        
        # 檢查是否已達到最大剔除數量
        if len(excluded_indices) >= max_exclude:
            print(f"\n已達到最大剔除數量限制 ({max_exclude})")
            break
        
        # 找出誤差最高的圖片
        per_image_errors.sort(reverse=True, key=lambda x: x[0])
        
        # 顯示誤差排名
        print("\n每張圖片的誤差 (由高到低):")
        for rank, (err, name, idx) in enumerate(per_image_errors[:5], 1):
            original_idx = used_indices[idx]
            print(f"  {rank}. {name}: {err:.4f}")
        
        # 剔除誤差最高的圖片
        worst_error, worst_name, worst_local_idx = per_image_errors[0]
        worst_original_idx = used_indices[worst_local_idx]
        
        print(f"\n剔除高誤差圖片: {worst_name} (誤差: {worst_error:.4f})")
        excluded_indices.append(worst_original_idx)
    
    return best_result


def print_detailed_analysis(data, result):
    """
    打印詳細的校正分析報告
    """
    print("\n" + "=" * 60)
    print("詳細校正分析報告")
    print("=" * 60)
    
    print(f"\n【校正結果】")
    print(f"  最終重投影誤差: {result['error']:.4f}")
    print(f"  使用圖片數: {len(result['used_indices'])}/{len(data['image_names'])}")
    print(f"  剔除圖片數: {len(result['excluded_indices'])}")
    
    print(f"\n【相機內參矩陣】")
    print(result['mtx'])
    
    print(f"\n【畸變係數】")
    print(f"  k1 = {result['dist'][0][0]:.6f}")
    print(f"  k2 = {result['dist'][0][1]:.6f}")
    print(f"  p1 = {result['dist'][0][2]:.6f}")
    print(f"  p2 = {result['dist'][0][3]:.6f}")
    print(f"  k3 = {result['dist'][0][4]:.6f}")
    
    # 顯示每張圖片的誤差
    print(f"\n【每張圖片的重投影誤差】")
    errors_sorted = sorted(result['per_image_errors'], key=lambda x: x[0])
    
    print("\n  使用的圖片 (由低到高):")
    for err, name, idx in errors_sorted:
        status = "✓"
        print(f"    {status} {name}: {err:.4f}")
    
    if result['excluded_indices']:
        print("\n  被剔除的圖片:")
        for idx in result['excluded_indices']:
            name = data['image_names'][idx]
            print(f"    ✗ {name}")
    
    # 給出建議
    print(f"\n【建議】")
    if result['error'] <= 0.3:
        print("  ✓ 校正精度很好！")
    elif result['error'] <= 0.5:
        print("  △ 校正精度可接受，但可以考慮:")
        print("    - 重新拍攝更清晰的校正圖片")
        print("    - 確保棋盤格完全平整")
        print("    - 覆蓋更多角度和位置")
    else:
        print("  ✗ 校正精度較差，建議:")
        print("    - 檢查棋盤格是否有變形")
        print("    - 確保圖片清晰度足夠")
        print("    - 增加更多不同角度的圖片")


def save_corner_detection(data, result, output_folder="corner_detection"):
    """保存角點檢測結果"""
    os.makedirs(output_folder, exist_ok=True)
    
    for i, path in enumerate(data['image_paths']):
        img = cv2.imread(path)
        if img is None:
            continue
        
        corners = data['imgpoints'][i]
        is_excluded = i in result['excluded_indices']
        
        # 繪製角點
        img_with_corners = cv2.drawChessboardCorners(
            img.copy(), data['checkerboard_size'], corners, True
        )
        
        # 添加狀態標籤
        if is_excluded:
            cv2.putText(img_with_corners, "EXCLUDED", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        else:
            # 找到對應的誤差
            for err, name, _ in result['per_image_errors']:
                if name == data['image_names'][i]:
                    cv2.putText(img_with_corners, f"Error: {err:.4f}", (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                    break
        
        output_path = os.path.join(output_folder, os.path.basename(path))
        cv2.imwrite(output_path, img_with_corners)


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


def save_calibration(result, checkerboard_size, filename="calibration_data.npz"):
    """儲存標定參數"""
    np.savez(filename, 
             mtx=result['mtx'], 
             dist=result['dist'], 
             checkerboard_size=checkerboard_size,
             reprojection_error=result['error'])
    print(f"\n標定參數已儲存至 {filename}")


def create_comparison(image_folder, output_folder="comparison"):
    """創建對比圖"""
    os.makedirs(output_folder, exist_ok=True)
    
    images = glob.glob(os.path.join(image_folder, "*.png"))
    if not images:
        return
    
    original = cv2.imread(images[0])
    undistorted_path = os.path.join("undistorted", os.path.basename(images[0]))
    
    if not os.path.exists(undistorted_path):
        return
    
    undistorted = cv2.imread(undistorted_path)
    
    h1, w1 = original.shape[:2]
    h2, w2 = undistorted.shape[:2]
    max_h = max(h1, h2)
    
    if h1 != max_h:
        scale = max_h / h1
        original = cv2.resize(original, (int(w1 * scale), max_h))
    if h2 != max_h:
        scale = max_h / h2
        undistorted = cv2.resize(undistorted, (int(w2 * scale), max_h))
    
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(original, "Original", (10, 30), font, 1, (0, 255, 0), 2)
    cv2.putText(undistorted, "Undistorted", (10, 30), font, 1, (0, 255, 0), 2)
    
    comparison = np.hstack([original, undistorted])
    comparison_path = os.path.join(output_folder, "comparison.png")
    cv2.imwrite(comparison_path, comparison)
    print(f"對比圖已儲存至 {comparison_path}")


def interactive_selection(data, result):
    """
    互動式選擇要剔除的圖片
    """
    print("\n" + "=" * 60)
    print("互動式圖片選擇")
    print("=" * 60)
    
    while True:
        print("\n選項:")
        print("  1. 顯示所有圖片的誤差")
        print("  2. 手動剔除圖片")
        print("  3. 恢復被剔除的圖片")
        print("  4. 重新校正")
        print("  5. 完成並保存")
        
        choice = input("\n請選擇 (1-5): ").strip()
        
        if choice == '1':
            print("\n所有圖片的誤差:")
            for err, name, _ in sorted(result['per_image_errors'], key=lambda x: x[0], reverse=True):
                print(f"  {name}: {err:.4f}")
            
            if result['excluded_indices']:
                print("\n已剔除的圖片:")
                for idx in result['excluded_indices']:
                    print(f"  {data['image_names'][idx]}")
        
        elif choice == '2':
            print("\n可剔除的圖片:")
            available = [(i, data['image_names'][i]) for i in range(len(data['image_names'])) 
                        if i not in result['excluded_indices']]
            for i, (idx, name) in enumerate(available, 1):
                print(f"  {i}. {name}")
            
            try:
                sel = input("\n請輸入要剔除的圖片編號 (多個用逗號分隔): ").strip()
                if sel:
                    indices = [int(x.strip()) - 1 for x in sel.split(',')]
                    for idx in indices:
                        if 0 <= idx < len(available):
                            result['excluded_indices'].append(available[idx][0])
                            print(f"已剔除: {available[idx][1]}")
            except:
                print("輸入無效")
        
        elif choice == '3':
            if not result['excluded_indices']:
                print("\n沒有被剔除的圖片")
            else:
                print("\n被剔除的圖片:")
                for i, idx in enumerate(result['excluded_indices'], 1):
                    print(f"  {i}. {data['image_names'][idx]}")
                
                try:
                    sel = input("\n請輸入要恢復的圖片編號 (多個用逗號分隔, 或輸入 'all' 恢復全部): ").strip()
                    if sel.lower() == 'all':
                        result['excluded_indices'] = []
                        print("已恢復所有圖片")
                    elif sel:
                        indices = [int(x.strip()) - 1 for x in sel.split(',')]
                        to_restore = [result['excluded_indices'][i] for i in indices 
                                     if 0 <= i < len(result['excluded_indices'])]
                        for idx in to_restore:
                            result['excluded_indices'].remove(idx)
                            print(f"已恢復: {data['image_names'][idx]}")
                except:
                    print("輸入無效")
        
        elif choice == '4':
            print("\n重新校正中...")
            mtx, dist, rvecs, tvecs, error, used_indices = calibrate_with_data(
                data, result['excluded_indices']
            )
            
            if mtx is not None:
                used_names = [data['image_names'][i] for i in used_indices]
                used_objpoints = [data['objpoints'][i] for i in used_indices]
                used_imgpoints = [data['imgpoints'][i] for i in used_indices]
                
                per_image_errors, _ = calculate_per_image_error(
                    used_objpoints, used_imgpoints, rvecs, tvecs, mtx, dist, used_names
                )
                
                result['mtx'] = mtx
                result['dist'] = dist
                result['rvecs'] = rvecs
                result['tvecs'] = tvecs
                result['error'] = error
                result['used_indices'] = used_indices
                result['per_image_errors'] = per_image_errors
                
                print(f"\n新的重投影誤差: {error:.4f}")
            else:
                print("校正失敗")
        
        elif choice == '5':
            return result
        
        else:
            print("無效選項")
    
    return result


def main():
    image_folder = "chest image"
    
    print("=" * 60)
    print("相機鏡頭畸變校正程式 - 優化版")
    print("=" * 60)
    print(f"圖像資料夾: {image_folder}")
    print("=" * 60 + "\n")
    
    # 收集校正數據
    data = collect_calibration_data(image_folder)
    
    if data is None:
        print("\n" + "=" * 60)
        print("數據收集失敗！")
        print("=" * 60)
        return
    
    # 選擇模式
    print("\n" + "=" * 60)
    print("請選擇校正模式:")
    print("  1. 自動優化 (自動剔除高誤差圖片)")
    print("  2. 互動式選擇 (手動選擇要剔除的圖片)")
    print("  3. 使用所有圖片 (不剔除任何圖片)")
    print("=" * 60)
    
    mode = input("\n請選擇 (1-3, 預設為1): ").strip() or '1'
    
    if mode == '1':
        # 自動優化模式
        target = input("目標誤差 (預設 0.3): ").strip()
        target_error = float(target) if target else 0.3
        
        result = optimize_calibration(data, target_error=target_error)
    
    elif mode == '2':
        # 先進行初始校正
        mtx, dist, rvecs, tvecs, error, used_indices = calibrate_with_data(data)
        
        if mtx is None:
            print("初始校正失敗")
            return
        
        used_names = [data['image_names'][i] for i in used_indices]
        used_objpoints = [data['objpoints'][i] for i in used_indices]
        used_imgpoints = [data['imgpoints'][i] for i in used_indices]
        
        per_image_errors, _ = calculate_per_image_error(
            used_objpoints, used_imgpoints, rvecs, tvecs, mtx, dist, used_names
        )
        
        result = {
            'mtx': mtx,
            'dist': dist,
            'rvecs': rvecs,
            'tvecs': tvecs,
            'error': error,
            'used_indices': used_indices,
            'excluded_indices': [],
            'per_image_errors': per_image_errors
        }
        
        print(f"\n初始重投影誤差: {error:.4f}")
        
        # 互動式選擇
        result = interactive_selection(data, result)
    
    else:
        # 使用所有圖片
        mtx, dist, rvecs, tvecs, error, used_indices = calibrate_with_data(data)
        
        if mtx is None:
            print("校正失敗")
            return
        
        used_names = [data['image_names'][i] for i in used_indices]
        used_objpoints = [data['objpoints'][i] for i in used_indices]
        used_imgpoints = [data['imgpoints'][i] for i in used_indices]
        
        per_image_errors, _ = calculate_per_image_error(
            used_objpoints, used_imgpoints, rvecs, tvecs, mtx, dist, used_names
        )
        
        result = {
            'mtx': mtx,
            'dist': dist,
            'rvecs': rvecs,
            'tvecs': tvecs,
            'error': error,
            'used_indices': used_indices,
            'excluded_indices': [],
            'per_image_errors': per_image_errors
        }
    
    if result is None:
        print("優化失敗")
        return
    
    # 打印詳細分析報告
    print_detailed_analysis(data, result)
    
    # 保存角點檢測結果
    print("\n正在保存角點檢測結果...")
    save_corner_detection(data, result)
    
    # 儲存標定參數
    save_calibration(result, data['checkerboard_size'])
    
    # 校正所有圖像
    undistort_all_images(image_folder, result['mtx'], result['dist'])
    
    # 創建對比圖
    print("\n正在創建對比圖...")
    create_comparison(image_folder)
    
    print("\n" + "=" * 60)
    print("校正完成！")
    print("=" * 60)
    print(f"棋盤格尺寸: {data['checkerboard_size'][0]}x{data['checkerboard_size'][1]}")
    print(f"最終重投影誤差: {result['error']:.4f}")
    print(f"使用圖片數: {len(result['used_indices'])}/{len(data['image_names'])}")
    print("-" * 60)
    print("輸出文件:")
    print("  - corner_detection/ : 角點檢測可視化 (含誤差標註)")
    print("  - undistorted/      : 校正後圖像")
    print("  - comparison/       : 校正前後對比")
    print("  - calibration_data.npz : 標定參數")


if __name__ == "__main__":
    main()
