"""
使用校正映射表對新影像進行校正
支援完整版或裁切版輸出
"""
import cv2
import numpy as np
import os

def load_rectification_map(npz_path):
    """載入校正映射表"""
    data = np.load(npz_path)
    return {
        'map_x': data['map_x'],
        'map_y': data['map_y'],
        'map_x_cropped': data['map_x_cropped'],
        'map_y_cropped': data['map_y_cropped'],
        'roi': data['roi'],
        'out_size': data['out_size'],
        'valid_region': data['valid_region'],
        'crop_margin': int(data['crop_margin'])
    }

def apply_rectification(img, rect_map, use_cropped=True):
    """
    對影像應用校正
    
    Parameters:
    -----------
    img : np.ndarray
        輸入影像 (BGR)
    rect_map : dict
        由 load_rectification_map() 載入的校正映射
    use_cropped : bool
        True: 輸出裁切版（只有有效區域，無黑邊）
        False: 輸出完整版（含黑邊）
    
    Returns:
    --------
    np.ndarray
        校正後的影像
    """
    # 提取 ROI
    x0, y0, w0, h0 = rect_map['roi']
    roi = img[y0:y0+h0, x0:x0+w0]
    
    if use_cropped:
        # 使用裁切版 remap
        rectified = cv2.remap(
            roi, 
            rect_map['map_x_cropped'], 
            rect_map['map_y_cropped'],
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT, 
            borderValue=(0, 0, 0)
        )
    else:
        # 使用完整版 remap
        rectified = cv2.remap(
            roi,
            rect_map['map_x'],
            rect_map['map_y'],
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(0, 0, 0)
        )
    
    return rectified

def get_valid_region_info(rect_map):
    """取得有效區域資訊"""
    valid_x0, valid_y0, valid_w, valid_h = rect_map['valid_region']
    return {
        'x0': valid_x0,
        'y0': valid_y0,
        'width': valid_w,
        'height': valid_h,
        'crop_margin': rect_map['crop_margin']
    }


# ==================== 使用範例 ====================
if __name__ == "__main__":
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
    
    # 載入映射表
    npz_path = os.path.join(SCRIPT_DIR, "tps_rectification_map.npz")
    rect_map = load_rectification_map(npz_path)
    
    # 顯示有效區域資訊
    valid_info = get_valid_region_info(rect_map)
    print("=== 校正映射表資訊 ===")
    print(f"原始 ROI: {rect_map['roi']}")
    print(f"輸出大小 (完整版): {rect_map['out_size']}")
    print(f"有效區域: ({valid_info['x0']}, {valid_info['y0']}) - size {valid_info['width']}x{valid_info['height']}")
    print(f"裁切 margin: {valid_info['crop_margin']}px")
    
    # 測試校正
    test_img_path = os.path.join(SCRIPT_DIR, "original.png")
    if os.path.exists(test_img_path):
        img = cv2.imread(test_img_path)
        
        # 使用裁切版（推薦）
        rectified_cropped = apply_rectification(img, rect_map, use_cropped=True)
        cv2.imwrite(os.path.join(SCRIPT_DIR, "test_rectified_cropped.png"), rectified_cropped)
        print(f"\n[OK] 裁切版輸出: test_rectified_cropped.png, size={rectified_cropped.shape[1]}x{rectified_cropped.shape[0]}")
        
        # 使用完整版
        rectified_full = apply_rectification(img, rect_map, use_cropped=False)
        cv2.imwrite(os.path.join(SCRIPT_DIR, "test_rectified_full.png"), rectified_full)
        print(f"[OK] 完整版輸出: test_rectified_full.png, size={rectified_full.shape[1]}x{rectified_full.shape[0]}")
    else:
        print(f"\n[INFO] 測試圖片不存在: {test_img_path}")
        print("[INFO] 使用方式:")
        print("    rect_map = load_rectification_map('tps_rectification_map_improved.npz')")
        print("    rectified = apply_rectification(your_image, rect_map, use_cropped=True)")
