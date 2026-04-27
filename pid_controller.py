# -*- coding: utf-8 -*-
"""
PID 控制模組
提供 Straight mode 姿態校正控制：
- 偵測角度偏移
- 自動切換到旋轉模式校正
- 校正完成後切回直走模式
"""

import time
import numpy as np


class PIDController:
    """通用 PID 控制器"""
    
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, setpoint=0.0, 
                 output_min=-1.0, output_max=1.0, deadband=0.0):
        """
        初始化 PID 控制器
        
        Args:
            kp: 比例增益
            ki: 積分增益
            kd: 微分增益
            setpoint: 目標值
            output_min: 輸出最小值
            output_max: 輸出最大值
            deadband: 死區（誤差小於此值時不輸出）
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.output_min = output_min
        self.output_max = output_max
        self.deadband = deadband
        
        # 內部狀態
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None
        self.enabled = False
        
    def reset(self):
        """重置 PID 狀態"""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None
        
    def set_setpoint(self, setpoint):
        """設定目標值"""
        self.setpoint = setpoint
        
    def set_gains(self, kp=None, ki=None, kd=None):
        """設定增益"""
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd
            
    def update(self, measurement, current_time=None):
        """
        更新 PID 控制器
        
        Args:
            measurement: 當前測量值
            current_time: 當前時間（如果為 None 則使用系統時間）
        
        Returns:
            控制輸出值
        """
        if not self.enabled:
            return 0.0
            
        if current_time is None:
            current_time = time.perf_counter()
            
        # 計算誤差
        error = self.setpoint - measurement
        
        # 死區處理
        if abs(error) < self.deadband:
            return 0.0
        
        # 計算時間間隔
        if self.prev_time is None:
            dt = 0.01  # 預設 10ms
        else:
            dt = current_time - self.prev_time
            if dt <= 0:
                dt = 0.01
                
        # 比例項
        p_term = self.kp * error
        
        # 積分項（帶有抗飽和）
        self.integral += error * dt
        # 積分限幅
        max_integral = (self.output_max - self.output_min) / max(self.ki, 1e-6)
        self.integral = np.clip(self.integral, -max_integral, max_integral)
        i_term = self.ki * self.integral
        
        # 微分項
        if self.prev_time is not None and dt > 0:
            derivative = (error - self.prev_error) / dt
        else:
            derivative = 0.0
        d_term = self.kd * derivative
        
        # 總輸出
        output = p_term + i_term + d_term
        
        # 輸出限幅
        output = np.clip(output, self.output_min, self.output_max)
        
        # 更新狀態
        self.prev_error = error
        self.prev_time = current_time
        
        return output


class StraightModeController:
    """
    Straight Mode 控制器
    目標：維持直線軌跡，減少橫向偏移
    控制策略：根據橫向偏移調整電壓比例來校正方向
    """
    
    def __init__(self, base_voltage=1.2, correction_range=0.3):
        """
        初始化 Straight Mode 控制器
        
        Args:
            base_voltage: 基準電壓 (V)
            correction_range: 校正電壓範圍 (V)，用於調整兩通道電壓差
        """
        self.base_voltage = base_voltage
        self.correction_range = correction_range
        
        # PID 控制器（控制橫向偏移 y）
        # 輸出為電壓補償值 (-correction_range ~ +correction_range)
        self.pid = PIDController(
            kp=0.05,           # 比例增益（mm 偏移 -> V 補償）
            ki=0.01,           # 積分增益
            kd=0.02,           # 微分增益
            setpoint=0.0,      # 目標：y = 0 (直線)
            output_min=-correction_range,
            output_max=correction_range,
            deadband=0.5       # 0.5mm 以內不校正
        )
        
        # 狀態
        self.origin_set = False
        self.origin_y = 0.0
        self.enabled = False
        
        # 運動方向偵測
        self.primary_axis = 'x'  # 主要運動方向（'x' 或 'y'）
        self.motion_direction = 1  # +1 或 -1
        
    def reset(self):
        """重置控制器"""
        self.pid.reset()
        self.origin_set = False
        self.origin_y = 0.0
        
    def enable(self, enabled=True):
        """啟用/停用控制器"""
        self.enabled = enabled
        self.pid.enabled = enabled
        if enabled:
            print("[StraightMode PID] 已啟用")
        else:
            print("[StraightMode PID] 已停用")
        
    def set_origin(self, y_mm):
        """設定原點（直線基準）"""
        self.origin_y = y_mm
        self.origin_set = True
        self.pid.set_setpoint(0.0)  # 目標是相對於原點的偏移為 0
        
    def detect_motion_direction(self, positions_mm):
        """
        偵測主要運動方向
        
        Args:
            positions_mm: [(x, y), ...] 位置序列
        """
        if len(positions_mm) < 10:
            return
            
        positions = np.array(positions_mm)
        x_range = np.ptp(positions[:, 0])
        y_range = np.ptp(positions[:, 1])
        
        if x_range > y_range:
            self.primary_axis = 'x'
            # 判斷方向
            if positions[-1, 0] > positions[0, 0]:
                self.motion_direction = 1
            else:
                self.motion_direction = -1
        else:
            self.primary_axis = 'y'
            if positions[-1, 1] > positions[0, 1]:
                self.motion_direction = 1
            else:
                self.motion_direction = -1
                
    def update(self, x_mm, y_mm, current_time=None):
        """
        更新控制器
        
        Args:
            x_mm: 當前 x 位置 (mm)
            y_mm: 當前 y 位置 (mm)
            current_time: 當前時間
        
        Returns:
            (ch1_voltage, ch2_voltage): 建議的通道電壓
        """
        if not self.enabled or not np.isfinite(x_mm) or not np.isfinite(y_mm):
            return self.base_voltage, self.base_voltage
            
        # 設定原點（第一次測量）
        if not self.origin_set:
            self.set_origin(y_mm)
            return self.base_voltage, self.base_voltage
        
        # 計算橫向偏移（相對於原點）
        if self.primary_axis == 'x':
            lateral_error = y_mm - self.origin_y
        else:
            lateral_error = x_mm - self.origin_y  # 此時 origin_y 實際存的是 x
        
        # PID 計算補償量
        correction = self.pid.update(lateral_error, current_time)
        
        # 應用到電壓（透過調整兩通道電壓差來校正方向）
        # 正偏移 -> 減少 ch1 或增加 ch2 來向負方向校正
        ch1_voltage = self.base_voltage - correction * 0.5
        ch2_voltage = self.base_voltage + correction * 0.5
        
        # 確保電壓在合理範圍
        ch1_voltage = np.clip(ch1_voltage, 0.5, 2.5)
        ch2_voltage = np.clip(ch2_voltage, 0.5, 2.5)
        
        return ch1_voltage, ch2_voltage
    
    def get_status(self):
        """獲取控制器狀態"""
        return {
            'enabled': self.enabled,
            'origin_set': self.origin_set,
            'origin_y': self.origin_y,
            'base_voltage': self.base_voltage,
            'pid_error': self.pid.prev_error,
            'pid_integral': self.pid.integral
        }


class RotationModeController:
    """
    Rotation Mode 智能控制器
    
    功能：
    - 旋轉到指定角度（預設 90 度）
    - 如果超過目標角度，自動反向旋轉校正
    - 達到目標後停止
    
    FG 模式對應：
    - Mode 2：順時針旋轉
    - Mode 4：逆時針旋轉
    """
    
    # 控制狀態
    STATE_ROTATING = 'rotating'       # 正向旋轉中
    STATE_CORRECTING = 'correcting'   # 超過目標，反向校正中
    STATE_COMPLETE = 'complete'       # 達到目標，停止
    
    def __init__(self, target_angle=90.0, angle_tolerance=2.0, base_voltage=1.2,
                 rotation_mode=2, overshoot_tolerance=5.0):
        """
        初始化 Rotation Mode 控制器
        
        Args:
            target_angle: 目標旋轉角度 (度)
            angle_tolerance: 角度容差 (度)，達到目標±容差即停止
            base_voltage: 基準電壓 (V)
            rotation_mode: 初始旋轉使用的 FG 模式 (2 或 4)
            overshoot_tolerance: 超過容差 (度)，超過此值觸發反向校正
        """
        self.target_angle = target_angle
        self.angle_tolerance = angle_tolerance
        self.base_voltage = base_voltage
        self.rotation_mode = rotation_mode  # 2 = 順時針, 4 = 逆時針
        self.overshoot_tolerance = overshoot_tolerance
        
        # 狀態
        self.state = self.STATE_ROTATING
        self.origin_angle = 0.0
        self.origin_set = False
        self.enabled = False
        self.current_rotation = 0.0
        
        # 旋轉方向：+1 = 順時針, -1 = 逆時針
        self.rotation_direction = 1 if rotation_mode == 2 else -1
        
        # 統計
        self.correction_count = 0
        
    def reset(self):
        """重置控制器"""
        self.state = self.STATE_ROTATING
        self.origin_set = False
        self.origin_angle = 0.0
        self.current_rotation = 0.0
        self.correction_count = 0
        
    def enable(self, enabled=True):
        """啟用/停用控制器"""
        self.enabled = enabled
        if enabled:
            self.state = self.STATE_ROTATING
            print(f"[RotationMode] 已啟用，目標: {self.target_angle}°, 容差: ±{self.angle_tolerance}°")
        else:
            print("[RotationMode] 已停用")
        
    def set_target_angle(self, angle):
        """設定目標角度"""
        self.target_angle = angle
        self.state = self.STATE_ROTATING
        
    def set_rotation_mode(self, mode):
        """設定旋轉 FG 模式"""
        self.rotation_mode = mode
        self.rotation_direction = 1 if mode == 2 else -1
        
    def set_origin(self, angle_deg):
        """設定原點角度"""
        self.origin_angle = angle_deg
        self.origin_set = True
        print(f"[RotationMode] 起始角度: {angle_deg:.1f}°")
        
    def update(self, angle_deg, current_time=None):
        """
        更新控制器
        
        Args:
            angle_deg: 當前角度（度）
            current_time: 當前時間
        
        Returns:
            dict: 控制結果
                - 'fg_mode': 建議的 FG 模式 (2 或 4)
                - 'voltage': 建議電壓
                - 'should_output': 是否應繼續輸出
                - 'state': 當前控制狀態
                - 'mode_changed': 是否切換了模式
                - 'complete': 是否完成
        """
        result = {
            'fg_mode': self.rotation_mode,
            'voltage': self.base_voltage,
            'should_output': True,
            'state': self.state,
            'mode_changed': False,
            'complete': False
        }
        
        if not self.enabled or not np.isfinite(angle_deg):
            return result
            
        # 設定原點（第一次測量）
        if not self.origin_set:
            self.set_origin(angle_deg)
            return result
        
        # 計算相對旋轉角度（有符號）
        rotation_from_origin = angle_deg - self.origin_angle
        self.current_rotation = rotation_from_origin
        
        # 計算與目標的差距
        # 對於順時針旋轉，目標是達到 +target_angle
        # 對於逆時針旋轉，目標是達到 -target_angle
        target_signed = self.target_angle * self.rotation_direction
        angle_error = target_signed - rotation_from_origin
        
        # 狀態機
        if self.state == self.STATE_ROTATING:
            # 正向旋轉中
            result['fg_mode'] = self.rotation_mode
            
            # 檢查是否達到目標範圍
            if abs(angle_error) <= self.angle_tolerance:
                # 達到目標！
                self.state = self.STATE_COMPLETE
                result['should_output'] = False
                result['complete'] = True
                result['state'] = self.state
                print(f"[RotationMode] ✓ 達到目標！旋轉了 {abs(rotation_from_origin):.1f}°")
                return result
            
            # 檢查是否超過目標（需要反向校正）
            if self.rotation_direction > 0:  # 順時針
                if rotation_from_origin > target_signed + self.overshoot_tolerance:
                    # 超過了，需要逆時針校正
                    self.state = self.STATE_CORRECTING
                    result['fg_mode'] = 4  # 逆時針
                    result['mode_changed'] = True
                    self.correction_count += 1
                    print(f"[RotationMode] 超過目標 {rotation_from_origin:.1f}° > {target_signed:.1f}°, 反向校正 (Mode 4)")
            else:  # 逆時針
                if rotation_from_origin < target_signed - self.overshoot_tolerance:
                    # 超過了，需要順時針校正
                    self.state = self.STATE_CORRECTING
                    result['fg_mode'] = 2  # 順時針
                    result['mode_changed'] = True
                    self.correction_count += 1
                    print(f"[RotationMode] 超過目標 {rotation_from_origin:.1f}° < {target_signed:.1f}°, 反向校正 (Mode 2)")
            
            # 根據剩餘角度調整電壓（接近目標時減速）
            remaining = abs(angle_error)
            if remaining < 20:  # 最後 20 度開始減速
                result['voltage'] = self.base_voltage * (remaining / 20.0)
                result['voltage'] = max(result['voltage'], 0.5)  # 最低維持 0.5V
                
        elif self.state == self.STATE_CORRECTING:
            # 反向校正中
            # 根據當前角度判斷校正方向
            if angle_error > 0:
                # 需要繼續順時針
                result['fg_mode'] = 2
            else:
                # 需要繼續逆時針
                result['fg_mode'] = 4
            
            # 檢查是否校正完成
            if abs(angle_error) <= self.angle_tolerance:
                self.state = self.STATE_COMPLETE
                result['should_output'] = False
                result['complete'] = True
                print(f"[RotationMode] ✓ 校正完成！最終角度: {abs(rotation_from_origin):.1f}°")
            
            # 減速
            remaining = abs(angle_error)
            if remaining < 15:
                result['voltage'] = self.base_voltage * (remaining / 15.0)
                result['voltage'] = max(result['voltage'], 0.5)
                
        elif self.state == self.STATE_COMPLETE:
            # 已完成
            result['should_output'] = False
            result['complete'] = True
        
        result['state'] = self.state
        return result
    
    def is_complete(self):
        """檢查旋轉是否完成"""
        return self.state == self.STATE_COMPLETE
    
    def get_status(self):
        """獲取控制器狀態"""
        return {
            'enabled': self.enabled,
            'state': self.state,
            'target_angle': self.target_angle,
            'current_rotation': self.current_rotation,
            'rotation_mode': self.rotation_mode,
            'rotation_direction': self.rotation_direction,
            'origin_set': self.origin_set,
            'origin_angle': self.origin_angle,
            'correction_count': self.correction_count
        }


class StraightWithCorrectionController:
    """
    Straight Mode 智能校正控制器
    
    功能：
    - 偵測角度偏移
    - 當角度偏離直線姿態過大時，自動切換到旋轉模式校正
    - 校正完成後自動切回直走模式
    
    FG 模式對應：
    - Mode 1, 3：直走/後退
    - Mode 2, 4：左右旋轉
    """
    
    # 控制狀態
    STATE_STRAIGHT = 'straight'      # 正常直走
    STATE_CORRECTING = 'correcting'  # 旋轉校正中
    STATE_RESUMING = 'resuming'      # 校正完成，恢復直走
    
    def __init__(self, base_voltage=1.2, angle_threshold=10.0, 
                 correction_tolerance=2.0, straight_mode=1, voltage_correction_range=0.3):
        """
        初始化控制器
        
        Args:
            base_voltage: 基準電壓 (V)
            angle_threshold: 角度偏移閾值 (度)，超過此值觸發旋轉校正
            correction_tolerance: 校正容差 (度)，角度回到此範圍內視為校正完成
            straight_mode: 直走使用的 FG 模式 (1 或 3)
            voltage_correction_range: 直走時的電壓校正範圍 (V)
        """
        self.base_voltage = base_voltage
        self.angle_threshold = angle_threshold
        self.correction_tolerance = correction_tolerance
        self.straight_mode = straight_mode  # 1 或 3
        self.voltage_correction_range = voltage_correction_range
        
        # 狀態
        self.state = self.STATE_STRAIGHT
        self.enabled = False
        
        # 基準角度（直走時的目標角度）
        self.reference_angle = 0.0
        self.reference_set = False
        
        # 橫向偏移控制（直走時的電壓調整）
        self.origin_y = 0.0
        self.origin_set = False
        
        # 旋轉校正相關
        self.correction_direction = 0  # +1 = 順時針, -1 = 逆時針
        self.correction_start_angle = 0.0
        
        # 電壓 PID
        self.voltage_pid = PIDController(
            kp=0.03,
            ki=0.005,
            kd=0.01,
            setpoint=0.0,
            output_min=-voltage_correction_range,
            output_max=voltage_correction_range,
            deadband=0.3  # 0.3mm 以內不校正
        )
        
        # 統計
        self.correction_count = 0
        
    def reset(self):
        """重置控制器"""
        self.state = self.STATE_STRAIGHT
        self.reference_set = False
        self.reference_angle = 0.0
        self.origin_set = False
        self.origin_y = 0.0
        self.correction_direction = 0
        self.correction_start_angle = 0.0
        self.voltage_pid.reset()
        self.correction_count = 0
        
    def enable(self, enabled=True):
        """啟用/停用控制器"""
        self.enabled = enabled
        self.voltage_pid.enabled = enabled
        if enabled:
            print(f"[StraightCorrection] 已啟用 (角度閾值: ±{self.angle_threshold}°)")
        else:
            print("[StraightCorrection] 已停用")
            
    def set_reference_angle(self, angle_deg):
        """設定基準角度（直走的目標方向）"""
        self.reference_angle = angle_deg
        self.reference_set = True
        print(f"[StraightCorrection] 基準角度設定: {angle_deg:.1f}°")
        
    def set_origin_y(self, y_mm):
        """設定 Y 軸原點"""
        self.origin_y = y_mm
        self.origin_set = True
        
    def _normalize_angle(self, angle):
        """將角度正規化到 [-90, 90] 範圍"""
        while angle > 90:
            angle -= 180
        while angle < -90:
            angle += 180
        return angle
        
    def update(self, x_mm, y_mm, angle_deg, current_time=None):
        """
        更新控制器
        
        Args:
            x_mm: 當前 x 位置 (mm)
            y_mm: 當前 y 位置 (mm)
            angle_deg: 當前角度 (度)
            current_time: 當前時間
        
        Returns:
            dict: 控制結果
                - 'fg_mode': 建議的 FG 模式 (1-4)
                - 'ch1_voltage': CH1 電壓
                - 'ch2_voltage': CH2 電壓
                - 'state': 當前控制狀態
                - 'angle_error': 角度誤差
                - 'mode_changed': 是否切換了模式
        """
        result = {
            'fg_mode': self.straight_mode,
            'ch1_voltage': self.base_voltage,
            'ch2_voltage': self.base_voltage,
            'state': self.state,
            'angle_error': 0.0,
            'mode_changed': False
        }
        
        if not self.enabled or not np.isfinite(angle_deg):
            return result
            
        # 設定基準角度（第一次測量）
        if not self.reference_set:
            self.set_reference_angle(angle_deg)
            
        # 設定 Y 軸原點
        if not self.origin_set and np.isfinite(y_mm):
            self.set_origin_y(y_mm)
            
        # 計算角度誤差（相對於基準角度）
        angle_error = self._normalize_angle(angle_deg - self.reference_angle)
        result['angle_error'] = angle_error
        
        # 狀態機
        if self.state == self.STATE_STRAIGHT:
            # 正常直走狀態
            result['fg_mode'] = self.straight_mode
            
            # 檢查是否需要校正
            if abs(angle_error) > self.angle_threshold:
                # 需要旋轉校正
                self.state = self.STATE_CORRECTING
                self.correction_start_angle = angle_deg
                
                # 決定旋轉方向
                if angle_error > 0:
                    # 順時針偏移，需要逆時針校正 (Mode 4)
                    self.correction_direction = -1
                    result['fg_mode'] = 4
                else:
                    # 逆時針偏移，需要順時針校正 (Mode 2)
                    self.correction_direction = 1
                    result['fg_mode'] = 2
                    
                result['mode_changed'] = True
                self.correction_count += 1
                print(f"[StraightCorrection] 觸發校正 #{self.correction_count}: "
                      f"角度誤差 {angle_error:+.1f}°, 切換到 Mode {result['fg_mode']}")
            else:
                # 正常直走，調整電壓來減少橫向偏移
                if self.origin_set and np.isfinite(y_mm):
                    lateral_error = y_mm - self.origin_y
                    correction = self.voltage_pid.update(lateral_error, current_time)
                    result['ch1_voltage'] = self.base_voltage - correction * 0.5
                    result['ch2_voltage'] = self.base_voltage + correction * 0.5
                    # 限幅
                    result['ch1_voltage'] = np.clip(result['ch1_voltage'], 0.5, 2.5)
                    result['ch2_voltage'] = np.clip(result['ch2_voltage'], 0.5, 2.5)
                    
        elif self.state == self.STATE_CORRECTING:
            # 旋轉校正中
            if self.correction_direction > 0:
                result['fg_mode'] = 2  # 順時針旋轉
            else:
                result['fg_mode'] = 4  # 逆時針旋轉
                
            # 檢查是否校正完成
            if abs(angle_error) <= self.correction_tolerance:
                # 校正完成，切回直走
                self.state = self.STATE_RESUMING
                result['fg_mode'] = self.straight_mode
                result['mode_changed'] = True
                print(f"[StraightCorrection] 校正完成，角度誤差 {angle_error:+.1f}°, 切回 Mode {self.straight_mode}")
                
        elif self.state == self.STATE_RESUMING:
            # 恢復直走（給一小段時間穩定後回到 STRAIGHT 狀態）
            result['fg_mode'] = self.straight_mode
            self.state = self.STATE_STRAIGHT
            
        result['state'] = self.state
        return result
    
    def get_status(self):
        """獲取控制器狀態"""
        return {
            'enabled': self.enabled,
            'state': self.state,
            'reference_angle': self.reference_angle,
            'reference_set': self.reference_set,
            'angle_threshold': self.angle_threshold,
            'correction_tolerance': self.correction_tolerance,
            'correction_count': self.correction_count,
            'straight_mode': self.straight_mode
        }


class UnifiedPIDController:
    """
    統一 PID 控制介面
    根據運動模式自動選擇控制器
    """
    
    def __init__(self, mode='straight', base_voltage=1.2, straight_mode=1,
                 angle_threshold=10.0, correction_tolerance=2.0):
        """
        初始化統一控制器
        
        Args:
            mode: 'straight' 或 'rotation'
            base_voltage: 基準電壓
            straight_mode: 直走使用的 FG 模式 (1 或 3)
            angle_threshold: 角度偏移閾值（度）
            correction_tolerance: 校正容差（度）
        """
        self.mode = mode.lower()
        self.base_voltage = base_voltage
        
        # 建立控制器
        self.straight_ctrl = StraightModeController(base_voltage=base_voltage)
        self.rotation_ctrl = RotationModeController(base_voltage=base_voltage)
        self.smart_straight_ctrl = StraightWithCorrectionController(
            base_voltage=base_voltage,
            angle_threshold=angle_threshold,
            correction_tolerance=correction_tolerance,
            straight_mode=straight_mode
        )
        
        # 使用智能校正控制器作為 straight mode 的預設
        self.use_smart_correction = True
        
    def set_mode(self, mode):
        """設定運動模式"""
        self.mode = mode.lower()
        
    def set_straight_mode(self, fg_mode):
        """設定直走使用的 FG 模式"""
        self.smart_straight_ctrl.straight_mode = fg_mode
        
    def reset(self):
        """重置控制器"""
        self.straight_ctrl.reset()
        self.rotation_ctrl.reset()
        self.smart_straight_ctrl.reset()
        
    def enable(self, enabled=True):
        """啟用/停用控制器"""
        if self.mode == 'straight':
            if self.use_smart_correction:
                self.smart_straight_ctrl.enable(enabled)
            else:
                self.straight_ctrl.enable(enabled)
        else:
            self.rotation_ctrl.enable(enabled)
        
    def update(self, x_mm, y_mm, angle_deg, current_time=None):
        """
        更新控制器
        
        Args:
            x_mm: 當前 x 位置 (mm)
            y_mm: 當前 y 位置 (mm)
            angle_deg: 當前角度（已解包裹）
            current_time: 當前時間
        
        Returns:
            dict: 控制結果
                - 'ch1_voltage': CH1 電壓
                - 'ch2_voltage': CH2 電壓
                - 'should_output': 是否應繼續輸出
                - 'complete': 是否完成（僅 rotation mode）
                - 'fg_mode': 建議的 FG 模式（僅 smart straight mode）
                - 'mode_changed': 是否需要切換 FG 模式
        """
        result = {
            'ch1_voltage': self.base_voltage,
            'ch2_voltage': self.base_voltage,
            'should_output': True,
            'complete': False,
            'fg_mode': None,
            'mode_changed': False
        }
        
        if self.mode == 'straight':
            if self.use_smart_correction:
                # 使用智能校正控制器
                smart_result = self.smart_straight_ctrl.update(x_mm, y_mm, angle_deg, current_time)
                result['ch1_voltage'] = smart_result['ch1_voltage']
                result['ch2_voltage'] = smart_result['ch2_voltage']
                result['fg_mode'] = smart_result['fg_mode']
                result['mode_changed'] = smart_result['mode_changed']
                result['state'] = smart_result['state']
                result['angle_error'] = smart_result['angle_error']
            else:
                # 使用簡單控制器
                ch1, ch2 = self.straight_ctrl.update(x_mm, y_mm, current_time)
                result['ch1_voltage'] = ch1
                result['ch2_voltage'] = ch2
        else:  # rotation
            rot_result = self.rotation_ctrl.update(angle_deg, current_time)
            result['ch1_voltage'] = rot_result['voltage']
            result['ch2_voltage'] = rot_result['voltage']
            result['should_output'] = rot_result['should_output']
            result['complete'] = rot_result['complete']
            result['fg_mode'] = rot_result['fg_mode']
            result['mode_changed'] = rot_result['mode_changed']
            result['state'] = rot_result['state']
            
        return result
    
    def get_status(self):
        """獲取控制器狀態"""
        return {
            'mode': self.mode,
            'use_smart_correction': self.use_smart_correction,
            'straight': self.straight_ctrl.get_status(),
            'rotation': self.rotation_ctrl.get_status(),
            'smart_straight': self.smart_straight_ctrl.get_status()
        }
