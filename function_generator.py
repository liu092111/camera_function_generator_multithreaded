# -*- coding: utf-8 -*-
"""
函數產生器控制模組
提供函數產生器的完整控制邏輯
"""

import time
import numpy as np
from config import FG_RESOURCE_STRING, FG_MODE_CONFIGS, HAVE_VISA

if HAVE_VISA:
    import pyvisa as visa


class FunctionGeneratorController:
    """函數產生器控制器"""
    
    def __init__(self):
        self.inst = None
        self.sampling_rates = {}
        self.current_mode = None
        self.connected = False
        self.continuous_output_setup = False
        self.current_wave_group = None  # '25k' or '47k' - 用於快速切換追蹤
        self.current_ch1_pol = None  # 追蹤當前 CH1 極性
        self.current_ch2_pol = None  # 追蹤當前 CH2 極性
        
    def connect(self):
        """連接函數產生器"""
        if not HAVE_VISA:
            print("PyVISA 未安裝，函數產生器功能已禁用")
            return False
            
        try:
            rm = visa.ResourceManager()
            self.inst = rm.open_resource(FG_RESOURCE_STRING)
            try:
                self.inst.control_ren(6)
            except:
                pass
            
            self.reset_instrument()
            self.sampling_rates = self.preload_waveforms()
            self.connected = True
            print("✓ 函數產生器已連接")
            return True
            
        except Exception as e:
            print(f"✗ 函數產生器連接失敗：{e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """斷開函數產生器"""
        if self.inst and self.connected:
            try:
                self.reset_instrument()
                self.inst.close()
                print("✓ 函數產生器已斷開")
            except:
                pass
        self.connected = False
    
    def reset_instrument(self):
        """重置儀器"""
        if not self.inst:
            return
            
        self.inst.write('OUTP1 OFF')
        self.inst.write('OUTP2 OFF')
        self.inst.write('SOUR1:TRACK OFF')
        self.inst.write('SOUR2:TRACK OFF')
        self.inst.write('*WAI')
        
        self.inst.write('SOUR1:FUNC SIN')
        self.inst.write('SOUR2:FUNC SIN')
        self.inst.write('SOUR1:FREQ 1000')
        self.inst.write('SOUR2:FREQ 1000')
        self.inst.write('SOUR1:VOLT 0.1')
        self.inst.write('SOUR2:VOLT 0.1')
        self.inst.write('*WAI')
        
        self.inst.write('SOUR1:DATA:VOL:CLE')
        self.inst.write('SOUR2:DATA:VOL:CLE')
        self.inst.write('*WAI')
    
    def load_waveform_csv(self, filename):
        """
        載入 CSV 波形檔案
        
        Args:
            filename: CSV 檔案路徑
        
        Returns:
            (times, values) 陣列
        """
        times = []
        values = []
        
        try:
            with open(filename, 'r') as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith('#'):
                        continue
                    
                    if ',' in line:
                        parts = line.split(',')
                    else:
                        parts = line.split()
                    
                    if len(parts) >= 2:
                        try:
                            t = float(parts[0])
                            v = float(parts[1])
                            times.append(t)
                            values.append(v)
                        except ValueError:
                            continue
        except FileNotFoundError:
            print(f"警告：找不到波形檔案 {filename}")
            return np.array([]), np.array([])
        
        return np.array(times), np.array(values)
    
    def align_waveforms(self, file1, file2):
        """
        對齊波形
        
        Args:
            file1: 第一個波形檔案
            file2: 第二個波形檔案
        
        Returns:
            (sig1, sig2, sRate, points) - 對齊後的波形和採樣率
        """
        times1, values1 = self.load_waveform_csv(file1)
        times2, values2 = self.load_waveform_csv(file2)
        
        if len(times1) == 0 or len(times2) == 0:
            print("使用預設正弦波")
            t = np.linspace(0, 1, 2000)
            sin1 = np.sin(2 * np.pi * t)
            sin2 = np.cos(2 * np.pi * t)
            return sin1.astype('f4'), sin2.astype('f4'), 2000.0, 2000
        
        t_start = max(times1[0], times2[0])
        t_end = min(times1[-1], times2[-1])
        
        dt1 = np.mean(np.diff(times1))
        dt2 = np.mean(np.diff(times2))
        dt_unified = min(dt1, dt2)
        
        unified_times = np.arange(t_start, t_end + dt_unified, dt_unified)
        
        aligned_values1 = np.interp(unified_times, times1, values1)
        aligned_values2 = np.interp(unified_times, times2, values2)
        
        sRate = 1 / dt_unified
        
        return (aligned_values1.astype('f4'), aligned_values2.astype('f4'), 
                sRate, len(unified_times))
    
    def preload_waveforms(self):
        """
        預載入所有波形
        
        Returns:
            採樣率資訊字典
        """
        if not self.inst:
            return {}
            
        print("載入函數產生器波形...")
        
        self.inst.write('OUTP1 OFF')
        self.inst.write('OUTP2 OFF')
        self.inst.write('*WAI')
        
        sampling_rates = {}
        
        for mode_num, config in FG_MODE_CONFIGS.items():
            try:
                sig1, sig2, sRate, points = self.align_waveforms(
                    config['file1'], config['file2']
                )
                
                freq = sRate / points
                
                sampling_rates[mode_num] = {
                    'sRate': sRate,
                    'points': points,
                    'freq': freq,
                    'name1': config['name1'],
                    'name2': config['name2']
                }
                
                self.inst.write_binary_values(
                    f'SOUR1:DATA:ARB {config["name1"]},', 
                    sig1, datatype='f', is_big_endian=False
                )
                self.inst.write('*WAI')
                
                self.inst.write_binary_values(
                    f'SOUR2:DATA:ARB {config["name2"]},', 
                    sig2, datatype='f', is_big_endian=False
                )
                self.inst.write('*WAI')
                
                print(f"  Mode {mode_num} 已載入 - 頻率: {freq:.1f} Hz")
                
            except Exception as e:
                print(f"  Mode {mode_num} 載入失敗: {e}")
        
        return sampling_rates
    
    def setup_continuous_output(self):
        """
        設定連續輸出模式
        
        Returns:
            成功返回 True，失敗返回 False
        """
        if not self.connected:
            return False
            
        try:
            print("設定連續輸出模式...")
            
            base_mode = self.sampling_rates[1]
            
            self.inst.write('SOUR1:FUNC ARB')
            self.inst.write('SOUR2:FUNC ARB')
            self.inst.write(f'SOUR1:FUNC:ARB {base_mode["name1"]}')
            self.inst.write(f'SOUR2:FUNC:ARB {base_mode["name2"]}')
            self.inst.write(f'SOUR1:FUNC:ARB:SRAT {base_mode["sRate"]:.0f}')
            self.inst.write(f'SOUR2:FUNC:ARB:SRAT {base_mode["sRate"]:.0f}')
            self.inst.write(f'SOUR1:FREQ {base_mode["freq"]}')
            self.inst.write(f'SOUR2:FREQ {base_mode["freq"]}')
            self.inst.write('SOUR1:PHAS 0')
            self.inst.write('SOUR2:PHAS 0')
            self.inst.write('SOUR1:VOLT:OFFS 0')
            self.inst.write('SOUR2:VOLT:OFFS 0')
            self.inst.write('*WAI')
            
            self.inst.write('SOUR1:TRACK OFF')
            self.inst.write('SOUR2:TRACK OFF')
            self.inst.write('SOUR2:TRACK ON')
            self.inst.write('SOUR2:PHAS:SYNC')
            self.inst.write('*WAI')
            
            self.inst.write('SOUR1:VOLT 0')
            self.inst.write('SOUR2:VOLT 0')
            self.inst.write('OUTP1:POL NORM')
            self.inst.write('OUTP2:POL INV')
            self.inst.write('*WAI')
            
            self.inst.write('OUTP1 ON')
            self.inst.write('OUTP2 ON')
            self.inst.write('*WAI')
            
            self.continuous_output_setup = True
            print("✓ 連續輸出模式已設定完成")
            return True
            
        except Exception as e:
            print(f"設定連續輸出模式失敗: {e}")
            return False
    
    def switch_mode(self, mode_num):
        """
        快速模式切換（舊版，保留兼容）
        
        Args:
            mode_num: 模式編號 (1-4)
        
        Returns:
            成功返回 True，失敗返回 False
        """
        # 使用新的超快速切換方法
        return self.fast_switch_mode(mode_num)
    
    def fast_switch_mode(self, mode_num, silent=False):
        """
        超快速模式切換（參考 dual_modal_4ms.py 的 ultra_fast_amplitude_switch 實現）
        
        優化策略：
        - 追蹤極性狀態，只在需要時才切換
        - 最小化 *WAI 呼叫（每個約 1-2ms 延遲）
        - 跳過已設定的冗餘指令（FUNC ARB, offset, phase, track）
        - 同頻率組（只換極性）：~1-2ms（Mode 1↔3 或 Mode 2↔4）
        - 跨頻率組（換波形+極性）：~3-4ms（Mode 1/3 ↔ Mode 2/4）
        
        Args:
            mode_num: 模式編號 (1-4)
            silent: 是否靜默模式
        
        Returns:
            成功返回 True，失敗返回 False
        """
        if not self.connected or mode_num not in self.sampling_rates:
            return False
        
        if not self.continuous_output_setup:
            if not self.setup_continuous_output():
                return False
        
        start_time = time.time()
        
        config = FG_MODE_CONFIGS[mode_num]
        mode_data = self.sampling_rates[mode_num]
        target_wave_group = '25k' if mode_num in [1, 3] else '47k'
        
        try:
            # 情況 1: 從待機狀態啟動（完整設定）
            if self.current_mode is None or self.current_wave_group is None:
                self.inst.write('SOUR1:FUNC ARB')
                self.inst.write('SOUR2:FUNC ARB')
                self.inst.write(f'SOUR1:FUNC:ARB {mode_data["name1"]}')
                self.inst.write(f'SOUR2:FUNC:ARB {mode_data["name2"]}')
                self.inst.write(f'SOUR1:FUNC:ARB:SRAT {mode_data["sRate"]:.0f}')
                self.inst.write(f'SOUR2:FUNC:ARB:SRAT {mode_data["sRate"]:.0f}')
                self.inst.write(f'SOUR1:VOLT {config["ch1_volt"]}')
                self.inst.write(f'SOUR2:VOLT {config["ch2_volt"]}')
                self.inst.write('SOUR1:VOLT:OFFS 0')  
                self.inst.write('SOUR2:VOLT:OFFS 0')
                self.inst.write(f'SOUR1:FREQ {mode_data["freq"]}')
                self.inst.write(f'SOUR2:FREQ {mode_data["freq"]}')
                self.inst.write('SOUR1:PHAS 0')
                self.inst.write('SOUR2:PHAS 0')
                self.inst.write('*WAI')
                
                self.inst.write('SOUR1:TRACK OFF')
                self.inst.write('SOUR2:TRACK OFF')
                self.inst.write('SOUR2:TRACK ON')
                self.inst.write('SOUR2:PHAS:SYNC')
                self.inst.write('*WAI')
                
                self.inst.write(f'OUTP1:POL {config["ch1_pol"]}')
                self.inst.write(f'OUTP2:POL {config["ch2_pol"]}')
                self.inst.write('*WAI')
                
                self.inst.write('OUTP1 ON')
                self.inst.write('OUTP2 ON') 
                self.inst.write('*WAI')
                
                switch_type = "完整啟動"
            else:
                # 判斷是否需要切換波形或極性
                need_waveform_switch = (self.current_wave_group != target_wave_group)
                need_polarity_change = (self.current_ch1_pol != config['ch1_pol'] or 
                                        self.current_ch2_pol != config['ch2_pol'])
                
                # 只在需要變更時才關閉輸出
                if need_waveform_switch or need_polarity_change:
                    self.inst.write('OUTP1 OFF')
                    self.inst.write('OUTP2 OFF')
                    
                    if need_waveform_switch:
                        # 切換波形、採樣率和頻率
                        self.inst.write(f'SOUR1:FUNC:ARB {mode_data["name1"]}')
                        self.inst.write(f'SOUR2:FUNC:ARB {mode_data["name2"]}')
                        self.inst.write(f'SOUR1:FUNC:ARB:SRAT {mode_data["sRate"]:.0f}')
                        self.inst.write(f'SOUR2:FUNC:ARB:SRAT {mode_data["sRate"]:.0f}')
                        self.inst.write(f'SOUR1:FREQ {mode_data["freq"]}')
                        self.inst.write(f'SOUR2:FREQ {mode_data["freq"]}')
                        self.inst.write('SOUR2:PHAS:SYNC')
                        self.inst.write('*WAI')
                    
                    if need_polarity_change:
                        self.inst.write(f'OUTP1:POL {config["ch1_pol"]}')
                        self.inst.write(f'OUTP2:POL {config["ch2_pol"]}')
                        self.inst.write('*WAI')
                    
                    # 重新開啟輸出
                    self.inst.write('OUTP1 ON')
                    self.inst.write('OUTP2 ON')
                    self.inst.write('*WAI')
                
                # 設定振幅（最快的操作，不需要關閉輸出）
                self.inst.write(f'SOUR1:VOLT {config["ch1_volt"]}')
                self.inst.write(f'SOUR2:VOLT {config["ch2_volt"]}')
                
                if need_waveform_switch and need_polarity_change:
                    switch_type = "波形+極性切換"
                elif need_waveform_switch:
                    switch_type = "波形切換"
                elif need_polarity_change:
                    switch_type = "極性切換"
                else:
                    switch_type = "振幅切換"
            
            switch_time = (time.time() - start_time) * 1000
            self.current_mode = mode_num
            self.current_wave_group = target_wave_group
            self.current_ch1_pol = config['ch1_pol']
            self.current_ch2_pol = config['ch2_pol']
            
            if not silent:
                print(f"FG Mode {mode_num} | {switch_time:.2f}ms | {switch_type} | "
                      f"Freq: {mode_data['freq']:.1f}Hz | "
                      f"CH1={config['ch1_pol']}/{config['ch1_volt']}V, "
                      f"CH2={config['ch2_pol']}/{config['ch2_volt']}V")
            return True
            
        except Exception as e:
            print(f"函數產生器模式切換失敗: {e}")
            return False
    
    def turn_off(self):
        """關閉函數產生器輸出"""
        if not self.connected:
            return
        try:
            self.inst.write('SOUR1:VOLT 0')
            self.inst.write('SOUR2:VOLT 0')
            self.inst.write('*WAI')
            self.current_mode = None
            # 保留 current_wave_group 以便下次快速切換
            print("函數產生器輸出已關閉 (電壓=0V)")
        except Exception as e:
            print(f"關閉函數產生器失敗: {e}")
    
    def ultra_fast_switch_mode(self, mode_num, silent=True):
        """
        用於 PID 控制的超快速模式切換（最小化延遲版本）
        
        專門為頻繁的 PID 控制切換優化：
        - 追蹤極性狀態，跳過不需要的指令
        - 最小化 *WAI 呼叫
        - 同組極性切換：~1-2ms
        - 跨組波形切換：~3-4ms
        
        Args:
            mode_num: 模式編號 (1-4)
            silent: 是否靜默模式（預設 True）
        
        Returns:
            成功返回 True，失敗返回 False
        """
        if not self.connected or mode_num not in self.sampling_rates:
            return False
        
        if not self.continuous_output_setup:
            if not self.setup_continuous_output():
                return False
        
        # 如果模式相同，不需要切換
        if self.current_mode == mode_num:
            return True
        
        start_time = time.time()
        prev_mode = self.current_mode
        
        config = FG_MODE_CONFIGS[mode_num]
        mode_data = self.sampling_rates[mode_num]
        target_wave_group = '25k' if mode_num in [1, 3] else '47k'
        
        try:
            # 情況 1: 從待機狀態啟動
            if self.current_mode is None or self.current_wave_group is None:
                result = self.fast_switch_mode(mode_num, silent=silent)
                return result
            
            # 判斷需要什麼變更
            need_waveform_switch = (self.current_wave_group != target_wave_group)
            need_polarity_change = (self.current_ch1_pol != config['ch1_pol'] or 
                                    self.current_ch2_pol != config['ch2_pol'])
            
            # 只在需要變更時才操作
            if need_waveform_switch or need_polarity_change:
                self.inst.write('OUTP1 OFF')
                self.inst.write('OUTP2 OFF')
                
                if need_waveform_switch:
                    self.inst.write(f'SOUR1:FUNC:ARB {mode_data["name1"]}')
                    self.inst.write(f'SOUR2:FUNC:ARB {mode_data["name2"]}')
                    self.inst.write(f'SOUR1:FUNC:ARB:SRAT {mode_data["sRate"]:.0f}')
                    self.inst.write(f'SOUR2:FUNC:ARB:SRAT {mode_data["sRate"]:.0f}')
                    self.inst.write(f'SOUR1:FREQ {mode_data["freq"]}')
                    self.inst.write(f'SOUR2:FREQ {mode_data["freq"]}')
                    self.inst.write('SOUR2:PHAS:SYNC')
                    self.inst.write('*WAI')
                
                if need_polarity_change:
                    self.inst.write(f'OUTP1:POL {config["ch1_pol"]}')
                    self.inst.write(f'OUTP2:POL {config["ch2_pol"]}')
                
                self.inst.write('OUTP1 ON')
                self.inst.write('OUTP2 ON')
                # 不使用 *WAI 以獲得最快速度
            
            # 設定振幅
            self.inst.write(f'SOUR1:VOLT {config["ch1_volt"]}')
            self.inst.write(f'SOUR2:VOLT {config["ch2_volt"]}')
            
            if need_waveform_switch:
                switch_type = "波形+極性切換"
            elif need_polarity_change:
                switch_type = "極性切換"
            else:
                switch_type = "振幅切換"
            
            switch_time = (time.time() - start_time) * 1000
            
            self.current_mode = mode_num
            self.current_wave_group = target_wave_group
            self.current_ch1_pol = config['ch1_pol']
            self.current_ch2_pol = config['ch2_pol']
            
            if not silent:
                print(f"[PID] Mode {prev_mode} → {mode_num} | {switch_time:.2f}ms | {switch_type}")
            
            return True
            
        except Exception as e:
            if not silent:
                print(f"超快速模式切換失敗: {e}")
            return False
    
    def set_voltages(self, ch1_volt, ch2_volt, silent=True):
        """
        動態設定兩通道電壓（用於 PID 控制）
        
        Args:
            ch1_volt: CH1 電壓 (V)
            ch2_volt: CH2 電壓 (V)
            silent: 是否靜默模式（不印出訊息）
        
        Returns:
            成功返回 True，失敗返回 False
        """
        if not self.connected or not self.continuous_output_setup:
            return False
        
        try:
            # 限制電壓範圍 (0 ~ 3V)
            ch1_volt = max(0.0, min(3.0, ch1_volt))
            ch2_volt = max(0.0, min(3.0, ch2_volt))
            
            self.inst.write(f'SOUR1:VOLT {ch1_volt:.3f}')
            self.inst.write(f'SOUR2:VOLT {ch2_volt:.3f}')
            # 不使用 *WAI 以減少延遲
            
            if not silent:
                print(f"FG 電壓更新: CH1={ch1_volt:.3f}V, CH2={ch2_volt:.3f}V")
            
            return True
            
        except Exception as e:
            if not silent:
                print(f"設定電壓失敗: {e}")
            return False
    
    def get_current_mode(self):
        """獲取當前模式"""
        return self.current_mode
    
    def is_output_active(self):
        """檢查輸出是否活躍"""
        return self.connected and self.continuous_output_setup and self.current_mode is not None
