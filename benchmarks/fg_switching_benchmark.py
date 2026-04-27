#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
fg_switching_benchmark.py — Keysight 33600A 模式切換時間 Benchmark

比較兩代切換實作的效能：
  第一代 (dual_modal_selector_4modes.py)：每次切換重新讀檔、上傳波形、設定全部參數
  第二代 (dual_modal_4ms.py)：啟動時預載波形，切換時只改極性或波形組
"""

import os
import sys
import time
import numpy as np
from datetime import datetime

try:
    import pyvisa as visa
except ImportError:
    print("錯誤：需要 pyvisa 套件。請執行 pip install pyvisa")
    sys.exit(1)


# ── Configuration ──────────────────────────────────────────────
VISA_RESOURCE = 'USB0::0x0957::0x5707::MY59000656::0::INSTR'
N_REPEATS = 50
VOLTAGE = 1.2

WAVEFORM_FILES = {
    '25k': {
        'file1': 'modal/ONEPERIOD_A_25k_50k_84p88deg_2000pts.csv',
        'file2': 'modal/ONEPERIOD_B_25k_50k_264p88deg_2000pts.csv',
        'name1': 'WF_25K_84',
        'name2': 'WF_25K_264',
    },
    '47k': {
        'file1': 'modal/ONEPERIOD_C_47k_94k_57p32deg_2000pts.csv',
        'file2': 'modal/ONEPERIOD_D_47k_94k_237p32deg_2000pts.csv',
        'name1': 'WF_47K_57',
        'name2': 'WF_47K_237',
    },
}

MODE_DEFS = {
    1: {'wave': '25k', 'ch1_pol': 'NORM', 'ch2_pol': 'INV'},
    2: {'wave': '47k', 'ch1_pol': 'NORM', 'ch2_pol': 'INV'},
    3: {'wave': '25k', 'ch1_pol': 'INV',  'ch2_pol': 'NORM'},
    4: {'wave': '47k', 'ch1_pol': 'INV',  'ch2_pol': 'NORM'},
}


# ── Waveform Utilities ────────────────────────────────────────

def load_waveform_csv(filename):
    """讀取 CSV 波形檔案，回傳 (times, values) numpy arrays。"""
    times, values = [], []
    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split(',') if ',' in line else line.split()
            if len(parts) >= 2:
                try:
                    times.append(float(parts[0]))
                    values.append(float(parts[1]))
                except ValueError:
                    continue
    return np.array(times), np.array(values)


def align_waveforms(file1, file2, invert_ch2=False):
    """讀取兩個波形檔並對齊至共同時間軸。"""
    t1, v1 = load_waveform_csv(file1)
    t2, v2 = load_waveform_csv(file2)
    t_start = max(t1[0], t2[0])
    t_end = min(t1[-1], t2[-1])
    dt = min(np.mean(np.diff(t1)), np.mean(np.diff(t2)))
    t_unified = np.arange(t_start, t_end + dt, dt)
    s1 = np.interp(t_unified, t1, v1).astype('f4')
    s2 = np.interp(t_unified, t2, v2).astype('f4')
    if invert_ch2:
        s2 = -s2
    srate = 1.0 / dt
    return s1, s2, srate, len(t_unified)


# ── Gen1: Full-Upload Switch ──────────────────────────────────

def gen1_switch(inst, mode_num):
    """
    複製 dual_modal_selector_4modes.py 的 run_mode() SCPI 序列。
    每次切換：關輸出 → 讀檔 → 上傳波形 → 設定參數 → 開輸出。
    計時：從第一個 write() 前到最後一個 write() 後。
    """
    mdef = MODE_DEFS[mode_num]
    wf = WAVEFORM_FILES[mdef['wave']]

    # ── 計時開始 ──
    t0 = time.perf_counter()

    inst.write('OUTP1 OFF')
    inst.write('OUTP2 OFF')
    inst.write('SOUR2:TRACK OFF')
    inst.write('*WAI')

    # 讀檔 + 對齊（gen1 每次切換都做，包含在計時範圍內）
    sig1, sig2, srate, npts = align_waveforms(
        wf['file1'], wf['file2'], invert_ch2=True
    )

    # 上傳 CH1 波形
    inst.write('SOUR1:DATA:VOL:CLE')
    inst.write_binary_values('SOUR1:DATA:ARB MODAL_84DEG,', sig1,
                             datatype='f', is_big_endian=False)
    inst.write('*WAI')
    inst.write('MMEM:STOR:DATA "INT:\\remoteAdded\\MODAL_84DEG.arb"')

    # 上傳 CH2 波形
    inst.write('SOUR2:DATA:VOL:CLE')
    inst.write_binary_values('SOUR2:DATA:ARB MODAL_264DEG,', sig2,
                             datatype='f', is_big_endian=False)
    inst.write('*WAI')
    inst.write('MMEM:STOR:DATA "INT:\\remoteAdded\\MODAL_264DEG.arb"')

    # ARB 參數設定
    inst.write('SOUR1:FUNC ARB')
    inst.write('SOUR1:FUNC:ARB MODAL_84DEG')
    inst.write(f'SOUR1:FUNC:ARB:SRAT {srate}')
    inst.write(f'SOUR1:VOLT {VOLTAGE}')
    inst.write('SOUR1:VOLT:OFFS 0')
    inst.write('SOUR2:FUNC ARB')
    inst.write('SOUR2:FUNC:ARB MODAL_264DEG')
    inst.write(f'SOUR2:FUNC:ARB:SRAT {srate}')
    inst.write(f'SOUR2:VOLT {VOLTAGE}')
    inst.write('SOUR2:VOLT:OFFS 0')

    freq = srate / npts
    inst.write(f'SOUR1:FREQ {freq}')
    inst.write(f'SOUR2:FREQ {freq}')
    inst.write('SOUR1:PHAS 0')
    inst.write('SOUR2:PHAS 0')
    inst.write('*WAI')

    # Sync / Track
    inst.write('SOUR1:TRACK OFF')
    inst.write('SOUR2:TRACK OFF')
    inst.write('SOUR2:TRACK ON')
    inst.write('*WAI')
    inst.write('SOUR2:PHAS:SYNC')
    inst.write('SOUR2:PHAS 0')
    inst.write('*WAI')

    # 極性
    inst.write(f'OUTP1:POL {mdef["ch1_pol"]}')
    inst.write(f'OUTP2:POL {mdef["ch2_pol"]}')

    # Sync 輸出
    inst.write('OUTP:SYNC ON')
    inst.write('OUTP:SYNC:SOURCE CH1')
    inst.write('OUTP:SYNC:MODE MARK')

    # 開啟輸出
    inst.write('OUTP1 ON')
    inst.write('OUTP2 ON')
    inst.write('*WAI')

    # ── 計時結束 ──
    t1 = time.perf_counter()

    # 等待儀器完成所有操作，確保下次量測不受影響
    inst.query('*OPC?')

    return (t1 - t0) * 1000.0


# ── Gen2: Preload All Waveforms ───────────────────────────────

def gen2_preload(inst):
    """
    複製 dual_modal_4ms.py 的 preload + setup 流程。
    一次性上傳所有波形，回傳 sampling_rates dict。
    """
    inst.write('SOUR1:DATA:VOL:CLE')
    inst.write('SOUR2:DATA:VOL:CLE')
    inst.write('*WAI')

    sampling_rates = {}
    uploaded = set()

    for mode_num in [1, 2, 3, 4]:
        wkey = MODE_DEFS[mode_num]['wave']
        wf = WAVEFORM_FILES[wkey]
        sig1, sig2, srate, npts = align_waveforms(
            wf['file1'], wf['file2'], invert_ch2=False
        )
        freq = srate / npts

        sampling_rates[mode_num] = {
            'sRate': srate,
            'points': npts,
            'freq': freq,
            'name1': wf['name1'],
            'name2': wf['name2'],
        }

        # 同波形組只上傳一次（Mode 1/3 共用 25k，Mode 2/4 共用 47k）
        if wkey not in uploaded:
            inst.write_binary_values(
                f'SOUR1:DATA:ARB {wf["name1"]},', sig1,
                datatype='f', is_big_endian=False
            )
            inst.write('*WAI')
            inst.write_binary_values(
                f'SOUR2:DATA:ARB {wf["name2"]},', sig2,
                datatype='f', is_big_endian=False
            )
            inst.write('*WAI')
            uploaded.add(wkey)

    # 設定初始狀態（Mode 1 波形，待機振幅）
    base = sampling_rates[1]
    inst.write('SOUR1:FUNC ARB')
    inst.write('SOUR2:FUNC ARB')
    inst.write(f'SOUR1:FUNC:ARB {base["name1"]}')
    inst.write(f'SOUR2:FUNC:ARB {base["name2"]}')
    inst.write(f'SOUR1:FUNC:ARB:SRAT {base["sRate"]:.0f}')
    inst.write(f'SOUR2:FUNC:ARB:SRAT {base["sRate"]:.0f}')
    inst.write(f'SOUR1:FREQ {base["freq"]}')
    inst.write(f'SOUR2:FREQ {base["freq"]}')
    inst.write('SOUR1:PHAS 0')
    inst.write('SOUR2:PHAS 0')
    inst.write('SOUR1:VOLT:OFFS 0')
    inst.write('SOUR2:VOLT:OFFS 0')
    inst.write(f'SOUR1:VOLT {VOLTAGE}')
    inst.write(f'SOUR2:VOLT {VOLTAGE}')
    inst.write('*WAI')

    # Track / Sync
    inst.write('SOUR1:TRACK OFF')
    inst.write('SOUR2:TRACK OFF')
    inst.write('SOUR2:TRACK ON')
    inst.write('SOUR2:PHAS:SYNC')
    inst.write('*WAI')

    # 預設極性 (Mode 1: NORM/INV)
    inst.write('OUTP1:POL NORM')
    inst.write('OUTP2:POL INV')
    inst.write('*WAI')

    # 開啟輸出
    inst.write('OUTP1 ON')
    inst.write('OUTP2 ON')
    inst.write('*WAI')

    inst.query('*OPC?')
    return sampling_rates


# ── Gen2: Mode Switch ─────────────────────────────────────────

def gen2_switch(inst, to_mode, from_mode, sampling_rates):
    """
    複製 dual_modal_4ms.py 的 ultra_fast_amplitude_switch() SCPI 序列。
    from_mode=None 代表從待機狀態啟動。

    條件邏輯：
      - 同組切換（如 1↔3）：只改極性
      - 跨組切換（如 1→2）：切換波形名稱 + 採樣率 + 頻率
      - 待機啟動（None→任意）：need_polarity=True，可能需要波形切換
    """
    to_cfg = MODE_DEFS[to_mode]

    if from_mode is None:
        need_wf = to_cfg['wave'] != '25k'  # 預設已載入 25k
        need_pol = True
    else:
        from_cfg = MODE_DEFS[from_mode]
        need_wf = from_cfg['wave'] != to_cfg['wave']
        need_pol = (from_cfg['ch1_pol'] != to_cfg['ch1_pol'] or
                    from_cfg['ch2_pol'] != to_cfg['ch2_pol'])

    # ── 計時開始 ──
    t0 = time.perf_counter()

    if need_wf or need_pol:
        inst.write('OUTP1 OFF; OUTP2 OFF')

        if need_wf:
            md = sampling_rates[to_mode]
            inst.write(f'SOUR1:FUNC:ARB {md["name1"]}')
            inst.write(f'SOUR2:FUNC:ARB {md["name2"]}')
            inst.write(f'SOUR1:FUNC:ARB:SRAT {md["sRate"]:.0f}')
            inst.write(f'SOUR2:FUNC:ARB:SRAT {md["sRate"]:.0f}')
            inst.write(f'SOUR1:FREQ {md["freq"]}')
            inst.write(f'SOUR2:FREQ {md["freq"]}')
            inst.write('SOUR2:PHAS:SYNC')
            inst.write('*WAI')

        if need_pol:
            inst.write(f'OUTP1:POL {to_cfg["ch1_pol"]}')
            inst.write(f'OUTP2:POL {to_cfg["ch2_pol"]}')
            inst.write('*WAI')

        inst.write('OUTP1 ON; OUTP2 ON')
        inst.write('*WAI')

    inst.write(f'SOUR1:VOLT {VOLTAGE}; SOUR2:VOLT {VOLTAGE}')
    inst.write('*WAI')

    # ── 計時結束 ──
    t1 = time.perf_counter()

    inst.query('*OPC?')
    return (t1 - t0) * 1000.0


# ── Utility ───────────────────────────────────────────────────

def safe_output_off(inst):
    """安全關閉輸出。"""
    try:
        inst.write('OUTP1 OFF')
        inst.write('OUTP2 OFF')
        inst.write('*WAI')
        inst.query('*OPC?')
    except Exception:
        try:
            inst.write('*RST')
            inst.write('*CLS')
        except Exception:
            pass


def enter_standby(inst):
    """進入待機：關閉輸出、振幅歸零。"""
    try:
        inst.write('OUTP1 OFF')
        inst.write('OUTP2 OFF')
        inst.write('SOUR1:VOLT 0')
        inst.write('SOUR2:VOLT 0')
        inst.write('*WAI')
        inst.query('*OPC?')
    except Exception:
        try:
            inst.write('*RST')
            inst.write('*CLS')
        except Exception:
            pass


def run_benchmark(label, switch_fn, n=N_REPEATS):
    """執行 n 次切換，收集時間統計。"""
    times = [switch_fn() for _ in range(n)]
    arr = np.array(times)
    return {
        'label': label,
        'mean': float(np.mean(arr)),
        'std': float(np.std(arr)),
        'min': float(np.min(arr)),
        'max': float(np.max(arr)),
    }


def fmt(r):
    """格式化統計結果。"""
    return (f"mean={r['mean']:6.1f} ms  std={r['std']:5.1f} ms  "
            f"min={r['min']:6.1f} ms  max={r['max']:6.1f} ms")


# ── Main ──────────────────────────────────────────────────────

def main():
    os.chdir(os.path.dirname(os.path.abspath(__file__)))

    header = [
        "=" * 56,
        "  函數產生器切換時間 Benchmark",
        "  設備：Keysight 33600A",
        f"  重複次數：{N_REPEATS} 次 / 情境",
        "=" * 56,
    ]
    for ln in header:
        print(ln)

    # ── 連線 ──
    print("\n連線中...")
    try:
        rm = visa.ResourceManager()
        inst = rm.open_resource(VISA_RESOURCE)
        inst.timeout = 30000  # 30 秒，避免波形上傳時超時
        try:
            inst.control_ren(6)
        except Exception:
            pass
        idn = inst.query('*IDN?').strip()
        print(f"已連線：{idn}")
    except Exception as e:
        print(f"\n錯誤：無法連線至函數產生器")
        print(f"  VISA resource: {VISA_RESOURCE}")
        print(f"  {e}")
        print("\n請確認：")
        print("  1. 儀器已開機且 USB 已連接")
        print("  2. Keysight IO Libraries 已安裝")
        print("  3. Resource string 正確")
        avail = rm.list_resources() if 'rm' in dir() else []
        if avail:
            print(f"  可用裝置：{avail}")
        sys.exit(1)

    # ── 初始化 ──
    print("初始化儀器...")
    inst.write('OUTP1 OFF')
    inst.write('OUTP2 OFF')
    inst.write('SOUR1:TRACK OFF')
    inst.write('SOUR2:TRACK OFF')
    inst.write('*WAI')
    inst.write('*CLS')
    inst.write('*WAI')
    inst.write('FORM:BORD SWAP')
    inst.write('MMEMORY:MDIR "INT:\\remoteAdded"')
    inst.query('*OPC?')

    results = []

    try:
        # ────────────────────────────────────────────────────────
        # 情境 1a：第一代切換 (Mode 1 ↔ Mode 2，每次重新上傳)
        # ────────────────────────────────────────────────────────
        print("\n[1a] 第一代切換 (Mode 1 ↔ Mode 2)...")
        gen1_switch(inst, 1)  # warm-up
        cur = [1]

        def bench_1a():
            target = 2 if cur[0] == 1 else 1
            dt = gen1_switch(inst, target)
            cur[0] = target
            return dt

        r = run_benchmark("1a", bench_1a)
        results.append(r)
        safe_output_off(inst)
        print(f"     {fmt(r)}")

        # ── 徹底重置儀器，準備 Gen2 ──
        print("\n重置儀器狀態...")
        inst.write('*RST')
        inst.write('*CLS')
        inst.query('*OPC?')
        time.sleep(2)
        inst.write('SOUR1:DATA:VOL:CLE')
        inst.write('SOUR2:DATA:VOL:CLE')
        inst.write('*WAI')
        inst.write('FORM:BORD SWAP')
        inst.query('*OPC?')

        # ────────────────────────────────────────────────────────
        # Gen2 預載波形
        # ────────────────────────────────────────────────────────
        print("\n預載波形（第二代初始化）...")
        sampling_rates = gen2_preload(inst)
        print("預載完成")

        # ────────────────────────────────────────────────────────
        # 情境 1b-i：同組切換 Mode 1 ↔ Mode 3（只改極性）
        # ────────────────────────────────────────────────────────
        print("\n[1b-i] 第二代同組切換 (Mode 1 ↔ Mode 3)...")
        gen2_switch(inst, 1, None, sampling_rates)  # 啟動到 Mode 1
        cur[0] = 1

        def bench_1bi():
            target = 3 if cur[0] == 1 else 1
            dt = gen2_switch(inst, target, cur[0], sampling_rates)
            cur[0] = target
            return dt

        r = run_benchmark("1b-i", bench_1bi)
        results.append(r)
        safe_output_off(inst)
        print(f"     {fmt(r)}")

        # ────────────────────────────────────────────────────────
        # 情境 1b-ii：同組切換 Mode 2 ↔ Mode 4（只改極性）
        # ────────────────────────────────────────────────────────
        print("\n[1b-ii] 第二代同組切換 (Mode 2 ↔ Mode 4)...")
        gen2_switch(inst, 2, 1, sampling_rates)  # 跨組到 Mode 2
        cur[0] = 2

        def bench_1bii():
            target = 4 if cur[0] == 2 else 2
            dt = gen2_switch(inst, target, cur[0], sampling_rates)
            cur[0] = target
            return dt

        r = run_benchmark("1b-ii", bench_1bii)
        results.append(r)
        safe_output_off(inst)
        print(f"     {fmt(r)}")

        # ────────────────────────────────────────────────────────
        # 情境 1b-iii：跨組切換 25k → 47k (Mode 1 → Mode 2)
        # ────────────────────────────────────────────────────────
        print("\n[1b-iii] 第二代跨組切換 (Mode 1 → Mode 2, 25k→47k)...")
        gen2_switch(inst, 2, None, sampling_rates)  # 初始化到 Mode 2

        def bench_1biii():
            gen2_switch(inst, 1, 2, sampling_rates)  # 不計時：回到 Mode 1
            return gen2_switch(inst, 2, 1, sampling_rates)  # 計時：1→2

        r = run_benchmark("1b-iii", bench_1biii)
        results.append(r)
        safe_output_off(inst)
        print(f"     {fmt(r)}")

        # ────────────────────────────────────────────────────────
        # 情境 1b-iv：跨組切換 47k → 25k (Mode 2 → Mode 1)
        # ────────────────────────────────────────────────────────
        print("\n[1b-iv] 第二代跨組切換 (Mode 2 → Mode 1, 47k→25k)...")
        gen2_switch(inst, 1, None, sampling_rates)  # 初始化到 Mode 1

        def bench_1biv():
            gen2_switch(inst, 2, 1, sampling_rates)  # 不計時：回到 Mode 2
            return gen2_switch(inst, 1, 2, sampling_rates)  # 計時：2→1

        r = run_benchmark("1b-iv", bench_1biv)
        results.append(r)
        safe_output_off(inst)
        print(f"     {fmt(r)}")

        # ────────────────────────────────────────────────────────
        # 情境 1c：從待機狀態啟動 (None → Mode 1)
        # ────────────────────────────────────────────────────────
        print("\n[1c] 第二代待機啟動 (None → Mode 1)...")

        def bench_1c():
            enter_standby(inst)
            return gen2_switch(inst, 1, None, sampling_rates)

        r = run_benchmark("1c", bench_1c)
        results.append(r)
        safe_output_off(inst)
        print(f"     {fmt(r)}")

    except Exception as e:
        print(f"\n測試中發生錯誤：{e}")
        safe_output_off(inst)
        inst.close()
        raise

    # ── 計算加速比 ──
    gen1_mean = results[0]['mean']
    gen2_same_mean = np.mean([results[1]['mean'], results[2]['mean']])
    gen2_cross_mean = np.mean([results[3]['mean'], results[4]['mean']])
    gen2_all_mean = np.mean([r['mean'] for r in results[1:]])
    speedup_all = gen1_mean / gen2_all_mean if gen2_all_mean > 0 else float('inf')
    speedup_same = gen1_mean / gen2_same_mean if gen2_same_mean > 0 else float('inf')
    speedup_cross = gen1_mean / gen2_cross_mean if gen2_cross_mean > 0 else float('inf')

    # ── 產生報告 ──
    report = []
    report.append("=" * 72)
    report.append("  函數產生器切換時間 Benchmark 報告")
    report.append("  設備：Keysight 33600A")
    report.append(f"  重複次數：{N_REPEATS} 次 / 情境")
    report.append(f"  測試時間：{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    report.append("=" * 72)
    report.append(f"情境 1a     第一代切換             {fmt(results[0])}")
    report.append(f"情境 1b-i   第二代同組(1↔3)        {fmt(results[1])}")
    report.append(f"情境 1b-ii  第二代同組(2↔4)        {fmt(results[2])}")
    report.append(f"情境 1b-iii 第二代跨組(1→2, 25k→47k) {fmt(results[3])}")
    report.append(f"情境 1b-iv  第二代跨組(2→1, 47k→25k) {fmt(results[4])}")
    report.append(f"情境 1c     第二代待機啟動           {fmt(results[5])}")
    report.append("=" * 72)
    report.append(f"第一代平均切換時間：        {gen1_mean:.1f} ms")
    report.append(f"第二代同組切換平均：        {gen2_same_mean:.1f} ms")
    report.append(f"第二代跨組切換平均：        {gen2_cross_mean:.1f} ms")
    report.append(f"第二代全情境平均：          {gen2_all_mean:.1f} ms")
    report.append("-" * 72)
    report.append(f"第一代 vs 第二代平均加速比：{speedup_all:.1f} 倍")
    report.append(f"第一代 vs 同組切換加速比：  {speedup_same:.1f} 倍")
    report.append(f"第一代 vs 跨組切換加速比：  {speedup_cross:.1f} 倍")
    report.append("=" * 72)

    report_text = "\n".join(report)
    print(f"\n{report_text}")

    # 儲存報告
    out_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        'fg_benchmark_results.txt'
    )
    with open(out_path, 'w', encoding='utf-8') as f:
        f.write(report_text + "\n")
    print(f"\n報告已儲存至：{out_path}")

    # 安全關閉
    safe_output_off(inst)
    inst.close()
    print("儀器已安全關閉")


if __name__ == "__main__":
    main()
