"""
Transfer Function Step Response with PID Controllers
Transfer Function: G(s) = (406.9s + 2553) / (s^3 + 9.298s^2 + 18.78s + 23.66)

This script plots the step response of the system with various controllers:
- Open loop (System Identification Fitting)
- PI Controller
- PD Controller  
- PID Controller

And calculates performance metrics:
- Rise Time
- Settling Time
- Overshoot
- Steady State Error
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

def get_plant():
    """Get the plant transfer function G(s)"""
    # G(s) = (406.9s + 2553) / (s^3 + 9.298s^2 + 18.78s + 23.66)
    num = [406.9, 2553]
    den = [1, 9.298, 18.78, 23.66]
    return num, den

def multiply_tf(num1, den1, num2, den2):
    """Multiply two transfer functions"""
    num = np.convolve(num1, num2)
    den = np.convolve(den1, den2)
    return num, den

def add_tf(num1, den1, num2, den2):
    """Add two transfer functions"""
    # (num1/den1) + (num2/den2) = (num1*den2 + num2*den1) / (den1*den2)
    num = np.polyadd(np.convolve(num1, den2), np.convolve(num2, den1))
    den = np.convolve(den1, den2)
    return num, den

def closed_loop_tf(controller_num, controller_den, plant_num, plant_den):
    """
    Calculate closed-loop transfer function
    T(s) = C(s)*G(s) / (1 + C(s)*G(s))
    """
    # C(s) * G(s)
    cg_num, cg_den = multiply_tf(controller_num, controller_den, plant_num, plant_den)
    
    # 1 + C(s)*G(s)
    one_plus_cg_num, one_plus_cg_den = add_tf([1], [1], cg_num, cg_den)
    
    # T(s) = (C*G) / (1 + C*G) = cg_num * one_plus_cg_den / (cg_den * one_plus_cg_num)
    # Simplified: num = cg_num, den = one_plus_cg_num (when denominators match)
    cl_num = cg_num
    cl_den = one_plus_cg_num
    
    return cl_num, cl_den

def pi_controller(Kp, Ki):
    """PI Controller: C(s) = Kp + Ki/s = (Kp*s + Ki) / s"""
    num = [Kp, Ki]
    den = [1, 0]
    return num, den

def pd_controller(Kp, Kd):
    """PD Controller: C(s) = Kp + Kd*s = (Kd*s + Kp)"""
    num = [Kd, Kp]
    den = [1]
    return num, den

def pid_controller(Kp, Ki, Kd):
    """PID Controller: C(s) = Kp + Ki/s + Kd*s = (Kd*s^2 + Kp*s + Ki) / s"""
    num = [Kd, Kp, Ki]
    den = [1, 0]
    return num, den

def calculate_step_response_metrics(t, y, target_value=1.0, settling_threshold=0.02):
    """
    Calculate step response performance metrics
    
    Parameters:
    -----------
    t : array
        Time vector
    y : array
        Response vector
    target_value : float
        The desired steady state value (usually 1 for unit step)
    settling_threshold : float
        Percentage threshold for settling time (default 2%)
    
    Returns:
    --------
    dict : Dictionary containing all metrics
    """
    metrics = {}
    
    # Steady State Value (use the final value)
    steady_state_value = y[-1]
    metrics['steady_state_value'] = steady_state_value
    
    # Steady State Error
    steady_state_error = abs(target_value - steady_state_value)
    metrics['steady_state_error'] = steady_state_error
    metrics['steady_state_error_percent'] = (steady_state_error / target_value) * 100 if target_value != 0 else 0
    
    # Rise Time (time from 10% to 90% of steady state)
    y_10 = 0.1 * steady_state_value
    y_90 = 0.9 * steady_state_value
    
    t_10 = None
    t_90 = None
    
    for i in range(len(y)):
        if t_10 is None and y[i] >= y_10:
            t_10 = t[i]
        if t_90 is None and y[i] >= y_90:
            t_90 = t[i]
            break
    
    if t_10 is not None and t_90 is not None:
        rise_time = t_90 - t_10
    else:
        rise_time = None
    metrics['rise_time'] = rise_time
    
    # Peak Value and Peak Time
    peak_value = np.max(y)
    peak_index = np.argmax(y)
    peak_time = t[peak_index]
    metrics['peak_value'] = peak_value
    metrics['peak_time'] = peak_time
    
    # Overshoot (percentage)
    if steady_state_value > 0:
        overshoot = ((peak_value - steady_state_value) / steady_state_value) * 100
    else:
        overshoot = 0
    metrics['overshoot_percent'] = max(0, overshoot)  # Overshoot is non-negative
    
    # Settling Time (time to stay within ±settling_threshold of steady state)
    settling_band_upper = steady_state_value * (1 + settling_threshold)
    settling_band_lower = steady_state_value * (1 - settling_threshold)
    
    settling_time = None
    for i in range(len(y) - 1, -1, -1):
        if y[i] > settling_band_upper or y[i] < settling_band_lower:
            if i < len(t) - 1:
                settling_time = t[i + 1]
            break
    
    if settling_time is None:
        settling_time = t[0]  # Already settled
    
    metrics['settling_time'] = settling_time
    metrics['settling_threshold_percent'] = settling_threshold * 100
    
    return metrics

def format_metrics(name, metrics):
    """Format metrics as string"""
    lines = []
    lines.append(f"\n{'='*60}")
    lines.append(f"{name}")
    lines.append(f"{'='*60}")
    lines.append(f"  Rise Time (10%-90%):        {metrics['rise_time']:.4f} s" if metrics['rise_time'] else "  Rise Time (10%-90%):        N/A")
    lines.append(f"  Peak Time:                  {metrics['peak_time']:.4f} s")
    lines.append(f"  Peak Value:                 {metrics['peak_value']:.4f}")
    lines.append(f"  Settling Time ({metrics['settling_threshold_percent']:.0f}%):      {metrics['settling_time']:.4f} s")
    lines.append(f"  Overshoot:                  {metrics['overshoot_percent']:.2f} %")
    lines.append(f"  Steady State Value:         {metrics['steady_state_value']:.4f}")
    lines.append(f"  Steady State Error:         {metrics['steady_state_error']:.4f}")
    lines.append(f"  Steady State Error (%):     {metrics['steady_state_error_percent']:.2f} %")
    return '\n'.join(lines)

def main():
    # Get plant transfer function
    plant_num, plant_den = get_plant()
    
    # Controller parameters (optimized for this system)
    Kp = 0.08   # Proportional gain
    Ki = 0.03   # Integral gain
    Kd = 0.015  # Derivative gain
    
    # Time vector for simulation
    t = np.linspace(0, 10, 1000)
    
    # Create figure
    plt.figure(figsize=(12, 8))
    
    # Store all results for txt file
    all_results = []
    all_results.append("Z Direction System Identification Simulation Results")
    all_results.append("="*60)
    all_results.append(f"\nPlant Transfer Function: G(s) = (406.9s + 2553) / (s^3 + 9.298s^2 + 18.78s + 23.66)")
    all_results.append(f"\nController Parameters:")
    all_results.append(f"  Kp = {Kp}")
    all_results.append(f"  Ki = {Ki}")
    all_results.append(f"  Kd = {Kd}")
    
    # 1. Open loop step response (System Identification Fitting)
    plant_system = signal.TransferFunction(plant_num, plant_den)
    t_ol, y_ol = signal.step(plant_system, T=t)
    # Normalize for comparison
    y_ol_norm = y_ol / y_ol[-1] if abs(y_ol[-1]) > 0.01 else y_ol
    plt.plot(t_ol, y_ol_norm, 'b-', linewidth=2, label='System Identification Fitting (Open Loop)')
    
    # Calculate metrics for open loop
    metrics_ol = calculate_step_response_metrics(t_ol, y_ol_norm, target_value=1.0)
    all_results.append(format_metrics("Open Loop (System Identification Fitting)", metrics_ol))
    
    # 2. PI Controller
    pi_num, pi_den = pi_controller(Kp, Ki)
    cl_pi_num, cl_pi_den = closed_loop_tf(pi_num, pi_den, plant_num, plant_den)
    try:
        pi_system = signal.TransferFunction(cl_pi_num, cl_pi_den)
        t_pi, y_pi = signal.step(pi_system, T=t)
        # Normalize
        y_pi_norm = y_pi / y_pi[-1] if abs(y_pi[-1]) > 0.01 else y_pi
        plt.plot(t_pi, y_pi_norm, 'r-', linewidth=2, label=f'PI Controller (Kp={Kp}, Ki={Ki})')
        
        metrics_pi = calculate_step_response_metrics(t_pi, y_pi_norm, target_value=1.0)
        all_results.append(format_metrics(f"PI Controller (Kp={Kp}, Ki={Ki})", metrics_pi))
    except Exception as e:
        print(f"PI Controller error: {e}")
        all_results.append(f"\nPI Controller: Error - {e}")
    
    # 3. PD Controller
    pd_num, pd_den = pd_controller(Kp, Kd)
    cl_pd_num, cl_pd_den = closed_loop_tf(pd_num, pd_den, plant_num, plant_den)
    try:
        pd_system = signal.TransferFunction(cl_pd_num, cl_pd_den)
        t_pd, y_pd = signal.step(pd_system, T=t)
        # Normalize
        y_pd_norm = y_pd / y_pd[-1] if abs(y_pd[-1]) > 0.01 else y_pd
        plt.plot(t_pd, y_pd_norm, 'g-', linewidth=2, label=f'PD Controller (Kp={Kp}, Kd={Kd})')
        
        metrics_pd = calculate_step_response_metrics(t_pd, y_pd_norm, target_value=1.0)
        all_results.append(format_metrics(f"PD Controller (Kp={Kp}, Kd={Kd})", metrics_pd))
    except Exception as e:
        print(f"PD Controller error: {e}")
        all_results.append(f"\nPD Controller: Error - {e}")
    
    # 4. PID Controller
    pid_num, pid_den = pid_controller(Kp, Ki, Kd)
    cl_pid_num, cl_pid_den = closed_loop_tf(pid_num, pid_den, plant_num, plant_den)
    try:
        pid_system = signal.TransferFunction(cl_pid_num, cl_pid_den)
        t_pid, y_pid = signal.step(pid_system, T=t)
        # Normalize
        y_pid_norm = y_pid / y_pid[-1] if abs(y_pid[-1]) > 0.01 else y_pid
        plt.plot(t_pid, y_pid_norm, 'm-', linewidth=2, label=f'PID Controller (Kp={Kp}, Ki={Ki}, Kd={Kd})')
        
        metrics_pid = calculate_step_response_metrics(t_pid, y_pid_norm, target_value=1.0)
        all_results.append(format_metrics(f"PID Controller (Kp={Kp}, Ki={Ki}, Kd={Kd})", metrics_pid))
    except Exception as e:
        print(f"PID Controller error: {e}")
        all_results.append(f"\nPID Controller: Error - {e}")
    
    # Plot formatting
    plt.xlabel('Time (s)', fontsize=24)
    plt.ylabel('Angle (degree)', fontsize=24)
    plt.title('System Identification of Step Response', fontsize=22)
    plt.legend(loc='best', fontsize=14)
    plt.tight_layout()
    
    # Save figure
    plt.savefig('step_response_with_controllers_optimized.png', dpi=1200)
    plt.show()
    
    # Save results to txt file
    output_text = '\n'.join(all_results)
    with open('z_direction_sys_ident_simulation_result_optimized.txt', 'w', encoding='utf-8') as f:
        f.write(output_text)
    
    # Print results
    print(output_text)
    print(f"\n\nResults saved to: z_direction_sys_ident_simulation_result_optimized.txt")

if __name__ == "__main__":
    main()
