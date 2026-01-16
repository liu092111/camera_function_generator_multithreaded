"""
System Identification for Angle Step Response
Identifies a transfer function with 3 poles and 1 zero from measured data.

Transfer Function Form: G(s) = (b1*s + b0) / (s^3 + a2*s^2 + a1*s + a0)

This script:
1. Reads angle vs time data from CSV
2. Detects the start of step response automatically
3. Uses optimization to find transfer function parameters
4. Compares identified model with measured data
5. Outputs results and visualization
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy import signal
from scipy.optimize import minimize, differential_evolution, curve_fit
from scipy.interpolate import interp1d

def load_angle_data(csv_path):
    """
    Load angle data from CSV file
    
    Returns:
    --------
    t : array - Time vector (seconds)
    angle : array - Angle values (degrees)
    """
    df = pd.read_csv(csv_path)
    
    # Extract time and angle
    t = df['t_s'].values
    angle = df['angle_deg_unwrapped'].values
    
    return t, angle

def detect_step_start(t, angle, threshold_velocity=5.0, window=5):
    """
    Detect the start of step response by looking for sudden change in angle
    
    Parameters:
    -----------
    t : array - Time vector
    angle : array - Angle values
    threshold_velocity : float - Angular velocity threshold (deg/s) to detect step start
    window : int - Number of samples to look ahead for confirmation
    
    Returns:
    --------
    idx : int - Index where step response starts
    """
    dt = np.diff(t)
    da = np.diff(angle)
    velocity = da / dt
    
    # Find where velocity exceeds threshold
    for i in range(len(velocity) - window):
        if np.abs(velocity[i]) > threshold_velocity:
            # Confirm this is sustained motion, not noise
            sustained = np.all(np.abs(velocity[i:i+window]) > threshold_velocity * 0.5)
            if sustained:
                return max(0, i - 2)  # Start a bit before detected motion
    
    return 0  # If no step detected, use full data

def tf_3p1z_step_response(params, t):
    """
    Compute step response of transfer function with 3 poles, 1 zero
    
    G(s) = (b1*s + b0) / (s^3 + a2*s^2 + a1*s + a0)
    
    Parameters:
    -----------
    params : array [b1, b0, a2, a1, a0]
    t : array - Time vector
    
    Returns:
    --------
    y : array - Step response
    """
    b1, b0, a2, a1, a0 = params
    
    # Numerator and denominator coefficients
    num = [b1, b0]
    den = [1, a2, a1, a0]
    
    try:
        # Check stability (all poles must have negative real parts)
        poles = np.roots(den)
        if np.any(np.real(poles) > 0):
            return np.full_like(t, 1e10, dtype=float)
        
        # Create transfer function
        sys = signal.TransferFunction(num, den)
        
        # Compute step response
        t_out, y = signal.step(sys, T=t)
        
        return y
    except Exception:
        return np.full_like(t, 1e10, dtype=float)

def cost_function(params, t_meas, y_meas, gain=1.0):
    """
    Cost function for optimization (Mean Squared Error)
    """
    # Compute model response
    y_model = tf_3p1z_step_response(params, t_meas)
    
    # Scale model to match DC gain
    if np.abs(y_model[-1]) > 1e-6:
        y_model = y_model * gain / y_model[-1]
    
    # Calculate MSE
    mse = np.mean((y_meas - y_model)**2)
    
    return mse

def identify_transfer_function(t, y, method='differential_evolution'):
    """
    Identify transfer function parameters using optimization
    """
    # Determine DC gain from steady state value
    dc_gain = y[-1] if np.abs(y[-1]) > 1e-6 else 1.0
    
    # Normalize output
    y_norm = y / dc_gain if np.abs(dc_gain) > 1e-6 else y
    
    # Resample data for faster computation
    num_points = min(500, len(t))
    t_resampled = np.linspace(t[0], t[-1], num_points)
    interp_func = interp1d(t, y_norm, kind='linear')
    y_resampled = interp_func(t_resampled)
    
    # Parameter bounds [b1, b0, a2, a1, a0]
    bounds = [
        (0.01, 500),     # b1 - zero coefficient
        (1, 500),        # b0 - zero constant term  
        (1, 100),        # a2 - s^2 coefficient
        (1, 500),        # a1 - s coefficient
        (1, 500)         # a0 - constant term
    ]
    
    print("Starting System Identification...")
    print(f"Optimization method: {method}")
    print(f"DC Gain (steady state value): {dc_gain:.4f}")
    print(f"Data points for optimization: {num_points}")
    
    if method == 'differential_evolution':
        # Global optimization using differential evolution
        result = differential_evolution(
            cost_function, 
            bounds, 
            args=(t_resampled, y_resampled, 1.0),
            seed=42,
            maxiter=3000,
            tol=1e-12,
            polish=True,
            workers=-1,
            disp=True,
            popsize=25
        )
    else:
        # Initial guess
        x0 = [10, 50, 10, 50, 50]
        result = minimize(
            cost_function,
            x0,
            args=(t_resampled, y_resampled, 1.0),
            method='Nelder-Mead',
            options={'maxiter': 10000, 'xatol': 1e-10, 'fatol': 1e-10}
        )
    
    print(f"\nOptimization completed:")
    print(f"  Success: {result.success}")
    print(f"  Final cost (MSE): {result.fun:.6e}")
    
    return result.x, dc_gain

def plot_comparison(t_meas, y_meas, t_model, y_model, params, dc_gain, save_path=None):
    """
    Plot comparison between measured and identified model response
    """
    fig, axes = plt.subplots(2, 1, figsize=(14, 10))
    
    # Plot 1: Direct comparison
    ax1 = axes[0]
    ax1.plot(t_meas, y_meas, 'b-', linewidth=1.5, label='Measured Data', alpha=0.8)
    ax1.plot(t_model, y_model, 'r--', linewidth=2, label='Identified Model (3P1Z)')
    ax1.set_xlabel('Time (s)', fontsize=12)
    ax1.set_ylabel('Angle (degrees)', fontsize=12)
    ax1.set_title('System Identification: Measured vs Identified Model', fontsize=14)
    ax1.legend(loc='best', fontsize=11)
    ax1.grid(True, linestyle='--', alpha=0.7)
    
    # Add transfer function text
    b1, b0, a2, a1, a0 = params
    tf_text = f'G(s) = ({b1:.2f}s + {b0:.2f}) / (s³ + {a2:.2f}s² + {a1:.2f}s + {a0:.2f})'
    ax1.text(0.02, 0.98, tf_text, transform=ax1.transAxes, fontsize=10,
             verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    # Plot 2: Error analysis
    ax2 = axes[1]
    
    # Interpolate model to match measurement time points
    interp_func = interp1d(t_model, y_model, kind='linear', fill_value='extrapolate')
    y_model_interp = interp_func(t_meas)
    
    error = y_meas - y_model_interp
    error_percent = (error / np.abs(y_meas[-1])) * 100 if np.abs(y_meas[-1]) > 1e-6 else error
    
    ax2.plot(t_meas, error_percent, 'g-', linewidth=1, alpha=0.8)
    ax2.axhline(y=0, color='k', linestyle='--', linewidth=1)
    ax2.set_xlabel('Time (s)', fontsize=12)
    ax2.set_ylabel('Error (%)', fontsize=12)
    ax2.set_title('Identification Error (Measured - Model) as Percentage of Final Value', fontsize=14)
    ax2.grid(True, linestyle='--', alpha=0.7)
    
    # Calculate and display error statistics
    rmse = np.sqrt(np.mean(error**2))
    max_error = np.max(np.abs(error))
    mean_error = np.mean(np.abs(error))
    rmse_percent = (rmse / np.abs(y_meas[-1])) * 100 if np.abs(y_meas[-1]) > 1e-6 else rmse
    
    stats_text = f'RMSE: {rmse:.4f}° ({rmse_percent:.2f}%)\nMax Error: {max_error:.4f}°\nMean Abs Error: {mean_error:.4f}°'
    ax2.text(0.98, 0.98, stats_text, transform=ax2.transAxes, fontsize=10,
             verticalalignment='top', horizontalalignment='right',
             bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.5))
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"\nFigure saved to: {save_path}")
    
    plt.show()
    
    return rmse, max_error, mean_error

def analyze_poles_zeros(params):
    """
    Analyze poles and zeros of the identified transfer function
    """
    b1, b0, a2, a1, a0 = params
    
    num = [b1, b0]
    den = [1, a2, a1, a0]
    
    zeros = np.roots(num)
    poles = np.roots(den)
    
    print("\n" + "="*60)
    print("Poles and Zeros Analysis")
    print("="*60)
    
    print("\nZeros (roots of numerator):")
    for i, z in enumerate(zeros):
        if np.abs(np.imag(z)) < 1e-6:
            print(f"  z{i+1} = {np.real(z):.4f}")
        else:
            print(f"  z{i+1} = {np.real(z):.4f} ± {np.abs(np.imag(z)):.4f}j")
    
    print("\nPoles (roots of denominator):")
    for i, p in enumerate(poles):
        if np.abs(np.imag(p)) < 1e-6:
            print(f"  p{i+1} = {np.real(p):.4f}")
            tau = -1/np.real(p) if np.real(p) != 0 else float('inf')
            print(f"       Time constant: τ = {tau:.4f} s")
        else:
            print(f"  p{i+1} = {np.real(p):.4f} ± {np.abs(np.imag(p)):.4f}j")
            wn = np.abs(p)
            zeta = -np.real(p) / wn
            print(f"       Natural frequency: ωn = {wn:.4f} rad/s")
            print(f"       Damping ratio: ζ = {zeta:.4f}")
    
    # Check stability
    if np.all(np.real(poles) < 0):
        print("\nSystem is STABLE (all poles have negative real parts)")
    else:
        print("\nWARNING: System is UNSTABLE")
    
    return zeros, poles

def generate_report(params, dc_gain, rmse, max_error, mean_error, zeros, poles, save_path=None):
    """
    Generate detailed identification report
    """
    b1, b0, a2, a1, a0 = params
    
    report = []
    report.append("="*70)
    report.append("SYSTEM IDENTIFICATION REPORT")
    report.append("Angle Step Response - 3 Poles, 1 Zero Transfer Function")
    report.append("="*70)
    
    report.append("\n" + "-"*70)
    report.append("IDENTIFIED TRANSFER FUNCTION")
    report.append("-"*70)
    report.append(f"\n           {b1:.4f}s + {b0:.4f}")
    report.append("G(s) = ────────────────────────────────────────────")
    report.append(f"       s³ + {a2:.4f}s² + {a1:.4f}s + {a0:.4f}")
    
    report.append("\n\nNumerator coefficients:")
    report.append(f"  b1 (s coefficient)  = {b1:.6f}")
    report.append(f"  b0 (constant)       = {b0:.6f}")
    
    report.append("\nDenominator coefficients:")
    report.append(f"  a2 (s² coefficient) = {a2:.6f}")
    report.append(f"  a1 (s coefficient)  = {a1:.6f}")
    report.append(f"  a0 (constant)       = {a0:.6f}")
    
    report.append(f"\nDC Gain (steady state) = {dc_gain:.6f}")
    
    report.append("\n" + "-"*70)
    report.append("POLES AND ZEROS")
    report.append("-"*70)
    
    report.append("\nZeros:")
    for i, z in enumerate(zeros):
        if np.abs(np.imag(z)) < 1e-6:
            report.append(f"  z{i+1} = {np.real(z):.6f}")
        else:
            report.append(f"  z{i+1} = {np.real(z):.6f} ± {np.abs(np.imag(z)):.6f}j")
    
    report.append("\nPoles:")
    for i, p in enumerate(poles):
        if np.abs(np.imag(p)) < 1e-6:
            report.append(f"  p{i+1} = {np.real(p):.6f}")
            tau = -1/np.real(p) if np.real(p) != 0 else float('inf')
            report.append(f"       Time constant: τ = {tau:.6f} s")
        else:
            report.append(f"  p{i+1} = {np.real(p):.6f} ± {np.abs(np.imag(p)):.6f}j")
            wn = np.abs(p)
            zeta = -np.real(p) / wn
            report.append(f"       Natural frequency: ωn = {wn:.6f} rad/s")
            report.append(f"       Damping ratio: ζ = {zeta:.6f}")
    
    # Stability
    if np.all(np.real(poles) < 0):
        report.append("\nSystem Stability: STABLE")
    else:
        report.append("\nSystem Stability: UNSTABLE")
    
    report.append("\n" + "-"*70)
    report.append("FITTING QUALITY METRICS")
    report.append("-"*70)
    rmse_percent = (rmse / np.abs(dc_gain)) * 100 if np.abs(dc_gain) > 1e-6 else rmse
    report.append(f"\n  Root Mean Square Error (RMSE): {rmse:.6f} degrees ({rmse_percent:.2f}%)")
    report.append(f"  Maximum Absolute Error:        {max_error:.6f} degrees")
    report.append(f"  Mean Absolute Error:           {mean_error:.6f} degrees")
    
    report.append("\n" + "-"*70)
    report.append("SCIPY TRANSFER FUNCTION FORMAT")
    report.append("-"*70)
    report.append(f"\nnumerator = [{b1:.6f}, {b0:.6f}]")
    report.append(f"denominator = [1, {a2:.6f}, {a1:.6f}, {a0:.6f}]")
    
    report.append("\n" + "="*70)
    
    report_text = '\n'.join(report)
    
    if save_path:
        with open(save_path, 'w', encoding='utf-8') as f:
            f.write(report_text)
        print(f"\nReport saved to: {save_path}")
    
    print(report_text)
    
    return report_text

def plot_raw_data(t, angle, step_start_idx=None):
    """
    Plot raw data to understand the signal characteristics
    """
    fig, ax = plt.subplots(figsize=(14, 6))
    
    ax.plot(t, angle, 'b-', linewidth=1.5, label='Angle Data')
    
    if step_start_idx is not None:
        ax.axvline(x=t[step_start_idx], color='r', linestyle='--', linewidth=2, 
                   label=f'Step Start (t={t[step_start_idx]:.3f}s)')
    
    ax.set_xlabel('Time (s)', fontsize=12)
    ax.set_ylabel('Angle (degrees)', fontsize=12)
    ax.set_title('Raw Angle Data with Step Detection', fontsize=14)
    ax.legend(loc='best', fontsize=11)
    ax.grid(True, linestyle='--', alpha=0.7)
    
    plt.tight_layout()
    plt.savefig('raw_angle_data.png', dpi=150)
    plt.show()

def main():
    # File paths
    csv_path = 'IMG_7110_pos_angle_speed.csv'
    figure_path = 'IMG_7110_system_identification.png'
    report_path = 'IMG_7110_system_identification_report.txt'
    
    print("="*60)
    print("ANGLE STEP RESPONSE SYSTEM IDENTIFICATION")
    print("Transfer Function: 3 Poles, 1 Zero")
    print("="*60)
    
    # Load data
    print("\nLoading data from CSV...")
    t_raw, angle_raw = load_angle_data(csv_path)
    
    print(f"  Data points: {len(t_raw)}")
    print(f"  Time range: {t_raw[0]:.4f} to {t_raw[-1]:.4f} seconds")
    print(f"  Duration: {t_raw[-1] - t_raw[0]:.4f} seconds")
    print(f"  Initial angle: {angle_raw[0]:.4f} degrees")
    print(f"  Final angle: {angle_raw[-1]:.4f} degrees")
    
    # Detect step start
    print("\nDetecting step response start...")
    step_start_idx = detect_step_start(t_raw, angle_raw, threshold_velocity=10.0, window=3)
    print(f"  Step detected at index {step_start_idx}, time = {t_raw[step_start_idx]:.4f}s")
    
    # Plot raw data with step detection
    print("\nPlotting raw data...")
    plot_raw_data(t_raw, angle_raw, step_start_idx)
    
    # Extract step response portion
    t_step = t_raw[step_start_idx:] - t_raw[step_start_idx]  # Start time from 0
    angle_step = angle_raw[step_start_idx:] - angle_raw[step_start_idx]  # Start angle from 0
    
    print(f"\nStep response data:")
    print(f"  Data points: {len(t_step)}")
    print(f"  Duration: {t_step[-1]:.4f} seconds")
    print(f"  Angle change: {angle_step[-1]:.4f} degrees")
    
    # Identify transfer function
    print("\n" + "="*60)
    params, dc_gain = identify_transfer_function(t_step, angle_step, method='differential_evolution')
    
    # Print identified parameters
    b1, b0, a2, a1, a0 = params
    print("\n" + "-"*60)
    print("IDENTIFIED TRANSFER FUNCTION PARAMETERS")
    print("-"*60)
    print(f"\n           {b1:.4f}s + {b0:.4f}")
    print("G(s) = ────────────────────────────────────────────")
    print(f"       s³ + {a2:.4f}s² + {a1:.4f}s + {a0:.4f}")
    
    # Generate model response for comparison
    t_model = np.linspace(0, t_step[-1], 1000)
    y_model_norm = tf_3p1z_step_response(params, t_model)
    y_model = y_model_norm * dc_gain / y_model_norm[-1] if np.abs(y_model_norm[-1]) > 1e-6 else y_model_norm
    
    # Analyze poles and zeros
    zeros, poles = analyze_poles_zeros(params)
    
    # Plot comparison
    rmse, max_error, mean_error = plot_comparison(
        t_step, angle_step, t_model, y_model, params, dc_gain, save_path=figure_path
    )
    
    # Generate report
    generate_report(params, dc_gain, rmse, max_error, mean_error, zeros, poles, save_path=report_path)
    
    print("\n" + "="*60)
    print("SYSTEM IDENTIFICATION COMPLETED")
    print("="*60)

if __name__ == "__main__":
    main()
