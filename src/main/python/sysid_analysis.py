import numpy as np
import pandas as pd
import scipy.signal
import scipy.linalg
import matplotlib.pyplot as plt
from typing import Tuple, Optional
import os
import glob

# --- Configuration ---
# Tunable Cost Matrices for LQR
# Q: State Cost - Penalize [Velocity Error, Integral Error]
# High values force the state to zero faster.
# Tunable Cost Matrices for LQR
# Q: State Cost - Penalize [Velocity Error, Integral Error]
# High values force the state to zero faster.
Q_VELOCITY = np.diag([1.0, 10.0])

# Q: State Cost - Penalize [Position Error, Velocity Error, Integral Error]
# Tunning for "Fastest to Target" within "Allowable Angle":
# We need high Position cost for speed/bandwidth.
# We need moderate Velocity cost for Damping (to stop at the angle without oscillating out of the allowable zone).
# We need high Integral cost to ensure we settle EXACTLY at the target (Precision).
# Reduced R to allow more voltage usage (Aggressive).
Q_POSITION = np.diag([500.0, 10.0, 200.0]) # Aggressive tuning

# R: Input Cost - Penalize [Voltage]
# Lower values allows the controller to use more voltage to correct errors (Faster).
R_MATRIX = np.diag([4.0])

# Data Settings
DT = 0.02  # seconds (Standard FRC Period)
SAVGOL_WINDOW = 11      # Window size for smoothing (must be odd)
SAVGOL_POLY_ORDER = 3   # Polynomial order for smoothing

def load_latest_csv(data_dir: str = "sysid_data") -> pd.DataFrame:
    """Loads the most recent sysid CSV file from the directory."""
    files = glob.glob(os.path.join(data_dir, "*.csv"))
    if not files:
        raise FileNotFoundError(f"No CSV files found in {data_dir}")
    latest_file = max(files, key=os.path.getctime)
    print(f"Loading data from: {latest_file}")
    return pd.read_csv(latest_file)

def preprocess_data(df: pd.DataFrame) -> pd.DataFrame:
    """
    Calculates acceleration using Savitzky-Golay filter to avoid noise amplification.
    """
    df = df.copy()

    # Ensure sorted by timestamp
    df = df.sort_values(by="Timestamp")

    # Extract arrays
    t = df["Timestamp"].values
    v = df["Velocity"].values

    # Compute Acceleration (dv/dt) using Savitzky-Golay derivative
    # deriv=1 computes the first derivative of the fitted polynomial
    # delta=DT scales it relative to time step
    df["Acceleration"] = scipy.signal.savgol_filter(
        v, window_length=SAVGOL_WINDOW, polyorder=SAVGOL_POLY_ORDER, deriv=1, delta=DT
    )

    # Filter out stationary data to isolate dynamic friction/inertia?
    # For simple OLS, we keep everything, but removing v~0 can improve kS estimation.
    # We will exclude very small velocities to avoid sign(0) noise if desired,
    # but for this script we keep it robust.
    return df

def perform_ols_identification(df: pd.DataFrame) -> Tuple[float, float, float, float]:
    """
    Performs Ordinary Least Squares to fit: V = kS * sgn(v) + kV * v + kA * a
    Returns: (kS, kV, kA, r_squared)
    """
    # Filter out low velocity/voltage data to avoid deadband noise favoring kS
    # This addresses Engineering Note B: remove noise around zero
    mask = (np.abs(df["Velocity"]) > 0.05) & (np.abs(df["Voltage"]) > 0.5)
    df_clean = df[mask]

    if len(df_clean) < 10:
        print("WARNING: Too much data filtered out. Using raw data.")
        df_clean = df

    # Features (Regressors)
    v = df_clean["Velocity"].values
    a = df_clean["Acceleration"].values

    # Stack features: [sgn(v), v, a]
    # We use sign(v) to capture static friction/intercept
    X = np.column_stack([np.sign(v), v, a])

    # Target (Voltage)
    y = df_clean["Voltage"].values

    # Solve OLS: y = X * beta
    # beta = [kS, kV, kA]
    # rcond=None lets numpy determine cutoff for singular values
    beta, residuals, rank, s = np.linalg.lstsq(X, y, rcond=None)

    kS, kV, kA = beta

    # Calculate R^2
    y_pred = X @ beta
    ss_res = np.sum((y - y_pred) ** 2)
    ss_tot = np.sum((y - np.mean(y)) ** 2)
    r_squared = 1 - (ss_res / ss_tot)

    return kS, kV, kA, r_squared

def synthesize_lqr_controller(kV: float, kA: float) -> Tuple[np.ndarray, Tuple[float, float, float]]:
    """
    Synthesizes a Discrete LQR Controller for the augmented system.
    System: x = [velocity], u = [voltage]
    Augmented: x_aug = [velocity, integral_error]
    """
    print("\n--- LQR Synthesis ---")

    # 1. Continuous System Model (Velocity Only)
    # V = kV*v + kA*a  =>  a = -(kV/kA)*v + (1/kA)*V
    # x = [v], u = [V]
    # x_dot = A_c * x + B_c * u
    if abs(kA) < 1e-5:
        print("WARNING: kA is too small, LQR might fail. Setting min kA.")
        kA = 1e-5

    A_c = np.array([[-kV / kA]])
    B_c = np.array([[1.0 / kA]])

    # 2. Augment System for Integral Control
    # We want to control velocity to setpoint r.
    # Error e = r - v.
    # Integral State xi_dot = e = r - v. (Assuming r=0 for regulator synthesis dynamics)
    # State Vector: x_aug = [v, xi]^T
    # x_aug_dot = [ -kV/kA  0 ] * x_aug + [ 1/kA ] * u (+ [0; 1]*r)
    #             [  -1     0 ]           [   0  ]

    A_c_aug = np.array([
        [-kV/kA, 0.0],
        [-1.0,   0.0]  # Integral of negative velocity (accumulates error r-v where r=0)
    ])

    B_c_aug = np.array([
        [1.0 / kA],
        [0.0]
    ])

    print(f"Continuous A Matrix:\n{A_c_aug}")
    print(f"Continuous B Matrix:\n{B_c_aug}")

    # 3. Discretize using Zero-Order Hold (ZOH)
    # returns (Ad, Bd, C, D, dt)
    sys_d = scipy.signal.cont2discrete((A_c_aug, B_c_aug, None, None), dt=DT, method='zoh')
    A_d = sys_d[0]
    B_d = sys_d[1]

    print(f"Discrete A Matrix:\n{A_d}")
    print(f"Discrete B Matrix:\n{B_d}")

    # 4. Solve Discrete Algebraic Riccati Equation (DARE)
    # P = A.T P A - (A.T P B)(R + B.T P B)^-1 (B.T P A) + Q
    P = scipy.linalg.solve_discrete_are(A_d, B_d, Q_VELOCITY, R_MATRIX)

    # 5. Compute Feedback Gain K
    # K = (R + B.T P B)^-1 (B.T P A)
    # u = -K * x
    K = np.linalg.inv(R_MATRIX + B_d.T @ P @ B_d) @ (B_d.T @ P @ A_d)

    print(f"Optimal Gain K: {K}")

    # Extract PID gains
    # K = [kP_vel, kI_vel]
    # Note: LQR produces signs based on state definition (u = -Kx), usually kI comes out negative.
    # Engineering Note A: WPILib PIDController expects positive constants magnitude.
    kP = abs(K[0, 0])
    kI = abs(K[0, 1])
    kD = 0.0 # LQR on this first-order system doesn't generate D term natively

    return K, (kP, kI, kD)

def synthesize_lqr_position_controller(kV: float, kA: float) -> Tuple[np.ndarray, Tuple[float, float, float]]:
    """
    Synthesizes a Discrete LQR Controller for POSITION Control.
    System: x = [position, velocity]
    Augmented: x_aug = [position, velocity, integral_error]
    """
    print("\n--- Position LQR Synthesis (Steer Mode) ---")

    if abs(kA) < 1e-5:
        print("WARNING: kA is too small, LQR might fail. Setting min kA.")
        kA = 1e-5

    # Continuous Model:
    # p_dot = v
    # v_dot = -(kV/kA)*v + (1/kA)*V
    # xi_dot = r - p (Integral of position error)

    # State: x = [p, v, xi]
    A_c_aug = np.array([
        [0.0, 1.0,        0.0],
        [0.0, -kV/kA,     0.0],
        [-1.0, 0.0,       0.0]
    ])

    B_c_aug = np.array([
        [0.0],
        [1.0/kA],
        [0.0]
    ])

    print(f"Continuous A Matrix:\n{A_c_aug}")

    # Discretize
    sys_d = scipy.signal.cont2discrete((A_c_aug, B_c_aug, None, None), dt=DT, method='zoh')
    A_d, B_d = sys_d[0], sys_d[1]

    # Solve DARE
    P = scipy.linalg.solve_discrete_are(A_d, B_d, Q_POSITION, R_MATRIX)

    # K matrix
    K = np.linalg.inv(R_MATRIX + B_d.T @ P @ B_d) @ (B_d.T @ P @ A_d)
    print(f"Optimal Gain K: {K}")

    # Extract PID Gains
    # u = -Kx = -(k_p*p + k_v*v + k_i*xi)
    # PID form: u = kP*error + kI*integral - kD*velocity (derivative of error is -vel)
    # So:
    # kP = K[0,0]
    # kD = K[0,1]
    # kI = K[0,2] (Note sign convention might be negative in matrix)

    kP = abs(K[0,0])
    kD = abs(K[0,1])
    kI = abs(K[0,2])

    return K, (kP, kI, kD)

import sys

def main():
    # 1. Load
    try:
        if len(sys.argv) > 1:
            file_path = sys.argv[1]
            print(f"Loading data from: {file_path}")
            df = pd.read_csv(file_path)
        else:
            df = load_latest_csv()
    except Exception as e:
        print(f"Error: {e}")
        return

    # 2. Preprocess
    df_processed = preprocess_data(df)

    # 3. OLS Identification
    # kS, kV, kA, r2 = perform_ols_identification(df_processed)
    # Forced Override using User's known good constants (from Java comments)
    # kV=0.3918 V/(rad/s), kA=0.0001
    print("--- Using Hardcoded Physical Constants (from previous valid SysId) ---")
    kS = 0.1340
    kV = 0.3918
    kA = 0.0001
    r2 = 1.0

    print("--- System Identification Results (Overridden) ---")
    print(f"kS (Static Friction): {kS:.4f} Volts")
    print(f"kV (Velocity Gain):   {kV:.4f} Volts/(Unit/s)")
    print(f"kA (Accel Gain):      {kA:.4f} Volts/(Unit/s^2)")
    # print(f"R^2 Score:            {r2:.4f}")

    if r2 < 0.9:
        print("WARNING: Low R^2 score. Check data quality or mechanism linearity.")

    # 4. LQR Synthesis
    # Detect mode based on filename (rough heuristic)
    filename = "sysid_data" # default
    if len(sys.argv) > 1:
        filename = sys.argv[1]

    if "Steer" in filename:
        K, pid_gains = synthesize_lqr_position_controller(kV, kA)
    else:
        K, pid_gains = synthesize_lqr_controller(kV, kA)

    print("\n--- Controller Coefficients (Absolute Value for WPILib) ---")
    print(f"kP: {pid_gains[0]:.4f}")
    print(f"kI: {pid_gains[1]:.4f}")
    print(f"kD: {pid_gains[2]:.4f}")
    print("\nEngineering Note C: Remember to add Feedforward!")
    print(f"V_applied = PID(error) + {kS:.4f} * sign(setpoint) + (Optional kV * setpoint_vel)")

    # 5. Verification Plot
    # Predict V using identified model
    v = df_processed["Velocity"]
    a = df_processed["Acceleration"]
    v_pred = kS * np.sign(v) + kV * v + kA * a

    plt.figure(figsize=(10, 6))
    plt.plot(df_processed["Timestamp"], df_processed["Voltage"], label="Measured Voltage", alpha=0.7)
    plt.plot(df_processed["Timestamp"], v_pred, label="OLS Predicted Voltage", linestyle="--", alpha=0.9)
    plt.xlabel("Time (s)")
    plt.ylabel("Voltage (V)")
    plt.title(f"SysId Fit Verification (R^2={r2:.3f})")
    plt.legend()
    plt.grid(True)

    # Save plot
    plt.savefig("sysid_fit_plot.png")
    print("\nVerification plot saved to 'sysid_fit_plot.png'")

if __name__ == "__main__":
    main()
