import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# Constants
# Filter window for acceleration calculation (simple finite difference can be noisy)
ACCEL_WINDOW = 5


def analyze_file(filepath):
    """
    Reads a SysId CSV file and returns a DataFrame with Velocity, Acceleration, and Voltage.
    """
    try:
        df = pd.read_csv(filepath)
    except Exception as e:
        print(f"Error reading {filepath}: {e}")
        return None

    # Calculate Acceleration
    # dt = df['Timestamp'].diff().fillna(0.02) # assume 20ms if missing, but usually 5ms
    # dv = df['Velocity'].diff().fillna(0)
    # df['Acceleration'] = dv / dt

    # Better: Use gradient for acceleration to reduce noise
    df["Acceleration"] = np.gradient(df["Velocity"], df["Timestamp"])

    # Apply sliding window filter to acceleration if needed
    # df['Acceleration'] = df['Acceleration'].rolling(window=ACCEL_WINDOW, center=True).mean().fillna(0)

    # Filter out data where velocity is very low (stiction zone) to avoid kS distortion?
    # Actually SysId relies on low speed data for kS.
    # But we should ignore the very beginning where voltage is ramping up but mechanism hasn't moved yet.
    # A simple way is to use the provided Quasistatic/Dynamic structure.

    return df


def main():
    data_dir = Path("src/main/python/sysid_data")
    if len(sys.argv) > 1:
        data_dir = Path(sys.argv[1])
    files = list(data_dir.glob("*.csv"))

    if not files:
        print("No CSV files found in src/main/python/sysid_data")
        return

    print(f"Found {len(files)} log files.")

    all_data = []
    for f in files:
        print(f"Processing {f.name}...")
        df = analyze_file(f)
        if df is not None:
            # Identify test type from filename
            if "QUASISTATIC" in f.name:
                df["TestType"] = "Quasistatic"
            else:
                df["TestType"] = "Dynamic"
            all_data.append(df)

    if not all_data:
        print("No valid data found.")
        return

    full_df = pd.concat(all_data, ignore_index=True)

    # Prepare data for OLS Regression
    # Model: Voltage = kS * sign(Velocity) + kV * Velocity + kA * Acceleration

    # Remove extremely small velocities to avoid sign flipping noise near zero
    # But keep enough to capture kS.
    # standard SysId tool uses a velocity threshold. Let's use 0.1 rot/s (check units).
    # The log writer likely writes raw units (rotations and rot/s as per Shooter.java).
    # Timestamp, Voltage, Position, Velocity

    # Filter out zero voltage (disabled state)
    full_df = full_df[np.abs(full_df["Voltage"]) > 0.0]

    # Create feature matrix
    X = np.column_stack(
        [np.sign(full_df["Velocity"]), full_df["Velocity"], full_df["Acceleration"]]
    )
    y = full_df["Voltage"].values

    # Perform OLS fit
    # coeffs = [kS, kV, kA]
    coeffs, residuals, rank, s = np.linalg.lstsq(X, y, rcond=None)

    kS, kV, kA = coeffs

    print("-" * 30)
    print("Optimization Results:")
    print(f"  kS: {kS:.5f} Volts")
    print(f"  kV: {kV:.5f} Volts * s / rot")
    print(f"  kA: {kA:.5f} Volts * s^2 / rot")
    print("-" * 30)

    # R-squared
    y_pred = X @ coeffs
    ss_res = np.sum((y - y_pred) ** 2)
    ss_tot = np.sum((y - np.mean(y)) ** 2)
    r2 = 1 - (ss_res / ss_tot)
    print(f"  R-squared: {r2:.4f}")

    # Estimate kP
    # Rule of thumb for velocity control:
    # A simple LQR approach or just standard P-controller tuning.
    # For a flywheel, we want to recover from a disturbance (ball shot) quickly.
    # Let's say we want to apply full voltage (12V) if velocity drops by N RPS.
    # Typical drop: ~2-5 RPS.
    # If we want 12V correction for 5 RPS error: kP = 12/5 = 2.4.
    # If we want 6V correction for 5 RPS error: kP = 1.2.
    # Let's calculate based on a time constant if possible.
    # Time constant tau = kV / kA (units: (V/(rot/s)) / (V/(rot/s^2)) = s) ?? No.
    # Voltage balance: V = V_emf + I*R
    # Torque balance: T_m = J * alpha + B * omega + T_fric
    # ...
    # From SysId model: V = kV * w + kA * dw/dt
    # Natural decay time constant (coast down): tau = kA / kV.
    # Check units: kA [V/(rot/s^2)], kV [V/(rot/s)].
    # kA/kV = [s].
    # This is the mechanical time constant.

    # If we want to reduce the error time constant to tau_cl (closed loop),
    # kP = (kA / tau_cl) - kV ?? No, that's for position?
    # For Velocity control:
    # u = kP * (w_ref - w) + kFF
    # dynamics: kA * dw/dt + kV * w = u
    # kA * dw/dt + kV * w = kP * (w_ref - w) + kV * w_ref + kA * dw_ref/dt
    # kA * dw/dt + (kV + kP) * w = (kV + kP) * w_ref
    # Time constant closed loop: tau_cl = kA / (kV + kP).
    # We want tau_cl << tau_open_loop (which is kA/kV).
    # Let's target a tau_cl of, say, 0.1s or 0.05s (very fast).
    # kP = (kA / tau_cl) - kV.

    if abs(kV) > 1e-5:
        tau_mechanism = kA / kV
        print(f"  Mechanism Time Constant: {tau_mechanism:.4f} s")

        # Suggest kP for different response times
        targets = [0.1, 0.05, 0.02]
        print("\nSuggested kP values:")
        for t in targets:
            kp_val = (kA / t) - kV
            if kp_val < 0:
                kp_val = 0
            print(f"  Target tau={t}s -> kP = {kp_val:.4f}")
    else:
        print("  kV too small to estimate time constant.")


if __name__ == "__main__":
    main()
