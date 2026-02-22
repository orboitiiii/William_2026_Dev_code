import glob
import os

import numpy as np
import pandas as pd


def derive_acceleration(df):
    # Calculate dt and dv
    df["dt"] = df["Timestamp"].diff()
    df["dv"] = df["Velocity"].diff()

    # Calculate acceleration
    # Filter out small time steps to avoid numerical instability
    df = df[df["dt"] > 0.001].copy()
    df["Acceleration"] = df["dv"] / df["dt"]

    # Apply rolling median filter to remove spikes
    df["Acceleration"] = df["Acceleration"].rolling(window=5, center=True).median()

    # Drop NaNs created by diff and rolling
    return df.dropna()


def analyze_sysid_data(directory="src/main/python"):
    # Find all SysId CSV files
    pattern = os.path.join(directory, "sysid_Shooter_*.csv")
    files = glob.glob(pattern)

    if not files:
        print("No SysId files found.")
        return

    print(f"Found {len(files)} files: {files}")

    all_data = []

    for f in files:
        print(f"Processing {f}...")
        try:
            df = pd.read_csv(f)
            # Basic validation
            required_cols = ["Timestamp", "Voltage", "Velocity"]
            if not all(col in df.columns for col in required_cols):
                print(f"Skipping {f}: Missing columns")
                continue

            # Filter out zero voltage (deadband/idle)
            # df = df[abs(df['Voltage']) > 0.1]

            # Derive acceleration
            df = derive_acceleration(df)

            if len(df) > 0:
                all_data.append(df)
        except Exception as e:
            print(f"Error reading {f}: {e}")

    if not all_data:
        print("No valid data to analyze.")
        return

    combined_df = pd.concat(all_data, ignore_index=True)

    # Filter out low velocity noise for regression (keep stiction region for kS?)
    # Usually we want steady motion for kV.
    # We remove very low velocities to avoid sign flipping noise, but need near-zero for kS.
    # Let's filter absolute velocity > 0.5 rot/s
    regression_df = combined_df[abs(combined_df["Velocity"]) > 0.5].copy()

    if len(regression_df) == 0:
        print("No data remaining after filtering.")
        return

    # Prepare features: V = kS * sign(v) + kV * v + kA * a
    # A = [sign_v, v, a]
    A = np.column_stack(
        [
            np.sign(regression_df["Velocity"]),
            regression_df["Velocity"],
            regression_df["Acceleration"],
        ]
    )
    y = regression_df["Voltage"].values

    # Perform Linear Regression using NumPy
    # x, residuals, rank, s = lstsq(A, y)
    beta, residuals, rank, s = np.linalg.lstsq(A, y, rcond=None)

    kS = beta[0]
    kV = beta[1]
    kA = beta[2]

    # R^2 Calculation
    y_pred = A @ beta
    ss_res = np.sum((y - y_pred) ** 2)
    ss_tot = np.sum((y - np.mean(y)) ** 2)
    r2 = 1 - (ss_res / ss_tot)

    print("\n--- SysId Analysis Results ---")
    print(f"R^2: {r2:.4f}")
    print(f"kS: {kS:.5f} Volts")
    print(f"kV: {kV:.5f} Volts * s / rot")
    print(f"kA: {kA:.5f} Volts * s^2 / rot")

    # Suggest kP
    # Simple rule: kP = 0.1 * kV (very rough) or based on error tolerance.
    # If we want to correct 1 RPS error with 0.1V, kP = 0.1.
    # Let's suggest a range.
    print(f"Suggested kP (Conservative): {kV * 0.5:.4f}")
    print(f"Suggested kP (Aggressive): {kV * 2.0:.4f}")


if __name__ == "__main__":
    analyze_sysid_data()
