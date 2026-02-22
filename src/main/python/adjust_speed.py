import math
import re
from pathlib import Path

# Target adjustments
# The user wants to lower the speeds slightly.
# Based on typical tuning, 1.5 - 2.0 RPS is a solid "slight" reduction at range.
# We will apply a scalar or a flat reduction. Let's do a flat -1.5 RPS reduction.
RPS_REDUCTION = 1.5
WHEEL_RADIUS_M = 0.0508
SLIP_FACTOR = 0.4993
EFF_R = WHEEL_RADIUS_M * SLIP_FACTOR

GRAVITY = 9.80665
RHO_AIR = 1.225
BALL_MASS_KG = 0.215
BALL_DIAMETER_M = 0.150
BALL_AREA_M2 = math.pi * (BALL_DIAMETER_M / 2) ** 2
CD = 0.485
DRAG_K = 0.5 * RHO_AIR * CD * BALL_AREA_M2 / BALL_MASS_KG

LAUNCH_HEIGHT_M = 0.50
TARGET_HEIGHT_M = 1.8288

DT = 0.0005
MAX_STEPS = int(8.0 / DT)


def simulate_tof_to_x(v0, theta_rad, target_x):
    vx = v0 * math.cos(theta_rad)
    vz = v0 * math.sin(theta_rad)
    x = 0.0
    z = LAUNCH_HEIGHT_M
    t = 0.0

    for _ in range(MAX_STEPS):
        spd = math.sqrt(vx * vx + vz * vz)
        ax1 = -DRAG_K * spd * vx
        az1 = -GRAVITY - DRAG_K * spd * vz

        # Simple Euler for finding crossing time quickly
        x_new = x + vx * DT
        z_new = z + vz * DT
        vx_new = vx + ax1 * DT
        vz_new = vz + az1 * DT
        t_new = t + DT

        if x < target_x and x_new >= target_x:
            # Linear interpolate time
            dx = x_new - x
            alpha = (target_x - x) / dx if dx > 1e-12 else 0.0
            ct = t + alpha * DT
            return ct

        x, z, vx, vz, t = x_new, z_new, vx_new, vz_new, t_new

    return -1.0  # Did not reach


# Current Table
table = {
    0.50: 35.21,
    0.75: 40.39,
    1.00: 45.56,
    1.25: 50.36,
    1.50: 50.54,
    1.75: 50.74,
    2.00: 50.98,
    2.25: 51.26,
    2.50: 51.56,
    2.75: 51.90,
    3.00: 52.80,
    3.25: 54.00,
    3.50: 55.40,
    3.75: 56.80,
    4.00: 58.60,
    4.25: 60.50,
    4.50: 62.50,
    4.75: 64.60,
    5.00: 66.80,
    5.25: 69.10,
    5.50: 71.50,
    5.75: 74.00,
    6.00: 76.50,
    6.25: 79.00,
    6.50: 81.50,
}

# Real angles from ShotTables.java
java_file = Path(
    "src/main/java/frc/robot/subsystems/shooter/ShotTables.java"
).read_text(encoding="utf-8")
angle_matches = re.findall(r"HOOD_ANGLE_MAP.put\(([\d\.]+),\s*([\d\.]+)\);", java_file)
angles = {float(m[0]): float(m[1]) for m in angle_matches}

print("Distance | Old RPS | New RPS | Old TOF | New TOF")
print("-" * 55)

for d, old_rps in table.items():
    if d not in angles:
        continue
    ang_rad = angles[d]

    # Let's do a 4% reduction for smooth scaling instead of a flat drop,
    # because -1.5 RPS at 0.5m (35 rps) is proportionately larger than at 6m (81 rps)
    new_rps = old_rps * 0.96
    if new_rps < 0:
        new_rps = 0

    old_v0 = old_rps * 2 * math.pi * EFF_R
    new_v0 = new_rps * 2 * math.pi * EFF_R

    old_tof = simulate_tof_to_x(old_v0, ang_rad, d)
    new_tof = simulate_tof_to_x(new_v0, ang_rad, d)

    print(
        f"{d:8.2f} | {old_rps:7.2f} | {new_rps:7.2f} | {old_tof:7.3f} | {new_tof:7.3f}"
    )

print("\n// JAVA CODE FOR FLYWHEEL SPEEDS")
for d, old_rps in table.items():
    if d in angles:
        new_rps = old_rps * 0.96
        print(f"    FLYWHEEL_SPEED_MAP.put({d:.2f}, {new_rps:.2f});")

print("\n// JAVA CODE FOR TOF")
for d, old_rps in table.items():
    if d in angles:
        ang_rad = angles[d]
        new_rps = old_rps * 0.96
        new_v0 = new_rps * 2 * math.pi * EFF_R
        t = simulate_tof_to_x(new_v0, ang_rad, d)
        print(f"    TIME_OF_FLIGHT_MAP.put({d:.2f}, {t:.6f});")
