import math
import re
from pathlib import Path

# Physics constants
GRAVITY = 9.80665
RHO_AIR = 1.225
BALL_MASS_KG = 0.215
BALL_DIAMETER_M = 0.150
BALL_AREA_M2 = math.pi * (BALL_DIAMETER_M / 2) ** 2
CD = 0.485
DRAG_K = 0.5 * RHO_AIR * CD * BALL_AREA_M2 / BALL_MASS_KG

LAUNCH_HEIGHT_M = 0.50
TARGET_HEIGHT_M = 1.8288
WHEEL_RADIUS_M = 0.0508
SLIP_FACTOR = 0.4993
EFF_R = WHEEL_RADIUS_M * SLIP_FACTOR

DT = 0.0005
MAX_STEPS = int(8.0 / DT)


def simulate_tof_to_x(v0, theta_rad, target_x):
    vx = v0 * math.cos(theta_rad)
    vz = v0 * math.sin(theta_rad)
    x = 0.0
    z = LAUNCH_HEIGHT_M
    t = 0.0

    for _ in range(MAX_STEPS):

        def deriv(vx_, vz_):
            spd = math.sqrt(vx_ * vx_ + vz_ * vz_)
            ax = -DRAG_K * spd * vx_
            az = -GRAVITY - DRAG_K * spd * vz_
            return ax, az

        # RK4 step
        ax1, az1 = deriv(vx, vz)
        hdt = 0.5 * DT
        ax2, az2 = deriv(vx + hdt * ax1, vz + hdt * az1)
        ax3, az3 = deriv(vx + hdt * ax2, vz + hdt * az2)
        ax4, az4 = deriv(vx + DT * ax3, vz + DT * az3)

        x_new = x + (DT / 6.0) * (
            vx + 2 * (vx + hdt * ax1) + 2 * (vx + hdt * ax2) + (vx + DT * ax3)
        )
        # z calculation omitted as we only care about x crossing for TOF
        vx_new = vx + (DT / 6.0) * (ax1 + 2 * ax2 + 2 * ax3 + ax4)
        vz_new = vz + (DT / 6.0) * (az1 + 2 * az2 + 2 * az3 + az4)
        t_new = t + DT

        if x < target_x and x_new >= target_x:
            dx = x_new - x
            alpha = (target_x - x) / dx if dx > 1e-12 else 0.0
            ct = t + alpha * DT
            return ct

        x, vx, vz, t = x_new, vx_new, vz_new, t_new

    return -1.0  # Did not reach


java_file = Path(
    "src/main/java/frc/robot/subsystems/shooter/ShotTables.java"
).read_text(encoding="utf-8")

speed_matches = re.findall(
    r"FLYWHEEL_SPEED_MAP.put\(([\d\.]+),\s*([\d\.]+)\);", java_file
)
speeds = {float(m[0]): float(m[1]) for m in speed_matches}

angle_matches = re.findall(r"HOOD_ANGLE_MAP.put\(([\d\.]+),\s*([\d\.]+)\);", java_file)
angles = {float(m[0]): float(m[1]) for m in angle_matches}

print("// JAVA CODE FOR FLYWHEEL SPEEDS")
for d, old_rps in sorted(speeds.items()):
    # Reduce by flat 1.5 RPS for ranges >= 3.0m, blend it for closer distances to avoid stalling
    reduction = 1.5 if d >= 2.0 else (d / 2.0) * 1.5
    new_rps = max(0, old_rps - reduction)
    print(f"    FLYWHEEL_SPEED_MAP.put({d:.2f}, {new_rps:.2f});")

print("\n// JAVA CODE FOR TOF")
for d, old_rps in sorted(speeds.items()):
    reduction = 1.5 if d >= 2.0 else (d / 2.0) * 1.5
    new_rps = max(0, old_rps - reduction)
    new_v0 = new_rps * 2 * math.pi * EFF_R

    if d in angles:
        ang_rad = angles[d]
    else:
        # Fallback for 0.5m-1.0m missing angles in map, assume ~1.2 rad
        ang_rad = 1.2

    t = simulate_tof_to_x(new_v0, ang_rad, d)

    # In ShotTables, close ranges are clamped to ~1.36s for the high apex shots
    # since TOF to x=d is meaningless if the ball is still falling.
    # The existing table uses ~1.36s from 1.25m to 2.75m. I will compute exact X-crossing TOF
    # because that's mechanically the time it takes the ball to travel horizontal distance.
    # Actually, if the ball travels up and down, time to horizontal 'd' is correct.

    # If t is negative (didn't reach), use old + 0.05
    if t < 0:
        t = 1.5

    print(f"    TIME_OF_FLIGHT_MAP.put({d:.2f}, {t:.6f});")
