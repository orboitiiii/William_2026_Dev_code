#!/usr/bin/env python3
"""
Recalibrate manually-tuned ShotTables parameters.

Given the user's hand-tuned hood angles and flywheel speeds (which produce the
desired trajectory *shape*), this script:
  1. Simulates each (distance, hood_angle, flywheel_rps) triple using the RK4 engine
  2. Reports where the ball actually crosses TARGET_HEIGHT on descent
  3. For each distance, binary-searches the flywheel speed that makes the ball
     cross TARGET_HEIGHT exactly at the correct ground distance
  4. Outputs corrected flywheel speeds and true ToF values

This keeps the user's hood angles fixed (trajectory shape preserved) and only
adjusts flywheel speed to ensure the ball actually hits the target.
"""

import math

import numpy as np

# ========== Physics Constants (match generate_shot_polynomials.py) ==========
GRAVITY = 9.80665
AIR_DENSITY = 1.225

PROJ_MASS = 0.215
PROJ_DIAM = 0.150
PROJ_CD = 0.485
PROJ_CM = 0.50
PROJ_MOI = 0.0024
PROJ_SDECAY = 0.001
PROJ_AREA = math.pi * (PROJ_DIAM / 2) ** 2
PROJ_R = PROJ_DIAM / 2

WHEEL_RADIUS = 0.0508
SLIP_FACTOR = 0.380  # Match current generate_shot_polynomials.py
EFF_RADIUS = WHEEL_RADIUS * SLIP_FACTOR
TURRET_HEIGHT = 0.438744

# Target = Hub opening height
TARGET_HEIGHT_M = 2.41 * 0.0254 * 39.3701  # 2.41m (95 inches)
# Actually let me just use the value from the main script
TARGET_HEIGHT_M = 95.0 * 0.0254  # 95 inches = 2.413 m

_drag_k = 0.5 * AIR_DENSITY * PROJ_CD * PROJ_AREA / PROJ_MASS
_magnus_k = 0.5 * AIR_DENSITY * PROJ_CM * PROJ_AREA * PROJ_R / PROJ_MASS

SPIN_RPM = 0.0  # Match current config (no backspin)
RPM_TO_RADPS = 2 * math.pi / 60


def _rps_to_vel(rps):
    return rps * 2.0 * math.pi * EFF_RADIUS


def _vel_to_rps(vel):
    return vel / (2.0 * math.pi * EFF_RADIUS)


# ========== RK4 Engine ==========
def _accel(vx, vy, vz, ox, oy, oz):
    vm = math.sqrt(vx * vx + vy * vy + vz * vz)
    df = -_drag_k * vm if vm > 1e-6 else 0.0
    ax = df * vx
    ay = df * vy
    az = -GRAVITY + df * vz
    if abs(ox) + abs(oy) + abs(oz) > 1e-8:
        ax += _magnus_k * (oy * vz - oz * vy)
        ay += _magnus_k * (oz * vx - ox * vz)
        az += _magnus_k * (ox * vy - oy * vx)
    om = math.sqrt(ox * ox + oy * oy + oz * oz)
    if om > 1e-6:
        sd = -PROJ_SDECAY * om / PROJ_MOI
        dox, doy, doz = sd * ox, sd * oy, sd * oz
    else:
        dox = doy = doz = 0.0
    return (ax, ay, az, dox, doy, doz)


def _rk4_sim(pitch, vel, dist, dt=0.002):
    """Simulate ball launched from (dist, 0) aiming at origin.
    Returns list of (t, gx, gy, z, vx, vy, vz) where gx,gy are field coords.
    """
    yaw = math.pi  # Aiming from +X toward origin (yaw = pi)
    # Actually: robot at (dist, 0), target at origin
    # launch direction = toward target = (-1, 0) in field frame
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    vx = vel * cp * cy  # = vel * cp * (-1) = -vel*cp
    vy = vel * cp * sy  # = vel * cp * 0 = 0
    vz = vel * sp

    spin_rad = SPIN_RPM * RPM_TO_RADPS
    ox, oy, oz = 0.0, spin_rad, 0.0

    x, y, z = dist, 0.0, TURRET_HEIGHT
    t = 0.0
    pts = []

    for _ in range(int(5.0 / dt) + 1):
        pts.append((t, x, y, z, vx, vy, vz))

        a1 = _accel(vx, vy, vz, ox, oy, oz)
        dt2 = dt * 0.5
        a2 = _accel(
            vx + dt2 * a1[0],
            vy + dt2 * a1[1],
            vz + dt2 * a1[2],
            ox + dt2 * a1[3],
            oy + dt2 * a1[4],
            oz + dt2 * a1[5],
        )
        a3 = _accel(
            vx + dt2 * a2[0],
            vy + dt2 * a2[1],
            vz + dt2 * a2[2],
            ox + dt2 * a2[3],
            oy + dt2 * a2[4],
            oz + dt2 * a2[5],
        )
        a4 = _accel(
            vx + dt * a3[0],
            vy + dt * a3[1],
            vz + dt * a3[2],
            ox + dt * a3[3],
            oy + dt * a3[4],
            oz + dt * a3[5],
        )

        dt6 = dt / 6.0
        x += dt6 * (
            vx + 2 * (vx + dt2 * a1[0]) + 2 * (vx + dt2 * a2[0]) + (vx + dt * a3[0])
        )
        y += dt6 * (
            vy + 2 * (vy + dt2 * a1[1]) + 2 * (vy + dt2 * a2[1]) + (vy + dt * a3[1])
        )
        z += dt6 * (
            vz + 2 * (vz + dt2 * a1[2]) + 2 * (vz + dt2 * a2[2]) + (vz + dt * a3[2])
        )
        vx += dt6 * (a1[0] + 2 * a2[0] + 2 * a3[0] + a4[0])
        vy += dt6 * (a1[1] + 2 * a2[1] + 2 * a3[1] + a4[1])
        vz += dt6 * (a1[2] + 2 * a2[2] + 2 * a3[2] + a4[2])
        ox += dt6 * (a1[3] + 2 * a2[3] + 2 * a3[3] + a4[3])
        oy += dt6 * (a1[4] + 2 * a2[4] + 2 * a3[4] + a4[4])
        oz += dt6 * (a1[5] + 2 * a2[5] + 2 * a3[5] + a4[5])
        t += dt

        if z <= 0 and len(pts) > 5:
            break

    return pts


def find_crossing(pts, target_z):
    """Find where trajectory crosses target_z on descent.
    Returns (ground_x, tof, apex_z) or None.
    """
    apex_z = max(p[3] for p in pts)

    # Find descending crossing of target_z
    for i in range(1, len(pts)):
        z_prev, z_curr = pts[i - 1][3], pts[i][3]
        vz_curr = pts[i][6]
        if z_prev >= target_z and z_curr < target_z and vz_curr < 0:
            # Linear interpolation
            frac = (target_z - z_prev) / (z_curr - z_prev) if z_curr != z_prev else 0
            gx = pts[i - 1][1] + frac * (pts[i][1] - pts[i - 1][1])
            gy = pts[i - 1][2] + frac * (pts[i][2] - pts[i - 1][2])
            tof = pts[i - 1][0] + frac * (pts[i][0] - pts[i - 1][0])
            ground_dist = math.sqrt(gx * gx + gy * gy)
            return (gx, gy, ground_dist, tof, apex_z)
    return None


def solve_flywheel_for_distance(dist, hood_rad, initial_rps):
    """Binary search flywheel RPS so ball crosses TARGET_HEIGHT at ground_dist ≈ 0 (origin)."""
    rps_lo = 20.0
    rps_hi = 120.0
    best = None

    for _ in range(40):
        rps_mid = (rps_lo + rps_hi) / 2
        vel = _rps_to_vel(rps_mid)
        pts = _rk4_sim(hood_rad, vel, dist, dt=0.002)
        res = find_crossing(pts, TARGET_HEIGHT_M)

        if res is None:
            # Ball didn't reach target height on descent -> need more speed
            rps_lo = rps_mid
            continue

        gx, gy, ground_dist, tof, apex = res

        # gx should be near 0 (origin). If gx > 0, ball fell short (need more speed).
        # If gx < 0, ball overshot (need less speed).
        if abs(gx) < 0.02:  # Within 2cm of target
            best = (rps_mid, tof, apex, gx)
            break
        elif gx > 0:
            # Fell short of origin
            rps_lo = rps_mid
        else:
            # Overshot past origin
            rps_hi = rps_mid

        best = (rps_mid, tof, apex, gx)

    return best


# ========== User's Manual Data ==========
DISTANCES = [
    0.50,
    0.65,
    0.80,
    0.95,
    1.10,
    1.25,
    1.40,
    2.30,
    2.40,
    3.00,
    3.25,
    4.20,
    5.15,
    6.10,
    6.50,
]

HOOD_ANGLES = [
    1.448623,
    1.448623,
    1.448623,
    1.448623,
    1.448623,
    1.448623,
    1.448623,
    1.19,
    1.18,
    1.155,
    1.148,
    1.143,
    1.130,
    1.250,
    1.155,
]

FW_SPEEDS = [
    48.0,
    48.0,
    48.0,
    48.0,
    48.0,
    48.0,
    48.0,
    50.0,
    52.0,
    54.0,
    56.0,
    58.0,
    60.0,
    62.1673,
    63.0400,
]

TOF_OLD = [
    0.747701,
    0.788496,
    0.815415,
    0.834165,
    0.847636,
    0.857454,
    0.864603,
    0.875877,
    0.874970,
    0.863366,
    0.861842,
    1.030780,
    1.181754,
    1.320766,
    1.376595,
]


def main():
    print("=" * 80)
    print("  Manual ShotTable Recalibration")
    print(
        f"  Target Height: {TARGET_HEIGHT_M:.4f} m ({TARGET_HEIGHT_M/0.0254:.1f} inches)"
    )
    print(f"  Turret Height: {TURRET_HEIGHT:.4f} m")
    print(f"  Slip Factor:   {SLIP_FACTOR}")
    print("=" * 80)

    # Step 1: Diagnose current values
    print("\n  ── DIAGNOSIS: Where do current params actually land? ──\n")
    print(
        f"  {'Dist':>5}  {'Hood':>7}  {'FW':>6}  {'Apex':>6}  {'LandX':>7}  {'LandErr':>8}  {'SimToF':>7}  {'OldToF':>7}"
    )
    print(
        f"  {'[m]':>5}  {'[deg]':>7}  {'[rps]':>6}  {'[m]':>6}  {'[m]':>7}  {'[m]':>8}  {'[s]':>7}  {'[s]':>7}"
    )
    print(f"  {'-'*5}  {'-'*7}  {'-'*6}  {'-'*6}  {'-'*7}  {'-'*8}  {'-'*7}  {'-'*7}")

    for i, d in enumerate(DISTANCES):
        vel = _rps_to_vel(FW_SPEEDS[i])
        pts = _rk4_sim(HOOD_ANGLES[i], vel, d, dt=0.002)
        res = find_crossing(pts, TARGET_HEIGHT_M)
        if res:
            gx, gy, gdist, tof, apex = res
            err = gx  # Positive = short, negative = overshot
            print(
                f"  {d:5.2f}  {math.degrees(HOOD_ANGLES[i]):7.1f}  {FW_SPEEDS[i]:6.1f}  "
                f"{apex:6.2f}  {gx:7.3f}  {err:+8.3f}  {tof:7.4f}  {TOF_OLD[i]:7.4f}"
            )
        else:
            apex = max(p[3] for p in pts)
            print(
                f"  {d:5.2f}  {math.degrees(HOOD_ANGLES[i]):7.1f}  {FW_SPEEDS[i]:6.1f}  "
                f"{apex:6.2f}  {'N/A':>7}  {'N/A':>8}  {'N/A':>7}  {TOF_OLD[i]:7.4f}"
            )

    # Step 2: Solve correct flywheel speeds
    print(f"\n\n  ── SOLUTION: Corrected flywheel speeds (hood angles preserved) ──\n")
    print(
        f"  {'Dist':>5}  {'Hood':>7}  {'OldFW':>7}  {'NewFW':>7}  {'Apex':>6}  {'LandX':>7}  {'ToF':>7}"
    )
    print(
        f"  {'[m]':>5}  {'[deg]':>7}  {'[rps]':>7}  {'[rps]':>7}  {'[m]':>6}  {'[m]':>7}  {'[s]':>7}"
    )
    print(f"  {'-'*5}  {'-'*7}  {'-'*7}  {'-'*7}  {'-'*6}  {'-'*7}  {'-'*7}")

    new_fw = []
    new_tof = []

    for i, d in enumerate(DISTANCES):
        sol = solve_flywheel_for_distance(d, HOOD_ANGLES[i], FW_SPEEDS[i])
        if sol:
            rps, tof, apex, gx = sol
            new_fw.append(rps)
            new_tof.append(tof)
            print(
                f"  {d:5.2f}  {math.degrees(HOOD_ANGLES[i]):7.1f}  {FW_SPEEDS[i]:7.2f}  "
                f"{rps:7.2f}  {apex:6.2f}  {gx:7.3f}  {tof:7.4f}"
            )
        else:
            new_fw.append(FW_SPEEDS[i])
            new_tof.append(TOF_OLD[i])
            print(
                f"  {d:5.2f}  {math.degrees(HOOD_ANGLES[i]):7.1f}  {FW_SPEEDS[i]:7.2f}  "
                f"{'FAIL':>7}  {'N/A':>6}  {'N/A':>7}  {'N/A':>7}"
            )

    # Step 3: Output Java code
    print(f"\n\n  ── JAVA OUTPUT ──\n")

    print("    // Hood Angle [rad] vs Distance [m] (user-tuned)")
    for i, d in enumerate(DISTANCES):
        print(f"    HOOD_ANGLE_MAP.put({d:.2f}, {HOOD_ANGLES[i]:.6f});")

    print("\n    // Flywheel Speed [rot/s] vs Distance [m] (physics-corrected)")
    for i, d in enumerate(DISTANCES):
        print(f"    FLYWHEEL_SPEED_MAP.put({d:.2f}, {new_fw[i]:.4f});")

    print("\n    // Time of Flight [s] vs Distance [m] (physics-corrected)")
    for i, d in enumerate(DISTANCES):
        print(f"    TIME_OF_FLIGHT_MAP.put({d:.2f}, {new_tof[i]:.6f});")


if __name__ == "__main__":
    main()
