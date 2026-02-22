#!/usr/bin/env python3
"""
Mid-Air Collision BPS Limit Calculator
=======================================
Team 9427 — 2026

Uses the existing RK4 physics engine to compute the maximum safe BPS at each
distance, accounting for:
  1. Minimum ball-to-ball spacing along the trajectory (apex bottleneck)
  2. Gaussian noise safety margin (50% derating per CD thread analysis)
  3. Backspin trajectory effects

Physics Model:
  - Two consecutive balls follow identical trajectories offset by dt = 1/BPS
  - The spatial separation at any point is ~|v(t)| * dt
  - Minimum separation occurs at the apex where v_vertical = 0
  - Critical BPS = v_min / D_ball
  - Output as Spinner RPS using ratio: 25 RPS = 12 BPS → 1 BPS = 2.0833 RPS
  - Aggressive strategy: use critical BPS directly (no safety margin)

References:
  - ChiefDelphi: https://www.chiefdelphi.com/t/512598
  - SamCarlberg's analysis: critical BPS is the theoretical collision threshold.

Output:
  - Console: BPS limit table for all distances
  - Java: ShotTables spline coefficients for BPS_MAX(distance)
"""

import math

import numpy as np
from scipy.interpolate import CubicSpline

# ============================================================================
# Import the RK4 engine from the existing generator
# (copy the essential constants and functions to avoid import issues)
# ============================================================================

GRAVITY = 9.80665
AIR_DENSITY = 1.225

PROJ_MASS = 0.215
PROJ_DIAM = 0.150  # Ball diameter [m]
PROJ_CD = 0.485
PROJ_CM = 0.50
PROJ_MOI = 0.0024
PROJ_SDECAY = 0.001
PROJ_AREA = math.pi * (PROJ_DIAM / 2) ** 2
PROJ_R = PROJ_DIAM / 2

WHEEL_RADIUS = 0.0508
SLIP_FACTOR = 0.4345
EFF_RADIUS = WHEEL_RADIUS * SLIP_FACTOR
TURRET_HEIGHT = 0.438744

RPM_TO_RADPS = 2 * math.pi / 60

_drag_k = 0.5 * AIR_DENSITY * PROJ_CD * PROJ_AREA / PROJ_MASS
_magnus_k = 0.5 * AIR_DENSITY * PROJ_CM * PROJ_AREA * PROJ_R / PROJ_MASS

# Backspin RPM — the user confirmed "full backspin shooting"
# The spin is applied by the shooter wheels. Typical backspin rate
# correlates with flywheel surface speed difference.
BACKSPIN_RPM = 0  # Match generate_shot_polynomials.py (SPIN_RPM = 0.0)

# Spinner RPS to BPS conversion
# kSideRollerTargetVelocity = -25.0 RPS produces ~12 BPS
SPINNER_RPS_PER_BPS = 25.0 / 12.0  # 2.0833 RPS per BPS

# ============================================================================
# Shot table data points (from ShotTables.java)
# ============================================================================
DISTANCES = np.array(
    [
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
)

HOOD_ANGLES_RAD = np.array(
    [
        1.448623,
        1.423455,
        1.398287,
        1.373119,
        1.347951,
        1.322783,
        1.297615,
        1.146607,
        1.129828,
        1.029156,
        1.031068,
        1.038334,
        1.045599,
        1.052865,
        1.055924,
    ]
)

FLYWHEEL_RPS = np.array(
    [
        39.5075,
        40.4292,
        41.1517,
        41.7522,
        42.2778,
        42.7581,
        43.2124,
        45.9368,
        46.2712,
        48.5282,
        52.8332,
        57.7891,
        62.5810,
        67.1673,
        69.0400,
    ]
)


def _rps_to_vel(rps):
    """Convert flywheel RPS to ball exit velocity [m/s]."""
    return rps * 2.0 * math.pi * EFF_RADIUS


# ============================================================================
# RK4 simulation (simplified from generate_shot_polynomials.py)
# ============================================================================
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


def simulate_trajectory(pitch_rad, exit_vel, backspin_rpm=BACKSPIN_RPM, dt=0.001):
    """Simulate a single ball trajectory with backspin.

    Returns list of (t, x, z, speed) tuples.
    x = horizontal distance from launch, z = altitude.
    speed = total velocity magnitude at each point.
    """
    cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
    # Launch in +X direction (yaw = 0)
    vx = exit_vel * cp
    vy = 0.0
    vz = exit_vel * sp

    # Backspin axis: perpendicular to trajectory plane (+Y for yaw=0)
    spin_rps = backspin_rpm * RPM_TO_RADPS
    ox, oy, oz = 0.0, spin_rps, 0.0

    x, y, z = 0.0, 0.0, TURRET_HEIGHT
    pts = []

    for _ in range(int(5.0 / dt) + 1):
        speed = math.sqrt(vx * vx + vy * vy + vz * vz)
        pts.append((x, z, speed, vx, vz))

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

        if z <= 0 and len(pts) > 5:
            break

    return pts


# ============================================================================
# BPS limit computation
# ============================================================================
def compute_bps_limit(pitch_rad, exit_vel):
    """Compute the critical BPS and max Spinner RPS for a given launch config.

    Algorithm:
      1. Simulate full trajectory via RK4
      2. Find minimum speed along the path (occurs near apex)
      3. Critical BPS = v_min / D_ball (balls just touching at apex)
      4. Max Spinner RPS = Critical BPS * SPINNER_RPS_PER_BPS

    Aggressive strategy: use critical BPS directly (no safety margin).
    Balls at the collision threshold are "just touching" — most still score.

    Args:
        pitch_rad: Launch pitch angle [rad]
        exit_vel: Ball exit velocity [m/s]

    Returns:
        (critical_bps, max_spinner_rps, v_min)
    """
    pts = simulate_trajectory(pitch_rad, exit_vel)

    v_min = float("inf")
    for x, z, speed, vx, vz in pts:
        if z >= TURRET_HEIGHT * 0.5:
            if speed < v_min:
                v_min = speed

    critical_bps = v_min / PROJ_DIAM
    max_spinner_rps = critical_bps * SPINNER_RPS_PER_BPS

    return critical_bps, max_spinner_rps, v_min


# ============================================================================
# Main sweep
# ============================================================================
def main():
    print("=" * 75)
    print("  Mid-Air Collision Spinner RPS Limit Analysis")
    print("  Team 9427 | 2026")
    print(f"  Ball diameter: {PROJ_DIAM*100:.1f} cm")
    print(f"  Spinner ratio: {SPINNER_RPS_PER_BPS:.4f} RPS/BPS (25 RPS = 12 BPS)")
    print(f"  Strategy: AGGRESSIVE (critical BPS, no safety margin)")
    print("=" * 75)

    print(
        f"\n  {'Dist':>5}  {'Pitch':>7}  {'v_exit':>7}  {'v_min':>7}  "
        f"{'Crit BPS':>9}  {'Max RPS':>8}  {'Curr RPS':>8}  {'Status':>10}"
    )
    print(
        f"  {'[m]':>5}  {'[deg]':>7}  {'[m/s]':>7}  {'[m/s]':>7}  "
        f"{'[1/s]':>9}  {'[1/s]':>8}  {'25.0':>8}  {'':>10}"
    )
    print(f"  {'-'*5}  {'-'*7}  {'-'*7}  {'-'*7}  {'-'*9}  {'-'*8}  {'-'*8}  {'-'*10}")

    current_rps = 25.0  # kSideRollerTargetVelocity
    max_rps_list = []

    for i, d in enumerate(DISTANCES):
        pitch = HOOD_ANGLES_RAD[i]
        rps = FLYWHEEL_RPS[i]
        v_exit = _rps_to_vel(rps)

        crit_bps, max_rps, v_min = compute_bps_limit(pitch, v_exit)
        max_rps_list.append(max_rps)

        status = "OK" if current_rps <= max_rps else "LIMIT!"
        print(
            f"  {d:5.2f}  {math.degrees(pitch):7.1f}  {v_exit:7.2f}  {v_min:7.2f}  "
            f"{crit_bps:9.1f}  {max_rps:8.1f}  {current_rps:8.1f}  {status:>10}"
        )

    # Generate cubic spline coefficients for MAX_SPINNER_RPS
    max_rps_array = np.array(max_rps_list)
    cs = CubicSpline(DISTANCES, max_rps_array, bc_type="natural")

    print(f"\n\n{'='*75}")
    print("  CUBIC SPLINE COEFFICIENTS FOR MAX_SPINNER_RPS(distance)")
    print(f"{'='*75}")

    n = len(DISTANCES)
    print(f"\n  // Max Spinner RPS Spline Coefficients ({n} knots, {n-1} segments)")
    print(f"  // Aggressive: critical BPS * {SPINNER_RPS_PER_BPS:.4f} RPS/BPS")

    def fmt(v):
        return f"{v:.15e}"

    a_vals = [cs(DISTANCES[i]) for i in range(n - 1)]
    print(
        f"  private static final double[] MAXRPS_A = "
        f"{{{', '.join(fmt(v) for v in a_vals)}}};"
    )
    print(
        f"  private static final double[] MAXRPS_B = "
        f"{{{', '.join(fmt(v) for v in cs.c[2,:])}}};"
    )
    print(
        f"  private static final double[] MAXRPS_C = "
        f"{{{', '.join(fmt(v) for v in cs.c[1,:])}}};"
    )
    print(
        f"  private static final double[] MAXRPS_D = "
        f"{{{', '.join(fmt(v) for v in cs.c[0,:])}}};"
    )

    # Summary
    print(f"\n\n{'='*75}")
    print("  SUMMARY")
    print(f"{'='*75}")
    print(
        f"  Min max-spinner-RPS: {min(max_rps_list):.1f} RPS "
        f"({min(max_rps_list)/SPINNER_RPS_PER_BPS:.1f} BPS) "
        f"at d={DISTANCES[np.argmin(max_rps_list)]:.2f}m"
    )
    print(
        f"  Max max-spinner-RPS: {max(max_rps_list):.1f} RPS "
        f"({max(max_rps_list)/SPINNER_RPS_PER_BPS:.1f} BPS) "
        f"at d={DISTANCES[np.argmax(max_rps_list)]:.2f}m"
    )
    print(f"  Current fixed RPS:   {current_rps:.1f}")
    limited = [
        DISTANCES[i] for i in range(len(DISTANCES)) if max_rps_list[i] < current_rps
    ]
    if limited:
        print(f"  NEEDS LIMITING at:   {', '.join(f'{d:.2f}m' for d in limited)}")
    else:
        print(f"  No limiting needed at current {current_rps:.0f} RPS")


if __name__ == "__main__":
    main()
