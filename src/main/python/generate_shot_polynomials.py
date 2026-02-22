#!/usr/bin/env python3
"""
Orbit 1690 + 6328 Hybrid: Offline Simulation -> Interpolating Lookup Tables
=============================================================================
Team 9427 - 2026

Uses the RK4 physics engine (identical to C++ BallisticSolver) to sweep the
distance parameter space, then outputs InterpolatingDoubleTreeMap entries for
use on the robot.

Design Philosophy (from 6328):
  "We're relying on a set of maps that correlate distance to a set of manually
  tuned hood and flywheel setpoints. Compared to a full physics model, this
  approach allows us to gracefully handle real-world imperfections with minimal
  tuning effort. We can retune the setpoints for part of the range at
  competition without risking unwanted changes to other parts of the range."

Key Insight:
  - Initial values come from physics simulation (Orbit 1690's contribution)
  - Stored as InterpolatingDoubleTreeMap (6328's design) for point-by-point
    tunability at competition
  - Motion compensation is handled by the iterative ToF solver in
    ShotCalculator (no separate 2D correction needed)

Output:
  1. Java class `ShotTables.java` (InterpolatingDoubleTreeMap entries)
  2. Validation plots
  3. CSV of raw simulation data

Hardware:
  4" Orange (40A) AndyMark Stealth Wheel + Polycarbonate Backplate
  Slip factor = 0.60 | Max 5600 RPM

Usage:
  python generate_shot_polynomials.py
  python generate_shot_polynomials.py --step 0.25    # coarser table
"""

import argparse
import math
import sys
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

# ============================================================================
# Output directories
# ============================================================================
SCRIPT_DIR = Path(__file__).parent
OUT_DIR = SCRIPT_DIR / "ballistic_report"
JAVA_OUT_DIR = SCRIPT_DIR.parent / "java" / "frc" / "robot" / "subsystems" / "shooter"

# ============================================================================
# Physical Constants  (must match C++ BallisticSolver exactly)
# ============================================================================
GRAVITY = 9.80665
AIR_DENSITY = 1.225

# Projectile
PROJ_MASS = 0.215
PROJ_DIAM = 0.150
PROJ_CD = 0.485
PROJ_CM = 0.50
PROJ_MOI = 0.0024
PROJ_SDECAY = 0.001
PROJ_AREA = math.pi * (PROJ_DIAM / 2) ** 2
PROJ_R = PROJ_DIAM / 2

# Shooter
WHEEL_RADIUS = 0.0508
SLIP_FACTOR = 0.38  # Reduced from 0.4345 to increase overall RPM
EFF_RADIUS = WHEEL_RADIUS * SLIP_FACTOR
MAX_RPM = 5600.0
TURRET_HEIGHT = 0.438744  # 43.8744 cm — measured turret pivot height
PITCH_MIN = math.radians(62.5)  # 1.0908 rad — mechanical lower limit (safety margin)
PITCH_MAX = math.radians(83.0)  # 1.4486 rad — mechanical upper limit
TURRET_RATE = 6.0
SPIN_RPM = -2500.0  # Negative for backspin (lift)

# Target height bumped from ~2.1m to ~2.4m (95 inches) for extreme arc
TARGET_HEIGHT_M = 95.0 * 0.0254  # 2.413 m

RPM_TO_RADPS = 2 * math.pi / 60
INCH_TO_M = 0.0254

# Pre-computed force constants
_drag_k = 0.5 * AIR_DENSITY * PROJ_CD * PROJ_AREA / PROJ_MASS
_magnus_k = 0.5 * AIR_DENSITY * PROJ_CM * PROJ_AREA * PROJ_R / PROJ_MASS


def _rpm_to_vel(rpm):
    return rpm * RPM_TO_RADPS * EFF_RADIUS


def _vel_to_rpm(vel):
    return vel / (RPM_TO_RADPS * EFF_RADIUS)


# ============================================================================
# Forces  (match BallisticSolver.cpp:computeAcceleration)
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


# ============================================================================
# RK4 Trajectory Simulation  (match BallisticSolver.cpp:rk4Step)
# ============================================================================
def _rk4_sim(yaw, pitch, velocity, rx, ry, vx_robot, vy_robot, dt=0.005):
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    vx = velocity * cp * cy + vx_robot
    vy = velocity * cp * sy + vy_robot
    vz = velocity * sp
    spin_rps = SPIN_RPM * RPM_TO_RADPS
    ox, oy, oz = -sy * spin_rps, cy * spin_rps, 0.0
    x, y, z = rx, ry, TURRET_HEIGHT
    pts = []
    t = 0.0
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
        if z <= 0 and t > 0.01:
            break
    return pts


# ============================================================================
# Trajectory Helpers
# ============================================================================
def _find_z_at_dist(pts, launch_x, launch_y, target_dist):
    best_desc = None
    for i in range(1, len(pts)):
        d = math.hypot(pts[i][1] - launch_x, pts[i][2] - launch_y)
        dp = math.hypot(pts[i - 1][1] - launch_x, pts[i - 1][2] - launch_y)
        if d >= target_dist and dp < target_dist:
            a = (target_dist - dp) / (d - dp) if d != dp else 0
            z_at = pts[i - 1][3] + a * (pts[i][3] - pts[i - 1][3])
            vz_at = pts[i - 1][6] + a * (pts[i][6] - pts[i - 1][6])
            tof = pts[i - 1][0] + a * (pts[i][0] - pts[i - 1][0])
            if vz_at < 0:
                return z_at, vz_at, tof
            elif best_desc is None:
                best_desc = (z_at, vz_at, tof)
    return best_desc


_SQRT3 = 1.7320508075688772
_HEX_Y = 20.966
_HEX_D = 41.932


def _inside_hexagon(x_m, y_m):
    xi, yi = x_m / INCH_TO_M, y_m / INCH_TO_M
    if yi < -_HEX_Y or yi > _HEX_Y:
        return False
    if -_SQRT3 * xi + yi > _HEX_D:
        return False
    if _SQRT3 * xi + yi > _HEX_D:
        return False
    if -_SQRT3 * xi - yi > _HEX_D:
        return False
    if _SQRT3 * xi - yi > _HEX_D:
        return False
    return True


# ============================================================================
# Solver
# ============================================================================
def _find_descending_crossing(pts, target_z):
    """Find where trajectory crosses target_z on the descending phase.

    Returns (x_at, y_at, vz_at, tof) or None.
    x_at, y_at are the field coordinates where the ball crosses target_z
    while descending. The target is at the field origin (0, 0).
    """
    # Find apex index (highest z)
    apex_idx = 0
    for i in range(1, len(pts)):
        if pts[i][3] > pts[apex_idx][3]:
            apex_idx = i

    # Search only after apex for descending crossing at target_z
    for i in range(apex_idx + 1, len(pts)):
        z_prev, z_curr = pts[i - 1][3], pts[i][3]
        if z_prev >= target_z and z_curr < target_z:
            a = (target_z - z_prev) / (z_curr - z_prev) if z_curr != z_prev else 0
            x_at = pts[i - 1][1] + a * (pts[i][1] - pts[i - 1][1])
            y_at = pts[i - 1][2] + a * (pts[i][2] - pts[i - 1][2])
            vz_at = pts[i - 1][6] + a * (pts[i][6] - pts[i - 1][6])
            tof = pts[i - 1][0] + a * (pts[i][0] - pts[i - 1][0])
            return (x_at, y_at, vz_at, tof)
    return None


def _find_velocity_for_pitch(yaw, pitch, rx, ry, dist):
    """Binary-search exit velocity so the ball descends through TARGET_H
    at the target position (field origin).

    Uses two strategies:
    1. Height-crossing method: Find where z = TARGET_H on descent, then
       check if ground distance from origin is small enough.
    2. Distance-crossing fallback: Original method for longer ranges.
    """
    v_lo = _rpm_to_vel(100)  # Small minimum to avoid degenerate trajectories
    v_hi = _rpm_to_vel(MAX_RPM)
    best = None
    # Adaptive dt: finer for short distances where trajectories are more sensitive
    sim_dt = (
        0.002 if dist < 1.0 else 0.003 if dist < 2.0 else 0.005 if dist < 3.0 else 0.008
    )

    # --- Strategy 1: Height-crossing method ---
    # Binary search velocity so ground_dist_at_target_h ≈ 0
    # Key insight: velocity monotonically maps to how far the ball overshoots.
    # More velocity → ball overshoots → landing x goes more negative (past target).
    # Less velocity → ball undershoots → apex doesn't reach TARGET_H.
    for _ in range(40):
        vm = (v_lo + v_hi) / 2
        pts = _rk4_sim(yaw, pitch, vm, rx, ry, 0, 0, dt=sim_dt)

        apex_z = max(p[3] for p in pts)
        if apex_z < TARGET_HEIGHT_M:
            v_lo = vm
            continue

        res = _find_descending_crossing(pts, TARGET_HEIGHT_M)
        if res is None:
            v_lo = vm
            continue

        x_at, y_at, vz_at, tof = res
        ground_dist = math.hypot(x_at, y_at)

        if ground_dist < 0.05:  # Within 5cm of target center
            best = (vm, tof, TARGET_HEIGHT_M)
            v_hi = vm  # Try to find even lower velocity (shallowest descent)
        else:
            # Ball overshot target. The ball's x_at should be negative
            # (past origin in the robot→target direction).
            # We need to compute the signed error along the yaw direction.
            # Project the crossing point onto the launch→target axis to get signed err.
            # target is at origin, robot is at (rx, ry)
            # launch direction: toward target = (-rx, -ry) normalized
            launch_dir_x = -rx / dist if dist > 0 else 0
            launch_dir_y = -ry / dist if dist > 0 else 0
            # Signed projection: positive = still before target, negative = past target
            signed_proj = x_at * launch_dir_x + y_at * launch_dir_y
            if signed_proj < 0:
                # Ball overshot past target → reduce velocity
                v_hi = vm
            else:
                # Ball didn't reach target → increase velocity
                v_lo = vm

    # --- Strategy 2: Distance-based fallback (works well for d > ~1.5m) ---
    if best is None:
        v_lo2 = _rpm_to_vel(100)
        v_hi2 = _rpm_to_vel(MAX_RPM)
        for _ in range(30):
            vm = (v_lo2 + v_hi2) / 2
            pts = _rk4_sim(yaw, pitch, vm, rx, ry, 0, 0, dt=sim_dt)
            res = _find_z_at_dist(pts, pts[0][1], pts[0][2], dist)
            if res is None:
                v_lo2 = vm
                continue
            z_at, vz_at, tof = res
            if vz_at >= 0:
                v_hi2 = vm
                continue
            e = z_at - TARGET_HEIGHT_M
            if abs(e) < 0.02:
                best = (vm, tof, z_at)
                v_hi2 = vm
            elif e > 0:
                v_hi2 = vm
            else:
                v_lo2 = vm
    return best


def _solve_static(robot_x, robot_y):
    dist = math.hypot(robot_x, robot_y)
    yaw = math.atan2(-robot_y, -robot_x)

    # Force near distances (~1.1m) to prefer higher hood angles near PITCH_MAX.
    if dist <= 1.25:
        p_lo = math.radians(75.8)
        p_hi = PITCH_MAX
    else:
        p_lo = PITCH_MIN
        p_hi = PITCH_MAX

    best = None
    sim_dt = (
        0.002 if dist < 1.0 else 0.003 if dist < 2.0 else 0.005 if dist < 3.0 else 0.008
    )
    for _ in range(25):  # More iterations for narrow 15° pitch range
        p_mid = (p_lo + p_hi) / 2
        result = _find_velocity_for_pitch(yaw, p_mid, robot_x, robot_y, dist)
        rpm_ok = False
        if result is not None:
            vel, tof, z_at = result
            rpm = _vel_to_rpm(vel)
            if rpm <= MAX_RPM:
                rpm_ok = True
        if rpm_ok:
            pts = _rk4_sim(yaw, p_mid, vel, robot_x, robot_y, 0, 0, dt=sim_dt)
            apex = max(p[3] for p in pts)
            # Find actual impact coordinates using descending crossing
            dc = _find_descending_crossing(pts, TARGET_HEIGHT_M)
            if dc is not None:
                ix, iy = dc[0], dc[1]
            else:
                # Fallback to old distance-from-launch method
                lx, ly = pts[0][1], pts[0][2]
                ix, iy = 0.0, 0.0
                for j in range(1, len(pts)):
                    d = math.hypot(pts[j][1] - lx, pts[j][2] - ly)
                    dp = math.hypot(pts[j - 1][1] - lx, pts[j - 1][2] - ly)
                    if d >= dist and dp < dist:
                        a = (dist - dp) / (d - dp) if d != dp else 0
                        ix = pts[j - 1][1] + a * (pts[j][1] - pts[j - 1][1])
                        iy = pts[j - 1][2] + a * (pts[j][2] - pts[j - 1][2])
                        break
            if not _inside_hexagon(ix, iy):
                p_lo = p_mid
                continue
            best = dict(
                pitch=p_mid,
                vel=vel,
                rpm=rpm,
                tof=tof,
                yaw=yaw,
                apex=apex,
                z_at=z_at,
                dist=dist,
            )
            p_hi = p_mid
        else:
            p_lo = p_mid
    return best


# ============================================================================
# Sweep
# ============================================================================
def sweep_static(dist_min=1.5, dist_max=10.0, dist_step=0.05):
    distances = np.arange(dist_min, dist_max + dist_step / 2, dist_step)
    results = []
    n = len(distances)
    for i, d in enumerate(distances):
        if (i + 1) % 5 == 0 or i == 0:
            print(f"    Sweep: {i+1}/{n} (d={d:.2f}m)", end="\r", flush=True)
        sol = _solve_static(d, 0.0)
        if sol:
            results.append(
                (d, sol["pitch"], sol["vel"], sol["rpm"], sol["tof"], sol["apex"])
            )
    print(f"    Sweep: {n}/{n} done.              ")
    return results


# ============================================================================
# Adaptive table spacing: dense where gradient is steep, sparse where flat
# ============================================================================
def compute_adaptive_entries(static_data, max_hood_err_deg=0.15):
    """Select table entries with adaptive spacing based on gradient analysis.

    Strategy:
      1. Compute |d(pitch_deg)/d(distance)| at each simulation point.
      2. In regions with steep gradient, use dense spacing (0.10-0.25m).
         In flat regions, use sparse spacing (0.50-1.00m).
      3. Always include first, last, and the RPM-transition inflection point.

    Args:
        static_data: list of (d, pitch, vel, rpm, tof, apex) from dense sweep.
        max_hood_err_deg: target max interpolation error in degrees.

    Returns:
        List of (d, pitch, vel, rps, tof) for the selected table entries.
    """
    sd = np.array(static_data)
    all_d = sd[:, 0]
    all_pitch = sd[:, 1]
    all_rpm = sd[:, 3]

    # Compute gradient magnitude |d(pitch_deg)/d(dist)| via central differences
    pitch_deg = np.degrees(all_pitch)
    grad = np.abs(np.gradient(pitch_deg, all_d))

    # Also detect RPM transition zone (where RPM gradient is steepest)
    rpm_grad = np.abs(np.gradient(all_rpm, all_d))

    # Assign desired spacing per point: inversely proportional to gradient
    # max_err ~ gradient * (step/2)  =>  step ~ 2 * max_err / gradient
    min_step = 0.15  # never closer than 15cm
    max_step = 1.00  # never sparser than 1.0m

    desired_step = np.clip(
        2.0 * max_hood_err_deg / np.maximum(grad, 0.1), min_step, max_step
    )

    # In RPM transition zone (gradient > 400 RPM/m), force dense spacing
    rpm_transition = rpm_grad > 400
    desired_step[rpm_transition] = np.minimum(desired_step[rpm_transition], 0.20)

    # Walk through distances greedily, picking the next entry when accumulated
    # distance exceeds the local desired step
    selected_indices = [0]  # always include first
    accumulated = 0.0
    for i in range(1, len(all_d)):
        accumulated += all_d[i] - all_d[i - 1]
        local_step = desired_step[i]
        if accumulated >= local_step * 0.95:  # 5% tolerance
            selected_indices.append(i)
            accumulated = 0.0

    # Always include last point
    if selected_indices[-1] != len(all_d) - 1:
        selected_indices.append(len(all_d) - 1)

    # Build table entries
    table_entries = []
    for idx in selected_indices:
        d, pitch, vel, rpm, tof, apex = sd[idx]
        rps = rpm / 60.0
        table_entries.append((round(d, 2), pitch, vel, rps, tof))

    # Report spacing statistics
    dists = [e[0] for e in table_entries]
    steps = [dists[i + 1] - dists[i] for i in range(len(dists) - 1)]
    print(f"    Adaptive spacing: {len(table_entries)} entries")
    print(
        f"    Step range: [{min(steps):.2f}, {max(steps):.2f}]m, "
        f"mean={np.mean(steps):.2f}m"
    )
    print(f"    Range: [{dists[0]:.2f}, {dists[-1]:.2f}]m")

    return table_entries


# ============================================================================
# Generate Java: InterpolatingDoubleTreeMap entries
# ============================================================================
def generate_java(static_data, table_step, java_path):
    """Generate ShotTables.java with InterpolatingDoubleTreeMap entries."""
    sd = np.array(static_data)

    # Use adaptive spacing instead of uniform
    table_entries = compute_adaptive_entries(static_data, max_hood_err_deg=0.15)

    d_min = table_entries[0][0]
    d_max = table_entries[-1][0]

    L = []  # lines
    L.append("package frc.robot.subsystems.shooter;")
    L.append("")
    L.append("import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;")
    L.append("import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;")
    L.append("")
    L.append("/**")
    L.append(" * ShotTables - Simulation-Generated Shooting Lookup Tables.")
    L.append(" *")
    L.append(
        " * <p>Generated by {@code generate_shot_polynomials.py} using the RK4 physics engine"
    )
    L.append(
        " * (identical to C++ BallisticSolver). Values are stored as {@link InterpolatingDoubleTreeMap}"
    )
    L.append(" * entries, following the 6328 design philosophy:")
    L.append(" *")
    L.append(
        ' * <blockquote>"We can retune the setpoints for part of the range at competition without'
    )
    L.append(' * risking unwanted changes to other parts of the range."</blockquote>')
    L.append(" *")
    L.append(" * <h2>How to Tune at Competition</h2>")
    L.append(" * <ol>")
    L.append(
        " *   <li>Use SmartDashboard to monitor {@code ShotCalculator/DistanceMeters}</li>"
    )
    L.append(
        " *   <li>If over/under-shooting at a specific distance, modify the corresponding"
    )
    L.append(
        " *       {@code put()} entry in this file (or use the runtime offset)</li>"
    )
    L.append(
        " *   <li>Call {@link #setTuningOffsets} from SmartDashboard to apply a global offset</li>"
    )
    L.append(" * </ol>")
    L.append(" *")
    L.append(" * <h2>Hardware Configuration</h2>")
    L.append(" * <ul>")
    L.append(
        f' *   <li>Shooter: 4" Orange (40A) Stealth Wheel + Polycarbonate Backplate</li>'
    )
    L.append(
        f" *   <li>Slip Factor: {SLIP_FACTOR} | Effective Radius: {EFF_RADIUS*1000:.2f} mm</li>"
    )
    L.append(
        f" *   <li>Max RPM: {MAX_RPM:.0f} | Target Height: {TARGET_HEIGHT_M:.4f} m</li>"
    )
    L.append(" * </ul>")
    L.append(f" *")
    L.append(
        f" * <h2>Valid Range: [{d_min:.2f}, {d_max:.2f}] meters ({len(table_entries)} points)</h2>"
    )
    L.append(" *")
    L.append(" * @author Auto-generated by generate_shot_polynomials.py")
    L.append(" */")
    L.append("public final class ShotTables {")
    L.append("")
    L.append("  private ShotTables() {}")
    L.append("")
    L.append(f"  /** Minimum valid distance [m]. */")
    L.append(f"  public static final double MIN_DISTANCE_M = {d_min};")
    L.append("")
    L.append(f"  /** Maximum valid distance [m]. */")
    L.append(f"  public static final double MAX_DISTANCE_M = {d_max};")
    L.append("")

    # --- Maps ---
    L.append("  // ── Lookup Tables (distance [m] -> setpoint) ──")
    L.append(
        "  // Interpolation is handled automatically by InterpolatingDoubleTreeMap."
    )
    L.append(
        "  // To tune a specific distance at competition: modify the put() value below."
    )
    L.append("")
    L.append("  private static final InterpolatingDoubleTreeMap HOOD_ANGLE_MAP =")
    L.append("      new InterpolatingDoubleTreeMap();")
    L.append("  private static final InterpolatingDoubleTreeMap FLYWHEEL_SPEED_MAP =")
    L.append("      new InterpolatingDoubleTreeMap();")
    L.append("  private static final InterpolatingDoubleTreeMap TIME_OF_FLIGHT_MAP =")
    L.append("      new InterpolatingDoubleTreeMap();")
    L.append("")

    # --- Runtime tuning ---
    L.append(
        "  // ── Runtime Tuning Offsets (adjustable at competition via SmartDashboard) ──"
    )
    L.append("  private static double sHoodOffsetRad = 0.0;")
    L.append("  private static double sFlywheelOffsetRotPerSec = 0.0;")
    L.append("")

    # --- Static initializer ---
    L.append("  static {")
    L.append("    // Hood Angle [rad] vs Distance [m]")
    L.append("    // (distance, hood_angle_rad)  // hood_angle_deg  |  RPM   |  ToF")
    for d, pitch, vel, rps, tof in table_entries:
        rpm = rps * 60
        L.append(
            f"    HOOD_ANGLE_MAP.put({d:5.2f}, {pitch:.6f});  "
            f"// {math.degrees(pitch):5.1f} deg  |  {rpm:6.0f} RPM  |  {tof:.3f}s"
        )
    L.append("")
    L.append("    // Flywheel Speed [rot/s] vs Distance [m]")
    for d, pitch, vel, rps, tof in table_entries:
        L.append(
            f"    FLYWHEEL_SPEED_MAP.put({d:5.2f}, {rps:.4f});  // {rps * 60:.0f} RPM"
        )
    L.append("")
    L.append("    // Time of Flight [s] vs Distance [m]")
    for d, pitch, vel, rps, tof in table_entries:
        L.append(f"    TIME_OF_FLIGHT_MAP.put({d:5.2f}, {tof:.6f});")
    L.append("  }")
    L.append("")

    # --- Public API ---
    L.append("  // ══════════════════════════════════════════════════════════════════")
    L.append("  // Public API")
    L.append("  // ══════════════════════════════════════════════════════════════════")
    L.append("")
    L.append("  /**")
    L.append("   * Hood angle [rad] for the given distance, including tuning offset.")
    L.append("   *")
    L.append("   * @param distanceM Distance to target [m].")
    L.append("   * @return Hood angle [rad].")
    L.append("   */")
    L.append("  public static double hoodAngleRad(double distanceM) {")
    L.append("    return HOOD_ANGLE_MAP.get(clamp(distanceM)) + sHoodOffsetRad;")
    L.append("  }")
    L.append("")
    L.append("  /**")
    L.append(
        "   * Flywheel speed [rot/s] for the given distance, including tuning offset."
    )
    L.append("   *")
    L.append("   * @param distanceM Distance to target [m].")
    L.append("   * @return Flywheel speed [rot/s].")
    L.append("   */")
    L.append("  public static double flywheelSpeedRotPerSec(double distanceM) {")
    L.append(
        "    return FLYWHEEL_SPEED_MAP.get(clamp(distanceM)) + sFlywheelOffsetRotPerSec;"
    )
    L.append("  }")
    L.append("")
    L.append("  /**")
    L.append("   * Time of flight [s] for the given distance.")
    L.append("   *")
    L.append("   * @param distanceM Distance to target [m].")
    L.append("   * @return Estimated time of flight [s].")
    L.append("   */")
    L.append("  public static double timeOfFlightS(double distanceM) {")
    L.append("    return TIME_OF_FLIGHT_MAP.get(clamp(distanceM));")
    L.append("  }")
    L.append("")

    # --- Tuning API ---
    L.append("  // ══════════════════════════════════════════════════════════════════")
    L.append("  // Competition Tuning")
    L.append("  // ══════════════════════════════════════════════════════════════════")
    L.append("")
    L.append("  /**")
    L.append("   * Sets global tuning offsets applied to all lookups.")
    L.append("   *")
    L.append(
        "   * <p>Use this at competition to compensate for wheel wear, air density,"
    )
    L.append("   * or other real-world deviations. Positive hood offset = aim higher.")
    L.append("   *")
    L.append("   * @param hoodOffsetRad       Hood angle offset [rad].")
    L.append("   * @param flywheelOffsetRotPS  Flywheel speed offset [rot/s].")
    L.append("   */")
    L.append(
        "  public static void setTuningOffsets(double hoodOffsetRad, double flywheelOffsetRotPS) {"
    )
    L.append("    sHoodOffsetRad = hoodOffsetRad;")
    L.append("    sFlywheelOffsetRotPerSec = flywheelOffsetRotPS;")
    L.append("  }")
    L.append("")
    L.append("  /** Reads and applies tuning offsets from SmartDashboard. */")
    L.append("  public static void updateTuningFromDashboard() {")
    L.append(
        '    sHoodOffsetRad = SmartDashboard.getNumber("ShotTuning/HoodOffsetDeg", 0.0)'
    )
    L.append("        * Math.PI / 180.0;")
    L.append(
        '    sFlywheelOffsetRotPerSec = SmartDashboard.getNumber("ShotTuning/RPMOffset", 0.0)'
    )
    L.append("        / 60.0;")
    L.append("  }")
    L.append("")
    L.append("  /** Publishes current tuning state to SmartDashboard. */")
    L.append("  public static void publishTuningToDashboard() {")
    L.append('    SmartDashboard.putNumber("ShotTuning/HoodOffsetDeg",')
    L.append("        Math.toDegrees(sHoodOffsetRad));")
    L.append('    SmartDashboard.putNumber("ShotTuning/RPMOffset",')
    L.append("        sFlywheelOffsetRotPerSec * 60.0);")
    L.append("  }")
    L.append("")

    # --- Utility ---
    L.append("  // ── Utility ──")
    L.append("")
    L.append("  private static double clamp(double distanceM) {")
    L.append(
        "    return Math.max(MIN_DISTANCE_M, Math.min(MAX_DISTANCE_M, distanceM));"
    )
    L.append("  }")
    L.append("}")
    L.append("")

    java_code = "\n".join(L)
    java_path.parent.mkdir(parents=True, exist_ok=True)
    with open(java_path, "w", encoding="utf-8") as f:
        f.write(java_code)

    print(f"    [OK] Generated {java_path}")
    print(f"         {len(table_entries)} entries (adaptive spacing)")
    print(f"         Range: [{d_min:.2f}, {d_max:.2f}] m")

    print("\n--- NEW RAW DATA FOR generate_spline_coefficients.py ---")
    dists_str = "    " + ",\n        ".join(f"{t[0]:.2f}" for t in table_entries)
    hood_str = "    " + ",\n        ".join(f"{t[1]:.6f}" for t in table_entries)
    fw_str = "    " + ",\n        ".join(f"{t[3]*60.0:.4f}" for t in table_entries)
    tof_str = "    " + ",\n        ".join(f"{t[4]:.6f}" for t in table_entries)

    print(f"DISTANCES = np.array([\n{dists_str}\n])\n")
    print(f"HOOD_ANGLES = np.array([\n{hood_str}\n])\n")
    print(f"FLYWHEEL_SPEEDS = np.array([\n{fw_str}\n])\n")
    print(f"TIME_OF_FLIGHT = np.array([\n{tof_str}\n])\n")
    print("------------------------------------------------------\n")

    return table_entries


# ============================================================================
# Validation Plots
# ============================================================================
def generate_plots(static_data, table_entries, out_dir):
    out_dir.mkdir(parents=True, exist_ok=True)

    sd = np.array(static_data)
    d_sim = sd[:, 0]
    pitch_sim = sd[:, 1]
    vel_sim = sd[:, 2]
    rpm_sim = sd[:, 3]
    tof_sim = sd[:, 4]
    rps_sim = rpm_sim / 60.0

    te = np.array(table_entries)
    d_tab = te[:, 0]
    pitch_tab = te[:, 1]
    rps_tab = te[:, 3]
    tof_tab = te[:, 4]

    # Interpolated values at simulation distances (mimicking InterpolatingDoubleTreeMap)
    pitch_interp = np.interp(d_sim, d_tab, pitch_tab)
    rps_interp = np.interp(d_sim, d_tab, rps_tab)
    tof_interp = np.interp(d_sim, d_tab, tof_tab)

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle(
        "Lookup Table Validation: Interpolated Map vs Dense Simulation\n"
        f"Table: {len(table_entries)} entries | Simulation: {len(static_data)} points",
        fontsize=13,
        fontweight="bold",
    )

    # Hood Angle
    ax = axes[0, 0]
    ax.plot(
        d_sim,
        np.degrees(pitch_sim),
        color="#2563EB",
        linewidth=1,
        alpha=0.6,
        label="Simulation",
    )
    ax.plot(
        d_tab,
        np.degrees(pitch_tab),
        "o",
        color="#2563EB",
        markersize=5,
        label="Table entries",
    )
    ax.plot(
        d_sim,
        np.degrees(pitch_interp),
        "--",
        color="#DC2626",
        linewidth=1.5,
        label="Interpolated",
    )
    ax_err = ax.twinx()
    err = np.degrees(pitch_sim - pitch_interp)
    ax_err.fill_between(d_sim, 0, np.abs(err), alpha=0.1, color="red")
    ax_err.set_ylabel("Error (deg)", color="red", fontsize=8)
    ax_err.set_ylim(0, max(np.max(np.abs(err)) * 3, 0.1))
    ax.set_xlabel("Distance (m)")
    ax.set_ylabel("Hood Angle (deg)")
    ax.set_title(f"Hood Angle (max err = {np.max(np.abs(err)):.3f} deg)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # Flywheel RPM
    ax = axes[0, 1]
    ax.plot(d_sim, rpm_sim, color="#DC2626", linewidth=1, alpha=0.6, label="Simulation")
    ax.plot(
        d_tab, te[:, 3] * 60, "o", color="#DC2626", markersize=5, label="Table entries"
    )
    ax.plot(
        d_sim,
        rps_interp * 60,
        "--",
        color="#2563EB",
        linewidth=1.5,
        label="Interpolated",
    )
    ax_err = ax.twinx()
    err_rpm = rpm_sim - rps_interp * 60
    ax_err.fill_between(d_sim, 0, np.abs(err_rpm), alpha=0.1, color="red")
    ax_err.set_ylabel("Error (RPM)", color="red", fontsize=8)
    ax_err.set_ylim(0, max(np.max(np.abs(err_rpm)) * 3, 1))
    ax.set_xlabel("Distance (m)")
    ax.set_ylabel("Flywheel Speed (RPM)")
    ax.set_title(f"Flywheel Speed (max err = {np.max(np.abs(err_rpm)):.1f} RPM)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # Time of Flight
    ax = axes[1, 0]
    ax.plot(d_sim, tof_sim, color="#7C3AED", linewidth=1, alpha=0.6, label="Simulation")
    ax.plot(d_tab, tof_tab, "o", color="#7C3AED", markersize=5, label="Table entries")
    ax.plot(
        d_sim, tof_interp, "--", color="#DC2626", linewidth=1.5, label="Interpolated"
    )
    ax_err = ax.twinx()
    err_tof = tof_sim - tof_interp
    ax_err.fill_between(d_sim, 0, np.abs(err_tof), alpha=0.1, color="red")
    ax_err.set_ylabel("Error (s)", color="red", fontsize=8)
    ax_err.set_ylim(0, max(np.max(np.abs(err_tof)) * 3, 0.01))
    ax.set_xlabel("Distance (m)")
    ax.set_ylabel("Time of Flight (s)")
    ax.set_title(f"Time of Flight (max err = {np.max(np.abs(err_tof)):.4f} s)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # Exit Velocity
    ax = axes[1, 1]
    ax.plot(d_sim, vel_sim, color="#16A34A", linewidth=1, alpha=0.6, label="Simulation")
    ax.set_xlabel("Distance (m)")
    ax.set_ylabel("Exit Velocity (m/s)")
    ax.set_title("Exit Velocity (reference only)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    p = out_dir / "table_fit_validation.png"
    fig.savefig(p, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"    [OK] Saved {p}")


def save_csv(static_data, out_dir):
    out_dir.mkdir(parents=True, exist_ok=True)
    with open(out_dir / "table_static_sweep.csv", "w") as f:
        f.write("distance_m,pitch_rad,pitch_deg,velocity_mps,rpm,rps,tof_s,apex_m\n")
        for row in static_data:
            d, pitch, vel, rpm, tof, apex = row
            f.write(
                f"{d:.4f},{pitch:.8f},{math.degrees(pitch):.4f},"
                f"{vel:.6f},{rpm:.2f},{rpm/60:.4f},{tof:.6f},{apex:.4f}\n"
            )
    print(f"    [OK] Saved {out_dir / 'table_static_sweep.csv'}")


# ============================================================================
# Main
# ============================================================================
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--step",
        type=float,
        default=0.50,
        help="(legacy) uniform step size hint; adaptive spacing is used",
    )
    parser.add_argument(
        "--min", type=float, default=0.5, help="Min sweep distance (default: 0.5)"
    )
    parser.add_argument(
        "--max", type=float, default=6.5, help="Max sweep distance (default: 6.5)"
    )
    args = parser.parse_args()

    print("=" * 65)
    print("  Offline Simulation -> Interpolating Lookup Tables")
    print("  Orbit 1690 Physics  +  6328 Tuning Philosophy")
    print("  Team 9427 | 2026")
    print(f"  Sweep: [{args.min}, {args.max}]m  |  Spacing: adaptive")
    print("=" * 65)

    t0 = time.time()

    # Dense simulation sweep (fine step for accuracy)
    print(f"\n[1/4] Dense Simulation Sweep ({args.min}m - {args.max}m, step 0.05m)")
    static_data = sweep_static(dist_min=args.min, dist_max=args.max, dist_step=0.05)
    print(f"    {len(static_data)} valid solutions")

    if len(static_data) < 10:
        print("ERROR: Not enough valid solutions. Check physics parameters.")
        sys.exit(1)

    # Generate Java lookup tables
    print(f"\n[2/4] Generating Java Lookup Tables (adaptive spacing)")
    java_path = JAVA_OUT_DIR / "ShotTables.java"
    table_entries = generate_java(static_data, args.step, java_path)

    # Validation plots
    print("\n[3/4] Generating Validation Plots")
    generate_plots(static_data, table_entries, OUT_DIR)

    # Save CSV
    print("\n[4/4] Saving CSV Data")
    save_csv(static_data, OUT_DIR)

    elapsed = time.time() - t0
    print(f"\n{'=' * 65}")
    print(f"  Complete in {elapsed:.1f}s")
    print(f"  Java:  {java_path}")
    print(f"  Plots: {OUT_DIR}/table_fit_validation.png")
    print(f"{'=' * 65}")


if __name__ == "__main__":
    main()
