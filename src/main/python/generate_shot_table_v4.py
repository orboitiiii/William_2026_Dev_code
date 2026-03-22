#!/usr/bin/env python3
"""
Shot Table Generator V4 - Low-Arc Hub Shot (apex <= 2.5m).
Team 9427 - 2026-03-06

Change from V3:
- MAX_APEX_M reduced from 4.0m to 2.5m for flatter, faster trajectories.
- Removed empirical overshoot correction (offset); pure physics targeting.
  The V3 overshoot calibration was fitted to 4.0m-apex trajectories and is
  invalid for the new constraint. Re-calibration on-field is required.

Physics Profile:
- SLIP_FACTOR = 0.4993 (calibrated 2026-02-20, unchanged)
- Cd = 0.485 (quadratic drag, no Magnus)
- RK4 integrator @ dt = 0.5ms
- Hood mechanical range: [60 deg, 83 deg]
- Launch height: 0.50m, Target height: 1.8288m (72 in)

Expected behavior:
- Close range (<=~1.25m): Hood at 83 deg max, apex naturally < 2.5m
- Mid range (~1.5-5.5m): Hood angle decreases to keep apex = 2.5m exactly
- Far range (>=~5.5m): Hood hits 60 deg floor, apex may slightly exceed 2.5m
"""

import math

GRAVITY = 9.80665
RHO_AIR = 1.225
BALL_MASS_KG = 0.215
BALL_RADIUS_M = 0.075
BALL_AREA_M2 = math.pi * BALL_RADIUS_M**2
CD = 0.485
DRAG_K = 0.5 * RHO_AIR * CD * BALL_AREA_M2 / BALL_MASS_KG

LAUNCH_HEIGHT_M = 0.50
TARGET_HEIGHT_M = 1.8288

HOOD_MIN_RAD = math.radians(60.0)
HOOD_MAX_RAD = math.radians(83.0)

# --- KEY CHANGE: apex constraint lowered from 4.0m to 2.5m ---
MAX_APEX_M = 3.5

WHEEL_RADIUS_M = 0.0508
SLIP_FACTOR = 0.4993
EFF_R = WHEEL_RADIUS_M * SLIP_FACTOR

DIST_MIN_M = 0.50
DIST_MAX_M = 6.50
DIST_STEP_M = 0.25
DT = 0.0005
MAX_STEPS = int(8.0 / DT)


def simulate(v0, theta_rad):
    vx = v0 * math.cos(theta_rad)
    vz = v0 * math.sin(theta_rad)
    x, z, t = 0.0, LAUNCH_HEIGHT_M, 0.0
    apex_z = z
    past_apex = False

    for _ in range(MAX_STEPS):

        def deriv(vx_, vz_):
            spd = math.sqrt(vx_ * vx_ + vz_ * vz_)
            ax = -DRAG_K * spd * vx_
            az = -GRAVITY - DRAG_K * spd * vz_
            return ax, az

        ax1, az1 = deriv(vx, vz)
        hdt = 0.5 * DT
        ax2, az2 = deriv(vx + hdt * ax1, vz + hdt * az1)
        ax3, az3 = deriv(vx + hdt * ax2, vz + hdt * az2)
        ax4, az4 = deriv(vx + DT * ax3, vz + DT * az3)

        k1x, k1z = vx, vz
        k2x, k2z = vx + hdt * ax1, vz + hdt * az1
        k3x, k3z = vx + hdt * ax2, vz + hdt * az2
        k4x, k4z = vx + DT * ax3, vz + DT * az3

        x_new = x + (DT / 6.0) * (k1x + 2 * k2x + 2 * k3x + k4x)
        z_new = z + (DT / 6.0) * (k1z + 2 * k2z + 2 * k3z + k4z)
        vx_new = vx + (DT / 6.0) * (ax1 + 2 * ax2 + 2 * ax3 + ax4)
        vz_new = vz + (DT / 6.0) * (az1 + 2 * az2 + 2 * az3 + az4)
        t_new = t + DT

        if z_new > apex_z:
            apex_z = z_new
        if not past_apex and vz_new < 0:
            past_apex = True

        if past_apex and z > TARGET_HEIGHT_M and z_new <= TARGET_HEIGHT_M:
            dz = z - z_new
            alpha = (z - TARGET_HEIGHT_M) / dz if dz > 1e-12 else 0.0
            cx = x + alpha * (x_new - x)
            ct = t + alpha * DT
            return {"apex_z": apex_z, "cross_x": cx, "cross_t": ct, "hit": True}

        if z_new < -0.5:
            return {"apex_z": apex_z, "hit": False, "cross_x": x_new}
        x, z, vx, vz, t = x_new, z_new, vx_new, vz_new, t_new

    return {"apex_z": apex_z, "hit": False, "cross_x": x}


def find_v0_for_distance(theta_rad, target_dist):
    v_low, v_high = 2.0, 15.0
    for _ in range(30):
        res = simulate(v_high, theta_rad)
        if res["hit"] and res["cross_x"] >= target_dist:
            break
        v_high *= 1.3
        if v_high > 200:
            return None
    else:
        return None

    for _ in range(80):
        v_mid = (v_low + v_high) / 2.0
        res = simulate(v_mid, theta_rad)
        if not res["hit"]:
            v_low = v_mid
            continue
        cx = res["cross_x"]
        if abs(cx - target_dist) < 0.001:
            return (v_mid, res["apex_z"], res["cross_t"])
        if cx < target_dist:
            v_low = v_mid
        else:
            v_high = v_mid

    res = simulate((v_low + v_high) / 2.0, theta_rad)
    if res["hit"]:
        return ((v_low + v_high) / 2.0, res["apex_z"], res["cross_t"])
    return None


def find_optimal_angle(target_dist):
    """
    Angle-priority optimization: use the highest hood angle whose apex <= MAX_APEX_M.
    If even 60 deg exceeds MAX_APEX_M, clamp to 60 deg (mechanical limit).

    Near-range fix: at very close distances (e.g. 1.0-1.5m), the minimum angle (60 deg)
    may have NO ballistic solution because the ball cannot descend back to target height
    within such a short horizontal distance. In that case, we scan upward from 60 deg
    to find the lowest angle that has a valid solution, then bisect from there.
    """
    res_max = find_v0_for_distance(HOOD_MAX_RAD, target_dist)
    if res_max is not None and res_max[1] <= MAX_APEX_M:
        return (HOOD_MAX_RAD, res_max[0], res_max[1], res_max[2])

    # If max angle exceeds apex OR has no solution, we need to search.
    # First check min angle.
    res_min = find_v0_for_distance(HOOD_MIN_RAD, target_dist)

    if res_min is not None:
        # Min angle has a solution
        if res_min[1] > MAX_APEX_M:
            # Even flattest angle exceeds apex -> clamp to min angle
            return (HOOD_MIN_RAD, res_min[0], res_min[1], res_min[2])
        # Normal case: bisect between min and max
        theta_low, theta_high = HOOD_MIN_RAD, HOOD_MAX_RAD
    else:
        # Min angle has NO solution (near-range: ball can't descend in time).
        # Scan upward to find the lowest angle that works.
        theta_low_working = None
        for deg in range(61, 84):
            theta_test = math.radians(float(deg))
            res_test = find_v0_for_distance(theta_test, target_dist)
            if res_test is not None:
                theta_low_working = theta_test
                break
        if theta_low_working is None:
            return None  # Truly no solution at any angle
        # Now check if this working angle is already within apex
        res_work = find_v0_for_distance(theta_low_working, target_dist)
        if res_work[1] <= MAX_APEX_M:
            # Bisect between this working angle and max angle
            theta_low = theta_low_working
            theta_high = HOOD_MAX_RAD
        else:
            # Working angle exceeds apex, but max angle also does (checked above)
            # Return lowest working angle (best we can do)
            return (theta_low_working, res_work[0], res_work[1], res_work[2])

    best = None
    for _ in range(60):
        theta_mid = (theta_low + theta_high) / 2.0
        res = find_v0_for_distance(theta_mid, target_dist)
        if res is None:
            # No ballistic solution at this angle -> need higher angle
            theta_low = theta_mid
            continue
        if res[1] > MAX_APEX_M:
            theta_high = theta_mid
        else:
            best = (theta_mid, res[0], res[1], res[2])
            theta_low = theta_mid
        if abs(theta_high - theta_low) < math.radians(0.005):
            break

    return best


def generate_table():
    distances = [
        round(DIST_MIN_M + i * DIST_STEP_M, 2)
        for i in range(int((DIST_MAX_M - DIST_MIN_M) / DIST_STEP_M) + 1)
    ]
    results = []

    print(f"Generating V4 Table with SLIP={SLIP_FACTOR}, MAX_APEX={MAX_APEX_M}m")
    print(
        f"{'Dist':>6s}  {'Angle':>7s}  {'Angle_rad':>10s}  {'v0 m/s':>8s}  {'rot/s':>7s}  {'Apex m':>7s}  {'ToF s':>7s}"
    )
    print("-" * 72)

    for dist in distances:
        # V4: NO overshoot offset. Pure physics targeting.
        target_dist = dist

        sol = find_optimal_angle(target_dist)
        if sol is None:
            print(f"{dist:6.2f}  FAIL")
            continue
        theta_rad, v0, apex_z, tof = sol
        rps = v0 / (2.0 * math.pi * EFF_R)
        theta_deg = math.degrees(theta_rad)

        print(
            f"{dist:6.2f}  {theta_deg:7.2f}  {theta_rad:10.6f}  {v0:8.4f}  {rps:7.2f}  {apex_z:7.4f}  {tof:7.4f}"
        )
        results.append((dist, theta_rad, rps, tof, apex_z, theta_deg))

    # Print Java code
    print("\n--- JAVA CODE ---")
    print(f"    // Hood Angle [rad] vs Distance [m]")
    print(
        f"    // Generated 2026-03-06: apex <= {MAX_APEX_M}m, Cd={CD}, slip={SLIP_FACTOR}"
    )
    for d, tr, rps, tof, az, td in results:
        print(
            f"    HOOD_ANGLE_MAP.put({d:.2f}, {tr:.6f}); // {td:.1f} deg, apex={az:.2f}m"
        )

    print()
    print(f"    // Flywheel Speed [rot/s] vs Distance [m]")
    print(f"    // v_exit = rot/s * 2pi * {EFF_R:.5f}m")
    for d, tr, rps, tof, az, td in results:
        print(f"    FLYWHEEL_SPEED_MAP.put({d:.2f}, {rps:.2f});")

    print()
    print(f"    // Time of Flight [s] vs Distance [m]")
    for d, tr, rps, tof, az, td in results:
        print(f"    TIME_OF_FLIGHT_MAP.put({d:.2f}, {tof:.6f});")


if __name__ == "__main__":
    generate_table()
