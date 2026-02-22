#!/usr/bin/env python3
"""
Shot Table Generator - RK4 Trajectory with 3.5m Apex Constraint.
Calibrated with empirical overshoot data (2026-02-20).

Physics: Gravity + Quadratic Air Drag (no Magnus/backspin).
RK4 integration at dt=0.5ms.

Calibration approach:
  - User reported ~1.20m overshoot at all distances
  - At 4.9m ball enters hub at back edge (hub half-width ~0.53m)
  - This means effective overshoot at 4.9m is ~0.53m
  - Overshoot is caused by real exit velocity > model prediction
  - We calibrate by finding the slip factor that eliminates overshoot

Author: Team 9427, 2026-02-20
"""

import math

GRAVITY = 9.80665
RHO_AIR = 1.225

BALL_MASS_KG = 0.215
BALL_DIAMETER_M = 0.150
BALL_RADIUS_M = BALL_DIAMETER_M / 2.0
BALL_AREA_M2 = math.pi * BALL_RADIUS_M**2
CD = 0.485

DRAG_K = 0.5 * RHO_AIR * CD * BALL_AREA_M2 / BALL_MASS_KG

LAUNCH_HEIGHT_M = 0.50
TARGET_HEIGHT_M = 72.0 * 0.0254  # 1.8288 m

HOOD_MIN_DEG = 60.0
HOOD_MAX_DEG = 83.0
HOOD_MIN_RAD = math.radians(HOOD_MIN_DEG)
HOOD_MAX_RAD = math.radians(HOOD_MAX_DEG)

MAX_APEX_M = 3.5

WHEEL_RADIUS_M = 0.0508

# Calibration: find correct slip factor
# Old table used 0.60 and worked reasonably well
# User said 0.435 but all shots overshoot by ~1.20m
# We'll sweep to find the right value
SLIP_FACTOR = None  # Will be calibrated

DIST_MIN_M = 0.50
DIST_MAX_M = 6.50
DIST_STEP_M = 0.25

DT = 0.0005
MAX_STEPS = int(8.0 / DT)


def simulate(v0, theta_rad):
    vx = v0 * math.cos(theta_rad)
    vz = v0 * math.sin(theta_rad)
    x = 0.0
    z = LAUNCH_HEIGHT_M
    t = 0.0
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
    v_low = 2.0
    v_high = 15.0

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

    v_mid = (v_low + v_high) / 2.0
    res = simulate(v_mid, theta_rad)
    if res["hit"]:
        return (v_mid, res["apex_z"], res["cross_t"])
    return None


def find_optimal_angle(target_dist):
    res_max = find_v0_for_distance(HOOD_MAX_RAD, target_dist)
    if res_max is not None and res_max[1] <= MAX_APEX_M:
        v0, apex_z, tof = res_max
        return (HOOD_MAX_RAD, v0, apex_z, tof)

    res_min = find_v0_for_distance(HOOD_MIN_RAD, target_dist)
    if res_min is None:
        return None
    if res_min[1] > MAX_APEX_M:
        v0, apex_z, tof = res_min
        return (HOOD_MIN_RAD, v0, apex_z, tof)

    theta_low = HOOD_MIN_RAD
    theta_high = HOOD_MAX_RAD
    best = None

    for _ in range(60):
        theta_mid = (theta_low + theta_high) / 2.0
        res = find_v0_for_distance(theta_mid, target_dist)
        if res is None:
            theta_high = theta_mid
            continue
        v0, apex_z, tof = res
        if apex_z > MAX_APEX_M:
            theta_high = theta_mid
        else:
            best = (theta_mid, v0, apex_z, tof)
            theta_low = theta_mid
        if abs(theta_high - theta_low) < math.radians(0.005):
            break

    return best


def calibrate_slip():
    """
    Find the slip factor that makes the model match reality.

    At 4.9m the ball enters the hub at the back edge.
    Hub hexagon half-width ~0.533m. So the ball overshoots by ~0.40-0.53m.
    At other distances, overshoot is ~1.20m.

    Strategy: with the OLD slip=0.435, at 4.9m we assign 64.45 rps.
    The real ball at 64.45 rps travels to ~5.3-5.43m instead of 4.9m.
    So real v_exit = 64.45 * 2pi * R_wheel * slip_real, and this velocity
    at our model's angle (63.5 deg) reaches 5.3m.

    We find v0 that reaches 5.3m at 63.5 deg, then:
    slip_real = v0 / (64.45 * 2pi * R_wheel)
    """
    print("=" * 72)
    print("  SLIP FACTOR CALIBRATION")
    print("=" * 72)

    # Our model at 4.9m gave ~64.45 rps with slip=0.435
    # The ball at 4.9m reaches hub back edge, overshoot ~0.5m
    # So real landing distance ~5.4m

    # At other distances, overshoot is ~1.2m uniformly
    # Let's calibrate: find slip where our model's rps at
    # distance d lands at d+1.2m in reality

    # Approach: the model correctly computes physics.
    # The error is that slip_model != slip_real.
    # If we set the model's target to d+overshoot instead of d,
    # we get the correct v0. Then rps = v0 / (2pi * Reff_model).
    # But this doesn't change rps because both Reff and target change.
    #
    # Better approach: use the fact that all shots overshoot by ~1.2m.
    # This means: when model says "aim at d", ball actually hits at d+1.2.
    # So for ball to hit at d, model should aim at d-1.2.
    # But that changes the angle too, which changes apex.
    #
    # Simplest correct approach: find slip_real such that:
    #   rps_model(d) * 2pi * R * slip_real gives v0 that reaches d+1.2m
    #   at the angle computed for d.

    # Let's compute: at d=3.0m, our model gives angle=73.4 deg, v0=8.33 m/s
    # The ball actually reaches ~4.2m (3.0+1.2) at this angle and v0.
    # We need to find: what v0_corrected reaches 3.0m at 73.4 deg?
    # Then slip_corrected = v0_corrected / (rps_3m * 2pi * R_wheel)

    test_distances = [3.0, 4.0, 5.0]
    slip_estimates = []

    old_slip = 0.435
    old_eff_r = WHEEL_RADIUS_M * old_slip

    for d in test_distances:
        sol = find_optimal_angle(d)
        if sol is None:
            continue
        theta_rad, v0_model, apex_z, tof = sol
        rps_model = v0_model / (2.0 * math.pi * old_eff_r)
        theta_deg = math.degrees(theta_rad)

        # Empirical Non-Linear Correction:
        # User reported 3.79m goes in, 4.1m barely goes in, >4.1m falls short.
        # This implies our constant 1.2m overshoot assumption is too aggressive at long range,
        # causing the solver to prescribe too low 'rps'.
        # We need the model to think overshoot diminishes at distance.
        # Let's say overshoot is 1.2m at 3m, but drops to 0.4m at 6m.
        # overshoot(d) = max(0.4, 1.2 - 0.26 * (d - 3.0))
        overshoot = max(0.4, 1.2 - 0.3 * (d - 3.0)) if d > 3.0 else 1.2
        target_real = d + overshoot
        sol_real = find_v0_for_distance(theta_rad, target_real)
        if sol_real is None:
            print(
                f"  d={d}m: cannot find v0 for {target_real:.2f}m at {theta_deg:.1f} deg"
            )
            continue

        v0_real = sol_real[0]
        # v0_real = rps_model * 2pi * R_wheel * slip_real
        slip_real = v0_real / (rps_model * 2.0 * math.pi * WHEEL_RADIUS_M)
        print(
            f"  d={d:.1f}m: angle={theta_deg:.1f} deg, v0_model={v0_model:.3f}, "
            f"rps={rps_model:.1f}, v0_real={v0_real:.3f}, slip_real={slip_real:.4f}"
        )
        slip_estimates.append(slip_real)

    if slip_estimates:
        avg_slip = sum(slip_estimates) / len(slip_estimates)
        print(f"\n  Average calibrated slip factor: {avg_slip:.4f}")
        print(f"  (was 0.435, original code had 0.60)")
        return avg_slip
    else:
        print("  CALIBRATION FAILED, using 0.60 as fallback")
        return 0.60


def generate_table(slip):
    eff_r = WHEEL_RADIUS_M * slip

    distances = []
    d = DIST_MIN_M
    while d <= DIST_MAX_M + 0.001:
        distances.append(round(d, 2))
        d += DIST_STEP_M

    results = []
    hdr = f"{'Dist':>6s}  {'Angle':>7s}  {'Angle_rad':>10s}  {'v0 m/s':>8s}  {'rot/s':>7s}  {'Apex m':>7s}  {'ToF s':>7s}"
    print(hdr)
    print("-" * 72)

    for dist in distances:
        sol = find_optimal_angle(dist)
        if sol is None:
            print(f"{dist:6.2f}  FAIL")
            continue
        theta_rad, v0, apex_z, tof = sol
        rps = v0 / (2.0 * math.pi * eff_r)
        theta_deg = math.degrees(theta_rad)
        print(
            f"{dist:6.2f}  {theta_deg:7.2f}  {theta_rad:10.6f}  {v0:8.4f}  {rps:7.2f}  {apex_z:7.4f}  {tof:7.4f}"
        )
        results.append((dist, theta_rad, rps, tof, apex_z, v0, theta_deg))

    return results


def print_java(results, slip):
    eff_r = WHEEL_RADIUS_M * slip
    print()
    print("=" * 72)
    print(f"  JAVA CODE (slip={slip:.4f}, Reff={eff_r:.5f}m)")
    print("=" * 72)
    print()

    print(f"    // Hood Angle [rad] vs Distance [m]")
    print(f"    // Generated 2026-02-20: apex <= 3.5m, Cd=0.485, slip={slip:.4f}")
    for d, tr, rps, tof, az, v0, td in results:
        print(
            f"    HOOD_ANGLE_MAP.put({d:.2f}, {tr:.6f}); // {td:.1f} deg, apex={az:.2f}m"
        )
    print()

    print(f"    // Flywheel Speed [rot/s] vs Distance [m]")
    print(f"    // v_exit = rot/s * 2pi * {eff_r:.5f}m")
    for d, tr, rps, tof, az, v0, td in results:
        print(f"    FLYWHEEL_SPEED_MAP.put({d:.2f}, {rps:.2f});")
    print()

    print(f"    // Time of Flight [s] vs Distance [m]")
    for d, tr, rps, tof, az, v0, td in results:
        print(f"    TIME_OF_FLIGHT_MAP.put({d:.2f}, {tof:.6f});")

    print()
    print("// ---- Test Arrays ----")
    dists_str = ", ".join(f"{r[0]:.2f}" for r in results)
    angles_str = ", ".join(f"{r[1]:.6f}" for r in results)
    speeds_str = ", ".join(f"{r[2]:.2f}" for r in results)
    tofs_str = ", ".join(f"{r[3]:.6f}" for r in results)
    print(f"  double[] DISTANCES = {{{dists_str}}};")
    print(f"  double[] HOOD_ANGLES = {{{angles_str}}};")
    print(f"  double[] FLYWHEEL_SPEEDS = {{{speeds_str}}};")
    print(f"  double[] TIME_OF_FLIGHT = {{{tofs_str}}};")


def main():
    print("=" * 72)
    print("  SHOT TABLE GENERATOR v2 -- Empirically Calibrated")
    print("  Team 9427 - 2026-02-20")
    print("=" * 72)
    print()

    # Step 1: Calibrate slip factor
    calibrated_slip = calibrate_slip()

    print()
    print("=" * 72)
    print(f"  GENERATING TABLE WITH slip={calibrated_slip:.4f}")
    print("=" * 72)
    print()

    # Step 2: Generate table with calibrated slip
    results = generate_table(calibrated_slip)

    if results:
        print_java(results, calibrated_slip)

    # Verification
    print()
    print("=" * 72)
    print("  VERIFICATION")
    print("=" * 72)
    for d, tr, rps, tof, az, v0, td in results:
        issues = []
        if az > MAX_APEX_M + 0.01:
            issues.append(f"apex={az:.3f}")
        if td < HOOD_MIN_DEG - 0.1 or td > HOOD_MAX_DEG + 0.1:
            issues.append(f"angle OOB")
        status = ", ".join(issues) if issues else "OK"
        print(
            f"  {d:5.2f}m: angle={td:5.1f} deg  rps={rps:6.2f}  apex={az:5.3f}m  {status}"
        )


if __name__ == "__main__":
    main()
