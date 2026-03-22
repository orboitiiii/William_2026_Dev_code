#!/usr/bin/env python3
"""
Shot Table Generator V3 - High Arc & Recalibrated Range.
Team 9427 - 2026-03-01

User Feedback on V2 (0.4993 slip):
- 2.1m ~ 2.6m: Accurate.
- < 2.0m: Overshoots.
- > 3.0m: Falls short.
- Desire: "Higher arc" (loosen apex constraint from 3.5m to e.g. 4.0m)

Physics Profile:
- We will fix SLIP_FACTOR = 0.4993 (since it was accurate for 2.1-2.6m).
- The overshoot/undershoot issue is purely an aiming target projection problem.
- We will adjust the calibration curve `overshoot(d)`:
  - At d=2.5m, overshoot=0.0 (model is perfect).
  - At d<2.0m, model overshoots, so we need a POSITIVE overshoot target to reduce energy.
  - At d>3.0m, model falls short, so we need a NEGATIVE overshoot target to add energy.
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

# Relaxed Apex Constraint for "Higher Arc"
MAX_APEX_M = 4.0

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
    res_max = find_v0_for_distance(HOOD_MAX_RAD, target_dist)
    if res_max is not None and res_max[1] <= MAX_APEX_M:
        return (HOOD_MAX_RAD, res_max[0], res_max[1], res_max[2])

    res_min = find_v0_for_distance(HOOD_MIN_RAD, target_dist)
    if res_min is None:
        return None
    if res_min[1] > MAX_APEX_M:
        return (HOOD_MIN_RAD, res_min[0], res_min[1], res_min[2])

    theta_low, theta_high = HOOD_MIN_RAD, HOOD_MAX_RAD
    best = None
    for _ in range(60):
        theta_mid = (theta_low + theta_high) / 2.0
        res = find_v0_for_distance(theta_mid, target_dist)
        if res is None:
            theta_high = theta_mid
            continue
        if res[1] > MAX_APEX_M:
            theta_high = theta_mid
        else:
            best = (theta_mid, res[0], res[1], res[2])
            theta_low = theta_mid
        if abs(theta_high - theta_low) < math.radians(0.005):
            break

    return best


def get_empiric_overshoot(d):
    # Feedback:
    # 2.1 - 2.6m: accurate -> overshoot = 0.0
    # < 2.0m: overshoots -> need positive offset to pull power back
    # > 3.0m: falls short -> need negative offset to push power further

    # We'll use a smooth curve.
    if 2.1 <= d <= 2.6:
        return 0.0
    elif d < 2.1:
        # e.g. at 1.0m, if we overshoot by 0.5m, we want the model to target 1.5m to drop RPS.
        return 0.5 * (2.1 - d)
    else:  # d > 2.6
        # e.g. at 5.0m, if we fall short by 0.5m, we want the model to target 4.5m or similar?
        # WAIT, if the ball FALLS SHORT, it means real life ball doesn't travel as far as model predicts.
        # This implies: Model says RPS=X reaches 5.0m. But Reality at RPS=X only reaches 4.0m.
        # To make Reality reach 5.0m, Model must give the RPS that IT thinks reaches 6.0m.
        # So we need to ADD to the model's target distance. overshoot > 0 means model target is further.
        # Ah, let's rethink:
        # V2 generated 63.93 RPS for 6.5m. The user says it "falls short".
        # Why? Because V2 applied "overshoot = max(0.4, 1.2 - 0.3 * (d - 3.0))".
        # At 6.5m, V2 target = 6.5 + 0.4 = 6.9m. And it solved for 63.93 RPS.
        # Wait, if V2 fell short, it means 63.93 RPS wasn't enough energy.
        # We need MORE energy. We should give it a HIGHER target distance.
        # Or simply, fix SLIP_FACTOR and trust physics targeting.

        # Actually, let's just make target_real = d + offset
        # Close range (<2.0m) overshoots -> lower energy -> model must aim CLOSER -> negative offset?!
        # Wait...
        # If Real Ball overshoots: Real travels 2.0m. Model targeted 1.0m to give RPS=40. Real at RPS=40 travels 2.0m.
        # So if we want Real to hit 1.0m, we must give it LESS RPS. Model must aim at e.g. 0.5m.
        # But `target_real = d + overshoot`. If target_real = 0.5m, then overshoot = -0.5.

        # Let's map target_real = d + offset
        # IF d < 2.1 (Overshoots): offset is NEGATIVE. Model will target d - X, giving lower RPS.
        # IF 2.1 <= d <= 2.6: offset is 0.0.
        # IF d > 2.6 (Falls short): offset is POSITIVE. Model will target d + X, giving higher RPS.

        # Specifically, at 6.0m it falls short, we must bump the target up.
        return 0.4 * (d - 2.6)


def generate_table():
    distances = [
        round(DIST_MIN_M + i * DIST_STEP_M, 2)
        for i in range(int((DIST_MAX_M - DIST_MIN_M) / DIST_STEP_M) + 1)
    ]
    results = []

    print(f"Generating V3 Table with SLIP='{SLIP_FACTOR}', MAX_APEX={MAX_APEX_M}m")
    print(
        f"{'Dist':>6s}  {'Target':>6s}  {'Angle':>7s}  {'Angle_rad':>10s}  {'v0 m/s':>8s}  {'rot/s':>7s}  {'Apex m':>7s}  {'ToF s':>7s}"
    )
    print("-" * 72)

    for dist in distances:
        if dist < 2.1:
            offset = -1.2 * (2.1 - dist)  # Agressive ramp down to smooth out RPS < 2.1m
        elif 2.1 <= dist <= 2.6:
            offset = 0.0
        else:
            offset = 0.35 * (dist - 2.6)  # Ramp up to +1.36m target at 6.5m

        target_dist = dist + offset
        target_dist = max(0.5, target_dist)  # Prevent negative/zero targets

        sol = find_optimal_angle(target_dist)
        if sol is None:
            print(f"{dist:6.2f}  FAIL")
            continue
        theta_rad, v0, apex_z, tof = sol
        rps = v0 / (2.0 * math.pi * EFF_R)
        theta_deg = math.degrees(theta_rad)

        # Re-verify the apex based on actual launch physics (angle + v0)
        # We don't need to re-simulate because the angle/v0 gives that apex.

        print(
            f"{dist:6.2f}  {target_dist:6.2f}  {theta_deg:7.2f}  {theta_rad:10.6f}  {v0:8.4f}  {rps:7.2f}  {apex_z:7.4f}  {tof:7.4f}"
        )
        results.append((dist, theta_rad, rps, tof, apex_z, theta_deg))

    # Print Java code
    print("\n--- JAVA CODE ---")
    for d, tr, rps, tof, az, td in results:
        print(
            f"    HOOD_ANGLE_MAP.put({d:.2f}, {tr:.6f}); // {td:.1f} deg, apex={az:.2f}m"
        )

    print()
    for d, tr, rps, tof, az, td in results:
        print(f"    FLYWHEEL_SPEED_MAP.put({d:.2f}, {rps:.2f});")

    print()
    for d, tr, rps, tof, az, td in results:
        print(f"    TIME_OF_FLIGHT_MAP.put({d:.2f}, {tof:.6f});")


if __name__ == "__main__":
    generate_table()
