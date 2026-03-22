#!/usr/bin/env python3
"""
Shot Table Generator V5 - Hybrid: Near Low-Arc + Mid/Far High-Arc + Far Boost.
Team 9427 - 2026-03-06

Three-segment strategy:
  d <= 2.50m : apex <= 3.5m, NO offset.
               Pure physics. Field-validated on 2026-03-01.

  2.50 < d <= 4.50m : apex <= 4.0m, offset = 0.35 * (d - 2.6).
               Same offset as V3 (last field-tested version).
               V3 fell slightly short here but this range is not the priority.

  d > 4.50m : apex <= 4.0m, offset = 0.35 * (d - 2.6) + 0.30 * (d - 4.5).
               V3 baseline offset PLUS additional boost.
               The 0.30 coefficient adds extra energy to fix V3's far-range
               undershoot. At 6.5m, total offset = 1.365 + 0.60 = 1.965m.

Physics Profile (unchanged):
  SLIP_FACTOR = 0.4993, Cd = 0.485, RK4 @ dt=0.5ms
  Hood: [60 deg, 83 deg], Launch: 0.50m, Target: 1.8288m (72 in)
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

APEX_NEAR_M = 3.5  # d <= 2.50m
APEX_FAR_M = 4.0  # d >  2.50m

# V3 baseline offset coefficient (field-tested, accurate at 2.1-2.6m)
V3_OFFSET_COEFF = 0.35
V3_OFFSET_BASE = 2.6

# Additional far-range boost (on top of V3 baseline)
FAR_BOOST_COEFF = 0.30
FAR_BOOST_BASE = 4.5

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
            return -DRAG_K * spd * vx_, -GRAVITY - DRAG_K * spd * vz_

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
            return {
                "apex_z": apex_z,
                "cross_x": x + alpha * (x_new - x),
                "cross_t": t + alpha * DT,
                "hit": True,
            }

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


def find_optimal_angle(target_dist, max_apex_m):
    """Angle-priority: highest angle with apex <= max_apex_m."""
    res_max = find_v0_for_distance(HOOD_MAX_RAD, target_dist)
    if res_max is not None and res_max[1] <= max_apex_m:
        return (HOOD_MAX_RAD, res_max[0], res_max[1], res_max[2])

    res_min = find_v0_for_distance(HOOD_MIN_RAD, target_dist)
    if res_min is not None:
        if res_min[1] > max_apex_m:
            return (HOOD_MIN_RAD, res_min[0], res_min[1], res_min[2])
        theta_low, theta_high = HOOD_MIN_RAD, HOOD_MAX_RAD
    else:
        # Near-range: min angle has no solution, scan upward
        theta_low_working = None
        for deg in range(61, 84):
            res_test = find_v0_for_distance(math.radians(float(deg)), target_dist)
            if res_test is not None:
                theta_low_working = math.radians(float(deg))
                break
        if theta_low_working is None:
            return None
        res_work = find_v0_for_distance(theta_low_working, target_dist)
        if res_work[1] <= max_apex_m:
            theta_low, theta_high = theta_low_working, HOOD_MAX_RAD
        else:
            return (theta_low_working, res_work[0], res_work[1], res_work[2])

    best = None
    for _ in range(60):
        theta_mid = (theta_low + theta_high) / 2.0
        res = find_v0_for_distance(theta_mid, target_dist)
        if res is None:
            theta_low = theta_mid
            continue
        if res[1] > max_apex_m:
            theta_high = theta_mid
        else:
            best = (theta_mid, res[0], res[1], res[2])
            theta_low = theta_mid
        if abs(theta_high - theta_low) < math.radians(0.005):
            break
    return best


def compute_offset(dist):
    """
    Three-segment offset strategy:
      d <= 2.50m : 0 (pure physics)
      2.50 < d <= 4.50m : V3 baseline = 0.35 * (d - 2.6)
      d > 4.50m : V3 baseline + far boost = 0.35*(d-2.6) + 0.30*(d-4.5)

    Returns (offset, max_apex, segment_label).
    """
    if dist <= 2.50:
        return 0.0, APEX_NEAR_M, "NEAR"
    elif dist <= 4.50:
        offset = V3_OFFSET_COEFF * (dist - V3_OFFSET_BASE)
        return max(0.0, offset), APEX_FAR_M, "MID"
    else:
        v3_base = V3_OFFSET_COEFF * (dist - V3_OFFSET_BASE)
        boost = FAR_BOOST_COEFF * (dist - FAR_BOOST_BASE)
        return v3_base + boost, APEX_FAR_M, "FAR"


def generate_table():
    distances = [
        round(DIST_MIN_M + i * DIST_STEP_M, 2)
        for i in range(int((DIST_MAX_M - DIST_MIN_M) / DIST_STEP_M) + 1)
    ]
    results = []

    print(f"V5 Hybrid Table: SLIP={SLIP_FACTOR}")
    print(f"  NEAR  (d<=2.50m): apex<={APEX_NEAR_M}m, no offset")
    print(
        f"  MID   (2.50<d<=4.50m): apex<={APEX_FAR_M}m, offset={V3_OFFSET_COEFF}*(d-{V3_OFFSET_BASE})"
    )
    print(
        f"  FAR   (d>4.50m): apex<={APEX_FAR_M}m, offset={V3_OFFSET_COEFF}*(d-{V3_OFFSET_BASE})+{FAR_BOOST_COEFF}*(d-{FAR_BOOST_BASE})"
    )
    print(
        f"{'Dist':>6s}  {'Target':>6s}  {'Seg':>4s}  {'Apex#':>5s}  {'Angle':>7s}  "
        f"{'rad':>10s}  {'v0':>7s}  {'rot/s':>7s}  {'Apex':>6s}  {'ToF':>7s}"
    )
    print("-" * 90)

    for dist in distances:
        offset, max_apex, seg = compute_offset(dist)
        target_dist = max(0.5, dist + offset)

        sol = find_optimal_angle(target_dist, max_apex)
        if sol is None:
            print(f"{dist:6.2f}  {target_dist:6.2f}  {seg:>4s}  {max_apex:5.1f}  FAIL")
            continue
        theta_rad, v0, apex_z, tof = sol
        rps = v0 / (2.0 * math.pi * EFF_R)
        theta_deg = math.degrees(theta_rad)

        print(
            f"{dist:6.2f}  {target_dist:6.2f}  {seg:>4s}  {max_apex:5.1f}  {theta_deg:7.2f}  "
            f"{theta_rad:10.6f}  {v0:7.4f}  {rps:7.2f}  {apex_z:6.3f}  {tof:7.4f}"
        )
        results.append((dist, theta_rad, rps, tof, apex_z, theta_deg))

    # Java output
    print("\n--- JAVA CODE ---")
    print(f"    // Hood Angle [rad] vs Distance [m]")
    print(
        f"    // Generated 2026-03-06 V5: NEAR(d<=2.5m apex<={APEX_NEAR_M}m) / "
        f"MID(2.5<d<=4.5m apex<={APEX_FAR_M}m+V3offset) / "
        f"FAR(d>4.5m apex<={APEX_FAR_M}m+V3offset+boost)"
    )
    print(f"    // Cd={CD}, slip={SLIP_FACTOR}")
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
