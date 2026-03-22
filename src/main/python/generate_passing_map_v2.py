#!/usr/bin/env python3
"""
Passing Table Generator V2 - Empricially Boosted Speed & TOF
Team 9427 - 2026-03-03

User Feedback on V1 (0.4993 slip, exact distance):
- "Can you increase the passing speed a little bit? TOF also needs to be changed"

Physics Profile:
- We will fix SLIP_FACTOR = 0.4993.
- To increase speed and TOF structurally across the board, we will aim the
  simulator an additional `TARGET_OFFSET_M` further than the requested distance.
- This forces the RK4 solver to find a higher `v0` (speed) and a corresponding longer `TOF`
  to reach the imaginary point further away. We then map this back to the original `d`.
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
TARGET_HEIGHT_M = 0.0

HOOD_ANGLE_RAD = math.radians(60.0)
WHEEL_RADIUS_M = 0.0508
SLIP_FACTOR = 0.4993
EFF_R = WHEEL_RADIUS_M * SLIP_FACTOR

DT = 0.001
MAX_STEPS = int(8.0 / DT)

# === USER FEEDBACK CALIBRATION ===
# We will tell the physics engine to calculate the shot as if the target
# is 0.8 meters further away. This will universally boost RPM and TOF.
TARGET_OFFSET_M = 0.8


def simulate(v0, theta_rad):
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

        if z > TARGET_HEIGHT_M and z_new <= TARGET_HEIGHT_M:
            dz = z - z_new
            alpha = (z - TARGET_HEIGHT_M) / dz if dz > 1e-12 else 0.0
            cx = x + alpha * (x_new - x)
            ct = t + alpha * DT
            return {"cross_x": cx, "cross_t": ct, "hit": True}

        if z_new < -0.5:
            return {"hit": False, "cross_x": x_new}

        x, z, vx, vz, t = x_new, z_new, vx_new, vz_new, t_new

    return {"hit": False, "cross_x": x}


def find_v0_for_passing(target_dist):
    v_low, v_high = 1.0, 30.0
    for _ in range(30):
        res = simulate(v_high, HOOD_ANGLE_RAD)
        if res["hit"] and res["cross_x"] >= target_dist:
            break
        v_high *= 1.3
        if v_high > 200:
            return None
    else:
        return None

    for _ in range(80):
        v_mid = (v_low + v_high) / 2.0
        res = simulate(v_mid, HOOD_ANGLE_RAD)
        if not res["hit"]:
            v_low = v_mid
            continue
        cx = res["cross_x"]
        if abs(cx - target_dist) < 0.001:
            return (v_mid, res["cross_t"])
        if cx < target_dist:
            v_low = v_mid
        else:
            v_high = v_mid

    res = simulate((v_low + v_high) / 2.0, HOOD_ANGLE_RAD)
    if res["hit"]:
        return ((v_low + v_high) / 2.0, res["cross_t"])
    return None


def main():
    print(f"Generating Passing Maps V2 (Offset: +{TARGET_OFFSET_M}m)")
    dists = [x / 2 for x in range(2, 25)]  # 1.0, 1.5, ..., 12.0

    print("Distance | Target (m) | v0 (m/s) | RPS | ToF (s) | Capped RPS")
    print("-" * 65)

    java_code = []

    for d in dists:
        target_d = d + TARGET_OFFSET_M
        sol = find_v0_for_passing(target_d)
        if sol is None:
            continue

        v0, tof = sol
        rps = v0 / (2.0 * math.pi * EFF_R)
        capped_rps = min(80.0, rps)

        if rps > 80.0:
            v0_capped = 80.0 * 2.0 * math.pi * EFF_R
            res_capped = simulate(v0_capped, HOOD_ANGLE_RAD)
            tof_capped = res_capped["cross_t"]
            print(
                f"{d:8.1f} | {target_d:10.1f} | {v0:8.3f} | {rps:>5.1f} | {tof:5.3f} | {capped_rps:>5.1f} (falls short)"
            )
            java_code.append(
                f"    PASSING_HOOD_ANGLE_MAP.put({d:.1f}, 1.047198); // 60 deg"
            )
            java_code.append(
                f"    PASSING_FLYWHEEL_SPEED_MAP.put({d:.1f}, 80.00); // Need {rps:.1f}"
            )
            java_code.append(
                f"    PASSING_TIME_OF_FLIGHT_MAP.put({d:.1f}, {tof_capped:.4f});"
            )
        else:
            print(
                f"{d:8.1f} | {target_d:10.1f} | {v0:8.3f} | {rps:>5.1f} | {tof:5.3f} | {capped_rps:>5.1f}"
            )
            java_code.append(
                f"    PASSING_HOOD_ANGLE_MAP.put({d:.1f}, 1.047198); // 60 deg"
            )
            java_code.append(f"    PASSING_FLYWHEEL_SPEED_MAP.put({d:.1f}, {rps:.2f});")
            java_code.append(f"    PASSING_TIME_OF_FLIGHT_MAP.put({d:.1f}, {tof:.4f});")

    print("\n--- JAVA CODE ---")
    for line in java_code:
        print(line)


if __name__ == "__main__":
    main()
