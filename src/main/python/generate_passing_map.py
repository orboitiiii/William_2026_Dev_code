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
# For passing, our target is the floor (0.0m) or slightly above it (e.g. 0.1m for bounce)
TARGET_HEIGHT_M = 0.0

# 60 degrees is the minimum hood angle (flattest trajectory possible)
HOOD_ANGLE_RAD = math.radians(60.0)

WHEEL_RADIUS_M = 0.0508
SLIP_FACTOR = 0.4993  # Calibrated slip from generate_shot_table.py
EFF_R = WHEEL_RADIUS_M * SLIP_FACTOR

DT = 0.001
MAX_STEPS = int(8.0 / DT)


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
    v_low = 1.0
    v_high = 30.0

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

    v_mid = (v_low + v_high) / 2.0
    res = simulate(v_mid, HOOD_ANGLE_RAD)
    if res["hit"]:
        return (v_mid, res["cross_t"])
    return None


def main():
    print("Generating Passing Maps (3.5m to 12.0m)")
    dists = [x / 2 for x in range(7, 25)]  # 3.5, 4.0, ..., 12.0

    # User's current table already goes up to 6.5m but states "推算需 84.0，受限於馬達最大安全轉速 80" at 7.0m.
    # We will compute unbounded physics first, then cap it gracefully at 80.0 RPS.

    print("Distance | v0 (m/s) | RPS | ToF (s) | Capped RPS")
    print("-" * 55)

    java_code = []

    for d in dists:
        sol = find_v0_for_passing(d)
        if sol is None:
            continue
        v0, tof = sol
        rps = v0 / (2.0 * math.pi * EFF_R)

        capped_rps = min(80.0, rps)
        # If capped, re-simulate at 80.0 RPS to find actual landing distance & ToF?
        # A 80 RPS shot won't reach a 12m distance to the floor in the same way, it will fall short.
        # But the table structure maps *request distance* to *parameters*.
        # So we just cap RPS and keep the ToF for the 80 RPS trajectory.

        if rps > 80.0:
            v0_capped = 80.0 * 2.0 * math.pi * EFF_R
            res_capped = simulate(v0_capped, HOOD_ANGLE_RAD)
            tof_capped = res_capped["cross_t"]
            print(
                f"{d:8.1f} | {v0:8.3f} | {rps:>5.1f} | {tof:5.3f} | {capped_rps:>5.1f} (falls short at {res_capped['cross_x']:.1f}m)"
            )
            java_code.append(
                f"    PASSING_HOOD_ANGLE_MAP.put({d:.1f}, 1.047198); // 60 deg"
            )
            java_code.append(
                f"    PASSING_FLYWHEEL_SPEED_MAP.put({d:.1f}, 80.00); // Physics needed {rps:.1f}"
            )
            java_code.append(
                f"    PASSING_TIME_OF_FLIGHT_MAP.put({d:.1f}, {tof_capped:.4f});"
            )
        else:
            print(
                f"{d:8.1f} | {v0:8.3f} | {rps:>5.1f} | {tof:5.3f} | {capped_rps:>5.1f}"
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
