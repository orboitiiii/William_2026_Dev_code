#!/usr/bin/env python3
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
TARGET_HEIGHT_M = 0.0  # Passing to the ground

# Hood is fixed at 60 deg (mechanical minimum) for lowest possible shot
THETA_RAD = math.radians(60.0)

WHEEL_RADIUS_M = 0.0508
SLIP_FACTOR = 0.4993  # Calibrated value from ShotTables.java

DIST_MIN_M = 2.00
DIST_MAX_M = 12.00
DIST_STEP_M = 0.50

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

        if z > TARGET_HEIGHT_M and z_new <= TARGET_HEIGHT_M:
            dz = z - z_new
            alpha = (z - TARGET_HEIGHT_M) / dz if dz > 1e-12 else 0.0
            cx = x + alpha * (x_new - x)
            ct = t + alpha * DT
            return {"apex_z": apex_z, "cross_x": cx, "cross_t": ct, "hit": True}

        if z_new < -0.5:
            return {"apex_z": apex_z, "hit": False, "cross_x": x_new}

        x, z, vx, vz, t = x_new, z_new, vx_new, vz_new, t_new

    return {"apex_z": apex_z, "hit": False, "cross_x": x}


def find_v0_for_distance(target_dist):
    v_low = 1.0
    v_high = 20.0

    # Bisection
    for _ in range(80):
        v_mid = (v_low + v_high) / 2.0
        res = simulate(v_mid, THETA_RAD)
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
    res = simulate(v_mid, THETA_RAD)
    if res["hit"]:
        return (v_mid, res["apex_z"], res["cross_t"])
    return None


def main():
    eff_r = WHEEL_RADIUS_M * SLIP_FACTOR

    print("      // --- passing tables generated using 0.0m target height RK4 sim ---")
    distances = []
    d = DIST_MIN_M
    while d <= DIST_MAX_M + 0.001:
        sol = find_v0_for_distance(d)
        if sol is not None:
            v0, apex_z, tof = sol
            rps = v0 / (2.0 * math.pi * eff_r)
            # Add small safety buffer to RPS so it doesn't barely reach 0
            # and hard cap at 10.0 RPM for minimum physical wheel traction
            rps = max(rps, 10.0)

            print(
                f"      PASSING_HOOD_ANGLE_MAP.put({d:.1f}, {THETA_RAD:.6f}); // 60 deg"
            )
            print(
                f"      PASSING_FLYWHEEL_SPEED_MAP.put({d:.1f}, {rps:.2f}); // v0={v0:.2f} m/s"
            )
            print(f"      PASSING_TIME_OF_FLIGHT_MAP.put({d:.1f}, {tof:.4f});")
        d += DIST_STEP_M


if __name__ == "__main__":
    main()
