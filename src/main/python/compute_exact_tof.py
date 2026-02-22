import math

GRAVITY = 9.80665
RHO_AIR = 1.225
BALL_MASS_KG = 0.215
BALL_DIAMETER_M = 0.150
BALL_AREA_M2 = math.pi * (BALL_DIAMETER_M / 2.0) ** 2
CD = 0.485
DRAG_K = 0.5 * RHO_AIR * CD * BALL_AREA_M2 / BALL_MASS_KG

LAUNCH_HEIGHT_M = 0.50
TARGET_HEIGHT_M = 1.8288
WHEEL_RADIUS_M = 0.0508
SLIP = 0.4993
REFF = WHEEL_RADIUS_M * SLIP

DT = 0.0005
MAX_STEPS = int(8.0 / DT)


def simulate_tof(v0, theta_rad):
    vx = v0 * math.cos(theta_rad)
    vz = v0 * math.sin(theta_rad)
    z = LAUNCH_HEIGHT_M
    t = 0.0
    past_apex = False

    for _ in range(MAX_STEPS):

        def deriv(vx_, vz_):
            spd = math.sqrt(vx_ * vx_ + vz_ * vz_)
            return -DRAG_K * spd * vx_, -GRAVITY - DRAG_K * spd * vz_

        ax1, az1 = deriv(vx, vz)
        ax2, az2 = deriv(vx + 0.5 * DT * ax1, vz + 0.5 * DT * az1)
        ax3, az3 = deriv(vx + 0.5 * DT * ax2, vz + 0.5 * DT * az2)
        ax4, az4 = deriv(vx + DT * ax3, vz + DT * az3)

        z_new = z + (DT / 6.0) * (
            vz + 2 * (vz + 0.5 * DT * az1) + 2 * (vz + 0.5 * DT * az2) + (vz + DT * az3)
        )
        vx_new = vx + (DT / 6.0) * (ax1 + 2 * ax2 + 2 * ax3 + ax4)
        vz_new = vz + (DT / 6.0) * (az1 + 2 * az2 + 2 * az3 + az4)
        t_new = t + DT

        if not past_apex and vz_new < 0:
            past_apex = True

        if past_apex and z > TARGET_HEIGHT_M and z_new <= TARGET_HEIGHT_M:
            alpha = (z - TARGET_HEIGHT_M) / (z - z_new) if (z - z_new) > 1e-12 else 0.0
            return t + alpha * DT

        if z_new < -0.5:
            return None  # Hit ground before target height

        z, vx, vz, t = z_new, vx_new, vz_new, t_new
    return None


distances = [
    0.5,
    0.75,
    1.0,
    1.25,
    1.5,
    1.75,
    2.0,
    2.25,
    2.5,
    2.75,
    3.0,
    3.25,
    3.5,
    3.75,
    4.0,
    4.25,
    4.5,
    4.75,
    5.0,
    5.25,
    5.5,
    5.75,
    6.0,
    6.25,
    6.5,
]
angles = [
    1.448623,
    1.448623,
    1.448623,
    1.447937,
    1.423485,
    1.399376,
    1.375218,
    1.351256,
    1.327392,
    1.303822,
    1.280546,
    1.257466,
    1.234631,
    1.212041,
    1.189892,
    1.168037,
    1.146476,
    1.125160,
    1.104236,
    1.083802,
    1.063515,
    1.047198,
    1.047198,
    1.047198,
    1.047198,
]
rps = [
    35.21,
    40.39,
    45.56,
    50.36,
    50.54,
    50.74,
    50.98,
    51.26,
    51.56,
    51.90,
    52.80,
    54.00,
    55.40,
    56.80,
    58.60,
    60.50,
    62.50,
    64.60,
    66.80,
    69.10,
    71.50,
    74.00,
    76.50,
    79.00,
    81.50,
]

print("--------- EXACT TOF FOR EMPIRICAL RPS --------")
for d, ang, r in zip(distances, angles, rps):
    v0 = r * 2.0 * math.pi * REFF
    tof = simulate_tof(v0, ang)
    if tof:
        print(f"    TIME_OF_FLIGHT_MAP.put({d:.2f}, {tof:.6f});")
    else:
        print(f"    TIME_OF_FLIGHT_MAP.put({d:.2f}, 1.0); // FAILED")
