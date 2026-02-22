import sys

old_data = {
    5.00: {"rps": 62.10, "tof": 1.233076},
    5.25: {"rps": 63.00, "tof": 1.229285},
    5.50: {"rps": 63.85, "tof": 1.227447},
    5.75: {"rps": 64.70, "tof": 1.233261},
    6.00: {"rps": 65.50, "tof": 1.276686},
    6.25: {"rps": 66.25, "tof": 1.321245},
    6.50: {"rps": 67.00, "tof": 1.364455},
}

# y(x) = 1.8667*(x-5)^2 + 3.8*(x-5) + 62.10
new_data = {}
for dist in sorted(old_data.keys()):
    if dist < 5.00:
        continue
    dy = dist - 5.0
    new_rps = 1.8667 * dy**2 + 3.8 * dy + 62.10
    new_data[dist] = {"rps": new_rps}

    # TOF scales inversely with RPS (since exit angle is constant 60 deg for dist >= 5.75
    # and mostly flat from 5.0)
    old_rps = old_data[dist]["rps"]
    old_tof = old_data[dist]["tof"]
    new_tof = old_tof * (old_rps / new_rps)
    new_data[dist]["tof"] = new_tof

print("// Flywheel Speed Updates:")
for dist in sorted(new_data.keys()):
    print(f"    FLYWHEEL_SPEED_MAP.put({dist:.2f}, {new_data[dist]['rps']:.2f});")

print("\n// Time of Flight Updates:")
for dist in sorted(new_data.keys()):
    print(f"    TIME_OF_FLIGHT_MAP.put({dist:.2f}, {new_data[dist]['tof']:.6f});")
