#!/usr/bin/env python3
"""
TOF (Time-of-Flight) Analysis & Visualization — Publication-Quality Figure.
==========================================================================
Team 9427 · 2026-02-22

Physics Model
-------------
  2-D projectile with quadratic drag (no Magnus / backspin):

    a_x = -k·|v|·v_x
    a_z = -g - k·|v|·v_z

    where k = ½·ρ_air·C_d·A / m

  Integration: Classical 4th-order Runge-Kutta, dt = 0.5 ms.
  TOF is the time from launch to the descending-arc crossing of
  z = TARGET_HEIGHT (1.8288 m), interpolated linearly within the
  crossing step.

Data Source
-----------
  Lookup table entries from ShotTables.java (25 nodes, 0.50-6.50 m).
  Dense simulation curve from compute_exact_tof.py physics engine.

Output
------
  ballistic_report/tof_analysis.png  — 4-panel engineering figure.

Author: Team 9427, Auto-generated.
"""

import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np

# ═══════════════════════════════════════════════════════════════════════════
# Physical Constants — must match generate_shot_table.py / ShotTables.java
# ═══════════════════════════════════════════════════════════════════════════
GRAVITY = 9.80665  # m/s²
RHO_AIR = 1.225  # kg/m³
BALL_MASS_KG = 0.215
BALL_DIAMETER_M = 0.150
BALL_AREA_M2 = math.pi * (BALL_DIAMETER_M / 2.0) ** 2
CD = 0.485
DRAG_K = 0.5 * RHO_AIR * CD * BALL_AREA_M2 / BALL_MASS_KG

LAUNCH_HEIGHT_M = 0.50
TARGET_HEIGHT_M = 1.8288  # 72 in
WHEEL_RADIUS_M = 0.0508  # 4" Stealth Wheel
SLIP = 0.4993  # calibrated 2026-02-20
REFF = WHEEL_RADIUS_M * SLIP  # 0.02537 m

HOOD_MIN_DEG = 60.0
HOOD_MAX_DEG = 83.0
MAX_APEX_M = 3.5

DT = 0.0005
MAX_STEPS = int(8.0 / DT)

# ═══════════════════════════════════════════════════════════════════════════
# ShotTables.java Data — Lookup Table (25 calibration nodes)
# ═══════════════════════════════════════════════════════════════════════════
TABLE_DISTANCES = [
    0.50,
    0.75,
    1.00,
    1.25,
    1.50,
    1.75,
    2.00,
    2.25,
    2.50,
    2.75,
    3.00,
    3.25,
    3.50,
    3.75,
    4.00,
    4.25,
    4.50,
    4.75,
    5.00,
    5.25,
    5.50,
    5.75,
    6.00,
    6.25,
    6.50,
]
TABLE_ANGLES_RAD = [
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
TABLE_RPS = [
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
TABLE_TOF = [
    0.752875,
    0.995992,
    1.193931,
    1.360074,
    1.360013,
    1.360254,
    1.360015,
    1.360030,
    1.359597,
    1.359452,
    1.376321,
    1.401268,
    1.430713,
    1.458278,
    1.495994,
    1.534354,
    1.573217,
    1.612403,
    1.651914,
    1.691743,
    1.731414,
    1.775321,
    1.836694,
    1.896960,
    1.956165,
]


# ═══════════════════════════════════════════════════════════════════════════
# RK4 Simulator — exact copy of compute_exact_tof.py physics
# ═══════════════════════════════════════════════════════════════════════════
def simulate(v0, theta_rad):
    """
    RK4 trajectory integration.

    Returns dict with keys:
      hit    — True if ball crosses TARGET_HEIGHT on descent
      tof    — time of flight [s] (if hit)
      apex_z — maximum altitude [m]
      cross_x — horizontal distance at crossing [m] (if hit)
    """
    vx = v0 * math.cos(theta_rad)
    vz = v0 * math.sin(theta_rad)
    x, z, t = 0.0, LAUNCH_HEIGHT_M, 0.0
    apex_z = z
    past_apex = False

    def deriv(vx_, vz_):
        spd = math.sqrt(vx_ * vx_ + vz_ * vz_)
        return -DRAG_K * spd * vx_, -GRAVITY - DRAG_K * spd * vz_

    for _ in range(MAX_STEPS):
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
            return {"hit": True, "tof": ct, "apex_z": apex_z, "cross_x": cx}

        if z_new < -0.5:
            return {"hit": False, "tof": None, "apex_z": apex_z, "cross_x": x_new}

        x, z, vx, vz, t = x_new, z_new, vx_new, vz_new, t_new

    return {"hit": False, "tof": None, "apex_z": apex_z, "cross_x": x}


# ═══════════════════════════════════════════════════════════════════════════
# Dense Simulation Sweep
# ═══════════════════════════════════════════════════════════════════════════
def compute_dense_sweep():
    """Simulate at 0.01 m resolution for smooth curves."""
    # Per-node exact simulation
    node_d, node_tof, node_apex, node_angle_deg, node_v0 = [], [], [], [], []
    node_rps = []
    for d, ang, rps in zip(TABLE_DISTANCES, TABLE_ANGLES_RAD, TABLE_RPS):
        v0 = rps * 2.0 * math.pi * REFF
        res = simulate(v0, ang)
        if res["hit"]:
            node_d.append(d)
            node_tof.append(res["tof"])
            node_apex.append(res["apex_z"])
            node_angle_deg.append(math.degrees(ang))
            node_v0.append(v0)
            node_rps.append(rps)

    # Dense interpolated sweep between table nodes
    dense_d, dense_tof, dense_apex = [], [], []
    dense_angle_deg, dense_rps, dense_v0 = [], [], []
    for di in np.arange(0.50, 6.51, 0.01):
        ang_interp = np.interp(di, TABLE_DISTANCES, TABLE_ANGLES_RAD)
        rps_interp = np.interp(di, TABLE_DISTANCES, TABLE_RPS)
        v0_interp = rps_interp * 2.0 * math.pi * REFF
        res = simulate(v0_interp, ang_interp)
        if res["hit"]:
            dense_d.append(di)
            dense_tof.append(res["tof"])
            dense_apex.append(res["apex_z"])
            dense_angle_deg.append(math.degrees(ang_interp))
            dense_rps.append(rps_interp)
            dense_v0.append(v0_interp)

    return {
        "node_d": np.array(node_d),
        "node_tof": np.array(node_tof),
        "node_apex": np.array(node_apex),
        "node_angle_deg": np.array(node_angle_deg),
        "node_v0": np.array(node_v0),
        "node_rps": np.array(node_rps),
        "dense_d": np.array(dense_d),
        "dense_tof": np.array(dense_tof),
        "dense_apex": np.array(dense_apex),
        "dense_angle_deg": np.array(dense_angle_deg),
        "dense_rps": np.array(dense_rps),
        "dense_v0": np.array(dense_v0),
    }


# ═══════════════════════════════════════════════════════════════════════════
# Regime Annotation Helper
# ═══════════════════════════════════════════════════════════════════════════
# Regime boundaries derived from the physics:
#   I   0.50-1.25 m  : Hood at max (83 deg), velocity rising
#   II  1.25-2.75 m  : Apex constrained at 3.50 m, angle drops, speed ~constant
#   III 2.75-5.50 m  : Both angle and speed actively varying
#   IV  5.50-6.50 m  : Hood hits 60 deg mechanical limit
REGIMES = [
    {"xmin": 0.50, "xmax": 1.25, "label": "I", "color": "#EA580C"},  # ORANGE
    {"xmin": 1.25, "xmax": 2.75, "label": "II", "color": "#059669"},  # GREEN
    {"xmin": 2.75, "xmax": 5.50, "label": "III", "color": "#2563EB"},  # BLUE
    {"xmin": 5.50, "xmax": 6.50, "label": "IV", "color": "#7C3AED"},  # PURPLE
]


def _add_regime_bands(ax, y_frac=0.95):
    """Add translucent regime bands + Roman numeral labels to any axis."""
    for r in REGIMES:
        ax.axvspan(r["xmin"], r["xmax"], alpha=0.06, color=r["color"], zorder=1)
        # Label at the top of the plot
        ax.annotate(
            r["label"],
            xy=((r["xmin"] + r["xmax"]) / 2, y_frac),
            xycoords=("data", "axes fraction"),
            fontsize=8,
            fontweight="bold",
            ha="center",
            va="top",
            color=r["color"],
            alpha=0.7,
        )


# ═══════════════════════════════════════════════════════════════════════════
# Plotting — 4-Panel: TOF, Hood Angle, Flywheel RPS, Exit Velocity
# ═══════════════════════════════════════════════════════════════════════════
def create_figure():
    data = compute_dense_sweep()

    java_d = np.array(TABLE_DISTANCES)
    java_tof = np.array(TABLE_TOF)
    java_angle_deg = np.array([math.degrees(a) for a in TABLE_ANGLES_RAD])
    java_rps = np.array(TABLE_RPS)
    java_v0 = java_rps * 2.0 * math.pi * REFF

    # ── Style ──────────────────────────────────────────────────────────
    plt.rcParams.update(
        {
            "font.family": "serif",
            "font.size": 9,
            "axes.titlesize": 11,
            "axes.labelsize": 10,
            "legend.fontsize": 8,
            "xtick.labelsize": 8,
            "ytick.labelsize": 8,
            "axes.grid": True,
            "grid.alpha": 0.3,
            "figure.dpi": 150,
            "savefig.dpi": 200,
        }
    )

    BLUE = "#2563EB"
    RED = "#DC2626"
    GREEN = "#059669"
    PURPLE = "#7C3AED"
    ORANGE = "#EA580C"
    GRAY = "#6B7280"

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle(
        "Shot Parameter Analysis — RK4 Ballistic Simulation\n"
        f"Team 9427  |  Cd={CD}, slip={SLIP}, dt={DT*1000:.1f} ms  |  "
        f"h0={LAUNCH_HEIGHT_M} m, h_target={TARGET_HEIGHT_M:.4f} m  |  "
        f"R_eff={REFF*1000:.2f} mm",
        fontsize=11,
        fontweight="bold",
        y=0.98,
    )

    XLIM = (0.3, 6.7)

    # ────────────────────────────────────────────────────────────────────
    # (a) Time of Flight
    # ────────────────────────────────────────────────────────────────────
    ax = axes[0, 0]
    ax.plot(
        data["dense_d"],
        data["dense_tof"],
        color=BLUE,
        linewidth=1.5,
        label="Dense RK4 sim.",
        zorder=3,
    )
    ax.scatter(
        java_d,
        java_tof,
        color=RED,
        s=40,
        zorder=5,
        marker="o",
        edgecolors="black",
        linewidths=0.5,
        label=f"ShotTables.java ({len(java_d)} nodes)",
    )
    _add_regime_bands(ax)

    # Plateau annotation
    ax.annotate(
        "Plateau ~1.360 s\n(apex-constrained)",
        xy=(2.0, 1.36),
        xytext=(2.5, 1.15),
        fontsize=7,
        ha="center",
        color=GREEN,
        fontstyle="italic",
        arrowprops=dict(arrowstyle="->", color=GREEN, lw=0.8),
    )

    ax.set_xlabel("Distance to Target [m]")
    ax.set_ylabel("Time of Flight [s]")
    ax.set_title("(a) Time of Flight (TOF)")
    ax.legend(loc="upper left", framealpha=0.9)
    ax.set_xlim(XLIM)
    ax.set_ylim(0.6, 2.1)
    ax.xaxis.set_major_locator(ticker.MultipleLocator(0.5))
    ax.yaxis.set_major_locator(ticker.MultipleLocator(0.1))

    # ────────────────────────────────────────────────────────────────────
    # (b) Hood Angle
    # ────────────────────────────────────────────────────────────────────
    ax = axes[0, 1]
    ax.plot(
        data["dense_d"],
        data["dense_angle_deg"],
        color=RED,
        linewidth=1.5,
        label="Dense RK4 sim.",
        zorder=3,
    )
    ax.scatter(
        java_d,
        java_angle_deg,
        color=RED,
        s=40,
        zorder=5,
        marker="s",
        edgecolors="black",
        linewidths=0.5,
        label=f"ShotTables.java ({len(java_d)} nodes)",
    )
    _add_regime_bands(ax)

    # Mechanical limits
    ax.axhline(HOOD_MAX_DEG, color=GRAY, linestyle=":", linewidth=0.8, alpha=0.7)
    ax.axhline(HOOD_MIN_DEG, color=GRAY, linestyle=":", linewidth=0.8, alpha=0.7)
    ax.annotate(
        f"Mechanical max = {HOOD_MAX_DEG:.0f} deg",
        xy=(4.0, HOOD_MAX_DEG + 0.5),
        fontsize=7,
        color=GRAY,
        alpha=0.8,
    )
    ax.annotate(
        f"Mechanical min = {HOOD_MIN_DEG:.0f} deg",
        xy=(4.0, HOOD_MIN_DEG + 0.5),
        fontsize=7,
        color=GRAY,
        alpha=0.8,
    )

    # Saturated regions
    ax.annotate(
        "Saturated at 83 deg\n(Regime I)",
        xy=(0.875, 83.0),
        xytext=(1.5, 78.0),
        fontsize=7,
        ha="center",
        color=ORANGE,
        fontstyle="italic",
        arrowprops=dict(arrowstyle="->", color=ORANGE, lw=0.8),
    )
    ax.annotate(
        "Saturated at 60 deg\n(Regime IV)",
        xy=(6.0, 60.0),
        xytext=(5.2, 65.0),
        fontsize=7,
        ha="center",
        color=PURPLE,
        fontstyle="italic",
        arrowprops=dict(arrowstyle="->", color=PURPLE, lw=0.8),
    )

    ax.set_xlabel("Distance to Target [m]")
    ax.set_ylabel("Hood Angle [deg]")
    ax.set_title("(b) Hood Angle")
    ax.legend(loc="center right", framealpha=0.9)
    ax.set_xlim(XLIM)
    ax.set_ylim(55, 90)
    ax.xaxis.set_major_locator(ticker.MultipleLocator(0.5))
    ax.yaxis.set_major_locator(ticker.MultipleLocator(5))

    # ────────────────────────────────────────────────────────────────────
    # (c) Flywheel Speed (RPS)
    # ────────────────────────────────────────────────────────────────────
    ax = axes[1, 0]
    ax.plot(
        data["dense_d"],
        data["dense_rps"],
        color=GREEN,
        linewidth=1.5,
        label="Dense RK4 sim.",
        zorder=3,
    )
    ax.scatter(
        java_d,
        java_rps,
        color=GREEN,
        s=40,
        zorder=5,
        marker="D",
        edgecolors="black",
        linewidths=0.5,
        label=f"ShotTables.java ({len(java_d)} nodes)",
    )
    _add_regime_bands(ax)

    # RPM secondary axis
    ax2 = ax.twinx()
    rpm_ticks = np.arange(2000, 5500, 500)
    ax2.set_ylim(np.array(ax.get_ylim()) * 60)
    ax2.set_ylabel("Flywheel Speed [RPM]", color=GRAY, alpha=0.7)
    ax2.tick_params(axis="y", labelcolor=GRAY, labelsize=7)

    # Plateau annotation
    ax.annotate(
        "~50-52 RPS plateau\n(speed nearly constant,\nangle absorbs distance)",
        xy=(2.0, 51.0),
        xytext=(3.0, 42.0),
        fontsize=7,
        ha="center",
        color=GREEN,
        fontstyle="italic",
        arrowprops=dict(arrowstyle="->", color=GREEN, lw=0.8),
    )

    # Growth annotation
    ax.annotate(
        "Linear growth\n(drag compensation)",
        xy=(5.0, 67.0),
        xytext=(4.5, 75.0),
        fontsize=7,
        ha="center",
        color=BLUE,
        fontstyle="italic",
        arrowprops=dict(arrowstyle="->", color=BLUE, lw=0.8),
    )

    ax.set_xlabel("Distance to Target [m]")
    ax.set_ylabel("Flywheel Speed [rot/s]")
    ax.set_title("(c) Flywheel Speed")
    ax.legend(loc="upper left", framealpha=0.9)
    ax.set_xlim(XLIM)
    ax.xaxis.set_major_locator(ticker.MultipleLocator(0.5))
    ax.yaxis.set_major_locator(ticker.MultipleLocator(5))

    # ────────────────────────────────────────────────────────────────────
    # (d) Exit Velocity (actual v0 in m/s)
    # ────────────────────────────────────────────────────────────────────
    ax = axes[1, 1]
    ax.plot(
        data["dense_d"],
        data["dense_v0"],
        color=PURPLE,
        linewidth=1.5,
        label="Dense RK4 sim.",
        zorder=3,
    )
    ax.scatter(
        java_d,
        java_v0,
        color=PURPLE,
        s=40,
        zorder=5,
        marker="^",
        edgecolors="black",
        linewidths=0.5,
        label=f"ShotTables.java ({len(java_d)} nodes)",
    )
    _add_regime_bands(ax)

    # Conversion formula annotation (bottom-right, axes fraction coords)
    ax.annotate(
        f"v_exit = RPS x 2pi x R_eff\n"
        f"R_eff = R_wheel x slip\n"
        f"     = {WHEEL_RADIUS_M*100:.2f} cm x {SLIP}\n"
        f"     = {REFF*1000:.2f} mm",
        xy=(0.97, 0.05),
        xycoords="axes fraction",
        fontsize=7,
        ha="right",
        va="bottom",
        bbox=dict(
            boxstyle="round,pad=0.4",
            facecolor="lightyellow",
            edgecolor=GRAY,
            alpha=0.85,
        ),
        family="monospace",
    )

    # Horizontal velocity component (for reference)
    vx_arr = data["dense_v0"] * np.cos(np.radians(data["dense_angle_deg"]))
    vz_arr = data["dense_v0"] * np.sin(np.radians(data["dense_angle_deg"]))
    ax.plot(
        data["dense_d"],
        vx_arr,
        color=BLUE,
        linewidth=0.8,
        linestyle="--",
        alpha=0.6,
        label="Horizontal comp. (v_x)",
    )
    ax.plot(
        data["dense_d"],
        vz_arr,
        color=ORANGE,
        linewidth=0.8,
        linestyle="--",
        alpha=0.6,
        label="Vertical comp. (v_z)",
    )

    ax.set_xlabel("Distance to Target [m]")
    ax.set_ylabel("Exit Velocity [m/s]")
    ax.set_title("(d) Exit Velocity & Components")
    ax.legend(loc="upper left", framealpha=0.9, fontsize=7)
    ax.set_xlim(XLIM)
    ax.xaxis.set_major_locator(ticker.MultipleLocator(0.5))
    ax.yaxis.set_major_locator(ticker.MultipleLocator(1))

    # ── Final layout ─────────────────────────────────────────────────
    fig.tight_layout(rect=[0, 0.03, 1, 0.94])

    # Footer
    fig.text(
        0.5,
        0.005,
        f"Physics: Gravity + Quadratic Drag (Cd={CD}) | "
        f"Ball: m={BALL_MASS_KG} kg, d={BALL_DIAMETER_M*100:.0f} cm | "
        f'Shooter: 4" Stealth Wheel, slip={SLIP} (calibrated 2026-02-20) | '
        f"Solver: RK4 @ dt={DT*1000:.1f} ms, bisection on angle & velocity",
        ha="center",
        fontsize=7,
        color=GRAY,
        fontstyle="italic",
    )

    return fig


def main():
    out_dir = Path(__file__).parent / "ballistic_report"
    out_dir.mkdir(exist_ok=True)
    out_path = out_dir / "tof_analysis.png"

    print(f"Computing dense TOF simulation...")
    fig = create_figure()
    fig.savefig(str(out_path), bbox_inches="tight", facecolor="white")
    print(f"Saved: {out_path}")

    # Print summary table for verification
    print("\n" + "=" * 72)
    print("  TOF TABLE VERIFICATION (ShotTables.java entries)")
    print("=" * 72)
    print(
        f"{'Dist':>6s}  {'Ang(deg)':>8s}  {'RPS':>7s}  {'v0(m/s)':>8s}  "
        f"{'TOF_table':>10s}  {'TOF_sim':>9s}  {'dT(ms)':>8s}  {'Apex(m)':>8s}"
    )
    print("-" * 82)

    for d, ang, rps, tof_table in zip(
        TABLE_DISTANCES, TABLE_ANGLES_RAD, TABLE_RPS, TABLE_TOF
    ):
        v0 = rps * 2.0 * math.pi * REFF
        res = simulate(v0, ang)
        tof_sim = res["tof"] if res["hit"] else float("nan")
        apex = res["apex_z"]
        delta_ms = (tof_table - tof_sim) * 1000 if res["hit"] else float("nan")
        print(
            f"{d:6.2f}  {math.degrees(ang):7.2f}  {rps:7.2f}  {v0:8.4f}  "
            f"{tof_table:10.6f}  {tof_sim:9.6f}  {delta_ms:8.3f}  {apex:8.4f}"
        )


if __name__ == "__main__":
    main()
