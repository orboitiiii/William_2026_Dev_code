#!/usr/bin/env python3
"""
Ballistic Solver — Full Simulation Report Visualizer
=====================================================
Team 9427 · 2026

Reads CSV files produced by BallisticReport.java and generates 5 publication-
quality matplotlib figures:

  Figure 1: Distance Sweep       (pitch, RPM, velocity, ToF vs distance)
  Figure 2: Motion Compensation  (turret yaw, hood pitch, RPM over time)
  Figure 3: Trajectory Views     (XZ side view, XY top view)
  Figure 4: Sensitivity Analysis (pitch/velocity error → miss distance)
  Figure 5: Hit Zone Map         (field coverage + hexagon impact)

Usage:
  python plot_ballistic_report.py                   # read from default dir
  python plot_ballistic_report.py /path/to          # specify CSV directory
  python plot_ballistic_report.py --generate        # generate test data + plot

Hardware:
  4" Orange (40A) AndyMark Stealth Wheel + Polycarbonate Backplate
  Slip factor ≈ 0.60  ·  Max 4000 RPM
"""

import os
import sys
import glob
import math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import RegularPolygon, FancyArrowPatch
from matplotlib.gridspec import GridSpec
from pathlib import Path

# ── Configuration ───────────────────────────────────────────────────────────

DATA_DIR = Path(__file__).parent / "ballistic_report"
OUT_DIR = DATA_DIR  # save PNGs alongside CSVs

TARGET_HEIGHT_M = 72.0 * 0.0254   # 1.8288 m
HEXAGON_RADIUS_IN = 21.0           # approximate inner radius for viz

# Colour palette (team-friendly, colour-blind safe)
C_PRIMARY = "#2563EB"    # blue
C_SECONDARY = "#DC2626"  # red
C_ACCENT = "#F59E0B"     # amber
C_GREEN = "#16A34A"
C_PURPLE = "#7C3AED"
C_GRAY = "#6B7280"

plt.rcParams.update({
    "figure.dpi": 150,
    "savefig.dpi": 200,
    "font.size": 10,
    "axes.titlesize": 12,
    "axes.labelsize": 10,
    "legend.fontsize": 8,
    "figure.facecolor": "white",
    "axes.grid": True,
    "grid.alpha": 0.3,
})


# ── Utility ─────────────────────────────────────────────────────────────────

def load_csv(name: str) -> pd.DataFrame | None:
    """Load a CSV from DATA_DIR, returning None if missing."""
    p = DATA_DIR / name
    if not p.exists():
        print(f"  [SKIP] {p} not found")
        return None
    return pd.read_csv(p)


def save_fig(fig, name: str):
    p = OUT_DIR / name
    fig.savefig(p, bbox_inches="tight", facecolor="white")
    print(f"  [OK] Saved {p}")
    plt.close(fig)


# ── Figure 1: Distance Sweep ───────────────────────────────────────────────

def figure1_distance_sweep():
    """Static shot parameters as a function of distance."""
    df = load_csv("distance_sweep.csv")
    if df is None:
        return

    df = df[df["valid"] == True].copy()
    if df.empty:
        print("  [SKIP] No valid solutions in distance_sweep.csv")
        return

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle(
        "Distance Sweep — Static Shot Parameters\n"
        "4\" Orange Stealth (40A) + Backplate  ·  Slip Factor 0.60",
        fontsize=13, fontweight="bold",
    )

    d = df["distance_m"]

    # (0,0) Hood Pitch
    ax = axes[0, 0]
    ax.plot(d, df["pitch_deg"], color=C_PRIMARY, linewidth=2)
    ax.set_ylabel("Hood Pitch (°)")
    ax.set_xlabel("Distance to Target (m)")
    ax.set_title("Hood Pitch vs Distance")

    # (0,1) Shooter RPM
    ax = axes[0, 1]
    ax.plot(d, df["rpm"], color=C_SECONDARY, linewidth=2)
    ax.set_ylabel("Shooter Wheel RPM")
    ax.set_xlabel("Distance to Target (m)")
    ax.set_title("Shooter RPM vs Distance")
    ax.axhline(4000, color=C_GRAY, linestyle="--", alpha=0.5, label="Max RPM")
    ax.legend()

    # (1,0) Exit Velocity
    ax = axes[1, 0]
    ax.plot(d, df["velocity_mps"], color=C_GREEN, linewidth=2)
    ax.set_ylabel("Exit Velocity (m/s)")
    ax.set_xlabel("Distance to Target (m)")
    ax.set_title("Exit Velocity vs Distance")

    # (1,1) Time of Flight
    ax = axes[1, 1]
    ax.plot(d, df["tof_s"], color=C_PURPLE, linewidth=2)
    ax2 = ax.twinx()
    ax2.plot(d, df["apex_height_m"], color=C_ACCENT, linewidth=1.5, linestyle="--")
    ax2.set_ylabel("Apex Height (m)", color=C_ACCENT)
    ax.set_ylabel("Time of Flight (s)")
    ax.set_xlabel("Distance to Target (m)")
    ax.set_title("ToF & Apex Height vs Distance")
    ax.legend(["ToF"], loc="upper left")
    ax2.legend(["Apex Height"], loc="upper right")

    fig.tight_layout()
    save_fig(fig, "fig1_distance_sweep.png")


# ── Figure 2: Motion Compensation ──────────────────────────────────────────

SCENARIO_LABELS = {
    "approaching": "Approaching (vx = −2 m/s)",
    "retreating": "Retreating (vx = +2 m/s)",
    "strafing": "Strafing (vy = +2 m/s)",
    "rotating": "Rotating (ω = 90°/s)",
    "combined": "Combined (vx = −1.5, vy = +1.0)",
}

SCENARIO_ORDER = ["approaching", "retreating", "strafing", "rotating", "combined"]


def figure2_motion_compensation():
    """Turret yaw, hood pitch, and RPM compensation over time for each motion scenario."""
    # Collect available scenarios
    available = []
    for name in SCENARIO_ORDER:
        df = load_csv(f"motion_{name}.csv")
        if df is not None and not df.empty:
            available.append((name, df[df["valid"] == True]))

    if not available:
        print("  [SKIP] No motion CSVs found")
        return

    n = len(available)
    fig, axes = plt.subplots(n, 3, figsize=(16, 3.2 * n + 1.5), squeeze=False)
    fig.suptitle(
        "Motion Compensation — Turret & Hood Tracking\n"
        "Blue = Compensated (solver)  ·  Red dashed = Direct Aim (no compensation)",
        fontsize=13, fontweight="bold",
    )

    for row, (name, df) in enumerate(available):
        t = df["t"]

        # Col 0: Turret Yaw
        ax = axes[row, 0]
        ax.plot(t, df["turret_yaw_deg"], color=C_PRIMARY, linewidth=1.5, label="Compensated Yaw")
        ax.plot(t, df["direct_yaw_deg"], color=C_SECONDARY, linewidth=1, linestyle="--",
                label="Direct Aim")
        if "yaw_lead_deg" in df.columns:
            ax_lead = ax.twinx()
            ax_lead.fill_between(t, 0, df["yaw_lead_deg"], alpha=0.15, color=C_ACCENT)
            ax_lead.plot(t, df["yaw_lead_deg"], color=C_ACCENT, linewidth=0.8, alpha=0.7)
            ax_lead.set_ylabel("Yaw Lead (°)", color=C_ACCENT, fontsize=8)
            ax_lead.tick_params(axis="y", labelsize=7)
        ax.set_ylabel("Turret Yaw (°)")
        if row == 0:
            ax.legend(fontsize=7, loc="best")
        ax.set_title(SCENARIO_LABELS.get(name, name), fontsize=10, fontweight="bold")

        # Col 1: Hood Pitch
        ax = axes[row, 1]
        ax.plot(t, df["hood_pitch_deg"], color=C_GREEN, linewidth=1.5)
        ax.set_ylabel("Hood Pitch (°)")
        ax.set_title("Hood Pitch", fontsize=9)

        # Col 2: Shooter RPM
        ax = axes[row, 2]
        ax.plot(t, df["shooter_rpm"], color=C_PURPLE, linewidth=1.5)
        ax.set_ylabel("Shooter RPM")
        ax.set_title("Shooter RPM", fontsize=9)
        ax.axhline(4000, color=C_GRAY, linestyle=":", alpha=0.4)

        # X labels only on bottom row
        for c in range(3):
            if row == n - 1:
                axes[row, c].set_xlabel("Time (s)")

    fig.tight_layout()
    save_fig(fig, "fig2_motion_compensation.png")


# ── Figure 3: Trajectory Views ─────────────────────────────────────────────

def figure3_trajectories():
    """XZ side-view and XY top-view of representative trajectories."""
    traj_files = sorted(glob.glob(str(DATA_DIR / "trajectory_*.csv")))
    # Filter out _meta files
    traj_files = [f for f in traj_files if "_meta" not in f]

    if not traj_files:
        print("  [SKIP] No trajectory CSVs found")
        return

    n = len(traj_files)
    fig, axes = plt.subplots(n, 2, figsize=(14, 3.5 * n + 1.0), squeeze=False)
    fig.suptitle(
        "Trajectory Views — Side (XZ) and Top (XY)\n"
        "Green ▲ = ascending  ·  Red ▼ = descending  ·  ★ = apex  ·  ◆ = target",
        fontsize=13, fontweight="bold",
    )

    for row, fpath in enumerate(traj_files):
        df = pd.read_csv(fpath)
        fname = Path(fpath).stem  # e.g. trajectory_static_5m
        label = fname.replace("trajectory_", "").replace("_", " ").title()

        # Load metadata if available
        meta_path = fpath.replace(".csv", "_meta.csv")
        meta = None
        if os.path.exists(meta_path):
            meta = pd.read_csv(meta_path)

        # Compute derived columns
        launch_x, launch_y = df["x"].iloc[0], df["y"].iloc[0]
        hdist = np.sqrt((df["x"] - launch_x) ** 2 + (df["y"] - launch_y) ** 2)

        # Find apex
        apex_idx = df["z"].idxmax()

        # ── Left: XZ Side View ──
        ax = axes[row, 0]
        # Ascending segment
        asc = df.loc[:apex_idx]
        desc = df.loc[apex_idx:]
        hdist_asc = hdist.loc[:apex_idx]
        hdist_desc = hdist.loc[apex_idx:]

        ax.plot(hdist_asc, asc["z"], color=C_GREEN, linewidth=2, label="Ascending")
        ax.plot(hdist_desc, desc["z"], color=C_SECONDARY, linewidth=2, label="Descending")
        ax.plot(hdist.iloc[apex_idx], df["z"].iloc[apex_idx], marker="*", markersize=12,
                color=C_ACCENT, zorder=5, label=f"Apex ({df['z'].iloc[apex_idx]:.2f} m)")

        # Target line
        target_dist = np.sqrt(
            (meta["robot_x"].iloc[0]) ** 2 + (meta["robot_y"].iloc[0]) ** 2
        ) if meta is not None else hdist.iloc[-1]
        ax.axhline(TARGET_HEIGHT_M, color=C_GRAY, linestyle=":", alpha=0.5)
        ax.axvline(target_dist, color=C_GRAY, linestyle=":", alpha=0.5)
        ax.plot(target_dist, TARGET_HEIGHT_M, marker="D", markersize=10, color=C_SECONDARY,
                zorder=5, label="Target")

        # Ground
        ax.axhline(0, color="black", linewidth=0.5)
        ax.fill_between([0, hdist.max() * 1.1], 0, -0.1, color="#E5E7EB", alpha=0.5)

        ax.set_ylabel("Height (m)")
        ax.set_title(f"{label} — Side View (XZ)", fontsize=10, fontweight="bold")
        ax.legend(fontsize=7, loc="upper right")
        ax.set_xlim(0, hdist.max() * 1.1)
        ax.set_ylim(-0.15, df["z"].max() * 1.2)
        if row == len(traj_files) - 1:
            ax.set_xlabel("Horizontal Distance (m)")

        # ── Right: XY Top View ──
        ax = axes[row, 1]
        ax.plot(df["x"], df["y"], color=C_PRIMARY, linewidth=1.5, alpha=0.8)
        ax.plot(df["x"].iloc[0], df["y"].iloc[0], "o", markersize=8, color=C_GREEN,
                label="Launch", zorder=5)
        ax.plot(0, 0, "D", markersize=10, color=C_SECONDARY, label="Target", zorder=5)

        # Draw hexagon at target
        hex_patch = RegularPolygon(
            (0, 0), numVertices=6,
            radius=HEXAGON_RADIUS_IN * 0.0254,
            orientation=0,
            facecolor="none", edgecolor=C_SECONDARY, linewidth=1.5, linestyle="--",
        )
        ax.add_patch(hex_patch)

        # Robot velocity arrow
        if meta is not None and (abs(meta["robot_vx"].iloc[0]) > 0.01 or
                                  abs(meta["robot_vy"].iloc[0]) > 0.01):
            robot_x = meta["robot_x"].iloc[0]
            robot_y = meta["robot_y"].iloc[0]
            vx = meta["robot_vx"].iloc[0]
            vy = meta["robot_vy"].iloc[0]
            ax.annotate(
                "", xy=(robot_x + vx * 0.5, robot_y + vy * 0.5),
                xytext=(robot_x, robot_y),
                arrowprops=dict(arrowstyle="->", color=C_ACCENT, lw=2),
            )
            ax.text(robot_x + vx * 0.55, robot_y + vy * 0.55,
                    f"v=({vx:.1f},{vy:.1f})", fontsize=7, color=C_ACCENT)

        ax.set_ylabel("Y (m)")
        ax.set_title(f"{label} — Top View (XY)", fontsize=10, fontweight="bold")
        ax.set_aspect("equal", adjustable="datalim")
        ax.legend(fontsize=7, loc="best")
        if row == len(traj_files) - 1:
            ax.set_xlabel("X (m)")

    fig.tight_layout()
    save_fig(fig, "fig3_trajectories.png")


# ── Figure 4: Sensitivity Analysis ─────────────────────────────────────────

def figure4_sensitivity():
    """Impact of pitch/velocity perturbation on miss distance."""
    df = load_csv("sensitivity.csv")
    if df is None:
        return

    # Exclude the optimal row (0,0 offset)
    dfp = df[(df["pitch_offset_deg"].abs() > 0.01) | (df["velocity_offset_pct"].abs() > 0.01)]

    if dfp.empty:
        print("  [SKIP] Not enough sensitivity data")
        return

    fig = plt.figure(figsize=(15, 10))
    fig.suptitle(
        "Sensitivity Analysis — Shot Tolerance @ 5 m\n"
        "How pitch and velocity errors affect where the ball lands",
        fontsize=13, fontweight="bold",
    )
    gs = GridSpec(2, 3, figure=fig, hspace=0.35, wspace=0.35)

    # (0,0) Vertical miss vs pitch offset (at 0% velocity)
    ax = fig.add_subplot(gs[0, 0])
    mask = dfp["velocity_offset_pct"].abs() < 0.5
    sub = dfp[mask].sort_values("pitch_offset_deg")
    if not sub.empty:
        ax.plot(sub["pitch_offset_deg"], sub["vert_miss_m"] * 100, color=C_PRIMARY, linewidth=2)
        ax.axhline(0, color=C_GRAY, linewidth=0.5)
        ax.axhspan(-5, 5, color=C_GREEN, alpha=0.1, label="±5 cm")
    ax.set_xlabel("Pitch Offset (°)")
    ax.set_ylabel("Vertical Miss (cm)")
    ax.set_title("Pitch Error → Vertical Miss")
    ax.legend(fontsize=7)

    # (0,1) Vertical miss vs velocity offset (at 0° pitch)
    ax = fig.add_subplot(gs[0, 1])
    mask = dfp["pitch_offset_deg"].abs() < 0.1
    sub = dfp[mask].sort_values("velocity_offset_pct")
    if not sub.empty:
        ax.plot(sub["velocity_offset_pct"], sub["vert_miss_m"] * 100, color=C_SECONDARY,
                linewidth=2)
        ax.axhline(0, color=C_GRAY, linewidth=0.5)
        ax.axhspan(-5, 5, color=C_GREEN, alpha=0.1, label="±5 cm")
    ax.set_xlabel("Velocity Offset (%)")
    ax.set_ylabel("Vertical Miss (cm)")
    ax.set_title("Velocity Error → Vertical Miss")
    ax.legend(fontsize=7)

    # (0,2) 2D heatmap: pitch × velocity → |miss|
    ax = fig.add_subplot(gs[0, 2])
    pivot = dfp.pivot_table(
        index="pitch_offset_deg",
        columns="velocity_offset_pct",
        values="vert_miss_m",
        aggfunc="first",
    )
    if not pivot.empty:
        im = ax.imshow(
            np.abs(pivot.values) * 100,
            extent=[
                pivot.columns.min(), pivot.columns.max(),
                pivot.index.min(), pivot.index.max(),
            ],
            origin="lower", aspect="auto", cmap="RdYlGn_r",
            vmin=0, vmax=50,
        )
        plt.colorbar(im, ax=ax, label="| Vertical Miss | (cm)")
        # Contour at 5 cm
        ax.contour(
            pivot.columns, pivot.index, np.abs(pivot.values) * 100,
            levels=[5], colors=["white"], linewidths=[2],
        )
    ax.set_xlabel("Velocity Offset (%)")
    ax.set_ylabel("Pitch Offset (°)")
    ax.set_title("Combined Error → |Miss| Heatmap")

    # (1, 0:2) Hexagon impact scatter
    ax = fig.add_subplot(gs[1, :])
    hex_patch = RegularPolygon(
        (0, 0), numVertices=6,
        radius=HEXAGON_RADIUS_IN * 0.0254,
        orientation=0,
        facecolor="#F3F4F6", edgecolor=C_SECONDARY, linewidth=2,
    )
    ax.add_patch(hex_patch)

    # Scatter: colour = pitch offset, size = velocity offset
    sc = ax.scatter(
        dfp["impact_y_m"],  # lateral
        dfp["impact_z_m"] - TARGET_HEIGHT_M,  # vertical relative to target center
        c=dfp["pitch_offset_deg"],
        cmap="coolwarm", s=15, alpha=0.6, edgecolors="none",
    )
    plt.colorbar(sc, ax=ax, label="Pitch Offset (°)")

    # Optimal point
    opt = df[(df["pitch_offset_deg"].abs() < 0.01) & (df["velocity_offset_pct"].abs() < 0.01)]
    if not opt.empty:
        ax.plot(opt["impact_y_m"].iloc[0], (opt["impact_z_m"].iloc[0] - TARGET_HEIGHT_M),
                "*", markersize=15, color=C_ACCENT, zorder=5, label="Optimal")

    ax.set_xlabel("Lateral Deviation at Target (m)")
    ax.set_ylabel("Vertical Deviation from Target Center (m)")
    ax.set_title("Impact Point Distribution on Target (Hexagon)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.legend(loc="upper right")

    save_fig(fig, "fig4_sensitivity.png")


# ── Figure 5: Hit Zone Map ─────────────────────────────────────────────────

def figure5_hitzone():
    """Field coverage map: which positions can score and required RPM."""
    df = load_csv("hitzone.csv")
    if df is None:
        return

    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    fig.suptitle(
        "Field Coverage — Hit Zone Map (static shots)\n"
        "Robot positions where the solver finds a valid descending-arc solution",
        fontsize=13, fontweight="bold",
    )

    valid = df[df["valid"] == True]
    invalid = df[df["valid"] == False]

    # (0) RPM heatmap on field
    ax = axes[0]
    if not valid.empty:
        sc = ax.scatter(
            valid["robot_x"], valid["robot_y"],
            c=valid["rpm"], cmap="viridis", s=40, edgecolors="none",
        )
        plt.colorbar(sc, ax=ax, label="Shooter RPM")
    if not invalid.empty:
        ax.scatter(
            invalid["robot_x"], invalid["robot_y"],
            c="#E5E7EB", s=20, marker="x", alpha=0.5, label="No Solution",
        )
    ax.plot(0, 0, "D", markersize=12, color=C_SECONDARY, zorder=5, label="Target")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Required RPM by Position")
    ax.set_aspect("equal")
    ax.legend(fontsize=7)

    # (1) Pitch heatmap
    ax = axes[1]
    if not valid.empty:
        sc = ax.scatter(
            valid["robot_x"], valid["robot_y"],
            c=valid["pitch_deg"], cmap="plasma", s=40, edgecolors="none",
        )
        plt.colorbar(sc, ax=ax, label="Hood Pitch (°)")
    if not invalid.empty:
        ax.scatter(
            invalid["robot_x"], invalid["robot_y"],
            c="#E5E7EB", s=20, marker="x", alpha=0.5,
        )
    ax.plot(0, 0, "D", markersize=12, color=C_SECONDARY, zorder=5)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Required Hood Pitch by Position")
    ax.set_aspect("equal")

    # (2) ToF heatmap
    ax = axes[2]
    if not valid.empty:
        sc = ax.scatter(
            valid["robot_x"], valid["robot_y"],
            c=valid["tof_s"], cmap="cividis", s=40, edgecolors="none",
        )
        plt.colorbar(sc, ax=ax, label="Time of Flight (s)")
    if not invalid.empty:
        ax.scatter(
            invalid["robot_x"], invalid["robot_y"],
            c="#E5E7EB", s=20, marker="x", alpha=0.5,
        )
    ax.plot(0, 0, "D", markersize=12, color=C_SECONDARY, zorder=5)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Time of Flight by Position")
    ax.set_aspect("equal")

    fig.tight_layout()
    save_fig(fig, "fig5_hitzone.png")


# ══════════════════════════════════════════════════════════════════════════════
# Python-Side Physics Engine (for --generate mode)
# Matches C++ BallisticSolver exactly: RK4 integration, Gravity + Drag + Magnus,
# minimum-pitch solver, hexagon boundary, iterative yaw lead.
# ══════════════════════════════════════════════════════════════════════════════

# Physical constants — must match C++ BallisticSolver.h
GRAVITY      = 9.80665            # m/s^2  (CODATA 2018)
AIR_DENSITY  = 1.225              # kg/m^3 (ISA sea level)

# Projectile — 2026 game piece
PROJ_MASS    = 0.215              # kg
PROJ_DIAM    = 0.150              # m
PROJ_CD      = 0.485              # drag coefficient
PROJ_CM      = 0.50               # Magnus coefficient
PROJ_MOI     = 0.0024             # moment of inertia  kg·m^2
PROJ_SDECAY  = 0.001              # spin decay coefficient
PROJ_AREA    = math.pi * (PROJ_DIAM / 2) ** 2
PROJ_R       = PROJ_DIAM / 2

# Shooter — 4" Orange Stealth (40A) + backplate
WHEEL_RADIUS = 0.0508             # m (2" radius)
SLIP_FACTOR  = 0.60
EFF_RADIUS   = WHEEL_RADIUS * SLIP_FACTOR
MAX_RPM      = 4000.0
TURRET_HEIGHT = 0.50              # m
PITCH_MIN    = 0.1                # rad  (~5.7°)  — match C++ default
PITCH_MAX    = 1.2                # rad  (~68.8°) — match C++ default
TURRET_RATE  = 6.0                # rad/s max turret slew
SPIN_RPM     = 0.0                # backspin RPM (0 = no Magnus, matches C++ default)

RPM_TO_RADPS = 2 * math.pi / 60
INCH_TO_M    = 0.0254

# Pre-computed force constants
_drag_k   = 0.5 * AIR_DENSITY * PROJ_CD * PROJ_AREA / PROJ_MASS
_magnus_k = 0.5 * AIR_DENSITY * PROJ_CM * PROJ_AREA * PROJ_R / PROJ_MASS


def _rpm_to_vel(rpm):
    return rpm * RPM_TO_RADPS * EFF_RADIUS

def _vel_to_rpm(vel):
    return vel / (RPM_TO_RADPS * EFF_RADIUS)


# ── Forces (match BallisticSolver.cpp:computeAcceleration) ──────────────────

def _accel(vx, vy, vz, ox, oy, oz):
    """Returns (ax, ay, az, dox, doy, doz)."""
    vm = math.sqrt(vx*vx + vy*vy + vz*vz)
    # Drag: -0.5 * rho * Cd * A * |v| * v / m
    df = -_drag_k * vm if vm > 1e-6 else 0.0
    ax = df * vx
    ay = df * vy
    az = -GRAVITY + df * vz
    # Magnus: S0 * (omega x v) / m   (only when spin > 0)
    if abs(ox) + abs(oy) + abs(oz) > 1e-8:
        ax += _magnus_k * (oy*vz - oz*vy)
        ay += _magnus_k * (oz*vx - ox*vz)
        az += _magnus_k * (ox*vy - oy*vx)
    # Spin decay: -C * |omega| * omega / I
    om = math.sqrt(ox*ox + oy*oy + oz*oz)
    if om > 1e-6:
        sd = -PROJ_SDECAY * om / PROJ_MOI
        dox, doy, doz = sd*ox, sd*oy, sd*oz
    else:
        dox = doy = doz = 0.0
    return (ax, ay, az, dox, doy, doz)


# ── RK4 integration (match BallisticSolver.cpp:rk4Step) ─────────────────────

def _rk4_sim(yaw, pitch, velocity, rx, ry, vx_robot, vy_robot, dt=0.005):
    """RK4 trajectory simulation.  Returns [(t, x, y, z, vx, vy, vz), ...]."""
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    # Initial velocity = exit + robot  (match computeInitialVelocity)
    vx = velocity * cp * cy + vx_robot
    vy = velocity * cp * sy + vy_robot
    vz = velocity * sp
    # Initial spin  (match simulateTrajectory lines 428-434)
    spin_rps = SPIN_RPM * RPM_TO_RADPS
    ox, oy, oz = -sy * spin_rps, cy * spin_rps, 0.0

    x, y, z = rx, ry, TURRET_HEIGHT
    pts = []
    t = 0.0
    max_steps = int(5.0 / dt) + 1
    for _ in range(max_steps):
        pts.append((t, x, y, z, vx, vy, vz))
        # k1
        a1 = _accel(vx, vy, vz, ox, oy, oz)
        dt2 = dt * 0.5
        # k2
        a2 = _accel(vx+dt2*a1[0], vy+dt2*a1[1], vz+dt2*a1[2],
                     ox+dt2*a1[3], oy+dt2*a1[4], oz+dt2*a1[5])
        # k3
        a3 = _accel(vx+dt2*a2[0], vy+dt2*a2[1], vz+dt2*a2[2],
                     ox+dt2*a2[3], oy+dt2*a2[4], oz+dt2*a2[5])
        # k4
        a4 = _accel(vx+dt*a3[0], vy+dt*a3[1], vz+dt*a3[2],
                     ox+dt*a3[3], oy+dt*a3[4], oz+dt*a3[5])
        dt6 = dt / 6.0
        # Position update:  dx/dt = v  →  use velocity RK4
        # k1_pos=v, k2_pos=v+dt/2*a1, k3_pos=v+dt/2*a2, k4_pos=v+dt*a3
        x += dt6 * (vx + 2*(vx+dt2*a1[0]) + 2*(vx+dt2*a2[0]) + (vx+dt*a3[0]))
        y += dt6 * (vy + 2*(vy+dt2*a1[1]) + 2*(vy+dt2*a2[1]) + (vy+dt*a3[1]))
        z += dt6 * (vz + 2*(vz+dt2*a1[2]) + 2*(vz+dt2*a2[2]) + (vz+dt*a3[2]))
        # Velocity update
        vx += dt6 * (a1[0] + 2*a2[0] + 2*a3[0] + a4[0])
        vy += dt6 * (a1[1] + 2*a2[1] + 2*a3[1] + a4[1])
        vz += dt6 * (a1[2] + 2*a2[2] + 2*a3[2] + a4[2])
        # Spin update
        ox += dt6 * (a1[3] + 2*a2[3] + 2*a3[3] + a4[3])
        oy += dt6 * (a1[4] + 2*a2[4] + 2*a3[4] + a4[4])
        oz += dt6 * (a1[5] + 2*a2[5] + 2*a3[5] + a4[5])
        t += dt
        if z <= 0 and t > 0.01:
            break
    return pts


def simulate_trajectory(yaw, pitch, velocity, rx, ry, vx_robot, vy_robot, dt=0.005):
    """Public API for display trajectories (full accuracy)."""
    return _rk4_sim(yaw, pitch, velocity, rx, ry, vx_robot, vy_robot, dt)


# ── Trajectory queries ──────────────────────────────────────────────────────

def _find_z_at_dist(pts, launch_x, launch_y, target_dist):
    """Find (z, vz, tof) where ball crosses target_dist.  Prefers descending arc."""
    best_desc = None
    for i in range(1, len(pts)):
        d  = math.hypot(pts[i][1] - launch_x, pts[i][2] - launch_y)
        dp = math.hypot(pts[i-1][1] - launch_x, pts[i-1][2] - launch_y)
        if d >= target_dist and dp < target_dist:
            a = (target_dist - dp) / (d - dp) if d != dp else 0
            z_at  = pts[i-1][3] + a * (pts[i][3] - pts[i-1][3])
            vz_at = pts[i-1][6] + a * (pts[i][6] - pts[i-1][6])
            tof   = pts[i-1][0] + a * (pts[i][0] - pts[i-1][0])
            if vz_at < 0:                         # descending crossing — preferred
                return z_at, vz_at, tof
            elif best_desc is None:                # ascending fallback
                best_desc = (z_at, vz_at, tof)
    return best_desc  # may be None


# ── Hexagon boundary (match BallisticSolver.cpp:isInsideHexagon) ────────────

_SQRT3 = 1.7320508075688772
_HEX_Y  = 20.966     # inches
_HEX_D  = 41.932     # inches

def _inside_hexagon(x_m, y_m):
    """Check if impact (meters, relative to target center) is inside hexagon."""
    xi, yi = x_m / INCH_TO_M, y_m / INCH_TO_M
    if yi < -_HEX_Y or yi > _HEX_Y: return False
    if -_SQRT3*xi + yi > _HEX_D:    return False
    if  _SQRT3*xi + yi > _HEX_D:    return False
    if -_SQRT3*xi - yi > _HEX_D:    return False
    if  _SQRT3*xi - yi > _HEX_D:    return False
    return True


# ── Solver: find velocity for a given pitch ─────────────────────────────────

def _find_velocity_for_pitch(yaw, pitch, rx, ry, dist):
    """Binary-search velocity.  Returns (vel, tof, z_at) or None.  Matches C++ findVelocityForPitch."""
    v_lo = _rpm_to_vel(0)
    v_hi = _rpm_to_vel(MAX_RPM)
    best = None
    for _ in range(20):
        vm = (v_lo + v_hi) / 2
        pts = _rk4_sim(yaw, pitch, vm, rx, ry, 0, 0, dt=0.008)
        res = _find_z_at_dist(pts, pts[0][1], pts[0][2], dist)
        if res is None:
            v_lo = vm; continue
        z_at, vz_at, tof = res
        if vz_at >= 0:
            v_hi = vm; continue              # ascending → too fast
        err = z_at - TARGET_HEIGHT_M
        if abs(err) < 0.05:                  # 5 cm tolerance (match C++)
            best = (vm, tof, z_at)
            v_hi = vm                        # try lower
        elif err > 0: v_hi = vm
        else:         v_lo = vm
    return best


# ── Solver: minimum-pitch search (match BallisticSolver.cpp:solve) ──────────

def _solve_static(robot_x, robot_y):
    """Outer binary-search for LOWEST valid pitch.  Matches C++ lines 877-910."""
    dist = math.hypot(robot_x, robot_y)
    yaw = math.atan2(-robot_y, -robot_x)

    p_lo, p_hi = PITCH_MIN, PITCH_MAX
    best = None

    for _ in range(15):                      # match C++ PITCH_ITER = 15
        p_mid = (p_lo + p_hi) / 2
        result = _find_velocity_for_pitch(yaw, p_mid, robot_x, robot_y, dist)
        rpm_ok = False
        if result is not None:
            vel, tof, z_at = result
            rpm = _vel_to_rpm(vel)
            if rpm <= MAX_RPM:
                rpm_ok = True
        if rpm_ok:
            # Check hexagon (match C++ lines 949-953)
            pts = _rk4_sim(yaw, p_mid, vel, robot_x, robot_y, 0, 0, dt=0.008)
            apex = max(p[3] for p in pts)
            # Impact position approximation (ball at target distance)
            lx, ly = pts[0][1], pts[0][2]
            ix, iy = 0.0, 0.0
            for j in range(1, len(pts)):
                d  = math.hypot(pts[j][1]-lx, pts[j][2]-ly)
                dp = math.hypot(pts[j-1][1]-lx, pts[j-1][2]-ly)
                if d >= dist and dp < dist:
                    a = (dist - dp) / (d - dp) if d != dp else 0
                    ix = pts[j-1][1] + a * (pts[j][1] - pts[j-1][1])
                    iy = pts[j-1][2] + a * (pts[j][2] - pts[j-1][2])
                    break
            if not _inside_hexagon(ix, iy):
                p_lo = p_mid; continue       # outside hexagon → need higher pitch
            best = dict(pitch=p_mid, vel=vel, rpm=rpm, tof=tof,
                        yaw=yaw, apex=apex, z_at=z_at, dist=dist)
            p_hi = p_mid                     # valid → try flatter
        else:
            p_lo = p_mid                     # invalid → raise pitch
    return best


# ── Yaw lead: iterative (match BallisticSolver.cpp:computeYawWithLead) ──────

def _compute_yaw_lead(rx, ry, vx, vy, pitch, velocity):
    """Iterative yaw-lead refinement (up to 5 iterations).  Includes turret travel."""
    dx, dy = -rx, -ry
    yaw = math.atan2(dy, dx)
    v_horiz = max(velocity * math.cos(pitch), 1.0)

    for _ in range(5):
        distance = math.hypot(dx, dy)
        tof_est = distance / v_horiz
        # Turret travel time (bang-bang model, match C++ computeTurretTravelTime)
        direct = math.atan2(-ry, -rx)
        delta = yaw - direct
        while delta >  math.pi: delta -= 2*math.pi
        while delta < -math.pi: delta += 2*math.pi
        turret_travel = abs(delta) / TURRET_RATE
        total_delay = turret_travel
        # Predict robot position at launch + ToF
        pred_x = rx + vx * (total_delay + tof_est)
        pred_y = ry + vy * (total_delay + tof_est)
        dx, dy = -pred_x, -pred_y
        new_yaw = math.atan2(dy, dx)
        if abs(new_yaw - yaw) < 1e-4:
            return new_yaw
        yaw = new_yaw
    return yaw


def _solve_moving(robot_x, robot_y, vx, vy, omega, heading):
    """Moving solver: static solve + iterative yaw-lead.  Matches C++ solve() flow."""
    sol = _solve_static(robot_x, robot_y)
    if sol is None:
        return None
    compensated_yaw = _compute_yaw_lead(
        robot_x, robot_y, vx, vy, sol["pitch"], sol["vel"])
    direct_yaw = math.atan2(-robot_y, -robot_x)
    sol["yaw"] = compensated_yaw
    sol["yaw_lead"] = compensated_yaw - direct_yaw
    sol["direct_yaw"] = direct_yaw
    return sol


def generate_test_data():
    """Generate all CSV test data using Python physics engine."""
    out = DATA_DIR
    out.mkdir(parents=True, exist_ok=True)
    print(f"  Generating test data in {out}/\n")

    # ── 1. Distance Sweep ──
    print("  [1/4] Distance Sweep...")
    rows = []
    for d100 in range(200, 1001, 25):
        d = d100 / 100.0
        sol = _solve_static(d, 0.0)
        if sol:
            rows.append(dict(
                distance_m=d, pitch_deg=math.degrees(sol["pitch"]),
                velocity_mps=sol["vel"], rpm=sol["rpm"], tof_s=sol["tof"],
                apex_height_m=sol["apex"], impact_z_m=sol["z_at"],
                impact_x_m=0.0, impact_y_m=0.0, valid=True,
            ))
        else:
            rows.append(dict(
                distance_m=d, pitch_deg=float("nan"),
                velocity_mps=float("nan"), rpm=float("nan"), tof_s=float("nan"),
                apex_height_m=float("nan"), impact_z_m=float("nan"),
                impact_x_m=float("nan"), impact_y_m=float("nan"), valid=False,
            ))
    pd.DataFrame(rows).to_csv(out / "distance_sweep.csv", index=False)
    print(f"    → {out / 'distance_sweep.csv'}")

    # ── 2. Motion Compensation ──
    print("  [2/4] Motion Compensation...")
    scenarios = [
        ("approaching", 8.0, 0.0, -2.0, 0.0, 0.0, math.pi),
        ("retreating", 4.0, 0.0, 2.0, 0.0, 0.0, 0.0),
        ("strafing", 5.0, -4.0, 0.0, 2.0, 0.0, 0.0),
        ("rotating", 5.0, 2.0, 0.0, 0.0, math.radians(90), 0.0),
        ("combined", 7.0, -3.0, -1.5, 1.0, 0.0, math.pi),
    ]
    for name, sx, sy, svx, svy, omega, heading in scenarios:
        rows = []
        for ti in range(0, 101):  # 0 to 5s at 50ms
            t = ti * 0.05
            rx = sx + svx * t
            ry = sy + svy * t
            h = heading + omega * t
            dist = math.sqrt(rx**2 + ry**2)
            direct_yaw = math.atan2(-ry, -rx)
            sol = _solve_moving(rx, ry, svx, svy, omega, h)
            if sol:
                rows.append(dict(
                    t=round(t, 4), robot_x=round(rx, 4), robot_y=round(ry, 4),
                    robot_vx=svx, robot_vy=svy, heading_deg=round(math.degrees(h), 2),
                    distance_m=round(dist, 4),
                    turret_yaw_deg=round(math.degrees(sol["yaw"]), 4),
                    direct_yaw_deg=round(math.degrees(direct_yaw), 4),
                    yaw_lead_deg=round(math.degrees(sol["yaw_lead"]), 4),
                    hood_pitch_deg=round(math.degrees(sol["pitch"]), 4),
                    shooter_rpm=round(sol["rpm"], 1),
                    velocity_mps=round(sol["vel"], 4),
                    tof_s=round(sol["tof"], 4),
                    valid=True,
                ))
            else:
                rows.append(dict(
                    t=round(t, 4), robot_x=round(rx, 4), robot_y=round(ry, 4),
                    robot_vx=svx, robot_vy=svy, heading_deg=round(math.degrees(h), 2),
                    distance_m=round(dist, 4),
                    turret_yaw_deg=float("nan"), direct_yaw_deg=round(math.degrees(direct_yaw), 4),
                    yaw_lead_deg=float("nan"),
                    hood_pitch_deg=float("nan"), shooter_rpm=float("nan"),
                    velocity_mps=float("nan"), tof_s=float("nan"), valid=False,
                ))
        pd.DataFrame(rows).to_csv(out / f"motion_{name}.csv", index=False)
        print(f"    → motion_{name}.csv")

    # Trajectory CSVs for Figure 3
    for label, rx, ry, vx, vy, omega, heading in [
        ("trajectory_static_5m", 5.0, 0.0, 0, 0, 0, 0),
        ("trajectory_static_8m", 8.0, 0.0, 0, 0, 0, 0),
        ("trajectory_approaching", 6.0, 0.0, -2.0, 0.0, 0, math.pi),
        ("trajectory_strafing", 5.0, 2.0, 0.0, -2.0, 0, 0),
    ]:
        sol = _solve_moving(rx, ry, vx, vy, omega, heading) if (vx or vy) else _solve_static(rx, ry)
        if sol is None:
            continue
        pts = simulate_trajectory(sol["yaw"], sol["pitch"], sol["vel"], rx, ry, vx, vy)
        lx, ly = pts[0][1], pts[0][2]
        trows = []
        for p in pts:
            hdist = math.sqrt((p[1] - lx)**2 + (p[2] - ly)**2)
            speed = math.sqrt(p[4]**2 + p[5]**2 + p[6]**2)
            trows.append(dict(t=p[0], x=p[1], y=p[2], z=p[3],
                              vx=p[4], vy=p[5], vz=p[6],
                              horiz_dist=hdist, speed=speed))
        pd.DataFrame(trows).to_csv(out / f"{label}.csv", index=False)
        # Metadata
        meta = pd.DataFrame([dict(
            robot_x=rx, robot_y=ry, robot_vx=vx, robot_vy=vy,
            yaw_deg=math.degrees(sol["yaw"]), pitch_deg=math.degrees(sol["pitch"]),
            velocity_mps=sol["vel"], rpm=sol["rpm"], tof_s=sol["tof"],
            yaw_lead_deg=math.degrees(sol.get("yaw_lead", 0)), target_z=TARGET_HEIGHT_M,
        )])
        meta.to_csv(out / f"{label}_meta.csv", index=False)
        print(f"    → {label}.csv")

    # ── 3. Sensitivity ──
    print("  [3/4] Sensitivity Analysis...")
    sol = _solve_static(5.0, 0.0)
    if sol:
        rows = []
        opt_pitch = sol["pitch"]
        opt_vel = sol["vel"]
        opt_yaw = sol["yaw"]
        # Optimal row
        rows.append(dict(
            pitch_offset_deg=0, velocity_offset_pct=0,
            pitch_deg=math.degrees(opt_pitch), velocity_mps=opt_vel,
            impact_x_m=0.0, impact_y_m=0.0, impact_z_m=TARGET_HEIGHT_M,
            horiz_miss_m=0, vert_miss_m=0,
            opt_pitch_deg=math.degrees(opt_pitch), opt_velocity_mps=opt_vel, distance_m=5.0,
        ))
        for p_off_10 in range(-30, 31, 5):  # -3.0 to +3.0 in 0.5° steps
            p_off = p_off_10 / 10.0
            for v_off in range(-10, 11, 2):  # -10% to +10% in 2% steps
                if abs(p_off) < 0.05 and abs(v_off) < 0.05:
                    continue
                tp = opt_pitch + math.radians(p_off)
                tv = opt_vel * (1 + v_off / 100.0)
                pts = _rk4_sim(opt_yaw, tp, tv, 5.0, 0.0, 0, 0)
                res = _find_z_at_dist(pts, pts[0][1], pts[0][2], 5.0)
                if res is None:
                    continue
                z_at = res[0]
                # interpolate impact position
                lx, ly = pts[0][1], pts[0][2]
                ix, iy = 0.0, 0.0
                for i in range(1, len(pts)):
                    d = math.sqrt((pts[i][1] - lx)**2 + (pts[i][2] - ly)**2)
                    dp = math.sqrt((pts[i-1][1] - lx)**2 + (pts[i-1][2] - ly)**2)
                    if d >= 5.0 and dp < 5.0:
                        alpha = (5.0 - dp) / (d - dp) if d != dp else 0
                        ix = pts[i-1][1] + alpha * (pts[i][1] - pts[i-1][1])
                        iy = pts[i-1][2] + alpha * (pts[i][2] - pts[i-1][2])
                        break
                rows.append(dict(
                    pitch_offset_deg=p_off, velocity_offset_pct=v_off,
                    pitch_deg=math.degrees(tp), velocity_mps=tv,
                    impact_x_m=ix, impact_y_m=iy, impact_z_m=z_at,
                    horiz_miss_m=math.sqrt(ix**2 + iy**2), vert_miss_m=z_at - TARGET_HEIGHT_M,
                    opt_pitch_deg=math.degrees(opt_pitch), opt_velocity_mps=opt_vel, distance_m=5.0,
                ))
        pd.DataFrame(rows).to_csv(out / "sensitivity.csv", index=False)
        print(f"    → sensitivity.csv")

    # ── 4. Hit Zone ──
    print("  [4/4] Hit Zone Map...")
    rows = []
    for rx_10 in range(20, 91, 10):  # 2.0 to 9.0 step 1.0
        rx = rx_10 / 10.0
        for ry_10 in range(-50, 51, 10):  # -5.0 to 5.0 step 1.0
            ry = ry_10 / 10.0
            dist = math.sqrt(rx**2 + ry**2)
            sol = _solve_static(rx, ry)
            if sol:
                rows.append(dict(
                    robot_x=rx, robot_y=ry, distance_m=round(dist, 4), valid=True,
                    pitch_deg=round(math.degrees(sol["pitch"]), 4),
                    velocity_mps=round(sol["vel"], 4), rpm=round(sol["rpm"], 1),
                    tof_s=round(sol["tof"], 4),
                    impact_x_m=0.0, impact_y_m=0.0, impact_z_m=round(sol["z_at"], 4),
                    yaw_deg=round(math.degrees(sol["yaw"]), 4),
                ))
            else:
                rows.append(dict(
                    robot_x=rx, robot_y=ry, distance_m=round(dist, 4), valid=False,
                    pitch_deg=float("nan"), velocity_mps=float("nan"),
                    rpm=float("nan"), tof_s=float("nan"),
                    impact_x_m=float("nan"), impact_y_m=float("nan"),
                    impact_z_m=float("nan"), yaw_deg=float("nan"),
                ))
    pd.DataFrame(rows).to_csv(out / "hitzone.csv", index=False)
    print(f"    → hitzone.csv")

    print("\n  Test data generation complete.")


# ── Main ────────────────────────────────────────────────────────────────────

def main():
    global DATA_DIR, OUT_DIR

    generate_mode = "--generate" in sys.argv

    if generate_mode:
        sys.argv.remove("--generate")

    if len(sys.argv) > 1:
        DATA_DIR = Path(sys.argv[1])
        OUT_DIR = DATA_DIR

    print("=" * 60)
    print("  Ballistic Solver Report — Visualization")
    print("  Team 9427 · 2026")
    print(f"  Data dir: {DATA_DIR}")
    print("=" * 60)

    # Generate test data if requested or if no CSVs exist
    if generate_mode or not (DATA_DIR / "distance_sweep.csv").exists():
        if not generate_mode:
            print(f"\n  No CSV data found in {DATA_DIR}/.")
            print("  Generating test data with Python physics engine...")
            print("  (Run BallisticReport.java for real C++ solver data)\n")
        else:
            print("\n  --generate mode: creating test data with Python physics\n")
        generate_test_data()

    if not DATA_DIR.exists():
        print(f"\nERROR: {DATA_DIR} does not exist.")
        sys.exit(1)

    print("\n[1/5] Figure 1: Distance Sweep")
    figure1_distance_sweep()

    print("[2/5] Figure 2: Motion Compensation")
    figure2_motion_compensation()

    print("[3/5] Figure 3: Trajectory Views")
    figure3_trajectories()

    print("[4/5] Figure 4: Sensitivity Analysis")
    figure4_sensitivity()

    print("[5/5] Figure 5: Hit Zone Map")
    figure5_hitzone()

    print("\n" + "=" * 60)
    print(f"  All figures saved to: {OUT_DIR}/")
    print("=" * 60)


if __name__ == "__main__":
    main()
