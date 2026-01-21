"""
Premium Inertia Compensation Visualization (2D Top-Down)
Generates a side-by-side comparison: WITH vs WITHOUT compensation.
"""

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import os
import glob
from PIL import Image

# --- Configuration ---
SCENARIO = "strafing_fire"
OUTPUT_PATH = r"C:\Users\user\.gemini\antigravity\brain\e1f147c2-7a45-4905-8177-6bfe280a89d9\premium_inertia_demo.gif"
FRAMES_DIR = "temp_frames_premium"

# --- Physics Constants ---
GRAVITY = 9.81
DRAG_COEFF = 0.47
BALL_RADIUS = 0.075
BALL_MASS = 0.215
AIR_DENSITY = 1.225
DT_SIM = 0.01


def simulate_ball_2d(x0, y0, vx, vy, vz, duration=1.5):
    """Simulate ball trajectory, returning XY path (top-down projection)."""
    traj = []
    x, y, z = x0, y0, 0.5
    for _ in np.arange(0, duration, DT_SIM):
        traj.append((x, y))
        if z < 0:
            break
        v = np.sqrt(vx**2 + vy**2 + vz**2) + 1e-6
        A = np.pi * BALL_RADIUS**2
        drag = 0.5 * AIR_DENSITY * DRAG_COEFF * A * v**2 / BALL_MASS
        ax = -drag * (vx / v)
        ay = -drag * (vy / v)
        az = -GRAVITY - drag * (vz / v)
        vx += ax * DT_SIM
        vy += ay * DT_SIM
        vz += az * DT_SIM
        x += vx * DT_SIM
        y += vy * DT_SIM
        z += vz * DT_SIM
    return np.array(traj)


def main():
    print("Loading data...")
    robot_df = pd.read_csv(f"{SCENARIO}_robot.csv")
    proj_df = pd.read_csv(f"{SCENARIO}_projectiles.csv")

    if not os.path.exists(FRAMES_DIR):
        os.makedirs(FRAMES_DIR)
    else:
        for f in glob.glob(f"{FRAMES_DIR}/*.png"):
            os.remove(f)

    frames_t = robot_df["t"].unique()[::4]
    img_paths = []

    print(f"Rendering {len(frames_t)} frames (Pure 2D Top-Down)...")

    for i, t in enumerate(frames_t):
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8), facecolor="#1a1a2e")

        for ax in [ax1, ax2]:
            ax.set_facecolor("#1a1a2e")
            ax.set_xlim(-2, 8)
            ax.set_ylim(-16, 12)
            ax.set_xlabel("X (m)", color="white", fontsize=12)
            ax.set_ylabel("Y (m)", color="white", fontsize=12)
            ax.tick_params(colors="white")
            ax.set_aspect("equal")
            ax.grid(True, alpha=0.15, color="white")
            # Target
            target = plt.Circle((0, 0), 0.6, color="limegreen", fill=False, linewidth=4)
            ax.add_patch(target)
            ax.plot(0, 0, "o", color="limegreen", markersize=12)

        ax1.set_title("WITH Inertia Compensation", color="white", fontsize=16, pad=12)
        ax2.set_title("WITHOUT Compensation", color="white", fontsize=16, pad=12)

        # Get robot state
        idx = (np.abs(robot_df["t"] - t)).argmin()
        r = robot_df.iloc[idx]
        rx, ry = r["x"], r["y"]
        turret_yaw = r["turret_yaw"]
        pitch = r["pitch"] if "pitch" in r else 0.4

        # Robot velocity
        rvx, rvy = 0.0, -3.0

        # LOS and Lead Angle
        los_yaw = np.arctan2(0 - ry, 0 - rx)
        lead_angle = turret_yaw - los_yaw
        lead_angle = (lead_angle + np.pi) % (2 * np.pi) - np.pi
        lead_deg = np.degrees(lead_angle)

        # ========== LEFT PANEL (WITH Compensation) ==========
        # Robot body
        ax1.plot(rx, ry, "s", color="dodgerblue", markersize=22, zorder=10)

        # Motion arrow (Cyan) - shows direction robot is MOVING
        ax1.annotate(
            "",
            xy=(rx, ry - 2.0),
            xytext=(rx, ry),
            arrowprops=dict(arrowstyle="->", color="cyan", lw=4),
        )
        ax1.text(
            rx + 0.4,
            ry - 1.2,
            "Motion\n(Vy < 0)",
            color="cyan",
            fontsize=10,
            weight="bold",
        )

        # Turret aim (Magenta) - points LEFT of LOS to compensate
        tx = rx + 2.5 * np.cos(turret_yaw)
        ty = ry + 2.5 * np.sin(turret_yaw)
        ax1.annotate(
            "",
            xy=(tx, ty),
            xytext=(rx, ry),
            arrowprops=dict(arrowstyle="->", color="magenta", lw=5),
        )
        ax1.text(tx - 0.8, ty + 0.3, "Aim", color="magenta", fontsize=11, weight="bold")

        # LOS (Green Dashed)
        ax1.plot([rx, 0], [ry, 0], "--", color="lime", linewidth=2, alpha=0.7)
        ax1.text(rx / 2 - 0.5, ry / 2, "LOS", color="lime", fontsize=10, alpha=0.8)

        # Projectiles
        projs = proj_df[np.abs(proj_df["t"] - t) < 0.1]
        if not projs.empty:
            ax1.scatter(projs["x"], projs["y"], c="orange", s=120, marker="o", zorder=5)

        # Annotations
        ax1.text(
            0.03,
            0.95,
            f"Lead Angle: {lead_deg:.1f}deg",
            transform=ax1.transAxes,
            fontsize=16,
            color="white",
            weight="bold",
            bbox=dict(facecolor="#333", alpha=0.9, edgecolor="none", pad=5),
        )
        ax1.text(
            0.70,
            0.95,
            "HIT",
            transform=ax1.transAxes,
            fontsize=20,
            color="limegreen",
            weight="bold",
        )

        # ========== RIGHT PANEL (WITHOUT Compensation) ==========
        ax2.plot(rx, ry, "s", color="dodgerblue", markersize=22, zorder=10)
        ax2.annotate(
            "",
            xy=(rx, ry - 2.0),
            xytext=(rx, ry),
            arrowprops=dict(arrowstyle="->", color="cyan", lw=4),
        )
        ax2.text(
            rx + 0.4,
            ry - 1.2,
            "Motion\n(Vy < 0)",
            color="cyan",
            fontsize=10,
            weight="bold",
        )

        # Turret (Yellow) - aims DIRECTLY at target (no lead)
        tx_direct = rx + 2.5 * np.cos(los_yaw)
        ty_direct = ry + 2.5 * np.sin(los_yaw)
        ax2.annotate(
            "",
            xy=(tx_direct, ty_direct),
            xytext=(rx, ry),
            arrowprops=dict(arrowstyle="->", color="yellow", lw=5),
        )
        ax2.text(
            tx_direct - 0.8,
            ty_direct + 0.3,
            "Aim",
            color="yellow",
            fontsize=11,
            weight="bold",
        )
        ax2.plot([rx, 0], [ry, 0], "--", color="lime", linewidth=2, alpha=0.7)

        # Simulate missed trajectory
        v0 = 12.0
        mvx = v0 * np.cos(pitch) * np.cos(los_yaw)
        mvy = v0 * np.cos(pitch) * np.sin(los_yaw)
        mvz = v0 * np.sin(pitch)
        ballVx = mvx + rvx
        ballVy = mvy + rvy  # Drift!
        bad_traj = simulate_ball_2d(rx, ry, ballVx, ballVy, mvz)
        if len(bad_traj) > 5:
            ax2.plot(bad_traj[:, 0], bad_traj[:, 1], "--", color="red", linewidth=3)
            ax2.plot(
                bad_traj[-1, 0], bad_traj[-1, 1], "x", color="red", markersize=18, mew=4
            )
            ax2.text(
                bad_traj[-1, 0] + 0.3,
                bad_traj[-1, 1],
                "Miss!",
                color="red",
                fontsize=12,
                weight="bold",
            )

        ax2.text(
            0.03,
            0.95,
            "Lead Angle: 0.0deg (None)",
            transform=ax2.transAxes,
            fontsize=16,
            color="yellow",
            weight="bold",
            bbox=dict(facecolor="#333", alpha=0.9, edgecolor="none", pad=5),
        )
        ax2.text(
            0.70,
            0.95,
            "MISS",
            transform=ax2.transAxes,
            fontsize=20,
            color="red",
            weight="bold",
        )

        plt.tight_layout()
        fname = f"{FRAMES_DIR}/frame_{i:04d}.png"
        fig.savefig(fname, dpi=100, facecolor=fig.get_facecolor())
        img_paths.append(fname)
        plt.close(fig)

        if i % 10 == 0:
            print(f"  Frame {i}/{len(frames_t)}")

    print(f"Stitching GIF to {OUTPUT_PATH}...")
    frames = [Image.open(f) for f in img_paths]
    frames[0].save(
        OUTPUT_PATH,
        format="GIF",
        append_images=frames[1:],
        save_all=True,
        duration=80,
        loop=0,
    )
    print("Done!")

    for f in img_paths:
        try:
            os.remove(f)
        except:
            pass
    try:
        os.rmdir(FRAMES_DIR)
    except:
        pass


if __name__ == "__main__":
    main()
