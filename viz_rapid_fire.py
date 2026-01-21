import matplotlib

matplotlib.use("Agg")  # Force non-interactive backend
import pandas as pd
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import numpy as np
import sys
import os
import glob
from PIL import Image


def run_viz(scenario_name, view_mode, output_filename):
    print(f"Processing {scenario_name}...")

    # Load Data
    try:
        robot_df = pd.read_csv(f"{scenario_name}_robot.csv")
        proj_df = pd.read_csv(f"{scenario_name}_projectiles.csv")
    except FileNotFoundError:
        print(f"Error: CSV files for {scenario_name} not found.")
        return

    # Create frames directory
    frames_dir = f"temp_frames_{scenario_name}"
    if not os.path.exists(frames_dir):
        os.makedirs(frames_dir)
    else:
        # Clean up existing
        for f in glob.glob(f"{frames_dir}/*.png"):
            os.remove(f)

    # Setup Figure
    fig = plt.figure(figsize=(10, 8))
    # Standard 3D axes creation
    ax = fig.add_subplot(111, projection="3d")

    # Static Setup
    ax.set_xlim(-5, 25)
    ax.set_ylim(-15, 15)
    ax.set_zlim(0, 5)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")

    # View Settings
    if view_mode == "TOP":
        # User requested "Isometric" style similar to Continuous Tracking
        # Elev=30, Azim=-60 gives a good depth view
        ax.view_init(elev=30, azim=-60)
        ax.set_title(f"{scenario_name} (Iso View)")
    elif view_mode == "SIDE":
        # Left View (+Y)
        ax.view_init(elev=5, azim=90)
        ax.set_title(f"{scenario_name} (Side View - Left)")

    # Static Object: Hexagonal Target
    # Constraints:
    # y within [-20.966, 20.966] inches
    # +/- sqrt(3)x +/- y <= 41.932 inches
    # 1 inch = 0.0254 m
    scale = 0.0254
    h = 20.966 * scale
    C = 41.932 * scale
    sqrt3 = np.sqrt(3)

    # Vertices (calculated from intersections)
    # y = h connects to x such that sqrt(3)x + h = C => x = (C-h)/sqrt3 = h/sqrt3
    # Max x is when y=0 => sqrt(3)x = C = 2h => x = 2h/sqrt3

    v_x = [
        (C - h) / sqrt3,  # Top Right
        (C - (-h)) / sqrt3,  # Right tip (y=0?? No wait)
        # Vertices order: Top-Right, Right-Tip, Bot-Right, Bot-Left, Left-Tip, Top-Left
    ]

    # Let's just hardcode the 6 points for a regular hexagon defined by these lines
    # x1 = h/sqrt3, y1 = h
    # x2 = 2h/sqrt3, y2 = 0
    # x3 = h/sqrt3, y3 = -h
    # x4 = -h/sqrt3, y4 = -h
    # x5 = -2h/sqrt3, y5 = 0
    # x6 = -h/sqrt3, y6 = h

    hex_x = np.array(
        [
            h / sqrt3,
            2 * h / sqrt3,
            h / sqrt3,
            -h / sqrt3,
            -2 * h / sqrt3,
            -h / sqrt3,
            h / sqrt3,
        ]
    )
    hex_y = np.array([h, 0, -h, -h, 0, h, h])
    hex_z = np.full_like(hex_x, 1.8288)  # Ground height 1.83m

    # Plot Hexagon Frame (Red)
    ax.plot(hex_x, hex_y, hex_z, color="red", linewidth=3, label="Target Hex")

    # Center Cross
    ax.plot([-h, h], [0, 0], [1.8288, 1.8288], color="red", linewidth=1, linestyle="--")
    ax.plot([0, 0], [-h, h], [1.8288, 1.8288], color="red", linewidth=1, linestyle="--")

    # Initial Dynamic Objects
    (robot_line,) = ax.plot(
        [],
        [],
        [],
        marker="s",
        markersize=10,
        linestyle="None",
        color="blue",
        label="Robot",
    )

    # Turret Vector (Magenta - High Contrast)
    (turret_line,) = ax.plot(
        [], [], [], color="magenta", linewidth=4, label="Aim (Turret)"
    )

    # Line of Sight (Green Dashed - Where the target is)
    (los_line,) = ax.plot(
        [],
        [],
        [],
        color="lime",
        linewidth=1.5,
        linestyle="--",
        alpha=0.8,
        label="Direct LOS",
    )

    (proj_line,) = ax.plot(
        [],
        [],
        [],
        marker="o",
        markersize=4,
        linestyle="None",
        color="orange",
        label="Projectile",
    )

    frames_t = robot_df["t"].unique()
    # Skip frames to keep file size/time reasonable
    frames_to_render = frames_t[::3]

    # Dynamic Zoom based on Scenario
    if view_mode == "TOP":
        # Isometric View needs slightly different limits to look good
        # Robot X: 5 -> 5 (moves Y)
        # Y: 10 -> -14
        # Tighter limits as requested ("range doesn't need to be too much")
        ax.set_xlim(-2, 8)
        ax.set_ylim(-15, 12)
        ax.set_zlim(0, 5)
    elif view_mode == "SIDE":
        # Retreating: Robot X moves 2 -> 17, Y=0. Target X=0.
        ax.set_xlim(-2, 20)
        ax.set_ylim(-5, 5)

    print(f"Rendering {len(frames_to_render)} frames to {frames_dir}...")

    img_paths = []

    for i, frame_t in enumerate(frames_to_render):
        # Update Data
        # Robot
        idx = (np.abs(robot_df["t"] - frame_t)).argmin()
        r_state = robot_df.iloc[idx]
        rx, ry = r_state["x"], r_state["y"]
        rz = 0.5
        yaw = r_state["turret_yaw"]

        # Debug Frame 0
        if i == 0:
            print(f"DEBUG Frame 0: Robot pos ({rx:.2f}, {ry:.2f}, {rz}), Yaw {yaw:.2f}")

        # Robot
        robot_line.set_data([rx], [ry])
        robot_line.set_3d_properties([rz])

        # Turret
        tx = rx + 1.0 * np.cos(yaw)
        ty = ry + 1.0 * np.sin(yaw)
        turret_line.set_data([rx, tx], [ry, ty])
        turret_line.set_3d_properties([rz, rz])

        # Line of Sight (Robot -> Target center at 0,0,1.83)
        # Target coords: 0, 0, 1.8288
        los_line.set_data([rx, 0], [ry, 0])
        los_line.set_3d_properties([rz, 1.8288])

        # Calculate Angles for Annotation
        # LOS Yaw
        los_yaw = np.arctan2(0 - ry, 0 - rx)
        # Lead Angle = Turret Yaw - LOS Yaw (normalized to -pi, pi)
        lead_angle = yaw - los_yaw
        lead_angle = (lead_angle + np.pi) % (2 * np.pi) - np.pi
        lead_deg = np.degrees(lead_angle)

        # Pitch (from CSV)
        pitch_rad = r_state["pitch"] if "pitch" in r_state else 0.0
        pitch_deg = np.degrees(pitch_rad)

        # Debug Pitch
        if i % 20 == 0 and view_mode == "SIDE":
            print(f"DEBUG Frame {i}: Pitch {pitch_deg:.2f} deg")

        # Update Text Annotation (Fixed position in upper left)
        # Using text2D transform=ax.transAxes ensures it stays in place relative to the frame
        if hasattr(ax, "dynamic_annotation"):
            ax.dynamic_annotation.remove()

        annot_text = ""
        if view_mode == "TOP":
            annot_text = f"Lead Angle: {lead_deg:.1f}°"
        elif view_mode == "SIDE":
            annot_text = f"Pitch: {pitch_deg:.1f}°"

        ax.dynamic_annotation = ax.text(
            0.05,
            0.95,
            0,
            annot_text,
            transform=ax.transAxes,
            fontsize=16,
            color="black",
            weight="bold",
            bbox=dict(facecolor="white", alpha=0.8, edgecolor="none"),
            zorder=100,
        )

        # Ensure title remains static
        # ax.set_title(f"{scenario_name}")

        # Projectiles (Tolerance 0.05s)
        current_projs = proj_df[np.abs(proj_df["t"] - frame_t) < 0.05]
        if i == 0:
            print(f"DEBUG Frame 0: Found {len(current_projs)} projectiles")

        if not current_projs.empty:
            proj_line.set_data(current_projs["x"], current_projs["y"])
            proj_line.set_3d_properties(current_projs["z"])
        else:
            proj_line.set_data([], [])
            proj_line.set_3d_properties([])

        # Save Frame
        fname = f"{frames_dir}/frame_{i:04d}.png"
        fig.savefig(fname, dpi=80)
        img_paths.append(fname)

        if i % 20 == 0:
            print(f"Saved frame {i}/{len(frames_to_render)}")

    plt.close(fig)

    # Clean up and stitch logic (restored for final run)
    save_path = f"C:\\Users\\user\\.gemini\\antigravity\\brain\\e1f147c2-7a45-4905-8177-6bfe280a89d9\\{output_filename}"
    print(f"Stitching GIF to {save_path}...")

    if img_paths:
        print(f"First frame sizes: {[os.path.getsize(p) for p in img_paths[:5]]}")

        # Load images
        frames = [Image.open(f) for f in img_paths]
        # Save as GIF
        frames[0].save(
            save_path,
            format="GIF",
            append_images=frames[1:],
            save_all=True,
            duration=60,
            loop=0,
        )
        print("Done.")

        # Cleanup
        for f in frames:
            f.close()

        for f in img_paths:
            try:
                os.remove(f)
            except:
                pass
        try:
            os.rmdir(frames_dir)
        except:
            pass


if __name__ == "__main__":
    if os.path.exists("strafing_fire_robot.csv"):
        run_viz("strafing_fire", "TOP", "strafing_fire_top.gif")

    if os.path.exists("retreating_fire_robot.csv"):
        run_viz("retreating_fire", "SIDE", "retreating_fire_side.gif")
