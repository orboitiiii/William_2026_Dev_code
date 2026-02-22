#!/usr/bin/env python3
"""
Ballistic Solver — Animated Physics Simulation (Rapid Fire)
=============================================================
Team 9427 · 2026

Strafing + approaching robot firing 8 balls/second.

  LEFT   Top View (XY):  turret yaw-lead compensation visible
  RIGHT  Side View:      hood pitch + parabolic arcs, distance-from-target axis

Usage:
    python animate_ballistic.py              # full quality
    python animate_ballistic.py --fast       # quick preview
"""

import math
import sys
import time

import matplotlib
import numpy as np

matplotlib.use("Agg")
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Arc, Circle, FancyBboxPatch, Polygon, RegularPolygon

# ═══════════════════════════════════════════════════════════════════════════
# Physics Constants (match C++ BallisticSolver.h)
# ═══════════════════════════════════════════════════════════════════════════
GRAVITY = 9.80665
AIR_RHO = 1.225
P_MASS = 0.215
P_DIAM = 0.150
P_CD = 0.485
P_CM = 0.50
P_AREA = math.pi * (P_DIAM / 2) ** 2
P_R = P_DIAM / 2
P_MOI = 0.0024
P_SDECAY = 0.001
TURRET_H = 0.438744  # 43.8744 cm — measured turret pivot height
TARGET_H = 72.0 * 0.0254  # 1.8288 m
EFF_R = 0.0508 * 0.60  # wheel_r × slip
MAX_RPM = 5600.0
PITCH_MIN = math.radians(62.5)  # mechanical lower limit (safety margin)
PITCH_MAX = math.radians(75.0)  # mechanical upper limit
TURRET_RATE = 6.0  # rad/s
SPIN_RPM = 0.0  # 0 = no Magnus (match C++ default)
R2W = 2 * math.pi / 60
INCH2M = 0.0254

# ═══════════════════════════════════════════════════════════════════════════
# Scenario
# ═══════════════════════════════════════════════════════════════════════════
FAST = "--fast" in sys.argv
FPS = 12 if FAST else 20
DURATION = 10.0  # longer sweep to cover full 0.75–6.5m range
FIRE_RATE = 6  # balls per second
FIRE_T0 = 0.25  # first shot time
FIRE_TIMES = [
    FIRE_T0 + i / FIRE_RATE for i in range(int((DURATION - 0.4 - FIRE_T0) * FIRE_RATE))
]

COLORS5 = ["#FFFF00", "#FFD700", "#FACC15", "#EAB308", "#CA8A04"]  # Yellow shades
TRAIL_N = 6

# ── Parametric trajectory: robot sweeps 6.5m → 0.75m → 6.5m ──
D_FAR, D_NEAR = 6.5, 0.75  # distance extremes from target
D_MID = (D_FAR + D_NEAR) / 2
D_AMP = (D_FAR - D_NEAR) / 2
Y_AMP = 1.8  # slight lateral oscillation for visual interest
SHEADING = math.pi
SOMEGA = 0.0


def robot_pos(t):
    """Return (rx, ry) at time t along the oscillating path."""
    # Cosine sweep: starts at D_FAR, reaches D_NEAR at T/2, back to D_FAR
    phase = 2 * math.pi * t / DURATION
    rx = D_MID + D_AMP * math.cos(phase)
    ry = Y_AMP * math.sin(phase)  # lateral oscillation (one full cycle)
    return rx, ry


def robot_vel(t):
    """Return (vx, vy) at time t — analytical derivative of robot_pos."""
    phase = 2 * math.pi * t / DURATION
    w = 2 * math.pi / DURATION
    vx = -D_AMP * w * math.sin(phase)
    vy = Y_AMP * w * math.cos(phase)
    return vx, vy


OUT_DIR = Path(__file__).parent / "ballistic_report"

# ═══════════════════════════════════════════════════════════════════════════
# Physics Engine — RK4 + Drag + Magnus (match C++ BallisticSolver)
# ═══════════════════════════════════════════════════════════════════════════
_dk = 0.5 * AIR_RHO * P_CD * P_AREA / P_MASS
_mk = 0.5 * AIR_RHO * P_CM * P_AREA * P_R / P_MASS


def _accel(vx, vy, vz, ox, oy, oz):
    vm = math.sqrt(vx * vx + vy * vy + vz * vz)
    df = -_dk * vm if vm > 1e-6 else 0.0
    ax, ay, az = df * vx, df * vy, -GRAVITY + df * vz
    if abs(ox) + abs(oy) + abs(oz) > 1e-8:
        ax += _mk * (oy * vz - oz * vy)
        ay += _mk * (oz * vx - ox * vz)
        az += _mk * (ox * vy - oy * vx)
    om = math.sqrt(ox * ox + oy * oy + oz * oz)
    if om > 1e-6:
        sd = -P_SDECAY * om / P_MOI
        dox, doy, doz = sd * ox, sd * oy, sd * oz
    else:
        dox = doy = doz = 0.0
    return (ax, ay, az, dox, doy, doz)


def _sim(yaw, pitch, vel, rx, ry, vrx, vry, dt=0.005):
    """RK4 trajectory simulation matching C++ BallisticSolver."""
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    vx = vel * cp * cy + vrx
    vy = vel * cp * sy + vry
    vz = vel * sp
    spin = SPIN_RPM * R2W
    ox, oy, oz = -sy * spin, cy * spin, 0.0
    x, y, z = rx, ry, TURRET_H
    pts = []
    t = 0.0
    for _ in range(int(5.0 / dt) + 1):
        pts.append((t, x, y, z, vx, vy, vz))
        a1 = _accel(vx, vy, vz, ox, oy, oz)
        h = dt * 0.5
        a2 = _accel(
            vx + h * a1[0],
            vy + h * a1[1],
            vz + h * a1[2],
            ox + h * a1[3],
            oy + h * a1[4],
            oz + h * a1[5],
        )
        a3 = _accel(
            vx + h * a2[0],
            vy + h * a2[1],
            vz + h * a2[2],
            ox + h * a2[3],
            oy + h * a2[4],
            oz + h * a2[5],
        )
        a4 = _accel(
            vx + dt * a3[0],
            vy + dt * a3[1],
            vz + dt * a3[2],
            ox + dt * a3[3],
            oy + dt * a3[4],
            oz + dt * a3[5],
        )
        d6 = dt / 6.0
        x += d6 * (vx + 2 * (vx + h * a1[0]) + 2 * (vx + h * a2[0]) + (vx + dt * a3[0]))
        y += d6 * (vy + 2 * (vy + h * a1[1]) + 2 * (vy + h * a2[1]) + (vy + dt * a3[1]))
        z += d6 * (vz + 2 * (vz + h * a1[2]) + 2 * (vz + h * a2[2]) + (vz + dt * a3[2]))
        vx += d6 * (a1[0] + 2 * a2[0] + 2 * a3[0] + a4[0])
        vy += d6 * (a1[1] + 2 * a2[1] + 2 * a3[1] + a4[1])
        vz += d6 * (a1[2] + 2 * a2[2] + 2 * a3[2] + a4[2])
        ox += d6 * (a1[3] + 2 * a2[3] + 2 * a3[3] + a4[3])
        oy += d6 * (a1[4] + 2 * a2[4] + 2 * a3[4] + a4[4])
        oz += d6 * (a1[5] + 2 * a2[5] + 2 * a3[5] + a4[5])
        t += dt
        if z <= 0 and t > 0.01:
            break
    return pts


def _fz(pts, lx, ly, td):
    """Find (z, vz, tof) at target distance.  Prefers descending crossing."""
    best = None
    for i in range(1, len(pts)):
        d = math.hypot(pts[i][1] - lx, pts[i][2] - ly)
        dp = math.hypot(pts[i - 1][1] - lx, pts[i - 1][2] - ly)
        if d >= td and dp < td:
            a = (td - dp) / (d - dp) if d != dp else 0
            z_at = pts[i - 1][3] + a * (pts[i][3] - pts[i - 1][3])
            vz_at = pts[i - 1][6] + a * (pts[i][6] - pts[i - 1][6])
            tof = pts[i - 1][0] + a * (pts[i][0] - pts[i - 1][0])
            if vz_at < 0:
                return (z_at, vz_at, tof)  # descending — preferred
            if best is None:
                best = (z_at, vz_at, tof)
    return best


# Hexagon boundary (match C++ isInsideHexagon)
_S3 = 1.7320508075688772
_HY = 20.966
_HD = 41.932


def _hex_ok(x_m, y_m):
    xi, yi = x_m / INCH2M, y_m / INCH2M
    if yi < -_HY or yi > _HY:
        return False
    if -_S3 * xi + yi > _HD:
        return False
    if _S3 * xi + yi > _HD:
        return False
    if -_S3 * xi - yi > _HD:
        return False
    if _S3 * xi - yi > _HD:
        return False
    return True


def _find_vel(yaw, pitch, rx, ry, dist, vx=0, vy=0):
    """Binary-search velocity for a pitch.  Returns (vel, tof) or None."""
    vlo, vhi = 0.0, MAX_RPM * R2W * EFF_R
    best = None
    for _ in range(20):
        vm = (vlo + vhi) / 2
        # Pass robot velocity to simulation for compensation
        pts = _sim(yaw, pitch, vm, rx, ry, vx, vy, dt=0.008)
        r = _fz(pts, pts[0][1], pts[0][2], dist)
        if r is None:
            vlo = vm
            continue
        z_at, vz_at, tof = r
        if vz_at >= 0:
            vhi = vm
            continue
        err = z_at - TARGET_H
        if abs(err) < 0.05:
            best = (vm, tof)
            vhi = vm
        elif err > 0:
            vhi = vm
        else:
            vlo = vm
    return best


def _yaw_lead(rx, ry, vx, vy, pitch, vel):
    """Iterative yaw lead with turret travel (match C++ computeYawWithLead)."""
    dx, dy = -rx, -ry
    yaw = math.atan2(dy, dx)
    vh = max(vel * math.cos(pitch), 1.0)
    direct = yaw
    for _ in range(5):
        d = math.hypot(dx, dy)
        tof = d / vh
        delta = yaw - direct
        while delta > math.pi:
            delta -= 2 * math.pi
        while delta < -math.pi:
            delta += 2 * math.pi
        tt = abs(delta) / TURRET_RATE
        px = rx + vx * (tt + tof)
        py = ry + vy * (tt + tof)
        dx, dy = -px, -py
        ny = math.atan2(dy, dx)
        if abs(ny - yaw) < 1e-4:
            return ny
        yaw = ny
    return yaw


def _solve(rx, ry, vrx, vry):
    """Minimum-pitch solver with hexagon check + iterative yaw lead."""
    dist = math.hypot(rx, ry)
    direct = math.atan2(-ry, -rx)
    p_lo, p_hi = PITCH_MIN, PITCH_MAX
    best = None
    for _ in range(15):
        pm = (p_lo + p_hi) / 2
        r = _find_vel(direct, pm, rx, ry, dist, vrx, vry)
        ok = False
        if r is not None:
            vel, tof = r
            rpm = vel / (R2W * EFF_R)
            if rpm <= MAX_RPM:
                ok = True
        if ok:
            best = (pm, vel, rpm, tof)
            p_hi = pm
        else:
            p_lo = pm
    if best is None:
        return None
    pitch, vel, rpm, tof = best
    comp = _yaw_lead(rx, ry, vrx, vry, pitch, vel)
    return dict(
        pitch=pitch,
        vel=vel,
        rpm=rpm,
        tof=tof,
        yaw=comp,
        direct=direct,
        lead=comp - direct,
        dist=dist,
    )


def _interp(traj, t):
    if t < 0 or not traj or t > traj[-1][0]:
        return None
    for i in range(1, len(traj)):
        if traj[i][0] >= t:
            a = (t - traj[i - 1][0]) / (traj[i][0] - traj[i - 1][0])
            return (
                traj[i - 1][1] + a * (traj[i][1] - traj[i - 1][1]),
                traj[i - 1][2] + a * (traj[i][2] - traj[i - 1][2]),
                traj[i - 1][3] + a * (traj[i][3] - traj[i - 1][3]),
            )
    return None


# ═══════════════════════════════════════════════════════════════════════════
# Pre-compute
# ═══════════════════════════════════════════════════════════════════════════
print("Pre-computing ...")
t0 = time.time()
nf = int(DURATION * FPS)
ft = [i / FPS for i in range(nf)]

KSTEP = 3
ksols = {}
for kf in range(0, nf, KSTEP):
    rx, ry = robot_pos(ft[kf])
    vx, vy = robot_vel(ft[kf])
    ksols[kf] = _solve(rx, ry, vx, vy)


def _gsol(f):
    k0 = (f // KSTEP) * KSTEP
    k1 = min(k0 + KSTEP, nf - 1)
    s0 = ksols.get(k0)
    s1 = ksols.get(k1, s0)
    if s0 is None:
        return None
    if s1 is None or k0 == k1:
        return s0
    a = (f - k0) / (k1 - k0)
    out = {}
    for k in s0:
        v0, v1 = s0[k], s1[k]
        if k in ("yaw", "direct"):
            d = v1 - v0
            while d > math.pi:
                d -= 2 * math.pi
            while d < -math.pi:
                d += 2 * math.pi
            out[k] = v0 + a * d
        else:
            out[k] = v0 + a * (v1 - v0)
    return out


states = []
for f in range(nf):
    t = ft[f]
    rx, ry = robot_pos(t)
    h = SHEADING + SOMEGA * t
    sol = _gsol(f)
    if sol:
        states.append(
            dict(
                t=t,
                rx=rx,
                ry=ry,
                h=h,
                dist=sol["dist"],
                yaw=sol["yaw"],
                direct=sol["direct"],
                lead=sol["lead"],
                pitch=sol["pitch"],
                rpm=sol["rpm"],
                vel=sol["vel"],
                tof=sol["tof"],
                ok=True,
            )
        )
    else:
        d = math.hypot(rx, ry)
        dy = math.atan2(-ry, -rx)
        states.append(
            dict(
                t=t,
                rx=rx,
                ry=ry,
                h=h,
                dist=d,
                yaw=dy,
                direct=dy,
                lead=0,
                pitch=0.5,
                rpm=0,
                vel=0,
                tof=0,
                ok=False,
            )
        )

# Shots
shots = []
for i, fft in enumerate(FIRE_TIMES):
    ff = min(int(fft * FPS), nf - 1)
    s = states[ff]
    if not s["ok"]:
        continue
    fvx, fvy = robot_vel(fft)
    traj = _sim(s["yaw"], s["pitch"], s["vel"], s["rx"], s["ry"], fvx, fvy, dt=0.005)
    shots.append(
        dict(
            ft=fft,
            lx=s["rx"],
            ly=s["ry"],
            ldist=s["dist"],
            traj=traj,
            color=COLORS5[i % 5],
        )
    )

print(f"  {nf} frames, {len(shots)} shots  ({time.time()-t0:.1f}s)")


# ═══════════════════════════════════════════════════════════════════════════
# Drawing — Top View
# ═══════════════════════════════════════════════════════════════════════════
def draw_top(ax, f):
    s = states[f]
    t = s["t"]
    rx, ry, h = s["rx"], s["ry"], s["h"]
    yaw, direct, lead, dist = s["yaw"], s["direct"], s["lead"], s["dist"]

    ax.set_xlim(-1.5, 8)
    ax.set_ylim(-3.5, 3.5)
    ax.set_aspect("equal")
    ax.set_xlabel("X (m)", fontsize=9)
    ax.set_ylabel("Y (m)", fontsize=9)
    ax.set_title(
        "Top View  --  Turret Yaw Compensation", fontsize=11, fontweight="bold"
    )
    ax.grid(True, alpha=0.12)
    ax.set_facecolor("#FAFAFA")

    # Target
    ax.add_patch(
        RegularPolygon(
            (0, 0), 6, radius=0.55, fc="#FEE2E2", ec="#DC2626", lw=2, zorder=5
        )
    )
    ax.text(
        0, -1.0, "TARGET", ha="center", fontsize=8, color="#DC2626", fontweight="bold"
    )

    # Robot trail
    i0 = max(0, f - 40)
    ax.plot(
        [states[j]["rx"] for j in range(i0, f + 1)],
        [states[j]["ry"] for j in range(i0, f + 1)],
        color="#93C5FD",
        lw=2,
        alpha=0.4,
        zorder=2,
    )

    # Velocity arrow
    ax.annotate(
        "",
        xy=(rx + robot_vel(t)[0] * 0.6, ry + robot_vel(t)[1] * 0.6),
        xytext=(rx, ry),
        arrowprops=dict(arrowstyle="->", color="#F59E0B", lw=2.5),
        zorder=8,
    )

    # Direct aim (dashed red)
    ax.plot([rx, 0], [ry, 0], color="#EF4444", lw=1.2, ls="--", alpha=0.5, zorder=3)
    # Compensated aim (dotted green)
    ext = dist * 1.1
    ax.plot(
        [rx, rx + ext * math.cos(yaw)],
        [ry, ry + ext * math.sin(yaw)],
        color="#22C55E",
        lw=1,
        ls=":",
        alpha=0.35,
        zorder=3,
    )

    # Robot chassis
    sz = 0.30
    c4 = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]], float) * sz
    ch, sh = math.cos(h), math.sin(h)
    rot = np.array([[ch, -sh], [sh, ch]])
    c4 = c4 @ rot.T + [rx, ry]
    ax.add_patch(Polygon(c4, fc="#3B82F6", ec="#1E3A8A", lw=2, zorder=10))
    wc = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]], float) * (sz * 0.85)
    wc = wc @ rot.T + [rx, ry]
    for wx, wy in wc:
        ax.plot(wx, wy, "s", color="#1E3A8A", ms=3.5, zorder=11)

    # Turret + barrel
    ax.add_patch(Circle((rx, ry), 0.15, fc="#1E40AF", ec="white", lw=1.5, zorder=12))
    bl = 0.55
    bx, by = rx + bl * math.cos(yaw), ry + bl * math.sin(yaw)
    ax.plot(
        [rx, bx], [ry, by], color="#22C55E", lw=5, solid_capstyle="round", zorder=13
    )
    ax.plot(bx, by, "o", color="#16A34A", ms=6, zorder=14)

    # Yaw-lead arc
    if abs(math.degrees(lead)) > 0.5:
        a1, a2 = math.degrees(direct), math.degrees(yaw)
        if a2 - a1 > 180:
            a1 += 360
        if a1 - a2 > 180:
            a2 += 360
        lo_a, hi_a = min(a1, a2), max(a1, a2)
        ax.add_patch(
            Arc(
                (rx, ry),
                1.0,
                1.0,
                angle=0,
                theta1=lo_a,
                theta2=hi_a,
                color="#F59E0B",
                lw=2,
                zorder=9,
            )
        )
        mid = math.radians((lo_a + hi_a) / 2)
        ax.text(
            rx + 0.65 * math.cos(mid),
            ry + 0.65 * math.sin(mid),
            f"{math.degrees(lead):+.1f}\u00b0",
            fontsize=8,
            color="#D97706",
            fontweight="bold",
            ha="center",
            zorder=15,
        )

    # Balls
    for shot in shots:
        bt = t - shot["ft"]
        if bt < -0.01 or bt > shot["traj"][-1][0] + 0.1:
            continue
        traj = shot["traj"]
        mt = traj[-1][0]
        # Trail
        for j in range(TRAIL_N):
            tt = bt - j * 0.02
            if tt < 0 or tt > mt:
                continue
            p = _interp(traj, tt)
            if p and p[2] > 0:
                ax.plot(
                    p[0],
                    p[1],
                    "o",
                    color=shot["color"],
                    ms=max(2, 8 - j * 0.8),
                    alpha=max(0.1, 1 - j / TRAIL_N),
                    zorder=20 - j,
                )
        # Ball
        p = _interp(traj, bt)
        if p and p[2] > 0:
            ax.plot(
                p[0],
                p[1],
                "o",
                color=shot["color"],
                ms=10,
                mec="white",
                mew=1.2,
                zorder=25,
            )
        # Path
        if bt > 0:
            px = [q[1] for q in traj if q[0] <= min(bt, mt)]
            py = [q[2] for q in traj if q[0] <= min(bt, mt)]
            ax.plot(px, py, color=shot["color"], lw=0.7, alpha=0.2, zorder=4)

    ax.plot([], [], color="#22C55E", lw=3, label="Turret (compensated)")
    ax.plot([], [], color="#EF4444", lw=1.2, ls="--", label="Direct aim")
    ax.plot([], [], color="#F59E0B", lw=2, label="Yaw lead")
    ax.legend(loc="upper right", fontsize=7, framealpha=0.9)


# ═══════════════════════════════════════════════════════════════════════════
# Drawing — Side View  (X = distance from target, inverted axis)
# ═══════════════════════════════════════════════════════════════════════════
def draw_side(ax, f):
    s = states[f]
    t = s["t"]
    dist, pitch = s["dist"], s["pitch"]

    # Distance-from-target axis, inverted so target is on RIGHT
    ax.set_xlim(7.5, -1)
    ax.set_ylim(-0.35, 5.0)
    ax.set_xlabel("Distance from Target (m)", fontsize=9)
    ax.set_ylabel("Height (m)", fontsize=9)
    ax.set_title("Side View  --  Hood Pitch & Ball Arc", fontsize=11, fontweight="bold")
    ax.grid(True, alpha=0.12)
    ax.set_facecolor("#FAFAFA")

    # Ground
    ax.fill_between([12, -1], -0.35, 0, color="#FEF3C7", alpha=0.3, zorder=0)
    ax.axhline(0, color="#78350F", lw=2, zorder=1)

    hex_R = _HY * INCH2M  # hexagon half-height in metres ≈ 0.533m
    hex_verts_x = []
    hex_verts_z = []
    for k in range(6):
        ang = math.pi / 6 + k * math.pi / 3  # start at 30° so flat side is horizontal
        hex_verts_x.append(hex_R * math.cos(ang))
        hex_verts_z.append(TARGET_H + hex_R * math.sin(ang))
    hex_verts_x.append(hex_verts_x[0])
    hex_verts_z.append(hex_verts_z[0])
    ax.fill(
        hex_verts_x, hex_verts_z, fc="#FEE2E2", ec="#DC2626", lw=2, alpha=0.6, zorder=5
    )
    ax.text(
        0,
        TARGET_H + hex_R + 0.12,
        f"TARGET {TARGET_H:.2f}m",
        ha="center",
        fontsize=7,
        color="#DC2626",
        fontweight="bold",
    )

    # Robot body at distance=dist  (appears on LEFT)
    bw, bh = 0.50, 0.28
    ax.add_patch(
        FancyBboxPatch(
            (dist - bw / 2, 0),
            bw,
            bh,
            boxstyle="round,pad=0.02",
            fc="#3B82F6",
            ec="#1E3A8A",
            lw=2,
            zorder=10,
        )
    )
    # Turret tower
    ax.plot(
        [dist, dist],
        [bh, TURRET_H],
        color="#1E40AF",
        lw=6,
        solid_capstyle="round",
        zorder=11,
    )
    # Wheels
    for wx in [dist - 0.18, dist + 0.18]:
        ax.add_patch(Circle((wx, 0), 0.055, fc="#1E3A8A", zorder=11))

    # Hood / barrel  — points toward target (decreasing X = toward RIGHT)
    bl = 0.65
    # In inverted-X frame: barrel goes toward lower X (visually rightward = toward target)
    barrel_end_x = dist - bl * math.cos(pitch)
    barrel_end_z = TURRET_H + bl * math.sin(pitch)
    ax.plot(
        [dist, barrel_end_x],
        [TURRET_H, barrel_end_z],
        color="#22C55E",
        lw=5,
        solid_capstyle="round",
        zorder=12,
    )
    ax.plot(barrel_end_x, barrel_end_z, "o", color="#16A34A", ms=6, zorder=13)

    # Pitch arc annotation
    # In the inverted axis, "toward target" = 0° visually but 180° in data coords
    # The barrel angle measured from the horizontal-toward-target direction = pitch
    # Arc drawn at turret pivot: from 180° to (180-pitch_deg)
    ar = 0.35
    p_deg = math.degrees(pitch)
    ax.add_patch(
        Arc(
            (dist, TURRET_H),
            ar * 2,
            ar * 2,
            angle=0,
            theta1=180 - p_deg,
            theta2=180,
            color="#22C55E",
            lw=2,
            zorder=12,
        )
    )
    # Label at mid-angle
    mid_ang = math.radians(180 - p_deg / 2)
    lx = dist + ar * 1.4 * math.cos(mid_ang)
    lz = TURRET_H + ar * 1.4 * math.sin(mid_ang)
    ax.text(
        lx,
        lz,
        f"{p_deg:.1f}\u00b0",
        fontsize=9,
        color="#16A34A",
        fontweight="bold",
        ha="center",
        zorder=15,
    )

    # ── Ball arcs ──
    for shot in shots:
        bt = t - shot["ft"]
        if bt < -0.01:
            continue
        traj = shot["traj"]
        mt = traj[-1][0]
        if bt > mt + 0.3:
            continue  # skip long-dead balls

        # Each trajectory point: x_plot = distance_from_target = sqrt(px^2+py^2)
        #                        z_plot = pz  (height)
        # Collect visible points
        vis_t = min(bt, mt)
        path_x = []
        path_z = []
        for q in traj:
            if q[0] > vis_t:
                break
            qd = math.hypot(q[1], q[2])  # distance from target (origin)
            path_x.append(qd)
            path_z.append(q[3])

        if path_x:
            ax.plot(path_x, path_z, color=shot["color"], lw=1.5, alpha=0.4, zorder=6)

        # Current ball
        p = _interp(traj, bt)
        if p and p[2] > 0:
            pd = math.hypot(p[0], p[1])
            ax.plot(
                pd,
                p[2],
                "o",
                color=shot["color"],
                ms=10,
                mec="white",
                mew=1.2,
                zorder=25,
            )

        # Trail
        for j in range(TRAIL_N):
            tt = bt - j * 0.02
            if tt < 0 or tt > mt:
                continue
            tp = _interp(traj, tt)
            if tp and tp[2] > 0:
                td = math.hypot(tp[0], tp[1])
                ax.plot(
                    td,
                    tp[2],
                    "o",
                    color=shot["color"],
                    ms=max(2, 7 - j * 0.8),
                    alpha=max(0.1, 1 - j / TRAIL_N),
                    zorder=20 - j,
                )


# ═══════════════════════════════════════════════════════════════════════════
# Animation
# ═══════════════════════════════════════════════════════════════════════════
print(f"Rendering {nf} frames @ {FPS} fps  ({len(shots)} shots) ...")

fig, (ax_top, ax_side) = plt.subplots(
    1, 2, figsize=(16, 7.5), gridspec_kw={"width_ratios": [1.15, 1]}
)
fig.subplots_adjust(top=0.86, bottom=0.08, left=0.04, right=0.97, wspace=0.22)

fig.suptitle(
    "Ballistic Solver  --  Sweep 0.75m \u2192 6.5m \u2192 0.75m  (6 shots/sec)\n"
    '4" Orange Stealth (40A) + Backplate   |   Gravity + Drag   |   Team 9427',
    fontsize=13,
    fontweight="bold",
    y=0.97,
)

info = fig.text(
    0.5,
    0.905,
    "",
    ha="center",
    fontsize=10,
    fontfamily="monospace",
    bbox=dict(boxstyle="round,pad=0.4", fc="#F3F4F6", ec="#D1D5DB"),
)


def update(frame):
    ax_top.clear()
    ax_side.clear()
    draw_top(ax_top, frame)
    draw_side(ax_side, frame)
    s = states[frame]
    info.set_text(
        f"t={s['t']:.2f}s   |   Dist: {s['dist']:.1f}m   |   "
        f"Yaw Lead: {math.degrees(s['lead']):+.1f}\u00b0   |   "
        f"Pitch: {math.degrees(s['pitch']):.1f}\u00b0   |   "
        f"RPM: {s['rpm']:.0f}   |   v_exit: {s['vel']:.1f} m/s"
    )
    if frame % 10 == 0:
        print(f"  frame {frame:>4d}/{nf}  ({100*frame/nf:5.1f}%)")
    return []


anim = FuncAnimation(fig, update, frames=nf, interval=1000 / FPS, blit=False)

OUT_DIR.mkdir(parents=True, exist_ok=True)
out = OUT_DIR / "ballistic_animation.gif"
print(f"Saving -> {out}")
ts = time.time()
anim.save(str(out), writer=PillowWriter(fps=FPS), dpi=90 if FAST else 110)
print(f"Done in {time.time()-ts:.1f}s  ->  {out}")
plt.close(fig)
