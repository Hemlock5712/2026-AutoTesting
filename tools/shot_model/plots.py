"""
Visual sanity checks for the shooter model. Saves PNGs to tools/shot_model/figures/.

  python3 tools/shot_model/plots.py

Figures:
  trajectories.png  - ball arcs at several distances vs the hub rim
  schedules.png     - hood / flywheel / ToF vs distance (at rest)
  flywheel_surface  - flywheel(distance, radial velocity) heatmap
  endtoend_miss.png - closed-loop landing miss over (radial, tangential) velocity
"""

import math
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import shot_model as sm
from verify import build_java_map, fire_3d

OUT = os.path.join(os.path.dirname(__file__), "figures")
os.makedirs(OUT, exist_ok=True)


def _trajectory(flywheel_rps, hood_deg, radial=0.0):
    elevation = math.radians(sm.HOOD_ZERO_ELEVATION_DEG - hood_deg)
    v0 = sm.exit_speed(flywheel_rps)
    omega = sm.backspin_rad_per_sec(flywheel_rps)
    x, z = 0.0, sm.LAUNCH_HEIGHT_M
    vx = v0 * math.cos(elevation) + radial
    vz = v0 * math.sin(elevation)
    xs, zs = [x], [z]
    t = 0.0
    while t < sm.MAX_TIME and z >= 0:
        ax, az = sm._accel_2d(vx, vz, omega)
        vx += ax * sm.DT
        vz += az * sm.DT
        x += vx * sm.DT
        z += vz * sm.DT
        t += sm.DT
        xs.append(x)
        zs.append(z)
        if z < 0:
            break
    return xs, zs


def plot_trajectories(smap):
    plt.figure(figsize=(9, 5))
    for d in [2.0, 3.0, 4.0, 5.0, 6.0]:
        hood = smap.hood_deg(d)
        fw = smap.flywheel_rps(d, 0.0)
        xs, zs = _trajectory(fw, hood)
        plt.plot(xs, zs, label=f"{d:.0f} m  (hood {hood:.0f} deg, {fw:.0f} RPS)")
        plt.plot(d, sm.GOAL_HEIGHT_M, "kx", ms=9)
    plt.axhline(sm.GOAL_HEIGHT_M, color="gray", ls="--", lw=0.8)
    plt.xlabel("horizontal distance (m)")
    plt.ylabel("height (m)")
    plt.title("Ball trajectories into the hub (x = rim height target)")
    plt.legend()
    plt.grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig(os.path.join(OUT, "trajectories.png"), dpi=110)
    plt.close()


def plot_schedules(smap):
    ds = np.linspace(sm.MIN_DIST_M, sm.MAX_DIST_M, 40)
    hood = [smap.hood_deg(d) for d in ds]
    fw = [smap.flywheel_rps(d, 0.0) for d in ds]
    tof = [smap.tof_seconds(d, 0.0) for d in ds]
    fig, ax = plt.subplots(1, 3, figsize=(13, 4))
    ax[0].plot(ds, hood); ax[0].set_title("hood (deg)"); ax[0].set_xlabel("distance (m)")
    ax[1].plot(ds, fw); ax[1].set_title("flywheel (RPS), at rest"); ax[1].set_xlabel("distance (m)")
    ax[2].plot(ds, tof); ax[2].set_title("time of flight (s)"); ax[2].set_xlabel("distance (m)")
    for a in ax:
        a.grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig(os.path.join(OUT, "schedules.png"), dpi=110)
    plt.close()


def plot_flywheel_surface(smap):
    ds = np.linspace(sm.MIN_DIST_M, sm.MAX_DIST_M, 60)
    vrs = np.linspace(sm.MIN_RADIAL_VEL, sm.MAX_RADIAL_VEL, 60)
    grid = np.array([[smap.flywheel_rps(d, vr) for d in ds] for vr in vrs])
    plt.figure(figsize=(8, 5))
    im = plt.pcolormesh(ds, vrs, grid, shading="auto", cmap="viridis")
    plt.colorbar(im, label="flywheel (RPS)")
    plt.xlabel("distance (m)")
    plt.ylabel("radial velocity (m/s, + = closing)")
    plt.title("flywheel(distance, radial velocity)")
    plt.tight_layout()
    plt.savefig(os.path.join(OUT, "flywheel_surface.png"), dpi=110)
    plt.close()


def plot_endtoend_miss(smap, d=4.0):
    vrs = np.linspace(-2.0, 4.0, 25)
    vts = np.linspace(-3.0, 3.0, 25)
    miss = np.full((len(vts), len(vrs)), np.nan)
    for a, vt in enumerate(vts):
        for b, vr in enumerate(vrs):
            aim = sm.swm_solve(smap, 0, 0, vr, vt, d, 0.0, 0.4, 0.0, 0.0)
            if not aim["feasible"]:
                continue
            land = fire_3d(aim["flywheel_rps"], aim["hood_deg"], aim["turret_rot"],
                           0, 0, vr, vt, 0.4, 0.0, 0.0)
            if land is not None:
                miss[a, b] = math.hypot(land[0] - d, land[1])
    plt.figure(figsize=(8, 5))
    im = plt.pcolormesh(vrs, vts, miss * 100, shading="auto", cmap="magma_r", vmin=0, vmax=40)
    plt.colorbar(im, label="landing miss (cm)")
    plt.contour(vrs, vts, miss, levels=[sm.GOAL_OPENING_RADIUS_M], colors="cyan")
    plt.xlabel("radial velocity (m/s, + = closing)")
    plt.ylabel("tangential velocity (m/s)")
    plt.title(f"closed-loop landing miss @ {d:.0f} m (cyan = hub opening edge)")
    plt.tight_layout()
    plt.savefig(os.path.join(OUT, "endtoend_miss.png"), dpi=110)
    plt.close()


if __name__ == "__main__":
    smap = build_java_map()  # the values that ship in Java ShooterMap
    plot_trajectories(smap)
    plot_schedules(smap)
    plot_flywheel_surface(smap)
    plot_endtoend_miss(smap)
    print("wrote figures to", OUT)
