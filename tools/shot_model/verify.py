"""
Offline verification of the shooter model.

Strategy: build the runtime map straight from the values baked into the Java
ShooterMap (the numbers that actually ship), then

1. cross_check_physics(): confirm the Python physics/solver reproduces each grid
   node the Java generator produced. Proves the Python mirror is faithful, so its
   end-to-end results transfer to the robot code. (Avoids re-running the slow
   make-window generation in pure Python.)

2. end_to_end(): the real test. For many robot states (moving, tilted), run the
   SwmTargeting solve, then fire the resulting (turret angle, hood, flywheel)
   commands through an INDEPENDENT 3D ball sim (gravity + drag + Magnus) with the
   robot's inherited velocity and tilt, and measure where it lands. Catches
   frame/sign/lead/integration bugs the per-function unit tests cannot.

Run: python3 tools/shot_model/verify.py
"""

import math
import sys

import shot_model as sm

# ---- Java baked values (from ShooterMap.java) ----
JAVA_HOOD_COEFFS = [-6.3365385, 7.3663836, -0.52297702]
JAVA_FLYWHEEL = [
    [57.727, 38.470, 29.457, 29.457, 29.457, 29.457, 29.457],
    [52.652, 39.301, 31.549, 29.027, 29.027, 29.027, 29.027],
    [50.806, 40.316, 33.518, 30.134, 30.134, 30.134, 30.134],
    [50.283, 41.485, 35.333, 31.641, 30.134, 30.134, 30.134],
    [50.498, 42.746, 37.055, 33.272, 31.211, 30.565, 30.565],
    [51.114, 44.100, 38.747, 34.933, 32.533, 31.365, 31.365],
    [52.036, 45.515, 40.408, 36.563, 33.979, 32.410, 31.703],
    [53.205, 47.022, 42.039, 38.194, 35.425, 33.610, 32.564],
    [54.528, 48.591, 43.700, 39.824, 36.932, 34.871, 33.549],
    [56.005, 50.221, 45.361, 41.454, 38.409, 36.194, 34.594],
    [57.635, 51.913, 47.053, 43.054, 39.916, 37.517, 35.733],
    [59.419, 53.667, 48.776, 44.684, 41.393, 38.840, 36.871],
]
JAVA_TOF = [
    [1.9480, 1.2011, 0.69472, 0.69472, 0.69472, 0.69472, 0.69472],
    [1.7395, 1.2131, 0.82037, 0.56412, 0.56412, 0.56412, 0.56412],
    [1.6426, 1.2301, 0.90713, 0.67145, 0.67145, 0.67145, 0.67145],
    [1.5933, 1.2514, 0.97345, 0.75741, 0.60071, 0.60071, 0.60071],
    [1.5714, 1.2753, 1.0290, 0.83060, 0.67641, 0.56690, 0.56690],
    [1.5651, 1.3025, 1.0799, 0.89531, 0.74603, 0.63060, 0.63060],
    [1.5715, 1.3327, 1.1278, 0.95311, 0.81086, 0.69290, 0.60048],
    [1.5890, 1.3677, 1.1743, 1.0086, 0.86902, 0.75371, 0.66085],
    [1.6148, 1.4070, 1.2232, 1.0636, 0.92788, 0.81158, 0.71706],
    [1.6493, 1.4514, 1.2739, 1.1195, 0.98443, 0.87013, 0.77139],
    [1.6931, 1.5013, 1.3285, 1.1758, 1.0434, 0.92789, 0.82917],
    [1.7463, 1.5572, 1.3874, 1.2360, 1.1029, 0.98656, 0.88616],
]


def build_java_map():
    return sm.ShooterMap({"hood_coeffs": list(JAVA_HOOD_COEFFS),
                          "flywheel": [row[:] for row in JAVA_FLYWHEEL],
                          "tof": [row[:] for row in JAVA_TOF]})


def _java_hood(d):
    c = JAVA_HOOD_COEFFS
    return c[0] + c[1] * d + c[2] * d * d


def cross_check_physics():
    """Re-solve each feasible grid node in Python and compare to the Java baked value."""
    max_fw_diff = 0.0
    max_tof_diff = 0.0
    checked = 0
    for i in range(sm.N_DIST):
        d = sm.distance_at(i)
        hood = _java_hood(d)
        for j in range(sm.N_VEL):
            vr = sm.radial_vel_at(j)
            fw = sm.on_target_flywheel(hood, d, vr)
            if fw is None:
                continue  # edge-filled (infeasible) node; nothing to compare
            tof = sm.simulate(fw, hood, d, vr)["tof"]
            max_fw_diff = max(max_fw_diff, abs(fw - JAVA_FLYWHEEL[i][j]))
            max_tof_diff = max(max_tof_diff, abs(tof - JAVA_TOF[i][j]))
            checked += 1
    print(f"cross-check: {checked} feasible nodes re-solved; "
          f"max flywheel diff={max_fw_diff:.4f} RPS, max tof diff={max_tof_diff:.4f} s")
    return max_fw_diff < 0.05 and max_tof_diff < 0.01


# ---- Independent 3D ball forward sim ----
def _accel_3d(vx, vy, vz, omega, wx, wy, wz):
    speed = math.sqrt(vx * vx + vy * vy + vz * vz)
    if speed < 1e-6:
        return 0.0, 0.0, -sm.GRAVITY
    r = sm.BALL_DIAMETER_M / 2.0
    area = math.pi * r * r
    drag = 0.5 * sm.AIR_DENSITY * sm.DRAG_COEFFICIENT * area * speed / sm.BALL_MASS_KG
    ax, ay, az = -drag * vx, -drag * vy, -drag * vz - sm.GRAVITY
    spin_ratio = omega * r / speed
    cl = min(sm.LIFT_COEFF_PER_SPIN * spin_ratio, sm.MAX_LIFT_COEFF)
    # Magnus: a = (0.5 rho A Cl |v| / m) * (w_hat x v); matches the 2D model in-plane.
    cx = wy * vz - wz * vy
    cy = wz * vx - wx * vz
    cz = wx * vy - wy * vx
    k = 0.5 * sm.AIR_DENSITY * area * cl * speed / sm.BALL_MASS_KG
    return ax + k * cx, ay + k * cy, az + k * cz


def fire_3d(flywheel_rps, hood_deg, turret_rot, tx, ty, tvx, tvy, yaw, pitch, roll):
    """Fire the commanded shot; return (x, y) where it crosses the rim height descending."""
    v0 = sm.exit_speed(flywheel_rps)
    az_robot = turret_rot * 2 * math.pi
    elev_robot = math.radians(sm.HOOD_ZERO_ELEVATION_DEG - hood_deg)
    dir_robot = [math.cos(elev_robot) * math.cos(az_robot),
                 math.cos(elev_robot) * math.sin(az_robot),
                 math.sin(elev_robot)]
    df = sm.robot_to_field(dir_robot[0], dir_robot[1], dir_robot[2], yaw, pitch, roll)

    vx, vy, vz = v0 * df[0] + tvx, v0 * df[1] + tvy, v0 * df[2]
    x, y, z = tx, ty, sm.LAUNCH_HEIGHT_M
    az_field = math.atan2(df[1], df[0])
    wx, wy, wz = math.sin(az_field), -math.cos(az_field), 0.0  # backspin axis
    omega = sm.backspin_rad_per_sec(flywheel_rps)

    t = 0.0
    past_apex = False
    dt = sm.DT
    while t < sm.MAX_TIME:
        pz, pvz, px, py = z, vz, x, y
        a1 = _accel_3d(vx, vy, vz, omega, wx, wy, wz)
        a2 = _accel_3d(vx + .5 * dt * a1[0], vy + .5 * dt * a1[1], vz + .5 * dt * a1[2], omega, wx, wy, wz)
        a3 = _accel_3d(vx + .5 * dt * a2[0], vy + .5 * dt * a2[1], vz + .5 * dt * a2[2], omega, wx, wy, wz)
        a4 = _accel_3d(vx + dt * a3[0], vy + dt * a3[1], vz + dt * a3[2], omega, wx, wy, wz)
        vx2, vy2, vz2 = vx + .5 * dt * a1[0], vy + .5 * dt * a1[1], vz + .5 * dt * a1[2]
        vx3, vy3, vz3 = vx + .5 * dt * a2[0], vy + .5 * dt * a2[1], vz + .5 * dt * a2[2]
        vx4, vy4, vz4 = vx + dt * a3[0], vy + dt * a3[1], vz + dt * a3[2]
        x += dt * (vx + 2 * vx2 + 2 * vx3 + vx4) / 6
        y += dt * (vy + 2 * vy2 + 2 * vy3 + vy4) / 6
        z += dt * (vz + 2 * vz2 + 2 * vz3 + vz4) / 6
        vx += dt * (a1[0] + 2 * a2[0] + 2 * a3[0] + a4[0]) / 6
        vy += dt * (a1[1] + 2 * a2[1] + 2 * a3[1] + a4[1]) / 6
        vz += dt * (a1[2] + 2 * a2[2] + 2 * a3[2] + a4[2]) / 6
        t += dt
        if pvz > 0 and vz <= 0:
            past_apex = True
        if past_apex and pz >= sm.GOAL_HEIGHT_M and z < sm.GOAL_HEIGHT_M:
            frac = (pz - sm.GOAL_HEIGHT_M) / (pz - z)
            return px + frac * (x - px), py + frac * (y - py)
        if z < 0:
            break
    return None


def end_to_end(smap):
    categories = {}

    def run(cat, tvx, tvy, yaw, pitch, roll, d):
        # turret at origin; target along +x at distance d. tvx = radial(+closing), tvy = tangential.
        aim = sm.swm_solve(smap, 0.0, 0.0, tvx, tvy, d, 0.0, yaw, pitch, roll)
        if not aim["feasible"]:
            return
        land = fire_3d(aim["flywheel_rps"], aim["hood_deg"], aim["turret_rot"],
                       0.0, 0.0, tvx, tvy, yaw, pitch, roll)
        miss = float("inf") if land is None else math.hypot(land[0] - d, land[1])
        categories.setdefault(cat, []).append(miss)

    for d in [1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0]:
        run("stationary", 0, 0, 0.0, 0, 0, d)
        for yaw in [0.0, 0.7, -1.2]:
            run("stationary+yaw", 0, 0, yaw, 0, 0, d)
        for vr in [-2, -1, 1, 2, 3]:
            run("radial", vr, 0, 0.4, 0, 0, d)
        for vt in [-3, -2, -1, 1, 2, 3]:
            run("tangential", 0, vt, 0.4, 0, 0, d)
        for vr, vt in [(1.5, 1.5), (-1.5, 2.0), (2.0, -2.0)]:
            run("diagonal", vr, vt, 0.4, 0, 0, d)
        for p, r in [(0.15, 0.0), (-0.15, 0.0), (0.0, 0.15), (0.12, -0.12)]:
            run("tilt", 0, 0, 0.4, p, r, d)
        for vr, vt, p, r in [(2.0, 1.5, 0.12, 0.0), (-1.0, -2.0, 0.0, 0.12)]:
            run("moving+tilt", vr, vt, 0.4, p, r, d)

    print(f"\n{'category':16} {'n':>4} {'mean miss':>10} {'max miss':>10} {'scored%':>8}")
    all_ok = True
    opening = sm.GOAL_OPENING_RADIUS_M
    for cat, misses in categories.items():
        finite = [m for m in misses if math.isfinite(m)]
        mean = sum(finite) / len(finite) if finite else float("inf")
        mx = max(misses)
        scored = 100.0 * sum(1 for m in misses if m <= opening) / len(misses)
        flag = "" if mx <= opening else "  <-- EXCEEDS OPENING"
        print(f"{cat:16} {len(misses):>4} {mean*100:>9.1f}c {mx*100:>9.1f}c {scored:>7.0f}%{flag}")
        if mx > opening:
            all_ok = False
    print(f"\n(misses in cm; hub opening radius = {opening*100:.0f} cm)")
    return all_ok


if __name__ == "__main__":
    print("=== Cross-check: Python physics reproduces the Java baked grid ===")
    ok1 = cross_check_physics()
    print("\n=== Closed-loop end-to-end: solve targeting -> fire ball -> measure miss ===")
    ok2 = end_to_end(build_java_map())
    print(f"\nRESULT: cross_check={'OK' if ok1 else 'FAIL'}, "
          f"end_to_end={'OK' if ok2 else 'FAIL (some shots miss the hub)'}")
    sys.exit(0 if (ok1 and ok2) else 1)
