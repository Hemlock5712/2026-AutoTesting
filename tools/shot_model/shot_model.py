"""
Offline Python mirror of the Java shooter targeting stack
(ShotPhysics / ShotSolver / ShotMapGenerator / ShooterMap / SwmTargeting).

Purpose: this environment cannot compile the WPILib Java, so we reproduce the
exact same logic in Python to (a) cross-check the Java math, (b) run a full
closed-loop end-to-end sim (verify.py), and (c) generate the model + plots
offline instead of on the robot.

Every constant and formula here intentionally matches the Java; keep them in
sync. Pure-stdlib + numpy only.
"""

import math

# ----------------------- CALIBRATION SURFACE (mirror ShotPhysics) -----------------------
SLIP_EFFICIENCY = 0.62
SPIN_FACTOR = 0.50
LIFT_COEFF_PER_SPIN = 0.30
MAX_LIFT_COEFF = 0.35
DRAG_COEFFICIENT = 0.50
BALL_MASS_KG = 0.2268
BALL_DIAMETER_M = 0.15

# Fixed geometry
WHEEL_RADIUS_M = 0.0508
HOOD_ZERO_ELEVATION_DEG = 75.0
LAUNCH_HEIGHT_M = 0.3556
GOAL_HEIGHT_M = 1.828
GOAL_OPENING_RADIUS_M = 0.34

# Environment / integration
GRAVITY = 9.81
AIR_DENSITY = 1.225
DT = 0.001
MAX_TIME = 3.0


def _ball_radius():
    return BALL_DIAMETER_M / 2.0


def surface_speed(flywheel_rps):
    return flywheel_rps * 2.0 * math.pi * WHEEL_RADIUS_M


def exit_speed(flywheel_rps):
    return SLIP_EFFICIENCY * surface_speed(flywheel_rps)


def backspin_rad_per_sec(flywheel_rps):
    return SPIN_FACTOR * surface_speed(flywheel_rps) / _ball_radius()


def _accel_2d(vx, vz, omega):
    """Acceleration (gravity + drag + Magnus) in the vertical shot plane. Mirrors ShotPhysics."""
    speed = math.hypot(vx, vz)
    if speed < 1e-6:
        return 0.0, -GRAVITY
    r = _ball_radius()
    area = math.pi * r * r
    drag = 0.5 * AIR_DENSITY * DRAG_COEFFICIENT * area * speed / BALL_MASS_KG
    ax = -drag * vx
    az = -drag * vz - GRAVITY
    spin_ratio = omega * r / speed
    cl = min(LIFT_COEFF_PER_SPIN * spin_ratio, MAX_LIFT_COEFF)
    lift = 0.5 * AIR_DENSITY * area * cl * speed * speed / BALL_MASS_KG
    ax += lift * (-vz / speed)
    az += lift * (vx / speed)
    return ax, az


# Returns dict mirroring ShotResult; range_at_rim is None when the ball never drops to the rim.
def simulate(flywheel_rps, hood_deg, target_distance_m, radial_vel_mps):
    elevation = math.radians(HOOD_ZERO_ELEVATION_DEG - hood_deg)
    v0 = exit_speed(flywheel_rps)
    omega = backspin_rad_per_sec(flywheel_rps)

    x, z = 0.0, LAUNCH_HEIGHT_M
    vx = v0 * math.cos(elevation) + radial_vel_mps
    vz = v0 * math.sin(elevation)
    t = 0.0
    past_apex = False

    while t < MAX_TIME:
        px, pz, pvz = x, z, vz
        a1 = _accel_2d(vx, vz, omega)
        k1x, k1z, k1vx, k1vz = vx, vz, a1[0], a1[1]
        vx2, vz2 = vx + 0.5 * DT * k1vx, vz + 0.5 * DT * k1vz
        a2 = _accel_2d(vx2, vz2, omega)
        k2x, k2z, k2vx, k2vz = vx2, vz2, a2[0], a2[1]
        vx3, vz3 = vx + 0.5 * DT * k2vx, vz + 0.5 * DT * k2vz
        a3 = _accel_2d(vx3, vz3, omega)
        k3x, k3z, k3vx, k3vz = vx3, vz3, a3[0], a3[1]
        vx4, vz4 = vx + DT * k3vx, vz + DT * k3vz
        a4 = _accel_2d(vx4, vz4, omega)
        k4x, k4z, k4vx, k4vz = vx4, vz4, a4[0], a4[1]

        x += DT * (k1x + 2 * k2x + 2 * k3x + k4x) / 6.0
        z += DT * (k1z + 2 * k2z + 2 * k3z + k4z) / 6.0
        vx += DT * (k1vx + 2 * k2vx + 2 * k3vx + k4vx) / 6.0
        vz += DT * (k1vz + 2 * k2vz + 2 * k3vz + k4vz) / 6.0
        t += DT

        if pvz > 0 and vz <= 0:
            past_apex = True
        if past_apex and pz >= GOAL_HEIGHT_M and z < GOAL_HEIGHT_M:
            frac = (pz - GOAL_HEIGHT_M) / (pz - z)
            x_cross = px + frac * (x - px)
            t_cross = (t - DT) + frac * DT
            margin = GOAL_OPENING_RADIUS_M - abs(x_cross - target_distance_m)
            entry = math.degrees(math.atan2(-vz, abs(vx)))
            return {"scored": margin >= 0.0, "range": x_cross, "tof": t_cross,
                    "margin": margin, "entry": entry}
        if z < 0:
            break
    return {"scored": False, "range": None, "tof": None, "margin": -1.0, "entry": 0.0}


# ----------------------- SOLVER (mirror ShotSolver) -----------------------
HOOD_MIN_DEG, HOOD_MAX_DEG, HOOD_STEP_DEG = 0.0, 32.0, 0.5
FW_MIN_RPS, FW_MAX_RPS = 12.0, 75.0
MIN_ENTRY_ANGLE_DEG = 25.0
HOOD_WINDOW_SCAN_MAX_DEG, HOOD_WINDOW_STEP_DEG = 45.0, 0.25


def _range_error(fw, hood, d, vr):
    r = simulate(fw, hood, d, vr)
    if r["range"] is None:
        return -1000.0
    return r["range"] - d


def on_target_flywheel(hood, d, vr):
    lo, hi = FW_MIN_RPS, FW_MAX_RPS
    if _range_error(lo, hood, d, vr) > 0:
        return None
    if _range_error(hi, hood, d, vr) < 0:
        return None
    i = 0
    while i < 40 and (hi - lo) > 0.05:
        mid = 0.5 * (lo + hi)
        if _range_error(mid, hood, d, vr) < 0:
            lo = mid
        else:
            hi = mid
        i += 1
    result = 0.5 * (lo + hi)
    check = simulate(result, hood, d, vr)
    if check["range"] is None or abs(check["range"] - d) > 0.1:
        return None
    return result


def _lands_in_opening(fw, hood, d, vr):
    r = simulate(fw, hood, d, vr)
    return r["range"] is not None and abs(r["range"] - d) <= GOAL_OPENING_RADIUS_M


def speed_window_rps(hood, d, vr):
    a = on_target_flywheel(hood, d - GOAL_OPENING_RADIUS_M, vr)
    b = on_target_flywheel(hood, d + GOAL_OPENING_RADIUS_M, vr)
    if a is None or b is None:
        return 0.0
    return abs(b - a)


def hood_window_deg(fw, d, vr, nominal_hood):
    lo = hi = nominal_hood
    h = nominal_hood
    while h >= 0.0:
        if _lands_in_opening(fw, h, d, vr):
            lo = h
        else:
            break
        h -= HOOD_WINDOW_STEP_DEG
    h = nominal_hood
    while h <= HOOD_WINDOW_SCAN_MAX_DEG:
        if _lands_in_opening(fw, h, d, vr):
            hi = h
        else:
            break
        h += HOOD_WINDOW_STEP_DEG
    return hi - lo


def solve_at_hood(d, vr, hood):
    fw = on_target_flywheel(hood, d, vr)
    if fw is None:
        return None
    r = simulate(fw, hood, d, vr)
    if not r["scored"]:
        return None
    sw = speed_window_rps(hood, d, vr)
    hw = hood_window_deg(fw, d, vr, hood)
    return {"hood": hood, "fw": fw, "tof": r["tof"], "entry": r["entry"],
            "speed_window": sw, "hood_window": hw, "robustness": sw * hw}


def solve(d, vr):
    best = None
    h = HOOD_MIN_DEG
    while h <= HOOD_MAX_DEG + 1e-9:
        shot = solve_at_hood(d, vr, h)
        if shot and shot["entry"] >= MIN_ENTRY_ANGLE_DEG:
            if best is None or shot["robustness"] > best["robustness"]:
                best = shot
        h += HOOD_STEP_DEG
    return best


# ----------------------- GENERATOR (mirror ShotMapGenerator) -----------------------
MIN_DIST_M, MAX_DIST_M, DIST_STEP_M = 1.0, 6.5, 0.5
MIN_RADIAL_VEL, MAX_RADIAL_VEL, RADIAL_VEL_STEP = -2.0, 4.0, 1.0
N_DIST = round((MAX_DIST_M - MIN_DIST_M) / DIST_STEP_M) + 1
N_VEL = round((MAX_RADIAL_VEL - MIN_RADIAL_VEL) / RADIAL_VEL_STEP) + 1


def distance_at(i):
    return MIN_DIST_M + i * DIST_STEP_M


def radial_vel_at(j):
    return MIN_RADIAL_VEL + j * RADIAL_VEL_STEP


def _edge_fill(grid):
    for row in grid:
        feasible = [j for j, v in enumerate(row) if v is not None]
        first, last = feasible[0], feasible[-1]
        for j in range(first):
            row[j] = row[first]
        for j in range(last + 1, len(row)):
            row[j] = row[last]


def generate():
    import numpy as np

    dists = [distance_at(i) for i in range(N_DIST)]
    hoods = []
    for d in dists:
        shot = solve(d, 0.0)
        if shot is None:
            raise RuntimeError(f"no robust shot at {d} m")
        hoods.append(shot["hood"])
    # quadratic fit hood(d): numpy returns highest-power first; flip to [c0, c1, c2]
    hood_coeffs = list(np.polyfit(dists, hoods, 2)[::-1])

    flywheel = [[None] * N_VEL for _ in range(N_DIST)]
    tof = [[None] * N_VEL for _ in range(N_DIST)]
    feasible = 0
    for i, d in enumerate(dists):
        hood = hood_coeffs[0] + hood_coeffs[1] * d + hood_coeffs[2] * d * d
        for j in range(N_VEL):
            vr = radial_vel_at(j)
            fw = on_target_flywheel(hood, d, vr)
            if fw is None:
                continue
            flywheel[i][j] = fw
            tof[i][j] = simulate(fw, hood, d, vr)["tof"]
            feasible += 1
    _edge_fill(flywheel)
    _edge_fill(tof)
    return {"hood_coeffs": hood_coeffs, "flywheel": flywheel, "tof": tof,
            "feasible": feasible, "total": N_DIST * N_VEL}


# ----------------------- RUNTIME MAP (mirror ShooterMap) -----------------------
class ShooterMap:
    def __init__(self, generated):
        self.hood_coeffs = generated["hood_coeffs"]
        self.flywheel = generated["flywheel"]
        self.tof = generated["tof"]

    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))

    def _bilinear(self, grid, d, vr):
        d = self._clamp(d, MIN_DIST_M, MAX_DIST_M)
        vr = self._clamp(vr, MIN_RADIAL_VEL, MAX_RADIAL_VEL)
        fi = (d - MIN_DIST_M) / DIST_STEP_M
        fj = (vr - MIN_RADIAL_VEL) / RADIAL_VEL_STEP
        i = min(max(int(math.floor(fi)), 0), N_DIST - 2)
        j = min(max(int(math.floor(fj)), 0), N_VEL - 2)
        td, tv = fi - i, fj - j
        v00, v01 = grid[i][j], grid[i][j + 1]
        v10, v11 = grid[i + 1][j], grid[i + 1][j + 1]
        top = v00 + (v01 - v00) * tv
        bot = v10 + (v11 - v10) * tv
        return top + (bot - top) * td

    def hood_deg(self, d):
        d = self._clamp(d, MIN_DIST_M, MAX_DIST_M)
        c = self.hood_coeffs
        return c[0] + c[1] * d + c[2] * d * d

    def flywheel_rps(self, d, vr):
        return self._bilinear(self.flywheel, d, vr)

    def tof_seconds(self, d, vr):
        return self._bilinear(self.tof, d, vr)

    def in_envelope(self, d, vr):
        return MIN_DIST_M <= d <= MAX_DIST_M and MIN_RADIAL_VEL <= vr <= MAX_RADIAL_VEL


# ----------------------- SWM TARGETING (mirror SwmTargeting) -----------------------
MIN_EFFECTIVE_RADIAL_SPEED = 0.5


def _k_drag():
    r = BALL_DIAMETER_M / 2.0
    return 0.5 * AIR_DENSITY * DRAG_COEFFICIENT * (math.pi * r * r)


def field_to_robot(x, y, z, yaw, pitch, roll):
    cz, sz = math.cos(yaw), math.sin(yaw)
    x1, y1, z1 = x * cz + y * sz, -x * sz + y * cz, z
    cp, sp = math.cos(pitch), math.sin(pitch)
    x2, z2 = x1 * cp - z1 * sp, x1 * sp + z1 * cp
    cr, sr = math.cos(roll), math.sin(roll)
    y3, z3 = y1 * cr + z2 * sr, -y1 * sr + z2 * cr
    return [x2, y3, z3]


def robot_to_field(x, y, z, yaw, pitch, roll):
    cr, sr = math.cos(roll), math.sin(roll)
    y1, z1 = y * cr - z * sr, y * sr + z * cr
    cp, sp = math.cos(pitch), math.sin(pitch)
    x2, z2 = x * cp + z1 * sp, -x * sp + z1 * cp
    cz, sz = math.cos(yaw), math.sin(yaw)
    x3, y3 = x2 * cz - y1 * sz, x2 * sz + y1 * cz
    return [x3, y3, z2]


def _input_modulus(v, lo, hi):
    mod = hi - lo
    r = (v - lo) % mod
    if r < 0:
        r += mod
    return r + lo


def swm_solve(smap, turret_x, turret_y, turret_vx, turret_vy, target_x, target_y,
              yaw, pitch, roll, slip_efficiency=None):
    """3D-vector shoot-while-moving + tilt.

    Give the ball the exact field velocity of the stationary scoring shot, then have the shooter
    provide (desired ball velocity - inherited robot velocity). This is exact for any robot motion
    (the ball flies the same field trajectory as a stationary shot at this distance) and preserves
    the robust entry angle; radial and tangential are handled together with no lead approximation.
    """
    if slip_efficiency is None:
        slip_efficiency = SLIP_EFFICIENCY  # reference (baked) slip
    dx, dy = target_x - turret_x, target_y - turret_y
    dist = math.hypot(dx, dy)
    azimuth_to_target = math.atan2(dy, dx)

    # Desired ball velocity (field) = the stationary scoring shot at this distance.
    hood_level = smap.hood_deg(dist)
    v0 = exit_speed(smap.flywheel_rps(dist, 0.0))
    elevation = math.radians(HOOD_ZERO_ELEVATION_DEG - hood_level)
    ce = math.cos(elevation)
    desired_x = v0 * ce * math.cos(azimuth_to_target)
    desired_y = v0 * ce * math.sin(azimuth_to_target)
    desired_z = v0 * math.sin(elevation)

    # Shooter must supply desired - inherited (robot velocity is horizontal).
    sx = desired_x - turret_vx
    sy = desired_y - turret_vy
    sz = desired_z
    shooter_speed = math.sqrt(sx * sx + sy * sy + sz * sz)
    flywheel = shooter_speed / (slip_efficiency * 2.0 * math.pi * WHEEL_RADIUS_M)

    # Shooter direction (field) -> robot frame for the turret/hood commands (tilt compensation).
    inv = 1.0 / shooter_speed
    rd = field_to_robot(sx * inv, sy * inv, sz * inv, yaw, pitch, roll)
    azimuth_robot = math.atan2(rd[1], rd[0])
    elevation_robot = math.atan2(rd[2], math.hypot(rd[0], rd[1]))
    hood_deg = HOOD_ZERO_ELEVATION_DEG - math.degrees(elevation_robot)
    turret_rot = _input_modulus(azimuth_robot / (2 * math.pi), -0.25, 0.75)

    tof = smap.tof_seconds(dist, 0.0)
    v_radial = turret_vx * math.cos(azimuth_to_target) + turret_vy * math.sin(azimuth_to_target)
    feasible = (smap.in_envelope(dist, 0.0)
                and FW_MIN_RPS <= flywheel <= FW_MAX_RPS
                and HOOD_MIN_DEG <= hood_deg <= HOOD_MAX_DEG)
    return {"turret_rot": turret_rot, "hood_deg": hood_deg, "flywheel_rps": flywheel,
            "tof": tof, "dist": dist, "v_radial": v_radial, "feasible": feasible}
