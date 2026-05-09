"""
Per-module slip/skid physics for swerve drive.

Reference implementation in Python — used to verify correctness before
porting to Java. Same constants and math will be mirrored in
AccelerationLimitedFieldSpeeds.java.

Coordinate convention:
  Robot frame: +X forward, +Y left (FRC standard).
  Field frame: +X away from blue alliance, +Y to blue's left.
  Heading: angle of robot's +X axis measured CCW from field +X.

Module layout (4 modules at corners of a rectangle):
  FL = (+a, +b),  FR = (+a, -b)
  BL = (-a, +b),  BR = (-a, -b)
where a = half-wheelbase, b = half-trackwidth.

Center of gravity at (cx, cy, cz) in the robot frame, measured from the
geometric center of the wheelbase rectangle, with cz above the ground.
"""

from dataclasses import dataclass
from math import cos, hypot, sin
from typing import Sequence


@dataclass(frozen=True)
class RobotPhysics:
    mass: float          # kg
    mu: float            # friction coefficient
    g: float             # m/s^2
    half_wheelbase: float  # a, meters (half front-to-rear distance)
    half_trackwidth: float # b, meters (half left-to-right distance)
    cog_x: float         # cx, meters from geometric center (+forward)
    cog_y: float         # cy, meters from geometric center (+left)
    cog_z: float         # cz, meters above ground
    module_positions: Sequence[tuple[float, float]]  # 4 modules in robot frame


def default_physics() -> RobotPhysics:
    """Matches TunerConstants.java module layout and AccelerationLimiter constants."""
    a = 0.276225  # 10.875 in
    b = 0.276225
    return RobotPhysics(
        mass=60.0,
        mu=1.1,
        g=9.81,
        half_wheelbase=a,
        half_trackwidth=b,
        cog_x=0.0,    # tune via balance test
        cog_y=0.0,    # tune via balance test
        cog_z=0.20,   # tune via tilt test
        module_positions=((+a, +b), (+a, -b), (-a, +b), (-a, -b)),
    )


def static_normal_forces(phys: RobotPhysics) -> list[float]:
    """
    Static N_i at each module given CoG (cx, cy).

    Derivation: load shifts proportionally to CoG offset within the rectangle.
    For module at (rx, ry):
      N_static = (mg/4) * (1 + rx*cx/a^2) * (1 + ry*cy/b^2)

    This product form satisfies vertical equilibrium and the front-rear / left-right
    moment balances. (For arbitrary CoG positions it implies a small cross-moment
    of mg*cx*cy that real rigid 4-wheel vehicles also exhibit.)
    """
    a = phys.half_wheelbase
    b = phys.half_trackwidth
    cx = phys.cog_x
    cy = phys.cog_y
    base = phys.mass * phys.g / 4.0
    return [
        base * (1.0 + rx * cx / (a * a)) * (1.0 + ry * cy / (b * b))
        for (rx, ry) in phys.module_positions
    ]


def normal_forces_with_transfer(
    phys: RobotPhysics, ax_robot: float, ay_robot: float
) -> list[float]:
    """
    N_i with both static (cx, cy) load distribution and dynamic weight transfer
    from CoG height during acceleration.

    Dynamic: forward acceleration shifts load to rear modules by m*ax*cz/(2a)
    (split across the two front modules, so half each).
    """
    statics = static_normal_forces(phys)
    a = phys.half_wheelbase
    b = phys.half_trackwidth
    cz = phys.cog_z
    out = []
    for n_static, (rx, ry) in zip(statics, phys.module_positions):
        # Load shift to the wheels OPPOSITE the acceleration direction.
        # Front modules (rx > 0): lose grip during +ax (forward accel).
        sign_x = 1.0 if rx > 0 else -1.0 if rx < 0 else 0.0
        sign_y = 1.0 if ry > 0 else -1.0 if ry < 0 else 0.0
        delta_x = phys.mass * ax_robot * cz * sign_x / (2.0 * a) / 2.0
        delta_y = phys.mass * ay_robot * cz * sign_y / (2.0 * b) / 2.0
        # The /2 at the end splits the per-axle delta across the two wheels on that axle.
        n = n_static - delta_x - delta_y
        out.append(max(n, 0.0))
    return out


def per_module_acceleration(
    phys: RobotPhysics, ax_robot: float, ay_robot: float, alpha: float
) -> list[tuple[float, float]]:
    """
    Linear acceleration vector at each module in robot frame.
      a_i = a_translation + alpha x r_i
      a_ix = ax - alpha * ry
      a_iy = ay + alpha * rx
    """
    return [
        (ax_robot - alpha * ry, ay_robot + alpha * rx)
        for (rx, ry) in phys.module_positions
    ]


def field_to_robot(ax_field: float, ay_field: float, heading: float) -> tuple[float, float]:
    """Rotate a field-frame vector into the robot frame given heading (radians)."""
    c = cos(heading)
    s = sin(heading)
    return ax_field * c + ay_field * s, -ax_field * s + ay_field * c


def robot_to_field(ax_robot: float, ay_robot: float, heading: float) -> tuple[float, float]:
    c = cos(heading)
    s = sin(heading)
    return ax_robot * c - ay_robot * s, ax_robot * s + ay_robot * c


def per_module_friction_check(
    phys: RobotPhysics,
    ax_field: float,
    ay_field: float,
    alpha: float,
    heading: float,
    prev_ax_robot: float = 0.0,
    prev_ay_robot: float = 0.0,
) -> tuple[float, float, float, list[float]]:
    """
    Apply per-module friction limit. Returns (ax_field, ay_field, alpha, ratios)
    where ratios[i] is |a_i| / friction_limit_i for each module before scaling.

    Algorithm:
      1. Rotate field accel to robot frame.
      2. Compute per-module accel vectors.
      3. Use previous-frame robot-frame accel to estimate weight transfer (decouples
         the nonlinear coupling between limit and accel — converges in 1-2 frames).
      4. Worst-module ratio determines uniform scale factor.
    """
    ax_r, ay_r = field_to_robot(ax_field, ay_field, heading)
    normals = normal_forces_with_transfer(phys, prev_ax_robot, prev_ay_robot)
    accels = per_module_acceleration(phys, ax_r, ay_r, alpha)

    worst_ratio = 0.0
    ratios: list[float] = []
    for (a_ix, a_iy), n in zip(accels, normals):
        # Per-module friction limit, expressed as max acceleration:
        # F_max = mu * N, equivalent accel = F_max / (m/4) = mu * 4 * N / m
        limit = phys.mu * 4.0 * n / phys.mass
        mag = hypot(a_ix, a_iy)
        ratio = mag / limit if limit > 1e-9 else float("inf")
        ratios.append(ratio)
        worst_ratio = max(worst_ratio, ratio)

    if worst_ratio > 1.0:
        scale = 1.0 / worst_ratio
        ax_field *= scale
        ay_field *= scale
        alpha *= scale

    return ax_field, ay_field, alpha, ratios


def old_scalar_friction_check(
    phys: RobotPhysics, ax: float, ay: float, alpha: float, drive_base_radius: float
) -> tuple[float, float, float]:
    """
    The current AccelerationLimiter scalar check, for comparison.
      hypot(linear, alpha * R) <= mu * g
    """
    linear = hypot(ax, ay)
    angular = abs(alpha) * drive_base_radius
    combined = hypot(linear, angular)
    limit = phys.mu * phys.g
    if combined > limit:
        scale = limit / combined
        return ax * scale, ay * scale, alpha * scale
    return ax, ay, alpha
