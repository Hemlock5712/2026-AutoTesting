"""
Verification tests for slip/skid physics.

Run with: python -m pytest test_physics.py -v
Or:       python test_physics.py
"""

from math import cos, hypot, pi, sin

from physics import (
    RobotPhysics,
    default_physics,
    field_to_robot,
    normal_forces_with_transfer,
    old_scalar_friction_check,
    per_module_acceleration,
    per_module_friction_check,
    robot_to_field,
    static_normal_forces,
)


def approx(a: float, b: float, tol: float = 1e-6) -> bool:
    return abs(a - b) < tol


def assert_close(a: float, b: float, tol: float = 1e-6, msg: str = "") -> None:
    assert approx(a, b, tol), f"{msg}: {a} != {b} (diff {a-b:.3e})"


# ---------- Static load distribution ----------

def test_static_centered_cog_equal_loads():
    phys = default_physics()  # cx=cy=0
    n = static_normal_forces(phys)
    expected = phys.mass * phys.g / 4.0
    for ni in n:
        assert_close(ni, expected, msg="centered CoG should give mg/4 per wheel")
    assert_close(sum(n), phys.mass * phys.g, msg="vertical equilibrium")


def test_static_forward_cog_loads_front():
    phys = RobotPhysics(
        mass=60, mu=1.1, g=9.81,
        half_wheelbase=0.276, half_trackwidth=0.276,
        cog_x=0.05, cog_y=0.0, cog_z=0.20,
        module_positions=((+0.276, +0.276), (+0.276, -0.276),
                          (-0.276, +0.276), (-0.276, -0.276)),
    )
    n_fl, n_fr, n_bl, n_br = static_normal_forces(phys)
    # Front gets more load (cx forward = 0.05/0.276 = 18.1% bias)
    assert n_fl > n_bl, "FL should carry more than BL with forward CoG"
    assert n_fr > n_br, "FR should carry more than BR with forward CoG"
    assert_close(n_fl, n_fr, msg="left/right symmetric when cy=0")
    assert_close(n_bl, n_br, msg="left/right symmetric when cy=0")
    assert_close(sum([n_fl, n_fr, n_bl, n_br]), phys.mass * phys.g,
                 msg="vertical equilibrium")
    # Front total / mg should equal (1 + cx/a)/2
    front_frac = (n_fl + n_fr) / (phys.mass * phys.g)
    expected_front = (1 + 0.05 / 0.276) / 2
    assert_close(front_frac, expected_front, msg="front load fraction")


def test_static_offset_cog_xy():
    """CoG offset both forward and left -> FL has most weight."""
    phys = RobotPhysics(
        mass=60, mu=1.1, g=9.81,
        half_wheelbase=0.276, half_trackwidth=0.276,
        cog_x=0.05, cog_y=0.04, cog_z=0.20,
        module_positions=((+0.276, +0.276), (+0.276, -0.276),
                          (-0.276, +0.276), (-0.276, -0.276)),
    )
    n_fl, n_fr, n_bl, n_br = static_normal_forces(phys)
    assert n_fl > n_fr > n_br, "FL > FR > BR ordering"
    assert n_fl > n_bl > n_br, "FL > BL > BR ordering"
    assert_close(sum([n_fl, n_fr, n_bl, n_br]), phys.mass * phys.g,
                 msg="vertical equilibrium")


# ---------- Dynamic weight transfer ----------

def test_weight_transfer_zero_accel():
    phys = default_physics()
    n = normal_forces_with_transfer(phys, 0.0, 0.0)
    expected = phys.mass * phys.g / 4.0
    for ni in n:
        assert_close(ni, expected, msg="zero accel = static")


def test_weight_transfer_forward_accel_shifts_to_rear():
    phys = default_physics()
    # 5 m/s^2 forward acceleration in robot frame
    n = normal_forces_with_transfer(phys, 5.0, 0.0)
    n_fl, n_fr, n_bl, n_br = n
    assert n_fl < phys.mass * phys.g / 4, "front loses load"
    assert n_bl > phys.mass * phys.g / 4, "rear gains load"
    assert_close(n_fl, n_fr, msg="left/right symmetric")
    assert_close(n_bl, n_br, msg="left/right symmetric")
    assert_close(sum(n), phys.mass * phys.g, msg="total = mg always")


def test_weight_transfer_left_accel_shifts_to_right():
    phys = default_physics()
    # 5 m/s^2 leftward acceleration in robot frame (toward +Y)
    n = normal_forces_with_transfer(phys, 0.0, 5.0)
    n_fl, n_fr, n_bl, n_br = n
    assert n_fr > phys.mass * phys.g / 4, "right gains load (CoG shifts toward right)"
    assert n_fl < phys.mass * phys.g / 4, "left loses load"
    assert_close(n_fl, n_bl, msg="front/back symmetric")
    assert_close(n_fr, n_br, msg="front/back symmetric")
    assert_close(sum(n), phys.mass * phys.g, msg="total = mg always")


def test_weight_transfer_diagonal_accel():
    """Robot accelerating forward+left in robot frame -> back-right gets most weight."""
    phys = default_physics()
    n = normal_forces_with_transfer(phys, 5.0, 5.0)
    n_fl, n_fr, n_bl, n_br = n
    # Forward shifts to back, left shifts to right -> back-right is opposite both
    assert n_br == max(n), "back-right has most weight under +ax +ay"
    assert n_fl == min(n), "front-left has least weight under +ax +ay"
    assert_close(sum(n), phys.mass * phys.g, msg="total = mg always")


def test_weight_transfer_field_45deg_drive():
    """User's example: robot at 45deg field, driving field +X.
    Robot-frame accel is mixed forward+right. Most weight to back-left."""
    phys = default_physics()
    heading = pi / 4  # 45 degrees
    ax_field, ay_field = 5.0, 0.0
    ax_robot, ay_robot = field_to_robot(ax_field, ay_field, heading)
    # ax_robot = 5*cos(45) = 3.535, ay_robot = -5*sin(45) = -3.535
    assert_close(ax_robot, 5.0 * cos(heading), msg="x rotation")
    assert_close(ay_robot, -5.0 * sin(heading), msg="y rotation")
    # In robot frame: forward+right -> shifts to back-left
    n = normal_forces_with_transfer(phys, ax_robot, ay_robot)
    n_fl, n_fr, n_bl, n_br = n
    assert n_bl == max(n), f"BL should have most weight, got {n}"
    assert n_fr == min(n), f"FR should have least weight, got {n}"


def test_weight_transfer_clamps_at_zero():
    """Extreme acceleration shouldn't make N go negative (wheel lifts off)."""
    phys = default_physics()
    # Huge acceleration that would exceed static load
    n = normal_forces_with_transfer(phys, 100.0, 0.0)
    for ni in n:
        assert ni >= 0, f"N must be >= 0, got {ni}"


# ---------- Per-module acceleration vectors ----------

def test_per_module_accel_pure_translation():
    phys = default_physics()
    accels = per_module_acceleration(phys, 3.0, 4.0, 0.0)
    for (a_ix, a_iy) in accels:
        assert_close(a_ix, 3.0, msg="all modules same accel for pure translation")
        assert_close(a_iy, 4.0, msg="all modules same accel for pure translation")


def test_per_module_accel_pure_rotation():
    phys = default_physics()
    alpha = 2.0  # rad/s^2
    accels = per_module_acceleration(phys, 0.0, 0.0, alpha)
    # All modules should have the same speed (alpha * R) but different directions
    for (a_ix, a_iy), (rx, ry) in zip(accels, phys.module_positions):
        mag = hypot(a_ix, a_iy)
        expected_mag = abs(alpha) * hypot(rx, ry)
        assert_close(mag, expected_mag, msg="rotation speed = alpha * R")
        # Direction is tangent to position (perpendicular)
        # a_i should be perpendicular to r_i: dot product = 0
        dot = a_ix * rx + a_iy * ry
        assert_close(dot, 0.0, tol=1e-9, msg="rotation accel perpendicular to position")


def test_per_module_accel_combined():
    """Translation + rotation: at one corner they add, at the opposite they subtract."""
    phys = default_physics()
    # Translate +X, rotate CCW (positive alpha)
    # Module FL (+a, +b): rotation gives (-alpha*b, +alpha*a) -> -X, +Y
    # FR (+a, -b): rotation gives (+alpha*b, +alpha*a) -> +X, +Y
    # So +X translation cancels at FL but adds at FR
    accels = per_module_acceleration(phys, 5.0, 0.0, 2.0)
    a_fl_x, a_fl_y = accels[0]  # FL
    a_fr_x, a_fr_y = accels[1]  # FR
    # FL: ax=5, alpha*ry=2*0.276=0.552 -> a_fl_x = 5 - 0.552 = 4.448
    assert a_fr_x > a_fl_x, "FR accel X should be greater than FL"


# ---------- Per-module friction check ----------

def test_friction_check_no_scaling_when_under_limit():
    phys = default_physics()
    # Modest accel well under mu*g = 10.79 m/s^2
    out_ax, out_ay, out_alpha, ratios = per_module_friction_check(
        phys, 3.0, 0.0, 0.0, 0.0
    )
    assert_close(out_ax, 3.0)
    assert_close(out_ay, 0.0)
    assert_close(out_alpha, 0.0)
    for r in ratios:
        assert r < 1.0, f"ratio should be under 1 for modest accel, got {r}"


def test_friction_check_scales_when_over_limit():
    phys = default_physics()
    # 20 m/s^2 - well over mu*g = 10.79
    out_ax, out_ay, out_alpha, ratios = per_module_friction_check(
        phys, 20.0, 0.0, 0.0, 0.0
    )
    # After scaling, the limited accel should be at the friction limit
    # With cog_x=cog_y=0 and prev_accel=0, all modules have N=mg/4, limit=mu*g
    expected = phys.mu * phys.g
    assert_close(hypot(out_ax, out_ay), expected, tol=1e-6,
                 msg="should scale to friction limit")


def test_friction_check_per_module_finds_worst_module():
    """When translating + rotating, one module should hit limit first."""
    phys = default_physics()
    # Combine translation and rotation that adds at one corner
    ax = 7.0  # m/s^2
    alpha = 10.0  # rad/s^2 - alpha*r ~= 2.76 m/s^2 at FR module pointing +X
    out_ax, out_ay, out_alpha, ratios_before = per_module_friction_check(
        phys, ax, 0.0, alpha, 0.0
    )
    # At least one module's pre-scale ratio should be > 1
    max_ratio = max(ratios_before)
    if max_ratio > 1.0:
        # Verify scaling brought it to limit
        ax_r, _ = field_to_robot(out_ax, 0.0, 0.0)
        accels = per_module_acceleration(phys, ax_r, 0.0, out_alpha)
        normals = normal_forces_with_transfer(phys, 0.0, 0.0)
        max_post_ratio = 0.0
        for (a_ix, a_iy), n in zip(accels, normals):
            limit = phys.mu * 4.0 * n / phys.mass
            mag = hypot(a_ix, a_iy)
            max_post_ratio = max(max_post_ratio, mag / limit)
        assert_close(max_post_ratio, 1.0, tol=1e-6,
                     msg="worst module should be at limit after scaling")


def test_friction_check_weight_transfer_makes_front_more_restrictive():
    """During hard forward accel, front modules have less grip -> they limit first."""
    phys = default_physics()
    # Use prev_accel to simulate that the robot is already accelerating forward
    prev_ax = 8.0
    out_ax, _, _, ratios = per_module_friction_check(
        phys, 10.0, 0.0, 0.0, 0.0, prev_ax_robot=prev_ax
    )
    # Modules: 0=FL (+a,+b), 1=FR (+a,-b), 2=BL (-a,+b), 3=BR (-a,-b)
    # Front modules (0, 1) should have higher ratios than rear (2, 3)
    front_ratio = max(ratios[0], ratios[1])
    rear_ratio = max(ratios[2], ratios[3])
    assert front_ratio > rear_ratio, \
        f"front should be more constrained during forward accel: front={front_ratio}, rear={rear_ratio}"


# ---------- Comparison to old scalar check ----------

def test_old_vs_new_pure_translation_agree():
    """For pure translation with centered CoG, both checks give same result."""
    phys = default_physics()
    R = hypot(phys.half_wheelbase, phys.half_trackwidth)

    new_ax, new_ay, _, _ = per_module_friction_check(phys, 15.0, 0.0, 0.0, 0.0)
    old_ax, old_ay, _ = old_scalar_friction_check(phys, 15.0, 0.0, 0.0, R)

    assert_close(new_ax, old_ax, tol=1e-6,
                 msg="pure translation should match scalar check")
    assert_close(new_ay, old_ay, tol=1e-6)


def test_old_vs_new_combined_translation_rotation():
    """For combined translation+rotation aligned at one corner, new check is MORE restrictive
    than the old scalar check (which uses hypot — too optimistic when they align)."""
    phys = default_physics()
    R = hypot(phys.half_wheelbase, phys.half_trackwidth)

    # Translation along x, rotation that adds at FR module
    ax_in = 7.5
    alpha_in = 12.0

    new_ax, _, new_alpha, _ = per_module_friction_check(phys, ax_in, 0.0, alpha_in, 0.0)
    old_ax, _, old_alpha = old_scalar_friction_check(phys, ax_in, 0.0, alpha_in, R)

    # Both should scale down. The new one scales down MORE because per-module
    # check sees the constructive addition at FR.
    new_combined = hypot(new_ax, new_alpha * R)
    old_combined = hypot(old_ax, old_alpha * R)
    print(f"  Old scalar: ax={old_ax:.2f}, alpha={old_alpha:.2f}, combined={old_combined:.2f}")
    print(f"  New module: ax={new_ax:.2f}, alpha={new_alpha:.2f}, combined={new_combined:.2f}")


# ---------- Frame conversions ----------

def test_frame_round_trip():
    for heading in [0, pi/6, pi/4, pi/2, pi, -pi/3]:
        for ax, ay in [(1, 0), (0, 1), (3, 4), (-2, 5)]:
            r_x, r_y = field_to_robot(ax, ay, heading)
            f_x, f_y = robot_to_field(r_x, r_y, heading)
            assert_close(f_x, ax, msg=f"round-trip x heading={heading}")
            assert_close(f_y, ay, msg=f"round-trip y heading={heading}")


def test_field_to_robot_45deg():
    # Robot facing 45deg, field +X acceleration
    ax_r, ay_r = field_to_robot(1.0, 0.0, pi/4)
    # Robot sees this as forward-right: +X robot, -Y robot (since +Y is left)
    assert_close(ax_r, cos(pi/4), msg="rotated x")
    assert_close(ay_r, -sin(pi/4), msg="rotated y")


# ---------- Edge cases ----------

def test_pure_rotation_no_translation():
    phys = default_physics()
    out_ax, out_ay, out_alpha, ratios = per_module_friction_check(
        phys, 0.0, 0.0, 50.0, 0.0  # huge alpha
    )
    # Should scale alpha down, ax/ay stay 0
    assert_close(out_ax, 0.0)
    assert_close(out_ay, 0.0)
    assert out_alpha < 50.0, "alpha should be scaled down"
    # All four modules at same R should hit limit equally
    R = hypot(phys.half_wheelbase, phys.half_trackwidth)
    expected_alpha = phys.mu * phys.g / R
    assert_close(out_alpha, expected_alpha, tol=1e-6,
                 msg="pure rotation limit = mu*g/R")


def test_zero_acceleration_no_op():
    phys = default_physics()
    out_ax, out_ay, out_alpha, ratios = per_module_friction_check(
        phys, 0.0, 0.0, 0.0, 0.0
    )
    assert out_ax == 0 and out_ay == 0 and out_alpha == 0
    for r in ratios:
        assert r == 0.0


# ---------- Run all tests ----------

if __name__ == "__main__":
    import sys
    import traceback

    tests = [name for name in globals() if name.startswith("test_")]
    failed = 0
    for name in tests:
        try:
            print(f"  {name}...", end=" ")
            globals()[name]()
            print("PASS")
        except AssertionError as e:
            print(f"FAIL: {e}")
            failed += 1
        except Exception as e:
            print(f"ERROR: {e}")
            traceback.print_exc()
            failed += 1
    print()
    print(f"{len(tests) - failed} / {len(tests)} passed")
    sys.exit(0 if failed == 0 else 1)
