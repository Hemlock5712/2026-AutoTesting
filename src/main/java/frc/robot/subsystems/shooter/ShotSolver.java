package frc.robot.subsystems.shooter;

import frc.robot.subsystems.shooter.ShotPhysics.ShotResult;

/**
 * Inverts {@link ShotPhysics} to find the flywheel speed (and resulting physical time-of-flight)
 * that lands a ball in the hub at a given target distance and robot radial velocity.
 *
 * <p>Two entry points:
 *
 * <ul>
 *   <li>{@link #solveAtHood} — for a fixed hood angle, bisects the flywheel speed so the ball's
 *       descending rim crossing lands exactly at the target distance. The primitive the table
 *       generator uses to build {@code flywheel(distance, radialVel)} and {@code tof(...)}.
 *   <li>{@link #solve} — also auto-selects the hood, maximizing a robustness metric: the area of the
 *       (flywheel, hood) make-window that still scores. This is HighTide's "shot most robust to
 *       errors" and yields a near-constant ~50 deg entry angle across distances.
 * </ul>
 *
 * <p>Runs offline (table generation / tests), not on the 50 Hz robot loop, so it favors clarity.
 */
public final class ShotSolver {
  private ShotSolver() {}

  public static final double HOOD_MIN_DEG = 0.0;
  public static final double HOOD_MAX_DEG = 32.0;
  public static final double HOOD_STEP_DEG = 0.5;
  public static final double FW_MIN_RPS = 12.0;
  public static final double FW_MAX_RPS = 75.0;

  /** Reject auto-selected shots flatter than this entry angle (they skim rather than drop in). */
  public static final double MIN_ENTRY_ANGLE_DEG = 25.0;

  // Bounds for measuring the hood make-window (physical error tolerance, not command limits).
  private static final double HOOD_WINDOW_SCAN_MAX_DEG = 45.0;
  private static final double HOOD_WINDOW_STEP_DEG = 0.25;

  /** A solved shot. {@code valid} is false when the hood/speed range cannot reach the target. */
  public record Shot(
      boolean valid,
      double hoodDeg,
      double flywheelRps,
      double timeOfFlight,
      double entryAngleDeg,
      double speedWindowRps,
      double hoodWindowDeg,
      double robustness) {
    public static final Shot INVALID = new Shot(false, 0, 0, 0, 0, 0, 0, -Double.MAX_VALUE);
  }

  /**
   * Solve for the on-target flywheel speed at a specific hood angle. Given distance, radial velocity
   * and hood, returns the flywheel speed and physical time-of-flight, plus the make-window sizes.
   * This is the trustworthy primitive used to generate the lookup table from a hood schedule.
   */
  public static Shot solveAtHood(double distanceM, double radialVelMps, double hoodDeg) {
    double fw = onTargetFlywheel(hoodDeg, distanceM, radialVelMps);
    if (Double.isNaN(fw)) {
      return Shot.INVALID;
    }
    ShotResult r = ShotPhysics.simulate(fw, hoodDeg, distanceM, radialVelMps);
    if (!r.scored()) {
      return Shot.INVALID;
    }
    double speedWindow = speedWindowRps(hoodDeg, distanceM, radialVelMps);
    double hoodWindow = hoodWindowDeg(fw, distanceM, radialVelMps, hoodDeg);
    double robustness = speedWindow * hoodWindow;
    return new Shot(
        true, hoodDeg, fw, r.timeOfFlight(), r.entryAngleDeg(), speedWindow, hoodWindow, robustness);
  }

  /**
   * Auto-select the most robust scoring shot, choosing the hood angle as well. Robustness is the
   * make-window area (speed tolerance in RPS x hood tolerance in deg) — the shot that tolerates the
   * most combined flywheel-speed and hood-angle error while still scoring.
   */
  public static Shot solve(double distanceM, double radialVelMps) {
    Shot best = Shot.INVALID;
    for (double hood = HOOD_MIN_DEG; hood <= HOOD_MAX_DEG + 1e-9; hood += HOOD_STEP_DEG) {
      Shot shot = solveAtHood(distanceM, radialVelMps, hood);
      if (!shot.valid() || shot.entryAngleDeg() < MIN_ENTRY_ANGLE_DEG) {
        continue;
      }
      if (shot.robustness() > best.robustness()) {
        best = shot;
      }
    }
    return best;
  }

  /** Width of the flywheel-speed interval that still lands within the hub opening (RPS). */
  static double speedWindowRps(double hoodDeg, double distanceM, double radialVelMps) {
    double r = ShotPhysics.GOAL_OPENING_RADIUS_M;
    double fwNear = onTargetFlywheel(hoodDeg, distanceM - r, radialVelMps); // near edge: less speed
    double fwFar = onTargetFlywheel(hoodDeg, distanceM + r, radialVelMps); // far edge: more speed
    if (Double.isNaN(fwNear) || Double.isNaN(fwFar)) {
      return 0.0;
    }
    return Math.abs(fwFar - fwNear);
  }

  /**
   * Width of the contiguous hood-angle band, at a fixed flywheel speed, that still lands within the
   * hub opening (deg). Measured by expanding outward from the nominal hood.
   */
  static double hoodWindowDeg(
      double flywheelRps, double distanceM, double radialVelMps, double nominalHoodDeg) {
    double lo = nominalHoodDeg;
    double hi = nominalHoodDeg;
    for (double h = nominalHoodDeg; h >= 0.0; h -= HOOD_WINDOW_STEP_DEG) {
      if (landsInOpening(flywheelRps, h, distanceM, radialVelMps)) {
        lo = h;
      } else {
        break;
      }
    }
    for (double h = nominalHoodDeg; h <= HOOD_WINDOW_SCAN_MAX_DEG; h += HOOD_WINDOW_STEP_DEG) {
      if (landsInOpening(flywheelRps, h, distanceM, radialVelMps)) {
        hi = h;
      } else {
        break;
      }
    }
    return hi - lo;
  }

  private static boolean landsInOpening(
      double flywheelRps, double hoodDeg, double distanceM, double radialVelMps) {
    ShotResult r = ShotPhysics.simulate(flywheelRps, hoodDeg, distanceM, radialVelMps);
    return !Double.isNaN(r.rangeAtRim())
        && Math.abs(r.rangeAtRim() - distanceM) <= ShotPhysics.GOAL_OPENING_RADIUS_M;
  }

  /**
   * Bisect the flywheel speed so the descending rim crossing lands at {@code distanceM}. Returns NaN
   * if the target is unreachable within the flywheel speed range for this hood angle.
   */
  static double onTargetFlywheel(double hoodDeg, double distanceM, double radialVelMps) {
    double lo = FW_MIN_RPS;
    double hi = FW_MAX_RPS;
    // rangeError is negative when undershooting (need more speed), positive when overshooting.
    if (rangeError(lo, hoodDeg, distanceM, radialVelMps) > 0) {
      return Double.NaN; // even minimum speed overshoots
    }
    if (rangeError(hi, hoodDeg, distanceM, radialVelMps) < 0) {
      return Double.NaN; // even maximum speed undershoots
    }
    for (int i = 0; i < 40 && (hi - lo) > 0.05; i++) {
      double mid = 0.5 * (lo + hi);
      if (rangeError(mid, hoodDeg, distanceM, radialVelMps) < 0) {
        lo = mid;
      } else {
        hi = mid;
      }
    }
    double result = 0.5 * (lo + hi);
    // Guard the miss/overshoot discontinuity: for a flat shot at close range the minimum-energy
    // shot already overshoots, so no speed lands exactly on target. Reject rather than return the
    // boundary value.
    ShotResult check = ShotPhysics.simulate(result, hoodDeg, distanceM, radialVelMps);
    if (Double.isNaN(check.rangeAtRim()) || Math.abs(check.rangeAtRim() - distanceM) > 0.1) {
      return Double.NaN;
    }
    return result;
  }

  /**
   * Signed range error at the rim crossing: {@code rangeAtRim - distance}. A ball too weak to reach
   * the rim is treated as a large undershoot so bisection drives the speed up.
   */
  private static double rangeError(
      double flywheelRps, double hoodDeg, double distanceM, double radialVelMps) {
    ShotResult r = ShotPhysics.simulate(flywheelRps, hoodDeg, distanceM, radialVelMps);
    if (Double.isNaN(r.rangeAtRim())) {
      return -1000.0;
    }
    return r.rangeAtRim() - distanceM;
  }
}
