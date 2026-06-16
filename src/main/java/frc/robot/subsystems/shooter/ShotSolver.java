package frc.robot.subsystems.shooter;

import frc.robot.subsystems.shooter.ShotPhysics.ShotResult;

/**
 * Inverts {@link ShotPhysics} to find the flywheel speed (and the resulting physical time-of-flight)
 * that lands a ball in the hub at a given target distance and robot radial velocity.
 *
 * <p>The core primitive is {@link #solveAtHood}: for a fixed hood angle it bisects the flywheel
 * speed so the ball's descending rim crossing lands exactly at the target distance. This is what the
 * table generator uses, anchored to a known-good hood schedule, to produce a physically consistent
 * {@code (distance, radial velocity) -> (flywheel, time-of-flight)} map — which both eliminates the
 * hand-fudged ToF multiplier and adds radial-velocity compensation for free.
 *
 * <p>{@link #solve} additionally auto-selects a hood by a robustness metric, but that metric is
 * provisional (see its docs) until we have the real hub opening geometry and field make-window data.
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

  /** A solved shot. {@code valid} is false when the hood/speed range cannot reach the target. */
  public record Shot(
      boolean valid,
      double hoodDeg,
      double flywheelRps,
      double timeOfFlight,
      double entryAngleDeg,
      double speedWindowRps,
      double robustness) {
    public static final Shot INVALID = new Shot(false, 0, 0, 0, 0, 0, -Double.MAX_VALUE);
  }

  /**
   * Solve for the on-target flywheel speed at a specific hood angle. This is the trustworthy
   * primitive used to generate the lookup table from a fixed (known-good) hood schedule: given
   * distance, radial velocity, and hood, it returns the flywheel speed and physical time-of-flight.
   * {@code speedWindowRps} is the flywheel-speed make-window (the band of RPS that still score).
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
    double window = speedWindowRps(hoodDeg, distanceM, radialVelMps);
    return new Shot(true, hoodDeg, fw, r.timeOfFlight(), r.entryAngleDeg(), window, window);
  }

  /**
   * Auto-select the most robust scoring shot, choosing the hood angle as well.
   *
   * <p>PROVISIONAL metric: maximizes the flywheel-speed make-window (the same quantity you measure
   * on the field by sweeping RPS until it stops scoring). It is the most defensible single scalar of
   * robustness, but under the current top-opening hub assumption it trends toward steep lobs. Once we
   * have the real hub opening geometry and your make-window data — which also capture hood/aim
   * tolerance and reward shorter time-of-flight — this metric will be extended and the optimum will
   * move flatter, toward your existing hand-tuned table. Until then, prefer {@link #solveAtHood} with
   * your known-good hood schedule for table generation.
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
  private static double speedWindowRps(double hoodDeg, double distanceM, double radialVelMps) {
    double r = ShotPhysics.GOAL_OPENING_RADIUS_M;
    double fwNear = onTargetFlywheel(hoodDeg, distanceM - r, radialVelMps); // near edge: less speed
    double fwFar = onTargetFlywheel(hoodDeg, distanceM + r, radialVelMps); // far edge: more speed
    if (Double.isNaN(fwNear) || Double.isNaN(fwFar)) {
      return 0.0;
    }
    return Math.abs(fwFar - fwNear);
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
    return 0.5 * (lo + hi);
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
