package frc.robot.utils.path;

import frc.robot.commands.AccelerationLimiter;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.Motor;
import java.util.function.DoubleUnaryOperator;

/**
 * Computes the fastest safe speed at each point on a path.
 *
 * <p>Respects:
 *
 * <ul>
 *   <li>The robot's max speed
 *   <li>How hard the wheels can grip (friction circle)
 *   <li>How fast the motors can push at the current speed
 *   <li>Required start and end speeds
 * </ul>
 *
 * <p>Works in three passes: cap speed in turns, then limit how fast we can speed up, then how fast
 * we can slow down. The smallest speed at each point wins.
 */
public final class VelocityProfiler {

  /** Below this curvature, we consider the path straight (no turning slowdown). */
  private static final double STRAIGHT_CURVATURE_EPS = 1e-9;

  /**
   * Constraints for profile generation.
   *
   * @param maxVelocity Top speed of the robot (m/s)
   * @param maxFrictionAccel How hard the wheels can grip (m/s^2)
   * @param longitudinalAccelLimit Optional: how fast the motors can push at a given speed. Use
   *     {@code null} to ignore motor limits. Only matters when speeding up - braking is
   *     friction-limited.
   */
  public record Constraints(
      double maxVelocity, double maxFrictionAccel, DoubleUnaryOperator longitudinalAccelLimit) {

    /** Default settings using the Kraken X60 motor curves and AccelerationLimiter values. */
    public static Constraints defaults() {
      return new Constraints(
          AccelerationLimiter.MAX_VELOCITY,
          AccelerationLimiter.MAX_FRICTION_ACCEL,
          v ->
              Motor.KRAKEN_X60_FOC.getMaxAcceleration(
                  v,
                  TunerConstants.FrontLeft.DriveMotorGearRatio,
                  TunerConstants.FrontLeft.WheelRadius,
                  60.0,
                  4,
                  150.0));
    }

    /** Like defaults() but ignores motor limits (only friction limits speed). */
    public Constraints withoutMotorLimit() {
      return new Constraints(maxVelocity, maxFrictionAccel, null);
    }
  }

  private VelocityProfiler() {}

  /**
   * Computes the safe speed at every point along a path.
   *
   * @param curvature How tight the path is at each sample (1/m). Length n.
   * @param ds Distance between adjacent samples (m). Length n-1.
   * @param startVelocity Speed at the first sample (m/s).
   * @param endVelocity Speed at the last sample (m/s).
   * @param c Constraints.
   * @return One speed per sample.
   */
  public static double[] profile(
      double[] curvature, double[] ds, double startVelocity, double endVelocity, Constraints c) {
    int n = curvature.length;
    if (n < 2) {
      throw new IllegalArgumentException("Need at least 2 samples, got " + n);
    }
    if (ds.length != n - 1) {
      throw new IllegalArgumentException(
          "ds length " + ds.length + " must equal curvature.length-1 = " + (n - 1));
    }

    double[] v = new double[n];
    double aTotalSq = c.maxFrictionAccel * c.maxFrictionAccel;

    // Pass 1: cap speed in turns so the robot doesn't slide.
    for (int i = 0; i < n; i++) {
      double absK = Math.abs(curvature[i]);
      double cap = c.maxVelocity;
      if (absK > STRAIGHT_CURVATURE_EPS) {
        cap = Math.min(cap, Math.sqrt(c.maxFrictionAccel / absK));
      }
      v[i] = cap;
    }

    // Pass 2: limit how fast we can speed up. The harder we're already turning, the less grip
    // is left for accelerating forward.
    v[0] = Math.min(v[0], startVelocity);
    for (int i = 0; i < n - 1; i++) {
      double aLat = v[i] * v[i] * Math.abs(curvature[i]);
      double aLong = Math.sqrt(Math.max(0.0, aTotalSq - aLat * aLat));
      if (c.longitudinalAccelLimit != null) {
        aLong = Math.min(aLong, c.longitudinalAccelLimit.applyAsDouble(v[i]));
      }
      double vNextSq = v[i] * v[i] + 2.0 * aLong * ds[i];
      double vNext = vNextSq > 0.0 ? Math.sqrt(vNextSq) : 0.0;
      v[i + 1] = Math.min(v[i + 1], vNext);
    }

    // Pass 3: walk backwards to limit how fast we can brake (friction only - motors can't help).
    v[n - 1] = Math.min(v[n - 1], endVelocity);
    for (int i = n - 2; i >= 0; i--) {
      double aLat = v[i + 1] * v[i + 1] * Math.abs(curvature[i + 1]);
      double aLong = Math.sqrt(Math.max(0.0, aTotalSq - aLat * aLat));
      double vPrevSq = v[i + 1] * v[i + 1] + 2.0 * aLong * ds[i];
      double vPrev = vPrevSq > 0.0 ? Math.sqrt(vPrevSq) : 0.0;
      v[i] = Math.min(v[i], vPrev);
    }

    return v;
  }
}
