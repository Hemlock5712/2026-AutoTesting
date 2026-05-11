package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Stick deadband and translation-magnitude scaling. Single-axis inputs go through {@link
 * #deadband(double)}; two-axis inputs go through {@link #rescaleTranslation(double, double)} which
 * applies a radial deadband and a squared-magnitude curve so small stick moves stay precise and
 * full deflection still reaches the rails.
 */
public final class DriverInput {

  public static final double DEADBAND = 0.05;

  private DriverInput() {}

  /** Single-axis deadband (e.g. for rotational rate). */
  public static double deadband(double input) {
    return MathUtil.applyDeadband(input, DEADBAND);
  }

  /**
   * Radial deadband + squared-magnitude curve on a translation stick. Returns a fresh {@link
   * Translation2d} so callers can safely store / pass it around.
   */
  public static Translation2d rescaleTranslation(double x, double y) {
    double mag = Math.hypot(x, y);
    if (mag < DEADBAND) return Translation2d.kZero;
    double normalized = Math.min(1.0, (mag - DEADBAND) / (1.0 - DEADBAND));
    double factor = normalized * normalized / mag;
    return new Translation2d(x * factor, y * factor);
  }
}
