package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Decomposes measured module velocities into a per-module robot-translation estimate.
 *
 * <p>Each module's velocity in the robot frame is {@code v_module = v_chassis + omega x r_module},
 * so the chassis-translation component is recoverable per module by subtracting the rotational
 * lever-arm term {@code omega x r_module}. Without skid these four estimates are identical (they're
 * four observations of the same chassis translation). Skid pulls them apart, and we surface that
 * spread as a scalar metric.
 *
 * <p>{@code maxOverMinRatio} climbs above 1 as one wheel skids relative to the others; {@code
 * magnitudeStdDev} catches more symmetric cases (e.g. all wheels slipping together). When the
 * chassis is barely translating ({@code mean.norm() < NOISE_FLOOR_MPS}) the ratio is forced to 1.0
 * — pure rotations leave each per-module translation near zero, where small/small ratios are
 * dominated by noise.
 */
public final class SkidDetection {

  /** Mean translation magnitudes below this gate the {@code maxOverMinRatio} to 1.0. */
  public static final double NOISE_FLOOR_MPS = 0.05;

  public record Result(
      Translation2d[] perModuleTranslation,
      Translation2d meanTranslation,
      double maxMagnitude,
      double minMagnitude,
      double maxOverMinRatio,
      double magnitudeStdDev) {}

  private SkidDetection() {}

  /**
   * @param states measured module states (speed + angle) in robot frame
   * @param moduleLocations (x, y) of each module relative to robot center, same order as {@code
   *     states}
   * @param omegaRadPerSec robot angular velocity; gyro-derived is preferred since
   *     kinematics-derived omega is itself corrupted by skid
   */
  public static Result compute(
      SwerveModuleState[] states, Translation2d[] moduleLocations, double omegaRadPerSec) {
    if (states.length != moduleLocations.length) {
      throw new IllegalArgumentException(
          "states and moduleLocations length mismatch: "
              + states.length
              + " vs "
              + moduleLocations.length);
    }

    Translation2d[] perModule = new Translation2d[states.length];
    double sumX = 0;
    double sumY = 0;
    double maxMag = 0.0;
    double minMag = Double.POSITIVE_INFINITY;
    double[] mags = new double[states.length];

    for (int i = 0; i < states.length; i++) {
      double s = states[i].speedMetersPerSecond;
      double a = states[i].angle.getRadians();
      double measuredX = s * Math.cos(a);
      double measuredY = s * Math.sin(a);
      // omega x r in 2D = (-omega*y, omega*x). Subtracting that from the measured module
      // velocity yields the chassis-translation contribution.
      double tx = measuredX + omegaRadPerSec * moduleLocations[i].getY();
      double ty = measuredY - omegaRadPerSec * moduleLocations[i].getX();
      perModule[i] = new Translation2d(tx, ty);
      sumX += tx;
      sumY += ty;
      double m = Math.hypot(tx, ty);
      mags[i] = m;
      if (m > maxMag) maxMag = m;
      if (m < minMag) minMag = m;
    }

    Translation2d mean = new Translation2d(sumX / states.length, sumY / states.length);

    double meanMag = 0.0;
    for (double m : mags) meanMag += m;
    meanMag /= mags.length;
    double sq = 0.0;
    for (double m : mags) {
      double d = m - meanMag;
      sq += d * d;
    }
    double stdDev = Math.sqrt(sq / mags.length);

    // Gate on the mean translation magnitude, not min magnitude: pure rotations leave each
    // per-module translation near zero where small/small ratios are noise-dominated.
    double ratio = (mean.getNorm() < NOISE_FLOOR_MPS) ? 1.0 : (maxMag / Math.max(minMag, 1.0e-6));

    return new Result(perModule, mean, maxMag, minMag, ratio, stdDev);
  }
}
