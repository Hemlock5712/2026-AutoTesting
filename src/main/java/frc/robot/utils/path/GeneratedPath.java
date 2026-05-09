package frc.robot.utils.path;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * A path created at runtime, with x/y/heading/curvature/velocity stored at each sample.
 *
 * <p>This is what {@link PathGenerator} produces. The follower can use it just like a Choreo
 * trajectory.
 */
public final class GeneratedPath implements FollowablePath {

  /** Step size when searching for the closest path point to the robot (m). */
  private static final double COARSE_SEARCH_STEP = 0.05;

  /** Max refinement steps to find the exact closest point. */
  private static final int MAX_NEWTON_ITERATIONS = 8;

  private final double[] sTable;
  private final double[] xSamples;
  private final double[] ySamples;
  private final double[] headingSamples;
  private final double[] curvatureSamples;
  private final double[] velocitySamples;
  private final double totalLength;
  private final int n;

  /**
   * @param x x positions (m)
   * @param y y positions (m)
   * @param headingRadians Robot heading at each sample (rad)
   * @param curvature How tight the curve is at each sample (1/m, positive = left turn)
   * @param velocity Target speed at each sample (m/s)
   */
  public GeneratedPath(
      double[] x, double[] y, double[] headingRadians, double[] curvature, double[] velocity) {
    n = x.length;
    if (n < 2) {
      throw new IllegalArgumentException("Need at least 2 samples, got " + n);
    }
    if (y.length != n
        || headingRadians.length != n
        || curvature.length != n
        || velocity.length != n) {
      throw new IllegalArgumentException("All sample arrays must have the same length");
    }

    xSamples = x.clone();
    ySamples = y.clone();
    headingSamples = headingRadians.clone();
    curvatureSamples = curvature.clone();
    velocitySamples = velocity.clone();
    sTable = new double[n];

    sTable[0] = 0.0;
    for (int i = 1; i < n; i++) {
      double dx = xSamples[i] - xSamples[i - 1];
      double dy = ySamples[i] - ySamples[i - 1];
      sTable[i] = sTable[i - 1] + Math.hypot(dx, dy);
    }
    totalLength = sTable[n - 1];
    if (totalLength < 1e-9) {
      throw new IllegalArgumentException("Path has zero length");
    }
  }

  @Override
  public Translation2d getPoint(double s) {
    s = clamp(s);
    int idx = bracketIndex(s);
    double frac = bracketFraction(s, idx);
    return new Translation2d(
        lerp(xSamples[idx], xSamples[idx + 1], frac), lerp(ySamples[idx], ySamples[idx + 1], frac));
  }

  @Override
  public Translation2d getTangent(double s) {
    s = clamp(s);
    int idx = bracketIndex(s);
    double frac = bracketFraction(s, idx);
    // Smooth the direction across sample boundaries by blending with the neighboring segment.
    double leftDx = xSamples[idx + 1] - xSamples[idx];
    double leftDy = ySamples[idx + 1] - ySamples[idx];
    double dx = leftDx;
    double dy = leftDy;
    if (frac < 0.5 && idx > 0) {
      double prevDx = xSamples[idx] - xSamples[idx - 1];
      double prevDy = ySamples[idx] - ySamples[idx - 1];
      double w = 0.5 + frac;
      dx = (1 - w) * prevDx + w * leftDx;
      dy = (1 - w) * prevDy + w * leftDy;
    } else if (frac >= 0.5 && idx + 2 < n) {
      double nextDx = xSamples[idx + 2] - xSamples[idx + 1];
      double nextDy = ySamples[idx + 2] - ySamples[idx + 1];
      double w = frac - 0.5;
      dx = (1 - w) * leftDx + w * nextDx;
      dy = (1 - w) * leftDy + w * nextDy;
    }
    double mag = Math.hypot(dx, dy);
    if (mag < 1e-12) return new Translation2d(1, 0);
    return new Translation2d(dx / mag, dy / mag);
  }

  @Override
  public double getCurvature(double s) {
    s = clamp(s);
    int idx = bracketIndex(s);
    double frac = bracketFraction(s, idx);
    return lerp(curvatureSamples[idx], curvatureSamples[idx + 1], frac);
  }

  @Override
  public double getVelocity(double s) {
    s = clamp(s);
    int idx = bracketIndex(s);
    double frac = bracketFraction(s, idx);
    return lerp(velocitySamples[idx], velocitySamples[idx + 1], frac);
  }

  @Override
  public Rotation2d getHeading(double s) {
    s = clamp(s);
    int idx = bracketIndex(s);
    double frac = bracketFraction(s, idx);
    double h0 = headingSamples[idx];
    double h1 = headingSamples[idx + 1];
    double diff = MathUtil.angleModulus(h1 - h0);
    return Rotation2d.fromRadians(h0 + frac * diff);
  }

  @Override
  public double getTotalLength() {
    return totalLength;
  }

  @Override
  public ProjectionResult getClosestPointInRange(Translation2d point, double sMin, double sMax) {
    sMin = Math.max(0, sMin);
    sMax = Math.min(totalLength, sMax);
    if (sMin >= sMax) {
      return buildProjectionResult(sMin, point);
    }

    double bestS = sMin;
    double bestDistSq = Double.MAX_VALUE;
    int numSteps = Math.max(1, (int) ((sMax - sMin) / COARSE_SEARCH_STEP));
    double step = (sMax - sMin) / numSteps;
    for (int i = 0; i <= numSteps; i++) {
      double s = sMin + i * step;
      Translation2d p = getPoint(s);
      double dx = point.getX() - p.getX();
      double dy = point.getY() - p.getY();
      double distSq = dx * dx + dy * dy;
      if (distSq < bestDistSq) {
        bestDistSq = distSq;
        bestS = s;
      }
    }

    bestS = refineProjection(bestS, point, sMin, sMax);
    return buildProjectionResult(bestS, point);
  }

  private int bracketIndex(double s) {
    int lo = 0;
    int hi = n - 1;
    while (lo + 1 < hi) {
      int mid = (lo + hi) >>> 1;
      if (sTable[mid] <= s) lo = mid;
      else hi = mid;
    }
    return lo;
  }

  private double bracketFraction(double s, int idx) {
    double range = sTable[idx + 1] - sTable[idx];
    if (range < 1e-12) return 0.0;
    return (s - sTable[idx]) / range;
  }

  private ProjectionResult buildProjectionResult(double s, Translation2d robotPosition) {
    Translation2d pathPoint = getPoint(s);
    Translation2d tangent = getTangent(s);
    Translation2d toRobot = robotPosition.minus(pathPoint);
    double crossTrack = tangent.getX() * toRobot.getY() - tangent.getY() * toRobot.getX();
    return new ProjectionResult(s, pathPoint, crossTrack, tangent);
  }

  private double refineProjection(double s, Translation2d robotPosition, double sMin, double sMax) {
    for (int iter = 0; iter < MAX_NEWTON_ITERATIONS; iter++) {
      Translation2d pathPoint = getPoint(s);
      Translation2d tangent = getTangent(s);
      Translation2d diff = pathPoint.minus(robotPosition);
      double f = diff.getX() * tangent.getX() + diff.getY() * tangent.getY();
      double curvature = getCurvature(s);
      Translation2d normal = new Translation2d(-tangent.getY(), tangent.getX());
      double fPrime = 1.0 + curvature * (diff.getX() * normal.getX() + diff.getY() * normal.getY());
      if (Math.abs(fPrime) < 1e-12) break;
      double ds = -f / fPrime;
      s = Math.max(sMin, Math.min(sMax, s + ds));
      if (Math.abs(ds) < 1e-6) break;
    }
    return s;
  }

  private double clamp(double s) {
    return Math.max(0, Math.min(s, totalLength));
  }

  private static double lerp(double a, double b, double t) {
    return a + t * (b - a);
  }
}
