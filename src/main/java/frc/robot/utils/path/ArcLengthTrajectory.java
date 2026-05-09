package frc.robot.utils.path;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;

/**
 * Converts a Choreo {@link Trajectory} from time-based to distance-based.
 *
 * <p>Choreo gives us a path indexed by time ("at 1.5 sec, be here"). This class converts it to be
 * indexed by distance along the path ("at 2.3 m, be here"). That way, if the robot gets bumped or
 * slows down, the path just waits for it to catch up - no fighting the clock.
 */
public final class ArcLengthTrajectory implements FollowablePath {

  /** Step size when searching for the closest path point to the robot (m). */
  private static final double COARSE_SEARCH_STEP = 0.05;

  /** Max number of refinement steps to find the exact closest point. */
  private static final int MAX_NEWTON_ITERATIONS = 8;

  private final double[] sTable;
  private final double[] xSamples;
  private final double[] ySamples;
  private final double[] headingSamples;
  private final double[] vxSamples;
  private final double[] vySamples;
  private final double[] speedSamples;
  private final double[] tSamples;
  private final double totalLength;
  private final int n; // number of samples

  /** Creates a distance-based path from a Choreo trajectory. */
  public static ArcLengthTrajectory fromChoreo(Trajectory<SwerveSample> trajectory) {
    return new ArcLengthTrajectory(trajectory.samples());
  }

  private ArcLengthTrajectory(List<SwerveSample> samples) {
    n = samples.size();
    if (n < 2) {
      throw new IllegalArgumentException("Trajectory must have at least 2 samples");
    }

    sTable = new double[n];
    xSamples = new double[n];
    ySamples = new double[n];
    headingSamples = new double[n];
    vxSamples = new double[n];
    vySamples = new double[n];
    speedSamples = new double[n];
    tSamples = new double[n];

    // Copy the data and compute the distance to each sample as we go.
    SwerveSample first = samples.get(0);
    sTable[0] = 0.0;
    xSamples[0] = first.x;
    ySamples[0] = first.y;
    headingSamples[0] = first.heading;
    vxSamples[0] = first.vx;
    vySamples[0] = first.vy;
    speedSamples[0] = Math.hypot(first.vx, first.vy);
    tSamples[0] = first.t;

    for (int i = 1; i < n; i++) {
      SwerveSample s = samples.get(i);
      xSamples[i] = s.x;
      ySamples[i] = s.y;
      headingSamples[i] = s.heading;
      vxSamples[i] = s.vx;
      vySamples[i] = s.vy;
      speedSamples[i] = Math.hypot(s.vx, s.vy);
      tSamples[i] = s.t;

      // Distance traveled in this step = average speed * time elapsed.
      double dt = s.t - tSamples[i - 1];
      double avgSpeed = (speedSamples[i] + speedSamples[i - 1]) / 2.0;
      sTable[i] = sTable[i - 1] + avgSpeed * dt;
    }

    totalLength = sTable[n - 1];
  }

  // ---- FollowablePath implementation ----

  @Override
  public Translation2d getPoint(double s) {
    s = clamp(s);
    double[] interp = interpolate(s);
    return new Translation2d(interp[0], interp[1]);
  }

  @Override
  public Translation2d getTangent(double s) {
    s = clamp(s);
    int idx = bracketIndex(s);
    double frac = bracketFraction(s, idx);

    double vx = lerp(vxSamples[idx], vxSamples[idx + 1], frac);
    double vy = lerp(vySamples[idx], vySamples[idx + 1], frac);
    double mag = Math.hypot(vx, vy);
    if (mag < 1e-12) {
      // Velocity is zero - fall back to position difference.
      double dx = xSamples[idx + 1] - xSamples[idx];
      double dy = ySamples[idx + 1] - ySamples[idx];
      mag = Math.hypot(dx, dy);
      if (mag < 1e-12) return new Translation2d(1, 0);
      return new Translation2d(dx / mag, dy / mag);
    }
    return new Translation2d(vx / mag, vy / mag);
  }

  @Override
  public double getCurvature(double s) {
    s = clamp(s);
    int idx = bracketIndex(s);
    double frac = bracketFraction(s, idx);

    // Curvature = how fast the path is turning. Computed from velocity and acceleration.
    double vx = lerp(vxSamples[idx], vxSamples[idx + 1], frac);
    double vy = lerp(vySamples[idx], vySamples[idx + 1], frac);

    double ds = sTable[idx + 1] - sTable[idx];
    if (ds < 1e-12) return 0.0;
    double dvx = vxSamples[idx + 1] - vxSamples[idx];
    double dvy = vySamples[idx + 1] - vySamples[idx];
    double speed = Math.hypot(vx, vy);
    if (speed < 1e-6) return 0.0;
    double ax = dvx / ds * speed;
    double ay = dvy / ds * speed;

    double v3 = speed * speed * speed;
    return (vx * ay - vy * ax) / v3;
  }

  @Override
  public double getVelocity(double s) {
    s = clamp(s);
    int idx = bracketIndex(s);
    double frac = bracketFraction(s, idx);
    return lerp(speedSamples[idx], speedSamples[idx + 1], frac);
  }

  @Override
  public Rotation2d getHeading(double s) {
    s = clamp(s);
    int idx = bracketIndex(s);
    double frac = bracketFraction(s, idx);

    // Always rotate the short way around (don't take the long way past 180°).
    double h0 = headingSamples[idx];
    double h1 = headingSamples[idx + 1];
    double diff = h1 - h0;
    diff = diff - 2 * Math.PI * Math.floor((diff + Math.PI) / (2 * Math.PI));
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

    // First, scan along the path to find the roughly closest point.
    double bestS = sMin;
    double bestDistSq = Double.MAX_VALUE;
    int numSteps = Math.max(1, (int) ((sMax - sMin) / COARSE_SEARCH_STEP));
    double step = (sMax - sMin) / numSteps;

    for (int i = 0; i <= numSteps; i++) {
      double s = sMin + i * step;
      double[] p = interpolate(clamp(s));
      double dx = point.getX() - p[0];
      double dy = point.getY() - p[1];
      double distSq = dx * dx + dy * dy;
      if (distSq < bestDistSq) {
        bestDistSq = distSq;
        bestS = s;
      }
    }

    // Then refine to find the exact closest point.
    bestS = refineProjection(bestS, point, sMin, sMax);
    return buildProjectionResult(bestS, point);
  }

  // ---- Choreo-specific utilities ----

  /**
   * Looks up which point on the path matches a Choreo timestamp. Used to convert Choreo event
   * markers (timestamp-based) into distance-based triggers.
   */
  public double getArcLengthAtTimestamp(double timestamp) {
    if (timestamp <= tSamples[0]) return 0.0;
    if (timestamp >= tSamples[n - 1]) return totalLength;

    // Binary search to find which two samples bracket this timestamp.
    int lo = 0;
    int hi = n - 1;
    while (lo + 1 < hi) {
      int mid = (lo + hi) >>> 1;
      if (tSamples[mid] <= timestamp) {
        lo = mid;
      } else {
        hi = mid;
      }
    }

    double frac =
        (tSamples[hi] - tSamples[lo]) > 1e-12
            ? (timestamp - tSamples[lo]) / (tSamples[hi] - tSamples[lo])
            : 0.0;
    return lerp(sTable[lo], sTable[hi], frac);
  }

  // ---- Internal helpers ----

  /** Finds which two samples surround the given distance s. */
  private int bracketIndex(double s) {
    int lo = 0;
    int hi = n - 1;
    while (lo + 1 < hi) {
      int mid = (lo + hi) >>> 1;
      if (sTable[mid] <= s) {
        lo = mid;
      } else {
        hi = mid;
      }
    }
    return lo;
  }

  /** How far between the two bracketing samples we are (0=at idx, 1=at idx+1). */
  private double bracketFraction(double s, int idx) {
    double range = sTable[idx + 1] - sTable[idx];
    if (range < 1e-12) return 0.0;
    return (s - sTable[idx]) / range;
  }

  /** Computes the (x, y) position at distance s along the path. */
  private double[] interpolate(double s) {
    int idx = bracketIndex(s);
    double frac = bracketFraction(s, idx);
    return new double[] {
      lerp(xSamples[idx], xSamples[idx + 1], frac), lerp(ySamples[idx], ySamples[idx + 1], frac)
    };
  }

  private ProjectionResult buildProjectionResult(double s, Translation2d robotPosition) {
    Translation2d pathPoint = getPoint(s);
    Translation2d tangent = getTangent(s);
    Translation2d toRobot = robotPosition.minus(pathPoint);

    // How far off the path the robot is. Positive = left of the path direction.
    double crossTrack = tangent.getX() * toRobot.getY() - tangent.getY() * toRobot.getX();
    return new ProjectionResult(s, pathPoint, crossTrack, tangent);
  }

  private double refineProjection(double s, Translation2d robotPosition, double sMin, double sMax) {
    for (int iter = 0; iter < MAX_NEWTON_ITERATIONS; iter++) {
      Translation2d pathPoint = getPoint(s);
      Translation2d tangent = getTangent(s);
      Translation2d diff = pathPoint.minus(robotPosition);

      // We're closest when the robot is directly off to the side of the path direction.
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
