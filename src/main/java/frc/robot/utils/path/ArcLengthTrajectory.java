package frc.robot.utils.path;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;

/**
 * Re-parameterizes a Choreo {@link Trajectory} from time to arc-length for distance-based path
 * following.
 *
 * <p>Choreo outputs time-optimal trajectories indexed by timestamp. This adapter integrates speed
 * over time to build a cumulative arc-length table, then provides all queries by arc-length {@code
 * s} (meters). The path follower tracks the robot's actual position on the path, not a clock — if
 * the robot gets hit or stalls, the path "waits."
 *
 * <p>Implements {@link FollowablePath} so it can be consumed by both {@link
 * frc.robot.commands.FollowPath} and future MPC controllers.
 */
public final class ArcLengthTrajectory implements FollowablePath {

  /** Coarse search step size in meters for closest-point projection. */
  private static final double COARSE_SEARCH_STEP = 0.05;

  /** Maximum Newton-Raphson iterations for closest-point refinement. */
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

  /**
   * Creates an arc-length-parameterized trajectory from Choreo samples.
   *
   * @param trajectory The Choreo trajectory to re-parameterize
   * @return A new ArcLengthTrajectory
   */
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

    // Extract sample data and build arc-length table via trapezoidal integration
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

      // Trapezoidal integration: s[i] = s[i-1] + avg(speed) * dt
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
      // Fallback: finite difference of position
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

    // Compute curvature from interpolated velocity and finite-difference acceleration
    double vx = lerp(vxSamples[idx], vxSamples[idx + 1], frac);
    double vy = lerp(vySamples[idx], vySamples[idx + 1], frac);

    // Finite difference for acceleration (use adjacent samples)
    double ds = sTable[idx + 1] - sTable[idx];
    if (ds < 1e-12) return 0.0;
    double dvx = vxSamples[idx + 1] - vxSamples[idx];
    double dvy = vySamples[idx + 1] - vySamples[idx];
    // Convert dt-based acceleration to ds-based
    double speed = Math.hypot(vx, vy);
    if (speed < 1e-6) return 0.0;
    double ax = dvx / ds * speed;
    double ay = dvy / ds * speed;

    // Signed curvature: κ = (vx * ay - vy * ax) / |v|³
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

    // Shortest-angle interpolation for heading
    double h0 = headingSamples[idx];
    double h1 = headingSamples[idx + 1];
    double diff = h1 - h0;
    // Wrap to [-pi, pi]
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

    // Coarse grid search
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

    // Newton-Raphson refinement
    bestS = refineProjection(bestS, point, sMin, sMax);
    return buildProjectionResult(bestS, point);
  }

  // ---- Choreo-specific utilities ----

  /**
   * Converts a Choreo timestamp to the corresponding arc-length position.
   *
   * <p>Used to convert Choreo event markers (which are timestamp-based) to arc-length triggers for
   * distance-based action scheduling.
   *
   * @param timestamp Timestamp in seconds from the Choreo trajectory
   * @return Arc-length in meters at that timestamp
   */
  public double getArcLengthAtTimestamp(double timestamp) {
    if (timestamp <= tSamples[0]) return 0.0;
    if (timestamp >= tSamples[n - 1]) return totalLength;

    // Binary search for the bracket
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

  /**
   * Returns the starting pose (position + heading) of this trajectory.
   *
   * @return Position at s=0
   */
  public Translation2d getStartingPosition() {
    return new Translation2d(xSamples[0], ySamples[0]);
  }

  /**
   * Returns the ending velocity of this trajectory (m/s).
   *
   * @return Speed at the last sample
   */
  public double getEndVelocity() {
    return speedSamples[n - 1];
  }

  // ---- Internal helpers ----

  /**
   * Binary search for the bracket index: sTable[idx] <= s < sTable[idx+1].
   *
   * @return Index of the lower bracket bound
   */
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

  /** Returns the interpolation fraction within the bracket. */
  private double bracketFraction(double s, int idx) {
    double range = sTable[idx + 1] - sTable[idx];
    if (range < 1e-12) return 0.0;
    return (s - sTable[idx]) / range;
  }

  /** Interpolates position at arc-length s. Returns [x, y]. */
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

    // Cross-track error: signed distance, positive = left of path direction
    double crossTrack = tangent.getX() * toRobot.getY() - tangent.getY() * toRobot.getX();
    return new ProjectionResult(s, pathPoint, crossTrack, tangent);
  }

  private double refineProjection(double s, Translation2d robotPosition, double sMin, double sMax) {
    for (int iter = 0; iter < MAX_NEWTON_ITERATIONS; iter++) {
      Translation2d pathPoint = getPoint(s);
      Translation2d tangent = getTangent(s);
      Translation2d diff = pathPoint.minus(robotPosition);

      // f(s) = dot(diff, tangent) — zero when diff is perpendicular to tangent
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
