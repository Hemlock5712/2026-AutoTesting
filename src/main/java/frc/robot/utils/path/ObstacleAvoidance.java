package frc.robot.utils.path;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Pose-based velocity clamp. For each obstacle, projects the commanded field-frame velocity onto
 * the robot→obstacle direction and limits that approach component to {@code sqrt(2·a·d_free)}.
 * Tangential motion is preserved so the robot slides along walls instead of pinning. Reads the
 * estimator pose — same belief the controller acts on — so avoiding contact keeps odometry
 * accurate.
 *
 * <p>Sequential per-obstacle clamping is conservative-but-not-optimal when obstacle normals are
 * non-orthogonal: the order of obstacles affects how aggressively the velocity is reduced. The
 * result is always safe (never lets through more motion than any single obstacle allows), but in
 * cluttered areas may pin harder than a joint QP solve would. Acceptable for the current set of
 * mostly axis-aligned field structures; revisit if dynamic-obstacle density grows.
 */
public final class ObstacleAvoidance {

  private final ObstacleField field;
  private final Footprint footprint;
  private final double brakeAccel;
  private final double safetyMargin;
  private final String logKey;

  /**
   * @param brakeAccel braking authority (m/s^2). Set below the friction limit so the chassis can
   *     actually decelerate in time.
   * @param safetyMargin buffer kept between robot edge and obstacle edge (m).
   * @param logKey AdvantageKit log prefix, or {@code null} to disable logging.
   */
  public ObstacleAvoidance(
      ObstacleField field,
      Footprint footprint,
      double brakeAccel,
      double safetyMargin,
      String logKey) {
    this.field = field;
    this.footprint = footprint;
    this.brakeAccel = brakeAccel;
    this.safetyMargin = safetyMargin;
    this.logKey = logKey;
  }

  /** Clamps the velocity so the robot at {@code pose} won't drive into any obstacle. */
  public Translation2d clamp(Pose2d pose, double vxField, double vyField) {
    double vx = vxField;
    double vy = vyField;
    double rx = pose.getX();
    double ry = pose.getY();
    double cosT = pose.getRotation().getCos();
    double sinT = pose.getRotation().getSin();

    double halfX = footprint.halfX();
    double halfY = footprint.halfY();
    double offX = footprint.offsetX();
    double offY = footprint.offsetY();

    double minFree = Double.POSITIVE_INFINITY;

    List<Obstacle> stat = field.staticObstacles();
    for (int i = 0, n = stat.size(); i < n; i++) {
      double[] r = applyOne(stat.get(i), rx, ry, cosT, sinT, halfX, halfY, offX, offY, vx, vy);
      vx = r[0];
      vy = r[1];
      if (r[2] < minFree) minFree = r[2];
    }
    List<Obstacle> dyn = field.dynamicObstacles();
    for (int i = 0, n = dyn.size(); i < n; i++) {
      double[] r = applyOne(dyn.get(i), rx, ry, cosT, sinT, halfX, halfY, offX, offY, vx, vy);
      vx = r[0];
      vy = r[1];
      if (r[2] < minFree) minFree = r[2];
    }

    if (logKey != null) {
      Logger.recordOutput(logKey + "/MinFreeDistance", minFree);
    }
    return new Translation2d(vx, vy);
  }

  /** Returns {@code [vx, vy, freeDistance]} after clamping against one obstacle. */
  private double[] applyOne(
      Obstacle obstacle,
      double rx,
      double ry,
      double cosT,
      double sinT,
      double halfX,
      double halfY,
      double offX,
      double offY,
      double vx,
      double vy) {
    // Bounding-box center in field frame (offset center for asymmetric extensions like intake).
    double bcx = rx + (offX * cosT - offY * sinT);
    double bcy = ry + (offX * sinT + offY * cosT);

    double sd = obstacle.signedDistance(bcx, bcy);
    Translation2d np = obstacle.nearestPoint(bcx, bcy);
    double dx = np.getX() - bcx;
    double dy = np.getY() - bcy;
    double dist = Math.hypot(dx, dy);
    if (dist < 1.0e-9) {
      // Numerically on the boundary. If the center is outside, one tick of pass-through is bounded
      // — next tick will be clearly outside and handled normally. If the center is inside (sd<0,
      // typically from a misplaced SIM_SPAWN_POSE), pass-through would let the robot drift further
      // in and never recover, so force a stop instead.
      if (sd < 0.0) {
        return new double[] {0.0, 0.0, sd};
      }
      return new double[] {vx, vy, sd};
    }

    // (nx, ny) points from the robot toward "more obstacle". When the center is outside, the
    // nearestPoint vector already points toward the boundary (= into the obstacle); when inside,
    // it points back out, so flip so the same brake-curve logic kills motion that would drive
    // deeper.
    double nx = dx / dist;
    double ny = dy / dist;
    if (sd < 0.0) {
      nx = -nx;
      ny = -ny;
    }

    // Bounding-box extent along the approach direction, computed in robot frame. This is an
    // approximation: at oblique heading near a rectangle corner the projection underestimates
    // the true OBB-rect distance (a rotated OBB corner can poke past where this formula
    // expects). For the 2026 field — axis-aligned obstacles, modest rotation while translating —
    // the error is small. Replace with full SAT distance if a denser obstacle set or persistent
    // diagonal traversal exposes the gap.
    double nxR = nx * cosT + ny * sinT;
    double nyR = -nx * sinT + ny * cosT;
    double extent = halfX * Math.abs(nxR) + halfY * Math.abs(nyR);
    double free = sd - extent;

    double approach = vx * nx + vy * ny;
    if (approach <= 0.0) {
      return new double[] {vx, vy, free};
    }

    double safe = Math.max(0.0, free - safetyMargin);
    double vMax = Math.sqrt(2.0 * brakeAccel * safe);
    if (approach <= vMax) {
      return new double[] {vx, vy, free};
    }
    double excess = approach - vMax;
    return new double[] {vx - excess * nx, vy - excess * ny, free};
  }
}
