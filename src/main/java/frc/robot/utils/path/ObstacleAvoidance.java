package frc.robot.utils.path;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Pose-based velocity clamp for teleop. For each axis-aligned obstacle, projects the commanded
 * field-frame velocity onto the robot→obstacle direction and limits that approach component to
 * {@code sqrt(2·a·d_free)}. Tangential motion is preserved so the robot slides along walls instead
 * of pinning. Reads the estimator pose — same belief the controller acts on — so avoiding contact
 * keeps odometry accurate.
 *
 * <p>Obstacles are the same AABB pairs PathPlanner's pathfinder consumes (see {@code
 * PathPlannerAutos.pushObstaclesToPathfinder}), so the teleop clamp and the planner can't disagree
 * on the field layout.
 *
 * <p>Sequential per-obstacle clamping is conservative-but-not-optimal when obstacle normals are
 * non-orthogonal: the order of obstacles affects how aggressively the velocity is reduced. The
 * result is always safe (never lets through more motion than any single obstacle allows), but in
 * cluttered areas may pin harder than a joint QP solve would. Acceptable for the current set of
 * mostly axis-aligned field structures.
 */
public final class ObstacleAvoidance {

  private final List<Pair<Translation2d, Translation2d>> obstacles;
  private final double halfX;
  private final double halfY;
  private final double brakeAccel;
  private final double safetyMargin;
  private final String logKey;

  /**
   * @param obstacles AABB obstacle list (min/max Translation2d pairs)
   * @param halfX robot bounding-box half-extent along robot X (m, including bumpers)
   * @param halfY robot bounding-box half-extent along robot Y (m, including bumpers)
   * @param brakeAccel braking authority (m/s²). Set below the friction limit so the chassis can
   *     actually decelerate in time.
   * @param safetyMargin buffer kept between robot edge and obstacle edge (m).
   * @param logKey AdvantageKit log prefix, or {@code null} to disable logging.
   */
  public ObstacleAvoidance(
      List<Pair<Translation2d, Translation2d>> obstacles,
      double halfX,
      double halfY,
      double brakeAccel,
      double safetyMargin,
      String logKey) {
    this.obstacles = obstacles;
    this.halfX = halfX;
    this.halfY = halfY;
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

    double minFree = Double.POSITIVE_INFINITY;
    for (int i = 0, n = obstacles.size(); i < n; i++) {
      double[] r = applyOne(obstacles.get(i), rx, ry, cosT, sinT, vx, vy);
      vx = r[0];
      vy = r[1];
      if (r[2] < minFree) minFree = r[2];
    }

    if (logKey != null) {
      Logger.recordOutput(logKey + "/MinFreeDistance", minFree);
    }
    return new Translation2d(vx, vy);
  }

  /** Returns {@code [vx, vy, freeDistance]} after clamping against one AABB. */
  private double[] applyOne(
      Pair<Translation2d, Translation2d> box,
      double rx,
      double ry,
      double cosT,
      double sinT,
      double vx,
      double vy) {
    Translation2d min = box.getFirst();
    Translation2d max = box.getSecond();

    // AABB signed distance from the robot center (negative inside, positive outside).
    double dxMin = min.getX() - rx;
    double dxMax = rx - max.getX();
    double dyMin = min.getY() - ry;
    double dyMax = ry - max.getY();
    double outX = Math.max(dxMin, dxMax);
    double outY = Math.max(dyMin, dyMax);
    double sd;
    double nearX;
    double nearY;
    if (outX > 0.0 || outY > 0.0) {
      // Outside (or on a face). Nearest point is the clamp of the robot center into the AABB.
      double clampedX = Math.max(min.getX(), Math.min(max.getX(), rx));
      double clampedY = Math.max(min.getY(), Math.min(max.getY(), ry));
      double dx = clampedX - rx;
      double dy = clampedY - ry;
      sd = Math.hypot(Math.max(outX, 0.0), Math.max(outY, 0.0));
      nearX = clampedX;
      nearY = clampedY;
      // Keep the sign convention consistent.
      if (sd < 1.0e-9) {
        // Touching the boundary; nudge perpendicular to the closer face so direction is defined.
        nearX = rx + dx;
        nearY = ry + dy;
      }
    } else {
      // Inside the AABB. Nearest edge is the one with the smallest perpendicular distance.
      double dRight = max.getX() - rx;
      double dLeft = rx - min.getX();
      double dTop = max.getY() - ry;
      double dBottom = ry - min.getY();
      double m = Math.min(Math.min(dRight, dLeft), Math.min(dTop, dBottom));
      sd = -m;
      if (m == dRight) {
        nearX = max.getX();
        nearY = ry;
      } else if (m == dLeft) {
        nearX = min.getX();
        nearY = ry;
      } else if (m == dTop) {
        nearX = rx;
        nearY = max.getY();
      } else {
        nearX = rx;
        nearY = min.getY();
      }
    }

    double dx = nearX - rx;
    double dy = nearY - ry;
    double dist = Math.hypot(dx, dy);
    if (dist < 1.0e-9) {
      // Numerically on the boundary. If inside, force a stop; if exactly on a face from outside,
      // one tick of pass-through is bounded.
      if (sd < 0.0) {
        return new double[] {0.0, 0.0, sd};
      }
      return new double[] {vx, vy, sd};
    }

    // (nx, ny) points from the robot toward "more obstacle". Flip when inside so the brake-curve
    // logic kills motion that would drive deeper.
    double nx = dx / dist;
    double ny = dy / dist;
    if (sd < 0.0) {
      nx = -nx;
      ny = -ny;
    }

    // Bounding-box extent along the approach direction, computed in robot frame. For axis-aligned
    // obstacles at modest robot rotation, the projection is exact.
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
