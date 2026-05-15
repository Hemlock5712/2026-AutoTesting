package frc.robot.utils.path;

import edu.wpi.first.math.geometry.Ellipse2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * A field obstacle. Backed by WPILib's {@link Rectangle2d} and {@link Ellipse2d} so the same shape
 * is shared between the planner, the avoidance clamp, the dyn4j sim, and AdvantageScope's native
 * 2D-field rendering. Rectangles are full OBBs (the {@link Pose2d} center carries rotation), so the
 * planner can reason about rotated walls — not just axis-aligned bumps.
 */
public sealed interface Obstacle permits Obstacle.Circle, Obstacle.Rectangle {

  /** Negative if (x, y) is inside the obstacle, positive outside. */
  double signedDistance(double x, double y);

  /**
   * Closest point on this obstacle's boundary to (x, y). For inside queries this is the nearest
   * edge point — the vector from (x, y) to the result is the "into-obstacle escape" direction.
   */
  Translation2d nearestPoint(double x, double y);

  /**
   * True if the robot's oriented bounding box (OBB — a rotated rectangle) at the given pose
   * overlaps this obstacle.
   */
  boolean intersectsObb(double cx, double cy, double cosT, double sinT, double halfX, double halfY);

  record Circle(Ellipse2d shape) implements Obstacle {

    /** Convenience constructor for circles defined by center + radius. */
    public Circle(double cx, double cy, double radius) {
      this(new Ellipse2d(new Translation2d(cx, cy), radius));
    }

    private double cx() {
      return shape.getCenter().getX();
    }

    private double cy() {
      return shape.getCenter().getY();
    }

    private double radius() {
      return shape.getXSemiAxis();
    }

    @Override
    public double signedDistance(double x, double y) {
      return Math.hypot(x - cx(), y - cy()) - radius();
    }

    @Override
    public Translation2d nearestPoint(double x, double y) {
      double dx = x - cx();
      double dy = y - cy();
      double dist = Math.hypot(dx, dy);
      if (dist < 1.0e-9) {
        return new Translation2d(cx() + radius(), cy());
      }
      double scale = radius() / dist;
      return new Translation2d(cx() + dx * scale, cy() + dy * scale);
    }

    @Override
    public boolean intersectsObb(
        double bcx, double bcy, double cosT, double sinT, double halfX, double halfY) {
      // Rotate the circle into the rectangle's local frame, then circle-vs-AABB.
      double dx = cx() - bcx;
      double dy = cy() - bcy;
      double localX = cosT * dx + sinT * dy;
      double localY = -sinT * dx + cosT * dy;
      double clampedX = Math.max(-halfX, Math.min(halfX, localX));
      double clampedY = Math.max(-halfY, Math.min(halfY, localY));
      double ddx = localX - clampedX;
      double ddy = localY - clampedY;
      double r = radius();
      return ddx * ddx + ddy * ddy < r * r;
    }
  }

  record Rectangle(Rectangle2d shape) implements Obstacle {

    /** Convenience constructor for axis-aligned rectangles defined by opposite corners. */
    public Rectangle(double minX, double minY, double maxX, double maxY) {
      this(
          new Rectangle2d(
              new Pose2d((minX + maxX) / 2.0, (minY + maxY) / 2.0, Rotation2d.kZero),
              maxX - minX,
              maxY - minY));
      if (minX >= maxX || minY >= maxY) {
        throw new IllegalArgumentException("Rectangle bounds must be ordered (min < max)");
      }
    }

    @Override
    public double signedDistance(double x, double y) {
      // Inlined local-frame transform (no allocation; signedDistance is hot — called per
      // costmap cell × obstacles during planning, and per obstacle on the 250 Hz avoidance tick).
      Pose2d center = shape.getCenter();
      double dx = x - center.getX();
      double dy = y - center.getY();
      double cos = center.getRotation().getCos();
      double sin = center.getRotation().getSin();
      double localX = cos * dx + sin * dy;
      double localY = -sin * dx + cos * dy;
      double halfX = shape.getXWidth() / 2.0;
      double halfY = shape.getYWidth() / 2.0;
      double outX = Math.max(Math.abs(localX) - halfX, 0.0);
      double outY = Math.max(Math.abs(localY) - halfY, 0.0);
      double outside = Math.hypot(outX, outY);
      if (outside > 0.0) return outside;
      return -Math.min(halfX - Math.abs(localX), halfY - Math.abs(localY));
    }

    @Override
    public Translation2d nearestPoint(double x, double y) {
      // Inlined local-frame transform. The returned Translation2d is unavoidable allocation
      // (matches the interface), but the intermediate local-frame coords are kept on the stack.
      Pose2d center = shape.getCenter();
      double dx = x - center.getX();
      double dy = y - center.getY();
      double cos = center.getRotation().getCos();
      double sin = center.getRotation().getSin();
      double lx = cos * dx + sin * dy;
      double ly = -sin * dx + cos * dy;
      double halfX = shape.getXWidth() / 2.0;
      double halfY = shape.getYWidth() / 2.0;
      double clampedX;
      double clampedY;
      if (Math.abs(lx) <= halfX && Math.abs(ly) <= halfY) {
        // Inside: snap to the nearest edge.
        double dxR = halfX - lx;
        double dxL = halfX + lx;
        double dyT = halfY - ly;
        double dyB = halfY + ly;
        double m = Math.min(Math.min(dxR, dxL), Math.min(dyT, dyB));
        if (m == dxR) {
          clampedX = halfX;
          clampedY = ly;
        } else if (m == dxL) {
          clampedX = -halfX;
          clampedY = ly;
        } else if (m == dyT) {
          clampedX = lx;
          clampedY = halfY;
        } else {
          clampedX = lx;
          clampedY = -halfY;
        }
      } else {
        clampedX = Math.max(-halfX, Math.min(halfX, lx));
        clampedY = Math.max(-halfY, Math.min(halfY, ly));
      }
      return new Translation2d(
          center.getX() + cos * clampedX - sin * clampedY,
          center.getY() + sin * clampedX + cos * clampedY);
    }

    @Override
    public boolean intersectsObb(
        double rcx, double rcy, double rCos, double rSin, double rHalfX, double rHalfY) {
      Pose2d center = shape.getCenter();
      double ocx = center.getX();
      double ocy = center.getY();
      double oCos = center.getRotation().getCos();
      double oSin = center.getRotation().getSin();
      double oHalfX = shape.getXWidth() / 2.0;
      double oHalfY = shape.getYWidth() / 2.0;

      // 4 corners of each rectangle in world coordinates.
      double[] oX = corners(ocx, oCos, oSin, oHalfX, oHalfY, true);
      double[] oY = corners(ocy, oCos, oSin, oHalfX, oHalfY, false);
      double[] rX = corners(rcx, rCos, rSin, rHalfX, rHalfY, true);
      double[] rY = corners(rcy, rCos, rSin, rHalfX, rHalfY, false);

      // Separating Axis Theorem: two convex shapes don't overlap iff there's at least one axis
      // where their 1D projections don't overlap. For two rectangles, the only axes worth
      // checking are the four edge-normals (each rectangle contributes two).
      double[][] axes = {{oCos, oSin}, {-oSin, oCos}, {rCos, rSin}, {-rSin, rCos}};
      for (double[] ax : axes) {
        if (!projectionsOverlap(oX, oY, rX, rY, ax[0], ax[1])) return false;
      }
      return true;
    }

    private static double[] corners(
        double c, double cos, double sin, double halfX, double halfY, boolean xComponent) {
      // Local (±halfX, ±halfY), rotated by (cos, sin), translated by center. xComponent picks x
      // vs y of the resulting world coordinate.
      if (xComponent) {
        return new double[] {
          c + halfX * cos - halfY * sin,
          c + halfX * cos + halfY * sin,
          c - halfX * cos + halfY * sin,
          c - halfX * cos - halfY * sin
        };
      }
      return new double[] {
        c + halfX * sin + halfY * cos,
        c + halfX * sin - halfY * cos,
        c - halfX * sin - halfY * cos,
        c - halfX * sin + halfY * cos
      };
    }

    private static boolean projectionsOverlap(
        double[] aX, double[] aY, double[] bX, double[] bY, double axX, double axY) {
      double aMin = Double.POSITIVE_INFINITY, aMax = Double.NEGATIVE_INFINITY;
      double bMin = Double.POSITIVE_INFINITY, bMax = Double.NEGATIVE_INFINITY;
      for (int i = 0; i < 4; i++) {
        double pa = aX[i] * axX + aY[i] * axY;
        if (pa < aMin) aMin = pa;
        if (pa > aMax) aMax = pa;
        double pb = bX[i] * axX + bY[i] * axY;
        if (pb < bMin) bMin = pb;
        if (pb > bMax) bMax = pb;
      }
      return aMax >= bMin && bMax >= aMin;
    }
  }
}
