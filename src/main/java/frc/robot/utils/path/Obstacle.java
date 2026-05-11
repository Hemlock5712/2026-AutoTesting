package frc.robot.utils.path;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * A field obstacle. Right now we support circles and axis-aligned rectangles - that's enough for
 * almost every FRC obstacle (posts as circles, walls/zones as rectangles, alliance robots as fat
 * circles).
 */
public sealed interface Obstacle permits Obstacle.Circle, Obstacle.Rectangle {

  /**
   * Distance from (x, y) to this obstacle's edge, in meters. Negative if the point is inside the
   * obstacle, positive if outside.
   */
  double signedDistance(double x, double y);

  /**
   * Closest point on this obstacle's boundary to (x, y). For points outside the obstacle, this is
   * the projection onto the nearest edge. For points inside, it's the nearest edge point too — the
   * vector from (x, y) to this point still gives the "into-obstacle" direction (with zero length if
   * the query is exactly on the boundary).
   */
  Translation2d nearestPoint(double x, double y);

  /**
   * True if a rectangle (the robot, rotated to some heading) overlaps this obstacle.
   *
   * @param cx Robot center x (m)
   * @param cy Robot center y (m)
   * @param cosT cos of the robot's heading
   * @param sinT sin of the robot's heading
   * @param halfX Half the robot's length (m)
   * @param halfY Half the robot's width (m)
   */
  boolean intersectsObb(double cx, double cy, double cosT, double sinT, double halfX, double halfY);

  /** Circle - good for posts and other robots (use bumper radius). */
  record Circle(double cx, double cy, double radius) implements Obstacle {
    @Override
    public double signedDistance(double x, double y) {
      double dx = x - cx;
      double dy = y - cy;
      return Math.hypot(dx, dy) - radius;
    }

    @Override
    public Translation2d nearestPoint(double x, double y) {
      double dx = x - cx;
      double dy = y - cy;
      double dist = Math.hypot(dx, dy);
      if (dist < 1.0e-9) {
        return new Translation2d(cx + radius, cy);
      }
      double scale = radius / dist;
      return new Translation2d(cx + dx * scale, cy + dy * scale);
    }

    @Override
    public boolean intersectsObb(
        double bcx, double bcy, double cosT, double sinT, double halfX, double halfY) {
      // Rotate the circle into the rectangle's local frame, then check circle vs rectangle.
      double dx = cx - bcx;
      double dy = cy - bcy;
      double localX = cosT * dx + sinT * dy;
      double localY = -sinT * dx + cosT * dy;
      double clampedX = Math.max(-halfX, Math.min(halfX, localX));
      double clampedY = Math.max(-halfY, Math.min(halfY, localY));
      double ddx = localX - clampedX;
      double ddy = localY - clampedY;
      return ddx * ddx + ddy * ddy < radius * radius;
    }
  }

  /** Rectangle aligned with the field axes. */
  record Rectangle(double minX, double minY, double maxX, double maxY) implements Obstacle {
    public Rectangle {
      if (minX >= maxX || minY >= maxY) {
        throw new IllegalArgumentException("Rectangle bounds must be ordered (min < max)");
      }
    }

    @Override
    public double signedDistance(double x, double y) {
      // Outside: distance to nearest edge. Inside: negative distance to nearest edge.
      double dx = Math.max(Math.max(minX - x, x - maxX), 0.0);
      double dy = Math.max(Math.max(minY - y, y - maxY), 0.0);
      double outside = Math.hypot(dx, dy);
      if (outside > 0.0) {
        return outside;
      }
      double inside = Math.min(Math.min(x - minX, maxX - x), Math.min(y - minY, maxY - y));
      return -inside;
    }

    @Override
    public Translation2d nearestPoint(double x, double y) {
      // Clamp the query into the rectangle. If the query is outside, the clamped point is the
      // closest boundary point. If it's inside, the clamp returns the query itself, so fall back
      // to the nearest edge.
      double cx = Math.max(minX, Math.min(maxX, x));
      double cy = Math.max(minY, Math.min(maxY, y));
      if (cx != x || cy != y) {
        return new Translation2d(cx, cy);
      }
      double dxL = x - minX;
      double dxR = maxX - x;
      double dyB = y - minY;
      double dyT = maxY - y;
      double m = Math.min(Math.min(dxL, dxR), Math.min(dyB, dyT));
      if (m == dxL) return new Translation2d(minX, y);
      if (m == dxR) return new Translation2d(maxX, y);
      if (m == dyB) return new Translation2d(x, minY);
      return new Translation2d(x, maxY);
    }

    @Override
    public boolean intersectsObb(
        double cx, double cy, double cosT, double sinT, double halfX, double halfY) {
      // Separating Axis Theorem: try to find an angle where the two rectangles don't overlap.
      // We only need to check 4 directions - the sides of each rectangle.
      double ux = cosT, uy = sinT;
      double vx = -sinT, vy = cosT;
      // Compute the 4 corners of the rotated robot rectangle.
      double[] obbX = {
        cx + halfX * ux + halfY * vx,
        cx + halfX * ux - halfY * vx,
        cx - halfX * ux - halfY * vx,
        cx - halfX * ux + halfY * vx
      };
      double[] obbY = {
        cy + halfX * uy + halfY * vy,
        cy + halfX * uy - halfY * vy,
        cy - halfX * uy - halfY * vy,
        cy - halfX * uy + halfY * vy
      };
      double[] aabbX = {minX, maxX, maxX, minX};
      double[] aabbY = {minY, minY, maxY, maxY};
      double[][] axes = {{ux, uy}, {vx, vy}, {1, 0}, {0, 1}};
      for (double[] axis : axes) {
        double obbMin = Double.POSITIVE_INFINITY, obbMax = Double.NEGATIVE_INFINITY;
        double aabbMin = Double.POSITIVE_INFINITY, aabbMax = Double.NEGATIVE_INFINITY;
        for (int i = 0; i < 4; i++) {
          double po = obbX[i] * axis[0] + obbY[i] * axis[1];
          if (po < obbMin) obbMin = po;
          if (po > obbMax) obbMax = po;
          double pa = aabbX[i] * axis[0] + aabbY[i] * axis[1];
          if (pa < aabbMin) aabbMin = pa;
          if (pa > aabbMax) aabbMax = pa;
        }
        if (obbMax < aabbMin || aabbMax < obbMin) return false;
      }
      return true;
    }
  }
}
