package frc.robot.utils.path;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * A list of obstacles on the field.
 *
 * <p>Static obstacles (reef, processor, etc.) are added once at startup. Dynamic obstacles (other
 * robots) get added/cleared each frame before replanning. Use {@link Costmap#build} to turn this
 * into a planning grid.
 */
public final class ObstacleField {

  private final List<Obstacle> staticObstacles = new ArrayList<>();
  private final List<Obstacle> dynamicObstacles = new ArrayList<>();

  /** Adds a static obstacle. Returns this for chaining. */
  public ObstacleField addStatic(Obstacle o) {
    staticObstacles.add(o);
    return this;
  }

  /** Adds a dynamic obstacle (cleared by {@link #clearDynamic}). Returns this for chaining. */
  public ObstacleField addDynamic(Obstacle o) {
    dynamicObstacles.add(o);
    return this;
  }

  /** Removes all dynamic obstacles. Static obstacles are unaffected. */
  public void clearDynamic() {
    dynamicObstacles.clear();
  }

  /** Read-only view of static obstacles. */
  public List<Obstacle> staticObstacles() {
    return Collections.unmodifiableList(staticObstacles);
  }

  /** Read-only view of dynamic obstacles. */
  public List<Obstacle> dynamicObstacles() {
    return Collections.unmodifiableList(dynamicObstacles);
  }

  /**
   * Distance to the nearest obstacle. Positive in free space, negative inside an obstacle. Returns
   * infinity if there are no obstacles.
   */
  public double signedDistance(double x, double y) {
    double best = Double.POSITIVE_INFINITY;
    for (int i = 0, n = staticObstacles.size(); i < n; i++) {
      double d = staticObstacles.get(i).signedDistance(x, y);
      if (d < best) best = d;
    }
    for (int i = 0, n = dynamicObstacles.size(); i < n; i++) {
      double d = dynamicObstacles.get(i).signedDistance(x, y);
      if (d < best) best = d;
    }
    return best;
  }

  /** True if a robot rectangle at this location/heading clips any obstacle. */
  public boolean clipsObb(
      double cx, double cy, double cosT, double sinT, double halfX, double halfY) {
    for (int i = 0, n = staticObstacles.size(); i < n; i++) {
      if (staticObstacles.get(i).intersectsObb(cx, cy, cosT, sinT, halfX, halfY)) return true;
    }
    for (int i = 0, n = dynamicObstacles.size(); i < n; i++) {
      if (dynamicObstacles.get(i).intersectsObb(cx, cy, cosT, sinT, halfX, halfY)) return true;
    }
    return false;
  }
}
