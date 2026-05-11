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
 *
 * <p>Calling the {@link #staticObstacles()} / {@link #dynamicObstacles()} accessors is safe from
 * any thread, including the 250 Hz drive fast loop. Dynamic obstacles use copy-on-write so
 * iterating the returned list is safe even during concurrent {@link #addDynamic} / {@link
 * #clearDynamic} calls. Static obstacles are intended to be built once at startup and not mutated
 * thereafter — the returned unmodifiable view does NOT protect concurrent iteration against late
 * {@link #addStatic} calls. If you ever need runtime-mutable static obstacles, switch the static
 * list to the same copy-on-write pattern.
 */
public final class ObstacleField {

  private final List<Obstacle> staticObstacles = new ArrayList<>();
  private final List<Obstacle> staticObstaclesView = Collections.unmodifiableList(staticObstacles);
  // Replaced wholesale on each mutation so concurrent readers see an immutable snapshot.
  private volatile List<Obstacle> dynamicSnapshot = List.of();

  /** Adds a static obstacle. Returns this for chaining. */
  public ObstacleField addStatic(Obstacle o) {
    staticObstacles.add(o);
    return this;
  }

  /** Adds a dynamic obstacle (cleared by {@link #clearDynamic}). Returns this for chaining. */
  public ObstacleField addDynamic(Obstacle o) {
    List<Obstacle> next = new ArrayList<>(dynamicSnapshot.size() + 1);
    next.addAll(dynamicSnapshot);
    next.add(o);
    dynamicSnapshot = Collections.unmodifiableList(next);
    return this;
  }

  /** Removes all dynamic obstacles. Static obstacles are unaffected. */
  public void clearDynamic() {
    dynamicSnapshot = List.of();
  }

  /** Read-only view of static obstacles. Cached — no allocation per call. */
  public List<Obstacle> staticObstacles() {
    return staticObstaclesView;
  }

  /** Immutable snapshot of dynamic obstacles. Safe to iterate concurrently with mutations. */
  public List<Obstacle> dynamicObstacles() {
    return dynamicSnapshot;
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
    List<Obstacle> dyn = dynamicSnapshot;
    for (int i = 0, n = dyn.size(); i < n; i++) {
      double d = dyn.get(i).signedDistance(x, y);
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
    List<Obstacle> dyn = dynamicSnapshot;
    for (int i = 0, n = dyn.size(); i < n; i++) {
      if (dyn.get(i).intersectsObb(cx, cy, cosT, sinT, halfX, halfY)) return true;
    }
    return false;
  }
}
