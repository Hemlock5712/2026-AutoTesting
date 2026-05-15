package frc.robot.simlib;

import edu.wpi.first.math.geometry.Ellipse2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import frc.robot.utils.path.Obstacle;
import frc.robot.utils.path.ObstacleField;
import java.util.ArrayList;
import java.util.List;
import org.dyn4j.dynamics.Body;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;

/**
 * Registers an {@link ObstacleField} as immovable dyn4j bodies in the {@link SimulatedArena}, so
 * the rigid-body sim resists driving through the same obstacles the avoidance clamp sees.
 *
 * <p>Idempotent: re-calling {@link #addObstacles} removes the previously-registered batch first so
 * a re-instantiated RobotContainer (sim hot-reload, integration tests) doesn't stack duplicate
 * bodies in the singleton {@link SimulatedArena}.
 */
public final class SimWorldSetup {

  // Bodies this class has previously added to the singleton arena. Cleared and replaced on each
  // addObstacles call. Static because SimulatedArena is itself a process-wide singleton.
  private static final List<Body> ADDED = new ArrayList<>();

  private SimWorldSetup() {}

  /** Adds every obstacle (static + dynamic) as a static body. */
  public static synchronized void addObstacles(SimulatedArena arena, ObstacleField field) {
    for (Body b : ADDED) arena.removeBody(b);
    ADDED.clear();

    for (Obstacle o : field.staticObstacles()) {
      Body b = toBody(o);
      arena.addStaticBody(b);
      ADDED.add(b);
    }
    for (Obstacle o : field.dynamicObstacles()) {
      Body b = toBody(o);
      arena.addStaticBody(b);
      ADDED.add(b);
    }
  }

  /** Builds an immovable dyn4j body matching the obstacle's geometry. */
  private static Body toBody(Obstacle o) {
    Body body = new Body();
    if (o instanceof Obstacle.Circle c) {
      Ellipse2d e = c.shape();
      // dyn4j has no native ellipse fixture for static bodies; circles use semi-axis as radius.
      body.addFixture(Geometry.createCircle(e.getXSemiAxis()));
      body.translate(e.getCenter().getX(), e.getCenter().getY());
    } else if (o instanceof Obstacle.Rectangle r) {
      Rectangle2d rect = r.shape();
      body.addFixture(Geometry.createRectangle(rect.getXWidth(), rect.getYWidth()));
      Pose2d c = rect.getCenter();
      body.translate(c.getX(), c.getY());
      double rad = c.getRotation().getRadians();
      if (rad != 0.0) body.rotate(rad);
    } else {
      throw new IllegalArgumentException("Unknown obstacle type: " + o.getClass());
    }
    body.setMass(MassType.INFINITE);
    return body;
  }
}
