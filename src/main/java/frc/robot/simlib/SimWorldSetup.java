package frc.robot.simlib;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;
import org.dyn4j.dynamics.Body;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;

/**
 * Registers a list of AABB obstacles as immovable dyn4j bodies in the {@link SimulatedArena}, so
 * the rigid-body sim resists driving through the same obstacles PathPlanner sees.
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

  /** Adds every obstacle as an immovable static body. */
  public static synchronized void addObstacles(
      SimulatedArena arena, List<Pair<Translation2d, Translation2d>> obstacles) {
    for (Body b : ADDED) arena.removeBody(b);
    ADDED.clear();

    for (Pair<Translation2d, Translation2d> box : obstacles) {
      Body b = toBody(box);
      arena.addStaticBody(b);
      ADDED.add(b);
    }
  }

  private static Body toBody(Pair<Translation2d, Translation2d> box) {
    Translation2d min = box.getFirst();
    Translation2d max = box.getSecond();
    double cx = (min.getX() + max.getX()) / 2.0;
    double cy = (min.getY() + max.getY()) / 2.0;
    double widthX = max.getX() - min.getX();
    double widthY = max.getY() - min.getY();
    Body body = new Body();
    body.addFixture(Geometry.createRectangle(widthX, widthY));
    body.translate(cx, cy);
    body.setMass(MassType.INFINITE);
    return body;
  }
}
