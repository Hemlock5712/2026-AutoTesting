package frc.robot.utils.path;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Optional;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Integration tests for the runtime path generator. These run identically in real and replay modes
 * because every component in the pipeline is a pure function — no clock reads, no random numbers,
 * no I/O. If a path passes here, it would pass on a roboRIO too.
 *
 * <p>The load-bearing assertion is {@link #assertNoCollision} — re-samples the generated path at 5x
 * the planner's sample spacing and verifies every point keeps the robot disc clear of every
 * obstacle. This is the "no part of the robot enters an obstacle" guarantee.
 */
class PathGenerationTest {

  private static final double FIELD_X = 17.55;
  private static final double FIELD_Y = 8.05;
  private static final double CELL_SIZE = 0.1;
  private static final double ROBOT_RADIUS = 0.5;
  private static final double PLAN_SAMPLE_SPACING = 0.05;
  private static final PathGenerator.Config DEFAULT_CONFIG =
      new PathGenerator.Config(PLAN_SAMPLE_SPACING, VelocityProfiler.Constraints.defaults());

  /** Re-sample path at 5x the planner's spacing to catch between-sample spline excursions. */
  private static final double VALIDATION_SPACING = PLAN_SAMPLE_SPACING / 5.0;

  /**
   * Slack on the clearance check (m). Re-sampled points are interpolated linearly between sparser
   * planner samples, so the test sample can sit fractionally closer to an obstacle than the
   * validated planner sample. 1 cm is plenty given a 0.5 m robot radius.
   */
  private static final double CLEARANCE_TOLERANCE = 0.01;

  private ObstacleField field;
  private Costmap map;

  @BeforeEach
  void setUp() {
    field =
        new ObstacleField()
            .addStatic(new Obstacle.Circle(8.27, 4.03, 1.0)) // central pillar
            .addStatic(new Obstacle.Rectangle(3.0, 1.0, 4.0, 5.0)) // left wall
            .addStatic(new Obstacle.Circle(13.0, 2.5, 0.6)); // small post
    map = Costmap.build(field, 0.0, 0.0, FIELD_X, FIELD_Y, CELL_SIZE, ROBOT_RADIUS);
  }

  @Test
  void straightShotInClearLane_clearsAllObstacles() {
    PathGenerator.Request req =
        new PathGenerator.Request(
            new Pose2d(1.0, 7.5, Rotation2d.kZero),
            new Pose2d(16.0, 7.5, Rotation2d.kZero),
            0.0,
            0.0);
    Optional<GeneratedPath> result = PathGenerator.generate(field, map, req, DEFAULT_CONFIG);
    assertTrue(result.isPresent(), "Top-of-field straight path should succeed");
    assertNoCollision(result.get());
    assertVelocityProfileSafe(result.get());
  }

  @Test
  void pathThroughObstacleBelt_routesAround() {
    // Goal is on the far side of the central pillar + small post; the planner must detour.
    PathGenerator.Request req =
        new PathGenerator.Request(
            new Pose2d(1.0, 4.0, Rotation2d.kZero),
            new Pose2d(16.0, 4.0, Rotation2d.fromDegrees(90)),
            0.0,
            0.0);
    Optional<GeneratedPath> result = PathGenerator.generate(field, map, req, DEFAULT_CONFIG);
    assertTrue(result.isPresent(), "Detour path should succeed");
    assertNoCollision(result.get());
    assertVelocityProfileSafe(result.get());
    assertEndpointsExact(result.get(), req);
  }

  @Test
  void dynamicObstacleOnPath_forcesNewRouteThatStillClears() {
    // Use a field with room on both sides of the dynamic obstacle so the detour is feasible.
    ObstacleField openField = new ObstacleField();
    Costmap openMap = Costmap.build(openField, 0.0, 0.0, FIELD_X, FIELD_Y, CELL_SIZE, ROBOT_RADIUS);

    PathGenerator.Request req =
        new PathGenerator.Request(
            new Pose2d(1.0, 4.0, Rotation2d.kZero),
            new Pose2d(16.0, 4.0, Rotation2d.kZero),
            0.0,
            0.0);
    Optional<GeneratedPath> firstResult =
        PathGenerator.generate(openField, openMap, req, DEFAULT_CONFIG);
    assertTrue(firstResult.isPresent());

    // Drop a vision-tracked ally on the previous path's midpoint.
    Translation2d midpoint = firstResult.get().getPoint(firstResult.get().getTotalLength() / 2.0);
    openField.addDynamic(new Obstacle.Circle(midpoint.getX(), midpoint.getY(), 0.6));
    Costmap updatedMap =
        Costmap.build(openField, 0.0, 0.0, FIELD_X, FIELD_Y, CELL_SIZE, ROBOT_RADIUS);

    Optional<GeneratedPath> secondResult =
        PathGenerator.generate(openField, updatedMap, req, DEFAULT_CONFIG);
    assertTrue(secondResult.isPresent(), "Replan around dynamic obstacle should succeed");
    assertNoCollision(secondResult.get(), openField);
    assertVelocityProfileSafe(secondResult.get());

    // Sanity check: the new path actually detours (max y-deviation > obstacle radius).
    double maxDeviation = 0.0;
    double total = secondResult.get().getTotalLength();
    for (int i = 0; i <= 100; i++) {
      Translation2d p = secondResult.get().getPoint(total * i / 100.0);
      maxDeviation = Math.max(maxDeviation, Math.abs(p.getY() - 4.0));
    }
    assertTrue(maxDeviation > 0.5, "Replanned path should visibly detour around the new obstacle");
  }

  @Test
  void startInsideObstacle_returnsEmpty() {
    PathGenerator.Request req =
        new PathGenerator.Request(
            new Pose2d(8.27, 4.03, Rotation2d.kZero), // dead-center of pillar
            new Pose2d(16.0, 7.0, Rotation2d.kZero),
            0.0,
            0.0);
    Optional<GeneratedPath> result = PathGenerator.generate(field, map, req, DEFAULT_CONFIG);
    assertFalse(result.isPresent(), "Start inside an obstacle must fail");
  }

  @Test
  void goalEnclosedByObstacles_returnsEmpty() {
    ObstacleField boxed = new ObstacleField();
    // Wall a 1m x 1m box around (10, 4) so no path can reach inside.
    boxed.addStatic(new Obstacle.Rectangle(9.4, 3.4, 9.6, 4.6));
    boxed.addStatic(new Obstacle.Rectangle(10.4, 3.4, 10.6, 4.6));
    boxed.addStatic(new Obstacle.Rectangle(9.4, 3.4, 10.6, 3.6));
    boxed.addStatic(new Obstacle.Rectangle(9.4, 4.4, 10.6, 4.6));
    Costmap boxedMap = Costmap.build(boxed, 0.0, 0.0, FIELD_X, FIELD_Y, CELL_SIZE, ROBOT_RADIUS);

    PathGenerator.Request req =
        new PathGenerator.Request(
            new Pose2d(1.0, 4.0, Rotation2d.kZero),
            new Pose2d(10.0, 4.0, Rotation2d.kZero),
            0.0,
            0.0);
    Optional<GeneratedPath> result = PathGenerator.generate(boxed, boxedMap, req, DEFAULT_CONFIG);
    assertFalse(result.isPresent(), "Walled-in goal must fail");
  }

  @Test
  void velocityProfileObeysTerminalConstraints() {
    PathGenerator.Request req =
        new PathGenerator.Request(
            new Pose2d(1.0, 7.5, Rotation2d.kZero),
            new Pose2d(16.0, 7.5, Rotation2d.kZero),
            1.5,
            0.3);
    Optional<GeneratedPath> result = PathGenerator.generate(field, map, req, DEFAULT_CONFIG);
    assertTrue(result.isPresent());
    GeneratedPath path = result.get();
    assertEquals(1.5, path.getVelocity(0.0), 0.05, "Start velocity should match request");
    assertEquals(
        0.3, path.getVelocity(path.getTotalLength()), 0.05, "End velocity should match request");
  }

  @Test
  void manyRandomQueries_allClearOrFailCleanly() {
    java.util.Random rng = new java.util.Random(42);
    int generated = 0;
    int attempts = 50;
    for (int i = 0; i < attempts; i++) {
      double sx = 0.6 + rng.nextDouble() * (FIELD_X - 1.2);
      double sy = 0.6 + rng.nextDouble() * (FIELD_Y - 1.2);
      double gx = 0.6 + rng.nextDouble() * (FIELD_X - 1.2);
      double gy = 0.6 + rng.nextDouble() * (FIELD_Y - 1.2);
      PathGenerator.Request req =
          new PathGenerator.Request(
              new Pose2d(sx, sy, Rotation2d.kZero), new Pose2d(gx, gy, Rotation2d.kZero), 0.0, 0.0);
      Optional<GeneratedPath> result = PathGenerator.generate(field, map, req, DEFAULT_CONFIG);
      if (result.isEmpty()) continue; // start/goal landed in obstacle, fine
      assertNoCollision(result.get());
      generated++;
    }
    // ~17% of the field is occupied by inflated obstacles, so ~30% of random (start, goal) pairs
    // have at least one endpoint in an obstacle. Floor at 20/50 leaves slack for the long tail
    // of pinched-corridor failures.
    assertTrue(generated >= 20, "Random queries should mostly yield paths; got " + generated);
  }

  /**
   * Re-samples the path at {@link #VALIDATION_SPACING} (finer than the planner's own validation
   * grid) and verifies every point keeps the robot disc clear of every obstacle. Failure here
   * indicates the planner produced a path that violates the "no part of the robot enters an
   * obstacle" guarantee.
   */
  private void assertNoCollision(GeneratedPath path) {
    assertNoCollision(path, field);
  }

  private static void assertNoCollision(GeneratedPath path, ObstacleField against) {
    double total = path.getTotalLength();
    int samples = Math.max(2, (int) Math.ceil(total / VALIDATION_SPACING) + 1);
    for (int i = 0; i < samples; i++) {
      double s = total * i / (samples - 1);
      Translation2d p = path.getPoint(s);
      double clearance = against.signedDistance(p.getX(), p.getY());
      assertTrue(
          clearance + CLEARANCE_TOLERANCE >= ROBOT_RADIUS,
          String.format(
              "Robot disc clips obstacle at s=%.3f, point=(%.3f, %.3f), clearance=%.4f m"
                  + " < %.3f m (radius)",
              s, p.getX(), p.getY(), clearance, ROBOT_RADIUS));
    }
  }

  /** Verifies the velocity profile respects max velocity and the centripetal lateral-accel cap. */
  private static void assertVelocityProfileSafe(GeneratedPath path) {
    double total = path.getTotalLength();
    VelocityProfiler.Constraints c = VelocityProfiler.Constraints.defaults();
    int samples = 200;
    for (int i = 0; i < samples; i++) {
      double s = total * i / (samples - 1);
      double v = path.getVelocity(s);
      double k = path.getCurvature(s);
      assertTrue(v >= -1e-9, "Velocity must be non-negative at s=" + s + " (got " + v + ")");
      assertTrue(
          v <= c.maxVelocity() + 1e-6,
          "Velocity " + v + " exceeds maxVelocity " + c.maxVelocity() + " at s=" + s);
      double aLat = v * v * Math.abs(k);
      // Allow 10% slack — interpolated curvature between samples is not exactly the value the
      // profiler used, and the friction circle is a quadratic constraint.
      assertTrue(
          aLat <= c.maxFrictionAccel() * 1.1 + 1e-3,
          String.format(
              "Lateral accel %.3f exceeds 1.1 * maxFrictionAccel %.3f at s=%.3f (v=%.3f, k=%.3f)",
              aLat, c.maxFrictionAccel(), s, v, k));
    }
  }

  private static void assertEndpointsExact(GeneratedPath path, PathGenerator.Request req) {
    Translation2d start = path.getPoint(0.0);
    Translation2d end = path.getPoint(path.getTotalLength());
    assertEquals(req.start().getX(), start.getX(), 1e-6);
    assertEquals(req.start().getY(), start.getY(), 1e-6);
    assertEquals(req.goal().getX(), end.getX(), 1e-6);
    assertEquals(req.goal().getY(), end.getY(), 1e-6);
  }
}
