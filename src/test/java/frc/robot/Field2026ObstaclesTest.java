package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utils.FieldInfo;
import frc.robot.utils.path.Footprint;
import frc.robot.utils.path.Obstacle;
import frc.robot.utils.path.ObstacleAvoidance;
import frc.robot.utils.path.ObstacleField;
import java.util.List;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class Field2026ObstaclesTest {

  // Pin the layout before Hub/Tower static fields capture aprilTags() values, so AprilTag-derived
  // positions stay deterministic across Andymark/Welded variants.
  @BeforeAll
  public static void pinLayout() {
    FieldInfo.setLayout(AprilTagFields.k2026RebuiltWelded);
  }

  private static final double HALF_X = Drive.ROBOT_HALF_X;
  private static final double HALF_Y = Drive.ROBOT_HALF_Y;
  private static final double DECEL = Drive.AVOIDANCE_DECEL_BUDGET;
  private static final double MARGIN = Drive.AVOIDANCE_SAFETY_MARGIN_M;
  private static final double EPS = 1e-6;

  @Test
  public void obstacleCountAndSymmetry() {
    // 4 perimeter + 5 blue + 5 red.
    assertEquals(14, Field2026Obstacles.build().staticObstacles().size());
  }

  @Test
  public void rotateMirrorPlacesRedHubAtMirroredCenter() {
    // Probe a point inside the expected red Hub footprint. If rotateMirror is broken (e.g. swapped
    // axes, identity, wrong L/W), this point falls in free space and signedDistance is positive.
    ObstacleField f = Field2026Obstacles.build();
    double L = FieldInfo.lengthMeters();
    double W = FieldInfo.widthMeters();
    double blueHubCx = Field2026Constants.Hub.nearLeftCorner.getX() + Field2026Constants.Hub.width / 2.0;
    double redHubCx = L - blueHubCx;
    double centerY = W / 2.0;
    assertTrue(
        f.signedDistance(redHubCx, centerY) < 0.0,
        "Red Hub interior at (" + redHubCx + ", " + centerY + ") must be inside an obstacle");
  }

  @Test
  public void clampReducesToBrakeCurveExactly() {
    // Single wall, robot square-on, margin zero — pins the brake-curve math: vMax = sqrt(2·a·d).
    // Existing clampReducesApproachAndPreservesTangent only asserts "less than commanded," which
    // would still pass under a regression that under-clamps.
    ObstacleField f = new ObstacleField();
    f.addStatic(new Obstacle.Rectangle(2.0, -5.0, 5.0, 5.0));
    double half = 0.3;
    double brake = 6.0;
    ObstacleAvoidance avoid =
        new ObstacleAvoidance(f, Footprint.fixed(half, half), brake, 0.0, null);

    Translation2d clamped = avoid.clamp(new Pose2d(1.0, 0.0, Rotation2d.kZero), 5.0, 0.0);
    // sd = 1.0 (wall at x=2, robot at x=1), extent = 0.3, free = 0.7.
    double expectedVMax = Math.sqrt(2.0 * brake * 0.7);
    assertEquals(expectedVMax, clamped.getX(), EPS);
  }

  @Test
  public void spawnPoseIsClear() {
    double sx = 8.27;
    double sy = 4.0;
    ObstacleField f = Field2026Obstacles.build();
    for (Obstacle o : f.staticObstacles()) {
      assertFalse(
          o.intersectsObb(sx, sy, 1.0, 0.0, HALF_X, HALF_Y),
          "Spawn (" + sx + ", " + sy + ") overlaps " + o);
    }
  }

  @Test
  public void clampReducesApproachAndPreservesTangent() {
    ObstacleAvoidance avoid =
        new ObstacleAvoidance(
            Field2026Obstacles.build(), Footprint.fixed(HALF_X, HALF_Y), DECEL, MARGIN, null);
    Pose2d pose = new Pose2d(10.8, 4.0, Rotation2d.kZero); // near the red Hub edge

    Translation2d clamped = avoid.clamp(pose, 3.0, 0.0);
    assertTrue(clamped.getX() < 3.0, "Approach should be clamped, got " + clamped.getX());

    Translation2d sliding = avoid.clamp(pose, 0.0, 1.5);
    assertEquals(1.5, sliding.getY(), EPS, "Tangential motion must pass through");
  }

  @Test
  public void clampZeroesApproachWhenBoundingBoxOverlapsObstacle() {
    // Robot center is outside the obstacle (sd > 0) but the OBB overlaps it (free < 0). vMax
    // collapses to 0 and the approach component must be removed entirely.
    ObstacleField f = new ObstacleField();
    f.addStatic(new Obstacle.Rectangle(0.3, -1.0, 1.0, 1.0));
    ObstacleAvoidance avoid = new ObstacleAvoidance(f, Footprint.fixed(0.4, 0.4), 6.0, 0.0, null);

    Translation2d clamped = avoid.clamp(Pose2d.kZero, 2.0, 0.0);
    assertEquals(0.0, clamped.getX(), EPS, "vMax should be 0 when OBB overlaps obstacle");
  }

  @Test
  public void clampInsideObstaclePreservesEscapeAndKillsDeepening() {
    // Robot center sits inside a 2×2 rectangle, closer to the +x edge than any other. Outward
    // motion (+x) must pass through; motion deeper into the obstacle (-x) must be killed.
    ObstacleField f = new ObstacleField();
    f.addStatic(new Obstacle.Rectangle(-1.0, -1.0, 1.0, 1.0));
    ObstacleAvoidance avoid = new ObstacleAvoidance(f, Footprint.fixed(0.4, 0.4), 6.0, 0.0, null);
    Pose2d pose = new Pose2d(0.5, 0.5, Rotation2d.kZero);

    Translation2d outward = avoid.clamp(pose, 2.0, 0.0);
    assertEquals(2.0, outward.getX(), EPS, "Outward escape must pass through");

    Translation2d inward = avoid.clamp(pose, -2.0, 0.0);
    assertEquals(0.0, inward.getX(), EPS, "Motion deeper into obstacle must be killed");
  }

  @Test
  public void clampHandlesPerpendicularCorner() {
    // East wall at x≥1, north wall at y≥1. Robot at (0.9, 0.9) moving diagonally into the corner —
    // both axes must clamp despite being processed sequentially.
    ObstacleField f = new ObstacleField();
    f.addStatic(new Obstacle.Rectangle(1.0, -5.0, 5.0, 5.0));
    f.addStatic(new Obstacle.Rectangle(-5.0, 1.0, 5.0, 5.0));
    ObstacleAvoidance avoid = new ObstacleAvoidance(f, Footprint.fixed(0.1, 0.1), 6.0, 0.0, null);

    Translation2d clamped = avoid.clamp(new Pose2d(0.9, 0.9, Rotation2d.kZero), 2.0, 2.0);
    assertEquals(0.0, clamped.getX(), EPS, "X approach toward east wall must clamp");
    assertEquals(0.0, clamped.getY(), EPS, "Y approach toward north wall must clamp");
  }

  @Test
  public void clampRespectsRotationInOBBExtent() {
    // 45° heading. OBB extent along the +x normal is halfX·|cos45| + halfY·|sin45| = half·√2.
    ObstacleField f = new ObstacleField();
    f.addStatic(new Obstacle.Rectangle(1.0, -5.0, 5.0, 5.0));
    double half = 0.3;
    double brake = 6.0;
    ObstacleAvoidance avoid =
        new ObstacleAvoidance(f, Footprint.fixed(half, half), brake, 0.0, null);
    Pose2d pose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(45));

    Translation2d clamped = avoid.clamp(pose, 3.0, 0.0);
    double expectedFree = 1.0 - half * Math.sqrt(2.0);
    double expectedVMax = Math.sqrt(2.0 * brake * expectedFree);
    assertEquals(
        expectedVMax,
        clamped.getX(),
        1e-6,
        "Rotated OBB extent should be halfX·|cos| + halfY·|sin|");
  }

  @Test
  public void clampUsesFootprintOffset() {
    // A 0.5 m forward offset (e.g. deployed intake) moves the effective bbox center toward the
    // wall, so the clamp must engage sooner than without an offset.
    ObstacleField f = new ObstacleField();
    f.addStatic(new Obstacle.Rectangle(2.0, -5.0, 5.0, 5.0));
    double half = 0.1;
    Footprint shifted = new Footprint(half, half, 0.5, 0.0);
    ObstacleAvoidance noOffset =
        new ObstacleAvoidance(f, Footprint.fixed(half, half), 6.0, 0.0, null);
    ObstacleAvoidance withOffset = new ObstacleAvoidance(f, shifted, 6.0, 0.0, null);

    Translation2d a = noOffset.clamp(Pose2d.kZero, 5.0, 0.0);
    Translation2d b = withOffset.clamp(Pose2d.kZero, 5.0, 0.0);
    assertTrue(
        b.getX() < a.getX(),
        "Forward-offset footprint must see the wall as closer and clamp harder ("
            + b.getX()
            + " vs "
            + a.getX()
            + ")");
  }

  @Test
  public void dynamicObstacleSnapshotIsImmutable() {
    ObstacleField f = new ObstacleField();
    f.addDynamic(new Obstacle.Circle(1.0, 0.0, 0.2));
    List<Obstacle> snapshot = f.dynamicObstacles();
    f.clearDynamic();
    f.addDynamic(new Obstacle.Circle(2.0, 0.0, 0.2));

    assertEquals(1, snapshot.size(), "Held snapshot must not see later mutations");
    assertEquals(1, f.dynamicObstacles().size(), "Latest snapshot reflects current state");
  }
}
