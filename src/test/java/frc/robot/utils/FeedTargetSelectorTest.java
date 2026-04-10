package frc.robot.utils;

import static edu.wpi.first.units.Units.Meters;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.Superstructure.FeedMode;
import frc.robot.utils.FeedTargetSelector.FeedSelection;
import frc.robot.utils.FeedTargetSelector.FeedSide;
import org.junit.jupiter.api.Test;

class FeedTargetSelectorTest {
  private static final Translation2d LEFT_FEED = new Translation2d(-4.0, 2.0);
  private static final Translation2d RIGHT_FEED = new Translation2d(-4.0, -2.0);
  private static final Translation2d FAR_HUB = new Translation2d(100.0, 100.0);

  @Test
  void forcedSideIgnoresRobotY() {
    FeedSelection upperLeft =
        FeedTargetSelector.selectTeleopTarget(
            FeedMode.FORCE_LEFT,
            new Translation2d(6.0, 3.0),
            new Translation2d(6.0, 3.0),
            LEFT_FEED,
            RIGHT_FEED,
            FAR_HUB);
    FeedSelection lowerRight =
        FeedTargetSelector.selectTeleopTarget(
            FeedMode.FORCE_RIGHT,
            new Translation2d(6.0, -3.0),
            new Translation2d(6.0, -3.0),
            LEFT_FEED,
            RIGHT_FEED,
            FAR_HUB);

    assertEquals(FeedSide.LEFT, upperLeft.side());
    assertEquals(LEFT_FEED, upperLeft.preferredTarget());
    assertEquals(LEFT_FEED, upperLeft.resolvedTarget());
    assertEquals(FeedSide.RIGHT, lowerRight.side());
    assertEquals(RIGHT_FEED, lowerRight.preferredTarget());
    assertEquals(RIGHT_FEED, lowerRight.resolvedTarget());
  }

  @Test
  void autoModeSelectsUpperAndLowerTargets() {
    FeedSelection upperSelection =
        FeedTargetSelector.selectTeleopTarget(
            FeedMode.AUTO,
            new Translation2d(6.0, 1.0),
            new Translation2d(6.0, 1.0),
            LEFT_FEED,
            RIGHT_FEED,
            FAR_HUB);
    FeedSelection lowerSelection =
        FeedTargetSelector.selectTeleopTarget(
            FeedMode.AUTO,
            new Translation2d(6.0, -1.0),
            new Translation2d(6.0, -1.0),
            LEFT_FEED,
            RIGHT_FEED,
            FAR_HUB);
    FeedSelection autoSelection =
        FeedTargetSelector.selectAutoTarget(new Translation2d(6.0, -1.0), LEFT_FEED, RIGHT_FEED);

    assertEquals(FeedSide.LEFT, upperSelection.side());
    assertEquals(LEFT_FEED, upperSelection.preferredTarget());
    assertEquals(FeedSide.RIGHT, lowerSelection.side());
    assertEquals(RIGHT_FEED, lowerSelection.preferredTarget());
    assertEquals(FeedSide.RIGHT, autoSelection.side());
    assertEquals(RIGHT_FEED, autoSelection.resolvedTarget());
  }

  @Test
  void shiftedTargetUsesNearestClearCandidate() {
    Translation2d shooter = new Translation2d(4.0, 0.0);
    Translation2d preferredTarget = new Translation2d(-4.0, 0.0);
    Translation2d alternateSideTarget = new Translation2d(-4.0, -2.0);
    Translation2d hub = new Translation2d(0.0, -0.61);

    FeedSelection selection =
        FeedTargetSelector.resolveShiftedTarget(
            shooter, preferredTarget, FeedSide.LEFT, preferredTarget, alternateSideTarget, hub);

    assertFalse(selection.blockedByHub());
    assertEquals(preferredTarget, selection.preferredTarget());
    assertEquals(0.1, selection.offset().in(Meters), 1e-9);
    assertEquals(0.1, selection.resolvedTarget().getDistance(preferredTarget), 1e-9);
    assertTrue(selection.clearance().in(Meters) > 0.0);
  }

  @Test
  void circularSearchFindsCandidateOutsideOriginalLine() {
    Translation2d shooter = new Translation2d(4.0, 0.0);
    Translation2d preferredTarget = new Translation2d(-3.0, 1.0);
    Translation2d alternateSideTarget = new Translation2d(-3.0, -1.0);
    Translation2d hub = new Translation2d(-0.4, 0.6);

    FeedSelection selection =
        FeedTargetSelector.resolveShiftedTarget(
            shooter, preferredTarget, FeedSide.LEFT, preferredTarget, alternateSideTarget, hub);

    assertFalse(selection.blockedByHub());
    assertEquals(1.0, selection.offset().in(Meters), 1e-9);
    assertEquals(1.0, selection.resolvedTarget().getDistance(preferredTarget), 1e-9);
    assertTrue(Math.abs(selection.resolvedTarget().getX() - preferredTarget.getX()) > 0.05);
    assertTrue(selection.clearance().in(Meters) > 0.0);
  }

  @Test
  void blockedSelectionFallsBackToPreferredTarget() {
    Translation2d shooter = new Translation2d(4.0, 0.0);
    Translation2d preferredTarget = new Translation2d(-4.0, 0.0);
    Translation2d alternateSideTarget = new Translation2d(-4.0, -2.0);
    Translation2d hub = Translation2d.kZero;

    FeedSelection selection =
        FeedTargetSelector.resolveShiftedTarget(
            shooter, preferredTarget, FeedSide.LEFT, preferredTarget, alternateSideTarget, hub);

    assertTrue(selection.blockedByHub());
    assertEquals(preferredTarget, selection.resolvedTarget());
    assertEquals(0.0, selection.offset().in(Meters), 1e-9);
    assertEquals(0.0, selection.clearance().in(Meters), 1e-9);
  }

  @Test
  void hubCorridorCheckMatchesExpectedGeometry() {
    assertTrue(
        FeedTargetSelector.isPathBlockedByHub(
            new Translation2d(4.0, 0.0), new Translation2d(-4.0, 0.0), Translation2d.kZero));
    assertFalse(
        FeedTargetSelector.isPathBlockedByHub(
            new Translation2d(4.0, 2.0), new Translation2d(-4.0, 2.0), Translation2d.kZero));
    assertTrue(
        FeedTargetSelector.isPathBlockedByHub(
            new Translation2d(2.0, 0.72), new Translation2d(-2.0, 0.72), Translation2d.kZero));
    assertFalse(
        FeedTargetSelector.isPathBlockedByHub(
            new Translation2d(2.0, 0.75), new Translation2d(-2.0, 0.75), Translation2d.kZero));
  }
}
