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
  private static final Translation2d FAR_NET = new Translation2d(100.0, 100.0);

  // Net center at (5.0, 5.0) means Y range is [3.5, 6.5] since NET_INSET = 3.5
  private static final Translation2d TEST_NET = new Translation2d(5.0, 5.0);

  @Test
  void forcedSideIgnoresRobotY() {
    FeedSelection upperLeft =
        FeedTargetSelector.selectTeleopTarget(
            FeedMode.FORCE_LEFT,
            new Translation2d(6.0, 3.0),
            new Translation2d(6.0, 3.0),
            LEFT_FEED,
            RIGHT_FEED,
            FAR_NET);
    FeedSelection lowerRight =
        FeedTargetSelector.selectTeleopTarget(
            FeedMode.FORCE_RIGHT,
            new Translation2d(6.0, -3.0),
            new Translation2d(6.0, -3.0),
            LEFT_FEED,
            RIGHT_FEED,
            FAR_NET);

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
            FAR_NET);
    FeedSelection lowerSelection =
        FeedTargetSelector.selectTeleopTarget(
            FeedMode.AUTO,
            new Translation2d(6.0, -1.0),
            new Translation2d(6.0, -1.0),
            LEFT_FEED,
            RIGHT_FEED,
            FAR_NET);
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
  void blockedPathFlagsSelection() {
    // Net at (0, 5) → Y range [3.5, 6.5], shot at y=5 crosses through center
    FeedSelection selection =
        FeedTargetSelector.selectTeleopTarget(
            FeedMode.FORCE_LEFT,
            new Translation2d(4.0, 5.0),
            new Translation2d(4.0, 5.0),
            new Translation2d(-4.0, 5.0),
            RIGHT_FEED,
            new Translation2d(0.0, 5.0));

    assertTrue(selection.blocked());
    assertEquals(0.0, selection.clearance().in(Meters), 1e-9);
  }

  @Test
  void netLineBlocksAndClearsCorrectly() {
    // TEST_NET at (5.0, 5.0) → Y range [3.5, 6.5]
    // Shot through center at y=5.0 — blocked
    assertTrue(
        FeedTargetSelector.isPathBlocked(
            new Translation2d(8.0, 5.0), new Translation2d(2.0, 5.0), TEST_NET));
    // Shot well below at y=2.0 — clear
    assertFalse(
        FeedTargetSelector.isPathBlocked(
            new Translation2d(8.0, 2.0), new Translation2d(2.0, 2.0), TEST_NET));
    // Shot just inside Y range at y=3.52 — blocked
    assertTrue(
        FeedTargetSelector.isPathBlocked(
            new Translation2d(8.0, 3.52), new Translation2d(2.0, 3.52), TEST_NET));
    // Shot just outside Y range at y=3.48 — clear
    assertFalse(
        FeedTargetSelector.isPathBlocked(
            new Translation2d(8.0, 3.48), new Translation2d(2.0, 3.48), TEST_NET));
  }
}
