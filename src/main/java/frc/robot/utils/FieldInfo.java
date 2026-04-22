package frc.robot.utils;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.utils.geometry.ExtTranslation;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public final class FieldInfo {

  public static enum SymmetryType {
    MIRROR,
    ROTATE
  }

  private static AprilTagFieldLayout layout;
  private static SymmetryType symmetryType;

  static {
    setLayout(AprilTagFields.kDefaultField);
  }

  public static void setLayout(AprilTagFields field) {
    try {
      layout = AprilTagFieldLayout.loadField(field);
      symmetryType =
          switch (field) {
            case k2026RebuiltAndymark -> SymmetryType.ROTATE;
            case k2026RebuiltWelded -> SymmetryType.ROTATE;
            case k2025ReefscapeWelded -> SymmetryType.ROTATE;
            case k2025ReefscapeAndyMark -> SymmetryType.ROTATE;
            case k2024Crescendo -> SymmetryType.MIRROR;
            case k2023ChargedUp -> SymmetryType.MIRROR;
            case k2022RapidReact -> SymmetryType.ROTATE;
          };
    } catch (Exception e) {
      layout = new AprilTagFieldLayout(List.of(), 0.0, 0.0);
      symmetryType = SymmetryType.MIRROR;

      DriverStation.reportError(
          "Failed to load layout from field \"" + field.name() + "\": " + e.getMessage(), true);
    }
  }

  public static void setLayout(String resourcePath, SymmetryType symmetryType) {
    try {
      layout = AprilTagFieldLayout.loadFromResource(resourcePath);
      FieldInfo.symmetryType = symmetryType;
    } catch (Exception e) {
      layout = new AprilTagFieldLayout(List.of(), 0.0, 0.0);
      symmetryType = SymmetryType.MIRROR;

      DriverStation.reportError(
          "[AprilTags] Unable to load layout from resource \""
              + resourcePath
              + "\": "
              + e.getMessage(),
          true);
    }
  }

  public static void setLayout(AprilTagFieldLayout layout, SymmetryType symmetryType) {
    FieldInfo.layout = layout;
    FieldInfo.symmetryType = symmetryType;
  }

  public static Distance length() {
    return Meters.of(layout.getFieldLength());
  }

  /** Returns the width (Y-axis) of the field in meters. */
  public static Distance width() {
    return Meters.of(layout.getFieldWidth());
  }

  /** Returns the direction in which the field is symmetric. */
  public static SymmetryType symmetryType() {
    return symmetryType;
  }

  /** Returns the current field's AprilTag layout. */
  public static AprilTagFieldLayout aprilTags() {
    return layout;
  }

  // ==================== Alliance Tag Filtering ====================

  private static final int[] BLUE_ALLIANCE_TAGS = {27, 18, 25, 26, 21, 24, 19, 20};
  private static final int[] RED_ALLIANCE_TAGS = {3, 4, 9, 5, 8, 10, 11, 2};

  /**
   * Returns AprilTag IDs trusted when on the given alliance. Returns an empty array (no filter /
   * accept all) if alliance is unknown.
   */
  public static int[] getAllianceTags(Optional<Alliance> alliance) {
    if (alliance.isEmpty()) {
      return new int[] {};
    }
    return alliance.get() == Alliance.Blue ? BLUE_ALLIANCE_TAGS : RED_ALLIANCE_TAGS;
  }

  // ==================== Field Positions (Blue Alliance Coordinates)
  // ====================
  // These are stored in blue alliance coordinates. Use flip() at call sites to
  // get
  // alliance-correct values.

  /** Hub/target position for turret tracking (blue alliance coordinates). */
  public static final Translation2d HUB_POSITION = new Translation2d(4.625594 + 0.152, 4.034);

  public static final Distance HUB_HEIGHT = Meters.of(1.828);

  public static final ExtTranslation RIGHT_FEED_POSITION = new ExtTranslation(1.5, 1.625);
  public static final ExtTranslation LEFT_FEED_POSITION = FieldFlip.overWidth(RIGHT_FEED_POSITION);

  public static final ExtTranslation RIGHT_FEED_POSITION_AUTO = new ExtTranslation(1.5, 2.75);
  public static final ExtTranslation LEFT_FEED_POSITION_AUTO =
      FieldFlip.overWidth(RIGHT_FEED_POSITION_AUTO);

  public static final Distance NET_INSET = Inches.of(58.0 + 31.0);

  public static Translation2d netLineCenter() {
    double w = layout.getFieldWidth();
    return new Translation2d(5.5, w / 2.0);
  }

  public static final double ALLIANCE_ZONE_X = 5.4;

  /** Y-axis lock position on the right side (blue alliance coordinates). */
  public static final double AXIS_LOCK_Y_RIGHT = 0.639445;

  public static final double AXIS_LOCK_Y_TRENCH_RIGHT = 2.6924;

  /** Y-axis lock position on the left side (blue alliance coordinates). */
  public static double axisLockYLeft() {
    return width().in(Meters) - AXIS_LOCK_Y_RIGHT;
  }

  public static double axisLockYTrenchLeft() {
    return width().in(Meters) - AXIS_LOCK_Y_TRENCH_RIGHT;
  }

  /** Rotation preset: facing toward opponent alliance wall (blue alliance coordinates). */
  public static final Rotation2d FACING_FORWARD = Rotation2d.kZero;

  /** Rotation preset: facing left (relative to blue alliance driver station). */
  public static final Rotation2d FACING_LEFT = Rotation2d.fromDegrees(90);

  /** Rotation preset: facing right (relative to blue alliance driver station). */
  public static final Rotation2d FACING_RIGHT = Rotation2d.fromDegrees(-90);

  /** Rotation preset: facing toward own alliance wall (blue alliance coordinates). */
  public static final Rotation2d FACING_BACK = Rotation2d.k180deg;

  // ==================== Flip Utilities ====================

  /** Returns true if coordinates should be flipped (red alliance). */
  public static boolean shouldFlip() {
    return DriverStation.getAlliance().map(alliance -> alliance == Alliance.Red).orElse(false);
  }

  /**
   * Flips joystick X/Y for BlueAlliance perspective driving. Negates both axes on red alliance so
   * "forward" = positive field X.
   *
   * @param x Joystick X value (forward/back)
   * @param y Joystick Y value (left/right)
   * @return Flipped [x, y] array
   */
  public static double[] flipJoystick(double x, double y) {
    if (shouldFlip()) {
      return new double[] {-x, -y};
    }
    return new double[] {x, y};
  }

  /** Flips rotation input for red alliance. */
  public static double flipJoystickRotation(double omega) {
    return shouldFlip() ? -omega : omega;
  }

  /** Flips a Pose2d based on alliance and symmetry type. */
  public static Pose2d flip(Pose2d pose) {
    if (shouldFlip()) {
      return switch (symmetryType) {
        case MIRROR ->
            new Pose2d(
                layout.getFieldLength() - pose.getX(),
                pose.getY(),
                new Rotation2d(Math.PI - pose.getRotation().getRadians()));
        case ROTATE ->
            new Pose2d(
                layout.getFieldLength() - pose.getX(),
                layout.getFieldWidth() - pose.getY(),
                pose.getRotation().rotateBy(Rotation2d.k180deg));
      };
    }
    return pose;
  }

  /** Flips a Translation2d based on alliance and symmetry type. */
  public static Translation2d flip(Translation2d translation) {
    if (shouldFlip()) {
      return switch (symmetryType) {
        case MIRROR ->
            new Translation2d(layout.getFieldLength() - translation.getX(), translation.getY());
        case ROTATE ->
            new Translation2d(
                layout.getFieldLength() - translation.getX(),
                layout.getFieldWidth() - translation.getY());
      };
    }
    return translation;
  }

  /** Flips a Rotation2d based on alliance and symmetry type. */
  public static Rotation2d flip(Rotation2d rotation) {
    if (shouldFlip()) {
      return switch (symmetryType) {
        case MIRROR -> new Rotation2d(Math.PI - rotation.getRadians());
        case ROTATE -> rotation.rotateBy(Rotation2d.k180deg);
      };
    }
    return rotation;
  }

  /** Flips an X coordinate based on alliance. Always flipped for both MIRROR and ROTATE. */
  public static double flipX(double x) {
    if (shouldFlip()) {
      return layout.getFieldLength() - x;
    }
    return x;
  }

  public static double flipX(Distance x) {
    return flipX(x.in(Meters));
  }

  /** Flips a Y coordinate based on alliance. Only flipped for ROTATE symmetry. */
  public static double flipY(double y) {
    if (shouldFlip() && symmetryType == SymmetryType.ROTATE) {
      return layout.getFieldWidth() - y;
    }
    return y;
  }

  public static double flipY(Distance y) {
    return flipY(y.in(Meters));
  }

  private static final Translation2d CENTER_OF_FIELD =
      new Translation2d(FieldInfo.length().div(2), FieldInfo.width().div(2));

  private static final Rectangle2d NEUTRAL_ZONE =
      new Rectangle2d(
          new Pose2d(CENTER_OF_FIELD, Rotation2d.kZero),
          FieldInfo.length().div(2).minus(Meters.of(5.304)).times(2),
          FieldInfo.width());
  private static final Rectangle2d ALLIANCE_ZONE =
      new Rectangle2d(
          new Translation2d(0, 0), new Translation2d(Meters.of(4.5), FieldInfo.width()));

  private static final Distance NEUTRAL_ZONE_DEADZONE_DEPTH = Meters.of(3);
  private static final Distance NEUTRAL_ZONE_DEADZONE_WIDTH = Meters.of(1.25);

  private static final Rectangle2d NEUTRAL_ZONE_DEADZONE =
      new Rectangle2d(
          new Pose2d(
              CENTER_OF_FIELD.minus(
                  new Translation2d(
                      NEUTRAL_ZONE_DEADZONE_DEPTH.div(2), NEUTRAL_ZONE_DEADZONE_WIDTH.div(2))),
              Rotation2d.kZero),
          NEUTRAL_ZONE_DEADZONE_DEPTH,
          NEUTRAL_ZONE_DEADZONE_WIDTH);

  private static final Distance TOWER_WIDTH = Meters.of(1.1);
  private static final Distance TOWER_DEPTH = Meters.of(1.1);
  private static final Translation2d TOWER_POSITION =
      new Translation2d(TOWER_DEPTH.div(2), width().div(2));
  private static final Rectangle2d TOWER_ZONE =
      new Rectangle2d(new Pose2d(TOWER_POSITION, Rotation2d.kZero), TOWER_WIDTH, TOWER_DEPTH);

  public static boolean isInNeutralZone(Translation2d translation) {
    return NEUTRAL_ZONE.contains(translation);
  }

  public static boolean isInNeutralZone(Pose2d pose) {
    return isInNeutralZone(pose.getTranslation());
  }

  public static boolean isInAllianceZone(Translation2d translation) {
    return ALLIANCE_ZONE.contains(flip(translation));
  }

  public static boolean isInAllianceZone(Pose2d pose) {
    return isInAllianceZone(pose.getTranslation());
  }

  public static boolean isUnderaTrench(Translation2d translation, double speedX) {
    Translation2d flippedTranslation = flip(translation);

    double TRENCHTOLERANCE = 0.25;

    double speedMulti = 0.3;

    double allianceSide =
        Math.signum(Math.signum(4.625594 - translation.getX()) + Math.signum(speedX));

    double otherSide =
        Math.signum(
            Math.signum(FieldInfo.length().in(Meters) - 4.625594 - translation.getX())
                + Math.signum(speedX));

    Rectangle2d LEFTALLIANCETRENCHZONE =
        new Rectangle2d(
            new Translation2d(
                4.625594 - TRENCHTOLERANCE - speedX * speedMulti * allianceSide, 1.27889),
            new Translation2d(
                4.625594 + TRENCHTOLERANCE + speedX * speedMulti * allianceSide, -999));

    Rectangle2d RIGHTALLIANCETRENCHZONE =
        new Rectangle2d(
            new Translation2d(
                4.625594 - TRENCHTOLERANCE - speedX * speedMulti * allianceSide,
                FieldInfo.width().in(Meters) - 1.27889),
            new Translation2d(
                4.625594 + TRENCHTOLERANCE + speedX * speedMulti * allianceSide,
                FieldInfo.width().in(Meters) + 999));

    Rectangle2d LEFTOTHERTRENCHZONE =
        new Rectangle2d(
            new Translation2d(
                FieldInfo.length().in(Meters)
                    - 4.625594
                    - TRENCHTOLERANCE
                    - speedX * speedMulti * otherSide,
                1.27889),
            new Translation2d(
                FieldInfo.length().in(Meters)
                    - 4.625594
                    + TRENCHTOLERANCE
                    + speedX * speedMulti * otherSide,
                0 - 999));

    Rectangle2d RIGHTOTHERTRENCHZONE =
        new Rectangle2d(
            new Translation2d(
                FieldInfo.length().in(Meters)
                    - 4.625594
                    - TRENCHTOLERANCE
                    - speedX * speedMulti * otherSide,
                FieldInfo.width().in(Meters) - 1.27889),
            new Translation2d(
                FieldInfo.length().in(Meters)
                    - 4.625594
                    + TRENCHTOLERANCE
                    + speedX * speedMulti * otherSide,
                FieldInfo.width().in(Meters) + 999));

    logRectangle(LEFTALLIANCETRENCHZONE, "LEFTALLIANCETRENCHZONE");
    logRectangle(RIGHTALLIANCETRENCHZONE, "RIGHTALLIANCETRENCHZONE");
    logRectangle(LEFTOTHERTRENCHZONE, "LEFTOTHERTRENCHZONE");
    logRectangle(RIGHTOTHERTRENCHZONE, "RIGHTOTHERTRENCHZONE");

    return LEFTALLIANCETRENCHZONE.contains(flippedTranslation)
        || RIGHTALLIANCETRENCHZONE.contains(flippedTranslation)
        || LEFTOTHERTRENCHZONE.contains(flippedTranslation)
        || RIGHTOTHERTRENCHZONE.contains(flippedTranslation);
  }

  public static void logRectangle(Rectangle2d rectangle, String name) {
    Pose2d topLeft =
        new Pose2d(
            new Translation2d(
                rectangle.getCenter().getX() - (rectangle.getXWidth() / 2),
                rectangle.getCenter().getY() + rectangle.getYWidth() / 2),
            new Rotation2d(0));

    Pose2d topRight =
        new Pose2d(
            new Translation2d(
                rectangle.getCenter().getX() + (rectangle.getXWidth() / 2),
                rectangle.getCenter().getY() + rectangle.getYWidth() / 2),
            new Rotation2d(0));

    Pose2d bottomLeft =
        new Pose2d(
            new Translation2d(
                rectangle.getCenter().getX() - (rectangle.getXWidth() / 2),
                rectangle.getCenter().getY() - rectangle.getYWidth() / 2),
            new Rotation2d(0));
    Pose2d bottomRight =
        new Pose2d(
            new Translation2d(
                rectangle.getCenter().getX() + (rectangle.getXWidth() / 2),
                rectangle.getCenter().getY() - rectangle.getYWidth() / 2),
            new Rotation2d(0));
    Pose2d[] point = {topLeft, topRight, bottomRight, bottomLeft, topLeft};
    Logger.recordOutput("Trench/" + name, point);
  }

  public static boolean isInNeutralZoneDeadzone(Translation2d translation) {
    return NEUTRAL_ZONE_DEADZONE.contains(translation);
  }

  public static boolean isInNeutralZoneDeadzone(Pose2d pose) {
    return isInNeutralZoneDeadzone(pose.getTranslation());
  }

  public static boolean isUnderTower(Translation2d translation) {
    return TOWER_ZONE.contains(flip(translation));
  }

  public static boolean isUnderTower(Pose2d pose) {
    return isUnderTower(pose.getTranslation());
  }
}
