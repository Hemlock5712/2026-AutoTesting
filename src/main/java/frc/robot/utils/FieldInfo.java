package frc.robot.utils;

import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;

import frc.robot.utils.geometry.ExtTranslation;
import java.util.List;
import java.util.Optional;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rectangle2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.units.measure.Distance;
import org.wpilib.vision.apriltag.AprilTagFieldLayout;
import org.wpilib.vision.apriltag.AprilTagFields;

public final class FieldInfo {

  public static enum SymmetryType {
    MIRROR,
    ROTATE
  }

  private static AprilTagFieldLayout layout;
  private static SymmetryType symmetryType;

  // Cached to avoid Optional + autoboxing allocations every 50Hz cycle
  private static Boolean cachedShouldFlip = null;
  private static Translation2d cachedNetLineCenter = null;
  private static final double[] FLIP_JOYSTICK_RESULT = new double[2];

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

      DriverStationErrors.reportError(
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

      DriverStationErrors.reportError(
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
    cachedNetLineCenter = null;
  }

  public static Distance length() {
    return Meters.of(layout.getFieldLength());
  }

  /** Returns the field length in meters as a primitive double. Zero allocations. */
  public static double lengthMeters() {
    return layout.getFieldLength();
  }

  /** Returns the width (Y-axis) of the field in meters. */
  public static Distance width() {
    return Meters.of(layout.getFieldWidth());
  }

  /** Returns the field width in meters as a primitive double. Zero allocations. */
  public static double widthMeters() {
    return layout.getFieldWidth();
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
    return alliance.get() == Alliance.BLUE ? BLUE_ALLIANCE_TAGS : RED_ALLIANCE_TAGS;
  }

  // ==================== Field Positions (Blue Alliance Coordinates)
  // ====================
  // These are stored in blue alliance coordinates. Use flip() at call sites to
  // get
  // alliance-correct values.

  /** Hub/target position for turret tracking (blue alliance coordinates). */
  public static final Translation2d HUB_POSITION = new Translation2d(4.625594 + 0.152, 4.034);

  public static final Distance HUB_HEIGHT = Meters.of(1.828);

  public static final ExtTranslation RIGHT_FEED_POSITION = new ExtTranslation(1, 2);
  public static final ExtTranslation LEFT_FEED_POSITION = FieldFlip.overWidth(RIGHT_FEED_POSITION);

  public static final ExtTranslation RIGHT_FEED_POSITION_AUTO = new ExtTranslation(2.85, 2.01);
  public static final ExtTranslation LEFT_FEED_POSITION_AUTO =
      FieldFlip.overWidth(RIGHT_FEED_POSITION_AUTO);

  public static final ExtTranslation RIGHT_FAR_FEED_POSITION = new ExtTranslation(6, 0.5);
  public static final ExtTranslation LEFT_FAR_FEED_POSITION =
      FieldFlip.overWidth(RIGHT_FAR_FEED_POSITION);

  public static final Distance NET_INSET = Inches.of(58.0 + 31.0);

  public static Translation2d netLineCenter() {
    if (cachedNetLineCenter == null) {
      cachedNetLineCenter = new Translation2d(5.5, layout.getFieldWidth() / 2.0);
    }
    return cachedNetLineCenter;
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

  /** Clears the cached alliance flip so the next {@link #shouldFlip()} re-reads from DS. */
  public static void resetAllianceCache() {
    cachedShouldFlip = null;
  }

  /**
   * Returns true if coordinates should be flipped (red alliance). Cached after first determination.
   */
  public static boolean shouldFlip() {
    if (cachedShouldFlip != null) return cachedShouldFlip;
    Optional<Alliance> alliance = MatchState.getAlliance();
    if (alliance.isPresent()) {
      cachedShouldFlip = alliance.get() == Alliance.RED;
      return cachedShouldFlip;
    }
    return false;
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
      FLIP_JOYSTICK_RESULT[0] = -x;
      FLIP_JOYSTICK_RESULT[1] = -y;
    } else {
      FLIP_JOYSTICK_RESULT[0] = x;
      FLIP_JOYSTICK_RESULT[1] = y;
    }
    return FLIP_JOYSTICK_RESULT;
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
  private static final Rectangle2d OPPONENT_ZONE =
      new Rectangle2d(
          new Translation2d(FieldInfo.length().minus(Meters.of(4.5)), Meters.of(0)),
          new Translation2d(FieldInfo.length(), FieldInfo.width()));

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

  public static boolean isInOpponentZone(Translation2d translation) {
    return OPPONENT_ZONE.contains(flip(translation));
  }

  public static boolean isInOpponentZone(Pose2d pose) {
    return isInOpponentZone(pose.getTranslation());
  }

  public static boolean isUnderaTrench(Translation2d translation, double speedX) {
    boolean shouldFlip = shouldFlip();
    double flipX = shouldFlip ? layout.getFieldLength() - translation.getX() : translation.getX();
    double flipY =
        shouldFlip && symmetryType == SymmetryType.ROTATE
            ? layout.getFieldWidth() - translation.getY()
            : translation.getY();
    double flipSpeedX = shouldFlip ? -speedX : speedX;

    double trenchTolerance = 0.25;
    double trenchToleranceFarSide = 0.5;
    double speedMulti = 1.0;
    double trenchX = 4.625594;
    double trenchY = 1.27889;
    double fieldLength = layout.getFieldLength();
    double fieldWidth = layout.getFieldWidth();
    double otherTrenchX = fieldLength - trenchX;

    double allianceSide = Math.signum(Math.signum(trenchX - flipX) + Math.signum(flipSpeedX));

    double otherSide = Math.signum(Math.signum(otherTrenchX - flipX) + Math.signum(flipSpeedX));

    double allianceMinX = trenchX - trenchTolerance - flipSpeedX * speedMulti * allianceSide;
    double allianceMaxX = trenchX + trenchTolerance + flipSpeedX * speedMulti * allianceSide;
    double otherMinX = otherTrenchX - trenchToleranceFarSide - flipSpeedX * speedMulti * otherSide;
    double otherMaxX = otherTrenchX + trenchToleranceFarSide + flipSpeedX * speedMulti * otherSide;

    return containsBounds(flipX, flipY, allianceMinX, -999, allianceMaxX, trenchY)
        || containsBounds(
            flipX, flipY, allianceMinX, fieldWidth - trenchY, allianceMaxX, fieldWidth + 999)
        || containsBounds(flipX, flipY, otherMinX, -999, otherMaxX, trenchY)
        || containsBounds(
            flipX, flipY, otherMinX, fieldWidth - trenchY, otherMaxX, fieldWidth + 999);
  }

  private static boolean containsBounds(
      double px, double py, double x1, double y1, double x2, double y2) {
    double minX = Math.min(x1, x2);
    double maxX = Math.max(x1, x2);
    double minY = Math.min(y1, y2);
    double maxY = Math.max(y1, y2);

    return px >= minX && px <= maxX && py >= minY && py <= maxY;
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
