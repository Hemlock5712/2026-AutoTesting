package frc.robot.utils;

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
import java.util.List;

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

  // ==================== Field Positions (Blue Alliance Coordinates) ====================
  // These are stored in blue alliance coordinates. Use flip() at call sites to get
  // alliance-correct values.

  /** Hub/target position for turret tracking (blue alliance coordinates). */
  public static final Translation2d HUB_POSITION = new Translation2d(4.621, 4.030);

  public static final Translation2d LEFT_FEED_POSITION = new Translation2d(1.5, 1.500);

  public static final Translation2d RIGHT_FEED_POSITION = new Translation2d(1.5, 6.500);

  /** X-axis threshold for alliance zone (meters from alliance wall). */
  public static final double ALLIANCE_ZONE_X_THRESHOLD = 4.625594;

  public static final Rectangle2d RED_ALLIANCE_ZONE =
      new Rectangle2d(
          new Translation2d(FieldInfo.length().baseUnitMagnitude(), 0),
          new Translation2d(
              FieldInfo.length().baseUnitMagnitude() - 4.625594,
              FieldInfo.width().baseUnitMagnitude()));

  public static final Rectangle2d BLUE_ALLIANCE_ZONE =
      new Rectangle2d(
          new Translation2d(0, 0),
          new Translation2d(4.625594, FieldInfo.width().baseUnitMagnitude()));

  /** Y-axis lock position on the right side (blue alliance coordinates). */
  public static final double AXIS_LOCK_Y_RIGHT = 0.639445;

  /** Y-axis lock position on the left side (blue alliance coordinates). */
  public static double axisLockYLeft() {
    return width().in(Meters) - 0.639445;
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

  public static Rectangle2d getAllianceZone() {
    return shouldFlip() ? RED_ALLIANCE_ZONE : BLUE_ALLIANCE_ZONE;
  }

  /** Returns true if the position is within the alliance zone based on X-axis threshold. */
  public static boolean isInAllianceZone(Translation2d position) {
    if (shouldFlip()) {
      // Red alliance: zone is on the far end of the field
      return position.getX() > length().baseUnitMagnitude() - ALLIANCE_ZONE_X_THRESHOLD;
    }
    // Blue alliance: zone is near X = 0
    return position.getX() < ALLIANCE_ZONE_X_THRESHOLD;
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
}
