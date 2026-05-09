package frc.robot.utils;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.List;
import java.util.Optional;

public final class FieldInfo {

  public static enum SymmetryType {
    MIRROR,
    ROTATE
  }

  private static AprilTagFieldLayout layout;
  private static SymmetryType symmetryType;

  // Cached to avoid Optional + autoboxing allocations every 50Hz cycle
  private static Boolean cachedShouldFlip = null;
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
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      cachedShouldFlip = alliance.get() == Alliance.Red;
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

  // ==================== 2026 Field Elements ====================
  // Adapted from FRC 6328 Mechanical-Advantage RobotCode2026Public/FieldConstants.java.
  // Blue-alliance perspective; pass through flip() for red.

  public static final double fuelDiameter = Units.inchesToMeters(5.91);

  public static Translation2d fieldCenter() {
    return new Translation2d(lengthMeters() / 2.0, widthMeters() / 2.0);
  }

  /** Central scoring structure with four faces and a 3D inner volume. */
  public static final class Hub {
    public static final double width = Units.inchesToMeters(47.0);
    public static final double height = Units.inchesToMeters(72.0);
    public static final double innerWidth = Units.inchesToMeters(41.7);
    public static final double innerHeight = Units.inchesToMeters(56.5);

    static final double centerX = aprilTags().getTagPose(26).get().getX() + width / 2.0;
    private static final double centerY = widthMeters() / 2.0;

    public static final Translation3d topCenterPoint = new Translation3d(centerX, centerY, height);
    public static final Translation3d innerCenterPoint =
        new Translation3d(centerX, centerY, innerHeight);

    public static final Translation2d nearLeftCorner =
        new Translation2d(centerX - width / 2.0, centerY + width / 2.0);
    public static final Translation2d nearRightCorner =
        new Translation2d(centerX - width / 2.0, centerY - width / 2.0);
    public static final Translation2d farLeftCorner =
        new Translation2d(centerX + width / 2.0, centerY + width / 2.0);
    public static final Translation2d farRightCorner =
        new Translation2d(centerX + width / 2.0, centerY - width / 2.0);

    public static final Pose2d nearFace = aprilTags().getTagPose(26).get().toPose2d();
    public static final Pose2d farFace = aprilTags().getTagPose(20).get().toPose2d();
    public static final Pose2d rightFace = aprilTags().getTagPose(18).get().toPose2d();
    public static final Pose2d leftFace = aprilTags().getTagPose(21).get().toPose2d();
  }

  /** Climb structure with three rung heights. */
  public static final class Tower {
    public static final double width = Units.inchesToMeters(49.25);
    public static final double depth = Units.inchesToMeters(45.0);
    public static final double height = Units.inchesToMeters(78.25);
    public static final double innerOpeningWidth = Units.inchesToMeters(32.250);
    public static final double frontFaceX = Units.inchesToMeters(43.51);
    public static final double uprightHeight = Units.inchesToMeters(72.1);

    public static final double lowRungHeight = Units.inchesToMeters(27.0);
    public static final double midRungHeight = Units.inchesToMeters(45.0);
    public static final double highRungHeight = Units.inchesToMeters(63.0);

    private static final double tagY = aprilTags().getTagPose(31).get().getY();

    public static final Translation2d centerPoint = new Translation2d(frontFaceX, tagY);
    public static final Translation2d leftUpright =
        new Translation2d(frontFaceX, tagY + innerOpeningWidth / 2.0 + Units.inchesToMeters(0.75));
    public static final Translation2d rightUpright =
        new Translation2d(frontFaceX, tagY - innerOpeningWidth / 2.0 - Units.inchesToMeters(0.75));
  }

  /** Floor obstacle on the left side of the hub. */
  public static final class LeftBump {
    public static final double width = Units.inchesToMeters(73.0);
    public static final double height = Units.inchesToMeters(6.513);
    public static final double depth = Units.inchesToMeters(44.4);

    public static final Translation2d nearLeftCorner =
        Hub.nearLeftCorner.plus(new Translation2d(0.0, width));
    public static final Translation2d nearRightCorner = Hub.nearLeftCorner;
    public static final Translation2d farLeftCorner =
        Hub.farLeftCorner.plus(new Translation2d(0.0, width));
    public static final Translation2d farRightCorner = Hub.farLeftCorner;
  }

  /** Floor obstacle on the right side of the hub. */
  public static final class RightBump {
    public static final double width = Units.inchesToMeters(73.0);
    public static final double height = Units.inchesToMeters(6.513);
    public static final double depth = Units.inchesToMeters(44.4);

    public static final Translation2d nearLeftCorner = Hub.nearRightCorner;
    public static final Translation2d nearRightCorner =
        Hub.nearRightCorner.minus(new Translation2d(0.0, width));
    public static final Translation2d farLeftCorner = Hub.farRightCorner;
    public static final Translation2d farRightCorner =
        Hub.farRightCorner.minus(new Translation2d(0.0, width));
  }

  /** Trench on the left side; opening points define the entrance plane. */
  public static final class LeftTrench {
    public static final double width = Units.inchesToMeters(65.65);
    public static final double depth = Units.inchesToMeters(47.0);
    public static final double height = Units.inchesToMeters(40.25);
    public static final double openingWidth = Units.inchesToMeters(50.34);
    public static final double openingHeight = Units.inchesToMeters(22.25);

    public static final Translation3d openingTopLeft =
        new Translation3d(Hub.centerX, widthMeters(), openingHeight);
    public static final Translation3d openingTopRight =
        new Translation3d(Hub.centerX, widthMeters() - openingWidth, openingHeight);
    public static final Translation2d center =
        openingTopLeft.toTranslation2d().interpolate(openingTopRight.toTranslation2d(), 0.5);
  }

  /** Trench on the right side; opening points define the entrance plane. */
  public static final class RightTrench {
    public static final double width = Units.inchesToMeters(65.65);
    public static final double depth = Units.inchesToMeters(47.0);
    public static final double height = Units.inchesToMeters(40.25);
    public static final double openingWidth = Units.inchesToMeters(50.34);
    public static final double openingHeight = Units.inchesToMeters(22.25);

    public static final Translation3d openingTopLeft =
        new Translation3d(Hub.centerX, openingWidth, openingHeight);
    public static final Translation3d openingTopRight =
        new Translation3d(Hub.centerX, 0.0, openingHeight);
    public static final Translation2d center =
        openingTopLeft.toTranslation2d().interpolate(openingTopRight.toTranslation2d(), 0.5);
  }

  /** Game-piece intake/storage zone behind the alliance wall. */
  public static final class Depot {
    public static final double width = Units.inchesToMeters(42.0);
    public static final double depth = Units.inchesToMeters(27.0);
    public static final double height = Units.inchesToMeters(1.125);
    public static final double distanceFromCenterY = Units.inchesToMeters(75.93);

    public static final Translation3d depotCenter =
        new Translation3d(depth, widthMeters() / 2.0 + distanceFromCenterY, height);
    public static final Translation3d leftCorner =
        new Translation3d(depth, widthMeters() / 2.0 + distanceFromCenterY + width / 2.0, height);
    public static final Translation3d rightCorner =
        new Translation3d(depth, widthMeters() / 2.0 + distanceFromCenterY - width / 2.0, height);
  }

  /** Alliance-wall feature with a low opening. */
  public static final class Outpost {
    public static final double width = Units.inchesToMeters(31.8);
    public static final double openingDistanceFromFloor = Units.inchesToMeters(28.1);
    public static final double height = Units.inchesToMeters(7.0);

    public static final Translation2d centerPoint =
        new Translation2d(0.0, aprilTags().getTagPose(29).get().getY());
  }

  /** Central neutral-zone region containing fuel game pieces. */
  public static final class FuelPool {
    public static final double width = Units.inchesToMeters(181.9);
    public static final double depth = Units.inchesToMeters(71.9);

    public static final Translation2d nearLeftCorner =
        new Translation2d(lengthMeters() / 2.0 - depth / 2.0, widthMeters() / 2.0 + width / 2.0);
    public static final Translation2d nearRightCorner =
        new Translation2d(lengthMeters() / 2.0 - depth / 2.0, widthMeters() / 2.0 - width / 2.0);
    public static final Translation2d leftCenter =
        new Translation2d(lengthMeters() / 2.0, widthMeters() / 2.0 + width / 2.0);
    public static final Translation2d rightCenter =
        new Translation2d(lengthMeters() / 2.0, widthMeters() / 2.0 - width / 2.0);
  }
}
