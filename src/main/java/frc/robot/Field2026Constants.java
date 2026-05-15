package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.FieldInfo;

/**
 * 2026 game-piece + field-element geometry, blue-alliance perspective. Pass through {@link
 * FieldInfo#flip} for red.
 *
 * <p>Adapted from FRC 6328's RobotCode2026Public. Move/delete this file when the season ends — it
 * is the only game-specific code in the project beyond {@link Field2026Obstacles}.
 */
public final class Field2026Constants {

  private Field2026Constants() {}

  /** Central scoring structure with four faces. */
  public static final class Hub {
    public static final double width = Units.inchesToMeters(47.0);
    public static final double height = Units.inchesToMeters(72.0);

    static final double centerX =
        FieldInfo.aprilTags().getTagPose(26).get().getX() + width / 2.0;
    private static final double centerY = FieldInfo.widthMeters() / 2.0;

    public static final Translation2d nearLeftCorner =
        new Translation2d(centerX - width / 2.0, centerY + width / 2.0);
    public static final Translation2d nearRightCorner =
        new Translation2d(centerX - width / 2.0, centerY - width / 2.0);
    public static final Translation2d farLeftCorner =
        new Translation2d(centerX + width / 2.0, centerY + width / 2.0);
    public static final Translation2d farRightCorner =
        new Translation2d(centerX + width / 2.0, centerY - width / 2.0);

    public static final Pose2d nearFace = FieldInfo.aprilTags().getTagPose(26).get().toPose2d();
    public static final Pose2d farFace = FieldInfo.aprilTags().getTagPose(20).get().toPose2d();
    public static final Pose2d rightFace = FieldInfo.aprilTags().getTagPose(18).get().toPose2d();
    public static final Pose2d leftFace = FieldInfo.aprilTags().getTagPose(21).get().toPose2d();
  }

  /** Climb structure with a central opening drivable from the front. */
  public static final class Tower {
    public static final double width = Units.inchesToMeters(49.25);
    public static final double depth = Units.inchesToMeters(45.0);
    public static final double innerOpeningWidth = Units.inchesToMeters(32.250);
    public static final double frontFaceX = Units.inchesToMeters(43.51);

    private static final double tagY = FieldInfo.aprilTags().getTagPose(31).get().getY();

    public static final Translation2d leftUpright =
        new Translation2d(frontFaceX, tagY + innerOpeningWidth / 2.0 + Units.inchesToMeters(0.75));
    public static final Translation2d rightUpright =
        new Translation2d(frontFaceX, tagY - innerOpeningWidth / 2.0 - Units.inchesToMeters(0.75));
  }

  /** Floor obstacle on the left side of the hub. */
  public static final class LeftBump {
    public static final double width = Units.inchesToMeters(73.0);

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

    public static final Translation2d nearLeftCorner = Hub.nearRightCorner;
    public static final Translation2d nearRightCorner =
        Hub.nearRightCorner.minus(new Translation2d(0.0, width));
    public static final Translation2d farLeftCorner = Hub.farRightCorner;
    public static final Translation2d farRightCorner =
        Hub.farRightCorner.minus(new Translation2d(0.0, width));
  }
}
