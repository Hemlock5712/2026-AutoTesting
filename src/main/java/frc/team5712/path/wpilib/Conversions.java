package frc.team5712.path.wpilib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.team5712.path.core.FieldFrame;
import frc.team5712.path.core.FieldSpeeds;
import frc.team5712.path.core.Pose;

/**
 * Between WPILib's types and the core's, and the alliance as a {@link FieldFrame}.
 *
 * <p><b>WPILib 2026 backport:</b> the core (and the editor) put the origin at the centre of the
 * field, as WPILib 2027 does; WPILib 2026 puts it at the blue alliance's corner. Every {@link
 * Pose2d} this package takes or gives is in the 2026 frame (the drivetrain's pose), and these
 * methods shift by half the field. {@link PathLibrary#loadAll()} sets the field size from
 * field.json; until then it is the 2026 field.
 */
public final class Conversions {
  private Conversions() {}

  private static double halfLength = PathLibrary.DEFAULT_FIELD_LENGTH / 2;
  private static double halfWidth = PathLibrary.DEFAULT_FIELD_WIDTH / 2;

  /** The field's size, for the shift between the 2026 corner origin and the core's centre one. */
  public static void setFieldSize(double lengthMeters, double widthMeters) {
    halfLength = lengthMeters / 2;
    halfWidth = widthMeters / 2;
  }

  /** A 2026-frame pose (origin at the blue corner) into the core's centre frame. */
  public static Pose toPose(Pose2d pose) {
    return new Pose(
        pose.getX() - halfLength, pose.getY() - halfWidth, pose.getRotation().getRadians());
  }

  /** A core (centre-frame) pose into the 2026 frame (origin at the blue corner). */
  public static Pose2d toPose2d(Pose pose) {
    return new Pose2d(
        pose.xMeters() + halfLength,
        pose.yMeters() + halfWidth,
        new Rotation2d(pose.headingRadians()));
  }

  /** A core pose as a Pose2d with no shift: the centre frame, as the editor and 2027 draw it. */
  public static Pose2d toCenterPose2d(Pose pose) {
    return new Pose2d(pose.xMeters(), pose.yMeters(), new Rotation2d(pose.headingRadians()));
  }

  /** Field-relative speeds in, field speeds out. */
  public static FieldSpeeds toFieldSpeeds(ChassisSpeeds fieldRelative) {
    return new FieldSpeeds(
        fieldRelative.vxMetersPerSecond,
        fieldRelative.vyMetersPerSecond,
        fieldRelative.omegaRadiansPerSecond);
  }

  /** Field speeds in, field-relative speeds out. */
  public static ChassisSpeeds toChassisSpeeds(FieldSpeeds speeds) {
    return new ChassisSpeeds(
        speeds.vxMetersPerSecond(), speeds.vyMetersPerSecond(), speeds.omegaRadPerSec());
  }

  /**
   * The frame for the alliance the driver station reports now: blue, or {@code redFrame} on red
   * (RED_HALF_TURN for rotationally symmetric fields, RED_MIRROR for mirrored ones). Unknown counts
   * as blue. Read it when the path starts, not when the auto is built.
   */
  public static FieldFrame allianceFrame(FieldFrame redFrame) {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
        ? redFrame
        : FieldFrame.BLUE;
  }
}
