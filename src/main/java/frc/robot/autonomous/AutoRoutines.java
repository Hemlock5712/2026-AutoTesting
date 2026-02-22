package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.utils.FieldInfo;
import java.util.function.Supplier;

public class AutoRoutines {

  private final AutoCommands autoCommands;
  private final Superstructure superstructure;

  public AutoRoutines(AutoCommands autoCommands, Superstructure superstructure) {
    this.autoCommands = autoCommands;
    this.superstructure = superstructure;
  }

  /**
   * Snaps a rotation angle to the nearest 90 degree increment.
   *
   * @param rotation The rotation to snap
   * @return The rotation snapped to the nearest 90 degrees (0, 90, 180, or 270)
   */
  private static Rotation2d snapToNearest90Degrees(Rotation2d rotation) {
    double degrees = rotation.getDegrees();
    double snapped = Math.round(degrees / 90.0) * 90.0;
    return Rotation2d.fromDegrees(snapped);
  }

  /**
   * Example autonomous using WAITING commands for sequential operations.
   *
   * <p>Demonstrates using AndWait variants to ensure mechanisms are ready before continuing. Good
   * for when you need precise timing and guaranteed completion.
   */
  public Command sequentialScoringAuto() {
    return Commands.sequence(
        Commands.print("=== Sequential Scoring Auto ==="),
        autoCommands.resetPose(Pose2d.kZero),
        autoCommands.driveTo(() -> FieldInfo.flip(new Pose2d(3.0, 0, Rotation2d.kZero))),
        autoCommands.driveTo(() -> FieldInfo.flip(new Pose2d(3.0, 3.0, Rotation2d.kZero))),
        autoCommands.driveTo(() -> FieldInfo.flip(new Pose2d(0, 0, Rotation2d.kZero))));
  }

  public Command DrivePointInRight(Supplier<Rotation2d> rotation) {
    return Commands.sequence(
        autoCommands.driveTo(
            1,
            () -> {
              Translation2d pos =
                  FieldInfo.flip(
                      new Translation2d(
                          FieldConstants.DRIVE_POINT_IN_X_START,
                          FieldConstants.DRIVE_POINT_IN_Y_RIGHT));
              return new Pose2d(pos, snapToNearest90Degrees(rotation.get()));
            }),
        autoCommands.driveTo(
            () -> {
              Translation2d pos =
                  FieldInfo.flip(
                      new Translation2d(
                          FieldConstants.DRIVE_POINT_IN_X_END,
                          FieldConstants.DRIVE_POINT_IN_Y_RIGHT));
              return new Pose2d(pos, snapToNearest90Degrees(rotation.get()));
            }));
  }

  public Command DrivePointInLeft(Supplier<Rotation2d> rotation) {
    return Commands.sequence(
        autoCommands.driveTo(
            () -> {
              Translation2d pos =
                  FieldInfo.flip(
                      new Translation2d(
                          FieldConstants.DRIVE_POINT_IN_X_START,
                          FieldConstants.DRIVE_POINT_IN_Y_LEFT));
              return new Pose2d(pos, snapToNearest90Degrees(rotation.get()));
            }),
        autoCommands.driveTo(
            () -> {
              Translation2d pos =
                  FieldInfo.flip(
                      new Translation2d(
                          FieldConstants.DRIVE_POINT_IN_X_END,
                          FieldConstants.DRIVE_POINT_IN_Y_LEFT));
              return new Pose2d(pos, snapToNearest90Degrees(rotation.get()));
            }));
  }
}
