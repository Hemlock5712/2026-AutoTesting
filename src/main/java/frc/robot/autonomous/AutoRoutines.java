package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Superstructure;
import frc.robot.utils.FieldInfo;

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
  public static Rotation2d snapToNearest90Degrees(Rotation2d rotation) {
    double degrees = rotation.getDegrees();
    double snapped = Math.round(degrees / 90.0) * 90.0;
    return Rotation2d.fromDegrees(snapped);
  }

  /**
   * Snaps a rotation angle to the nearest 90 degree increment.
   *
   * @param rotation The rotation to snap
   * @return The rotation snapped to the nearest 90 degrees (0, 90, 180, or 270)
   */
  public static Rotation2d snapToNearest180Degrees(Rotation2d rotation) {
    double degrees = rotation.getDegrees();
    double snapped = Math.round(degrees / 180.0) * 180.0;
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
        autoCommands.resetPose(() -> Pose2d.kZero),
        autoCommands.driveTo(() -> FieldInfo.flip(new Pose2d(3.0, 0, Rotation2d.kZero))),
        autoCommands.driveTo(() -> FieldInfo.flip(new Pose2d(3.0, 3.0, Rotation2d.kZero))),
        autoCommands.driveTo(() -> FieldInfo.flip(new Pose2d(0, 0, Rotation2d.kZero))));
  }

  // 33.25
  public Command AutoHumanPlayerSIMONLY() {
    return Commands.sequence(
        Commands.print("=== AutoHumanPlayerSIMONLY ==="),
        autoCommands.resetPose(
            () ->
                FieldInfo.flip(
                    new Pose2d(new Translation2d(4.400169, 0.639445), Rotation2d.kZero))),
        autoCommands
            .driveTo(() -> FieldInfo.flip(new Pose2d(6, 0.639445, Rotation2d.kZero)))
            .withWaypoint(4),
        autoCommands
            .driveTo(() -> FieldInfo.flip(new Pose2d(8, 3.0, Rotation2d.kCCW_90deg)))
            .withWaypoint(4),
        autoCommands
            .driveTo(() -> FieldInfo.flip(new Pose2d(6, 0.639445, Rotation2d.k180deg)))
            .withWaypoint(0.1, 0, 4),
        autoCommands.driveTo(() -> FieldInfo.flip(new Pose2d(2, 0.639445, Rotation2d.k180deg))),
        superstructure.beginShoot(),
        Commands.waitSeconds(5));
  }
}
