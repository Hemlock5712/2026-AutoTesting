package frc.robot.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.intake.IntakeCoordinator;
import frc.robot.utils.FieldInfo;

public class AutoRoutines {

  private final AutoCommands autoCommands;
  private final Superstructure superstructure;
  private final IntakeCoordinator intakeCoordinator;

  public AutoRoutines(
      AutoCommands autoCommands,
      Superstructure superstructure,
      IntakeCoordinator intakeCoordinator) {
    this.autoCommands = autoCommands;
    this.superstructure = superstructure;
    this.intakeCoordinator = intakeCoordinator;
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
   * Snaps a rotation angle to the nearest 180 degree increment.
   *
   * @param rotation The rotation to snap
   * @return The rotation snapped to the nearest 180 degrees (0 or 180)
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

  /** PathPlanner path version of AutoHumanPlayerSIMONLY. */
  public Command rightAuto() {
    PathPlannerPath rightToCenter = loadPath("right to center");
    PathPlannerPath centerToRight = loadPath("center to right");

    return Commands.sequence(
        Commands.print("=== Right Auto ==="),
        // autoCommands.resetPose(() -> FieldInfo.flip(new Pose2d(4.378, 0.639445,
        // Rotation2d.kZero))),
        Commands.parallel(
            intakeCoordinator.deployAndRun(),
            autoCommands
                .driveTo(() -> FieldInfo.flip(new Pose2d(4.378, 0.639445, Rotation2d.kZero)))
                .withWaypoint(2)),
        AutoBuilder.followPath(rightToCenter),
        AutoBuilder.followPath(centerToRight),
        autoCommands.driveTo(() -> FieldInfo.flip(new Pose2d(4.378, 0.639445, Rotation2d.kZero))),
        superstructure.shoot(),
        Commands.waitSeconds(5));
  }

  /** Loads a PathPlanner path file, converting checked exceptions to unchecked. */
  private static PathPlannerPath loadPath(String name) {
    try {
      return PathPlannerPath.fromPathFile(name);
    } catch (Exception e) {
      throw new RuntimeException("Failed to load path: " + name, e);
    }
  }
}
