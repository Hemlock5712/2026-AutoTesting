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

  private static final double RIGHT_TRENCH_CENTER = 0.639445; // Center of right trench
  private static final double BUMPERS_ON_LINE =
      4.378; // Under trench, bumpers just barely on the line, starting X

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

  public Command pizzaAuto() {
    return Commands.sequence(
        autoCommands.rightAutoSetup(),
        // Drive through trench
        autoCommands
            .driveTo(
                () ->
                    FieldInfo.flip(
                        new Pose2d(5.965, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(0))))
            .withWaypoint(5)
            .withPositionTolerance(0.25),
        // Drive to midline, right of balls
        autoCommands
            .driveTo(() -> FieldInfo.flip(new Pose2d(8.652, 1.036, Rotation2d.fromDegrees(90))))
            .withPositionTolerance(0.25)
            .withWaypoint(0.5)
            .alongWith(intakeCoordinator.deployAndRun()),
        // Drive left through balls at midline, at a slight backwards angle
        autoCommands
            .driveTo(() -> FieldInfo.flip(new Pose2d(8.481, 2.766, Rotation2d.fromDegrees(110))))
            .withWaypoint(0.5)
            .withMaxSpeed(1)
            .withPositionTolerance(0.25),
        // Drive back to trench
        autoCommands
            .driveTo(
                () ->
                    FieldInfo.flip(
                        new Pose2d(5.959, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(-180))))
            .withWaypoint(0.2)
            .withMaxSpeed(3),
        // Drive under trench
        autoCommands
            .driveTo(
                () ->
                    FieldInfo.flip(
                        new Pose2d(4.378, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(180))))
            .withWaypoint(2)
            .withMaxSpeed(4),
        // Drive to outpost
        autoCommands
            .driveTo(
                () ->
                    FieldInfo.flip(
                        new Pose2d(0.814, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(-180))))
            .withEndTargetSpeed(0)
            .withMaxSpeed(1.25)
            .alongWith(superstructure.shoot())
            .alongWith(intakeCoordinator.runIntake())
            .alongWith(Commands.waitSeconds(10).andThen(intakeCoordinator.intakeUp())));
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
