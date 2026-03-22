package frc.robot.autonomous;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.intake.IntakeCoordinator;
import frc.robot.utils.FieldInfo;
import frc.robot.utils.geometry.ExtPose;

public class AutoRoutines {

  private final AutoCommands autoCommands;
  private final Superstructure superstructure;
  private final IntakeCoordinator intakeCoordinator;

  public static final double RIGHT_TRENCH_CENTER = 0.639445; // Center of right trench
  public static final double LEFT_TRENCH_CENTER = FieldInfo.width().in(Meters) - 0.639445;
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
        autoCommands.driveTo(() -> new ExtPose(3.0, 0, Rotation2d.kZero).get()),
        autoCommands.driveTo(() -> new ExtPose(3.0, 3.0, Rotation2d.kZero).get()),
        autoCommands.driveTo(() -> new ExtPose(0, 0, Rotation2d.kZero).get()));
  }

  // ==================== Right-Side Shared Helpers ====================

  /** Setup + drive through trench to 5.965m (shared by all right autos). */
  private Command rightTrenchDrive() {
    return Commands.sequence(
        autoCommands.rightAutoSetup(),
        autoCommands
            .driveTo(() -> new ExtPose(5.965, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(0)).get())
            .withWaypoint(5));
  }

  /** Short midline approach at 7.6m (shared by rightShort and rightShortExtended). */
  private Command rightShortMidline() {
    return Commands.sequence(
        autoCommands
            .driveTo(() -> new ExtPose(7.6, 1.036, Rotation2d.fromDegrees(90)).get())
            .withWaypoint(0.5)
            .alongWith(intakeCoordinator.deployAndRun()),
        autoCommands
            .driveTo(() -> new ExtPose(7.6, 2.766, Rotation2d.fromDegrees(90)).get())
            .withWaypoint(0.5)
            .withMaxSpeed(1));
  }

  /** Drive under trench to 4.378m (shared by all right autos). */
  private Command rightUnderTrench() {
    return autoCommands
        .driveTo(() -> new ExtPose(4.378, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(180)).get())
        .withWaypoint(1.5);
  }

  /** Return to trench entrance at 5.959m. */
  private Command rightReturnToTrench(boolean waypointTolerance) {
    var cmd =
        autoCommands.driveTo(
            () -> new ExtPose(5.959, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(-180)).get());
    return waypointTolerance ? cmd.withWaypointTolerance() : cmd;
  }

  // ==================== Right-Side Auto Routines ====================

  public Command rightAuto() {
    return Commands.sequence(
        rightTrenchDrive(),
        // Drive to midline, right of balls (wide approach)
        autoCommands
            .driveTo(() -> new ExtPose(8.652, 1.036, Rotation2d.fromDegrees(90)).get())
            .withWaypoint(0.5)
            .alongWith(intakeCoordinator.deployAndRun()),
        // Drive left through balls at midline, at a slight backwards angle
        autoCommands
            .driveTo(() -> new ExtPose(8.481, 2.766, Rotation2d.fromDegrees(110)).get())
            .withWaypoint(0.5)
            .withMaxSpeed(1),
        rightReturnToTrench(true),
        rightUnderTrench(),
        // Drive to outpost
        autoCommands
            .driveTo(
                () -> new ExtPose(0.744, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(-180)).get())
            .withMaxSpeed(1.5)
            .alongWith(superstructure.shoot())
            .alongWith(Commands.waitSeconds(10).andThen(intakeCoordinator.upAndRun())));
  }

  public Command rightShortAuto() {
    return Commands.sequence(
        rightTrenchDrive(),
        rightShortMidline(),
        rightReturnToTrench(true),
        rightUnderTrench(),
        // Drive to outpost
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                autoCommands
                    .driveTo(
                        () ->
                            new ExtPose(0.744, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(-180))
                                .get())
                    .withMaxSpeed(1.5),
                new WaitCommand(3),
                autoCommands
                    .driveTo(
                        () ->
                            new ExtPose(3.5, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(180))
                                .get())
                    .withMaxSpeed(1.5)),
            superstructure.shoot()),
        superstructure.shoot());
  }

  public Command rightShortExtendedAuto() {
    return Commands.sequence(
        rightTrenchDrive(),
        rightShortMidline(),
        rightReturnToTrench(false),
        rightUnderTrench(),
        // Drive to outpost
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                autoCommands
                    .driveTo(
                        () ->
                            new ExtPose(0.744, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(180))
                                .get())
                    .withMaxSpeed(1.5),
                new WaitCommand(2),
                autoCommands
                    .driveTo(
                        () ->
                            new ExtPose(3.5, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(180))
                                .get())
                    .withMaxSpeed(1.5)
                    .withEndTargetSpeed(1.5)),
            superstructure.shoot()),
        superstructure.stopShoot(),
        // Extended: second pass
        autoCommands
            .driveTo(
                () -> new ExtPose(5.965, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(180)).get())
            .withWaypoint(5),
        autoCommands
            .driveTo(() -> new ExtPose(5.965, 4.0, Rotation2d.fromDegrees(90)).get())
            .withWaypoint(0)
            .withMaxSpeed(2),
        autoCommands.driveTo(
            () -> new ExtPose(5.959, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(-180)).get()),
        // Drive under trench
        autoCommands.driveTo(
            () -> new ExtPose(4.0, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(180)).get()),
        superstructure.shoot());
  }

  public Command pizzaAutoFeedBack() {
    return Commands.sequence(
        autoCommands.rightAutoSetup(),
        // Drive through trench
        autoCommands
            .driveTo(() -> new ExtPose(5.965, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(0)).get())
            .withWaypoint(5)
            .withWaypointTolerance(),
        // Drive to midline, right of balls
        autoCommands
            .driveTo(() -> new ExtPose(8.652, 1.036, Rotation2d.fromDegrees(90)).get())
            .withWaypointTolerance()
            .withWaypoint(0.5)
            .alongWith(intakeCoordinator.deployAndRun()),

        // Drive left through balls at midline, at a slight backwards angle
        autoCommands
            .driveTo(() -> new ExtPose(8.481, 3.6, Rotation2d.fromDegrees(110)).get())
            .withWaypoint(0.5)
            .withMaxSpeed(1)
            .withWaypointTolerance()
            .deadlineFor(superstructure.shoot()),

        // Drive back to trench
        autoCommands
            .driveTo(
                () -> new ExtPose(5.959, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(-180)).get())
            .withWaypoint(0.2)
            .withMaxSpeed(3)
            .deadlineFor(superstructure.stopShoot()),
        // Drive under trench
        autoCommands
            .driveTo(
                () -> new ExtPose(4.378, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(180)).get())
            .withWaypoint(2)
            .withMaxSpeed(4),
        Commands.sequence(
                // Drive to outpost
                autoCommands
                    .driveTo(
                        () ->
                            new ExtPose(0.814, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(180))
                                .get())
                    .withEndTargetSpeed(0)
                    .withMaxSpeed(1.75),
                // Give robot some time to get loaded from outpost
                Commands.waitSeconds(5),
                autoCommands
                    .driveTo(() -> new ExtPose(2.0, 1.5, Rotation2d.fromDegrees(180)).get())
                    .withWaypoint(0.5)
                    .withMaxSpeed(2),
                autoCommands
                    .driveTo(() -> new ExtPose(0.814, 1.5, Rotation2d.fromDegrees(180)).get())
                    .withEndTargetSpeed(0)
                    .withMaxSpeed(1.25))
            .alongWith(superstructure.shoot())
            .alongWith(intakeCoordinator.runIntake()));
  }

  public Command leftSideAuto() {
    return leftAutoCore(8.652);
  }

  public Command leftShortSideAuto() {
    return leftAutoCore(8.481);
  }

  private Command leftAutoCore(double midlineX) {
    return Commands.sequence(
        autoCommands.leftAutoSetup(),
        // Drive through trench
        autoCommands
            .driveTo(() -> new ExtPose(5.965, LEFT_TRENCH_CENTER, Rotation2d.fromDegrees(0)).get())
            .withWaypoint(5),
        // Drive to midline, left of balls
        autoCommands
            .driveTo(() -> new ExtPose(midlineX, 7.20, Rotation2d.fromDegrees(-90)).get())
            .withWaypoint(0.5)
            .alongWith(intakeCoordinator.deployAndRun()),
        // Drive right through balls at midline, at a slight backwards angle
        autoCommands
            .driveTo(() -> new ExtPose(8.481, 5.263, Rotation2d.fromDegrees(-110)).get())
            .withWaypoint(0.5)
            .withMaxSpeed(1),
        // Drive back to trench
        autoCommands.driveTo(
            () -> new ExtPose(6.2, LEFT_TRENCH_CENTER, Rotation2d.fromDegrees(0)).get()),
        // Drive under trench
        autoCommands.driveTo(
            () -> new ExtPose(3.8, LEFT_TRENCH_CENTER, Rotation2d.fromDegrees(0)).get()),
        // Shoot for 3 seconds
        Commands.deadline(Commands.sequence(Commands.waitSeconds(3)), superstructure.shoot()),
        superstructure.stopShoot(),
        intakeCoordinator.deployAndRun(),
        // Second pass: drive back through trench
        autoCommands
            .driveTo(() -> new ExtPose(5.965, LEFT_TRENCH_CENTER, Rotation2d.fromDegrees(0)).get())
            .withWaypoint(2),
        // Drive right through balls at midline, at a slight backwards angle
        autoCommands
            .driveTo(() -> new ExtPose(7.8, 4.0, Rotation2d.fromDegrees(-80)).get())
            .withWaypoint(0.5)
            .withMaxSpeed(1),
        // Drive back to trench
        autoCommands.driveTo(
            () -> new ExtPose(6.2, LEFT_TRENCH_CENTER, Rotation2d.fromDegrees(0)).get()),
        // Drive under trench
        autoCommands.driveTo(
            () -> new ExtPose(3.8, LEFT_TRENCH_CENTER, Rotation2d.fromDegrees(0)).get()),
        // Final shoot
        Commands.parallel(
            Commands.sequence(
                Commands.waitSeconds(3), intakeCoordinator.upAndRun(), Commands.waitSeconds(2)),
            superstructure.shoot()));
  }
}
