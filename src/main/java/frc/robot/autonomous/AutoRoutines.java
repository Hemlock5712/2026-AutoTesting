package frc.robot.autonomous;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

  private static final double RIGHT_TRENCH_CENTER = 0.639445; // Center of right trench
  private static final double LEFT_TRENCH_CENTER = FieldInfo.width().in(Meters) - 0.639445;
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

  public Command rightAutoPass() {
    return Commands.sequence(
        autoCommands.rightAutoSetup(),
        autoCommands
            .driveTo(
                () -> new ExtPose(5.965, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(90)).get())
            .withWaypoint(5),
        // Drive to midline, right of balls (wide approach)
        intakeCoordinator.deployAndRunAUTO(),
        autoCommands
            .driveTo(() -> new ExtPose(8.1, 1.036, Rotation2d.fromDegrees(90)).get())
            .withWaypoint(0.5),

        // Drive left through balls at midline, at a slight backwards angle
        Commands.sequence(
                autoCommands
                    .driveTo(
                        () ->
                            new ExtPose(
                                    8.1,
                                    FieldInfo.width().in(Meters) / 2.0,
                                    Rotation2d.fromDegrees(90))
                                .get())
                    .withMaxSpeed(1),
                autoCommands
                    .driveTo(
                        () ->
                            new ExtPose(6.5, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(-180))
                                .get())
                    .withMaxSpeed(2))
            .deadlineFor(
                superstructure.passToLocation(new Translation2d(Inches.of(48), Inches.of(32)))),
        superstructure.stopShoot(),
        rightUnderTrench(),
        // Drive to outpost
        autoCommands
            .driveTo(
                () -> new ExtPose(0.914, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(-180)).get())
            .withMaxSpeed(2)
            .alongWith(superstructure.hubShoot())
            .alongWith(Commands.waitSeconds(10).andThen(intakeCoordinator.upAndRun())));
  }

  public Command rightAuto() {
    return Commands.sequence(
        autoCommands.rightAutoSetup(),
        autoCommands
            .driveTo(
                () -> new ExtPose(5.965, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(90)).get())
            .withWaypoint(5),
        // Drive to midline, right of balls (wide approach)
        autoCommands
            .driveTo(() -> new ExtPose(8.1, 1.036, Rotation2d.fromDegrees(90)).get())
            .withWaypoint(1.5)
            .alongWith(intakeCoordinator.deployAndRun()),
        // Drive left through balls at midline, at a slight backwards angle
        autoCommands
            .driveTo(() -> new ExtPose(8.1, 2.766, Rotation2d.fromDegrees(110)).get())
            .withWaypoint(1.5)
            .withMaxSpeed(1.5),
        rightReturnToTrench(true),
        rightUnderTrench(),
        // Drive to outpost
        autoCommands
            .driveTo(
                () -> new ExtPose(0.744, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(-180)).get())
            .withMaxSpeed(2)
            .alongWith(superstructure.hubShoot())
            .alongWith(Commands.waitSeconds(10).andThen(intakeCoordinator.upAndRun())));
  }

  public Command rightShortAuto() {
    return Commands.sequence(
        autoCommands.rightAutoSetup(),
        autoCommands
            .driveTo(
                () -> new ExtPose(5.965, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(90)).get())
            .withWaypoint(5),
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
                    .withMaxSpeed(2),
                new WaitCommand(3),
                autoCommands
                    .driveTo(
                        () ->
                            new ExtPose(3.5, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(180))
                                .get())
                    .withMaxSpeed(2)),
            superstructure.hubShoot()),
        superstructure.hubShoot());
  }

  public Command rightShortExtendedAuto() {
    return Commands.sequence(
        autoCommands.rightAutoSetup(),
        autoCommands
            .driveTo(
                () -> new ExtPose(5.965, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(90)).get())
            .withWaypoint(5),
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
                    .withMaxSpeed(2),
                new WaitCommand(2),
                autoCommands
                    .driveTo(
                        () ->
                            new ExtPose(3.5, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(180))
                                .get())
                    .withMaxSpeed(2)
                    .withEndTargetSpeed(2)),
            superstructure.hubShoot()),
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
        superstructure.hubShoot());
  }

  public Command leftShortSideAuto() {
    return leftAutoCore(8.481);
  }

  private Command leftAutoCore(double midlineX) {
    return Commands.sequence(
        autoCommands.leftAutoSetup(),
        // Drive through trench
        autoCommands
            .driveTo(
                () -> new ExtPose(5.965, LEFT_TRENCH_CENTER, Rotation2d.fromDegrees(-90)).get())
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
            .withMaxSpeed(2),
        // Drive back to trench
        autoCommands.driveTo(
            () -> new ExtPose(6.2, LEFT_TRENCH_CENTER, Rotation2d.fromDegrees(0)).get()),
        // Drive under trench
        autoCommands.driveTo(
            () -> new ExtPose(3.8, LEFT_TRENCH_CENTER, Rotation2d.fromDegrees(0)).get()),
        // Shoot for 3 seconds
        Commands.deadline(Commands.sequence(Commands.waitSeconds(3)), superstructure.hubShoot()),
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
            .withMaxSpeed(2),
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
            superstructure.hubShoot()));
  }

  public Command throughRightTrench(boolean centerToZone) {
    return Commands.either(
        Commands.sequence(

            // Drive back to trench
            autoCommands
                .driveTo(
                    () ->
                        new ExtPose(5.959, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(-180)).get())
                .withWaypoint(0.2)
                .withMaxSpeed(3)
                .deadlineFor(superstructure.stopShoot()),
            // Drive under trench
            autoCommands
                .driveTo(
                    () ->
                        new ExtPose(3.517, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(180)).get())
                .withWaypoint(2)
                .withMaxSpeed(4)),
        Commands.sequence(
            // Drive under trench
            autoCommands
                .driveTo(
                    () ->
                        new ExtPose(3.517, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(180)).get())
                .withWaypoint(2)
                .withMaxSpeed(4),
            // Drive back to trench
            autoCommands
                .driveTo(
                    () ->
                        new ExtPose(5.959, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(-180)).get())
                .withWaypoint(0.2)
                .withMaxSpeed(3)
                .deadlineFor(superstructure.stopShoot())),
        () -> (centerToZone));
  }

  public Command throughLeftTrench(boolean centerToZone) {
    return Commands.either(
        Commands.sequence(

            // Drive back to trench
            autoCommands
                .driveTo(
                    () ->
                        new ExtPose(5.959, LEFT_TRENCH_CENTER, Rotation2d.fromDegrees(-180)).get())
                .withWaypoint(0.2)
                .withMaxSpeed(3)
                .deadlineFor(superstructure.stopShoot()),
            // Drive under trench
            autoCommands
                .driveTo(
                    () -> new ExtPose(3.517, LEFT_TRENCH_CENTER, Rotation2d.fromDegrees(180)).get())
                .withWaypoint(2)
                .withMaxSpeed(4)),
        Commands.sequence(
            // Drive under trench
            autoCommands
                .driveTo(
                    () -> new ExtPose(3.517, LEFT_TRENCH_CENTER, Rotation2d.fromDegrees(180)).get())
                .withWaypoint(2)
                .withMaxSpeed(4),
            // Drive back to trench
            autoCommands
                .driveTo(
                    () ->
                        new ExtPose(5.959, LEFT_TRENCH_CENTER, Rotation2d.fromDegrees(-180)).get())
                .withWaypoint(0.2)
                .withMaxSpeed(3)
                .deadlineFor(superstructure.stopShoot())),
        () -> (centerToZone));
  }

  public Command leftAutoFeed(double midlineX) {
    return Commands.sequence(
        autoCommands.leftAutoSetup(),
        // Drive to midline, left of balls
        autoCommands
            .driveTo(
                () -> new ExtPose(midlineX, LEFT_TRENCH_CENTER, Rotation2d.fromDegrees(-90)).get())
            .withWaypoint(1)
            .withMaxSpeed(5),
        // Drive right through balls at midline, at a slight backwards angle
        intakeCoordinator.deployAndRunAUTO(),
        autoCommands
            .driveTo(
                () ->
                    new ExtPose(
                            midlineX,
                            FieldInfo.width().div(2).plus(Feet.of(2)).in(Meters),
                            Rotation2d.fromDegrees(-100))
                        .get())
            .withWaypoint(1.5)
            .withMaxSpeed(1.5),
        autoCommands
            .driveTo(
                () ->
                    new ExtPose(
                            midlineX - Feet.of(2).in(Meters),
                            FieldInfo.width().div(2).plus(Feet.of(2)).in(Meters),
                            Rotation2d.fromDegrees(-235))
                        .get())
            .withWaypoint(1.5)
            .withMaxSpeed(1.5),
        autoCommands
            .driveTo(
                () ->
                    new ExtPose(
                            midlineX - Feet.of(2).in(Meters),
                            LEFT_TRENCH_CENTER,
                            Rotation2d.fromDegrees(-180))
                        .get())
            .withMaxSpeed(2),
        autoCommands
            .driveTo(() -> new ExtPose(3.8, LEFT_TRENCH_CENTER, Rotation2d.fromDegrees(-180)).get())
            .withPositionTolerance(Inches.of(4))
            .withMaxSpeed(4)
            .withWaypoint(1)
            .deadlineFor(superstructure.spinUpShooter()),
        // Clean up and shoot
        Commands.deadline(
            Commands.sequence(
                autoCommands
                    .driveTo(new ExtPose(0.76, 7.0, Rotation2d.fromDegrees(-120)))
                    .withMaxSpeed(1.25)
                    .withWaypoint(0.75),
                autoCommands
                    .driveTo(new ExtPose(0.76, 5.284, Rotation2d.fromDegrees(-120)))
                    .withMaxSpeed(1)
                    .withWaypoint(0.75),
                autoCommands
                    .driveTo(new ExtPose(3.5, LEFT_TRENCH_CENTER, Rotation2d.fromDegrees(0)))
                    .withMaxSpeed(1)
                    .withWaypoint(1)),
            superstructure.shoot()),
        intakeCoordinator.slowUpAndRun());
  }
}
