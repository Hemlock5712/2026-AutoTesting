package frc.robot.autonomous;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.intake.IntakeCoordinator;
import frc.robot.utils.FieldFlip;
import frc.robot.utils.FieldInfo;
import frc.robot.utils.geometry.ExtPose;
import frc.robot.utils.path.Paths;
import frc.robot.utils.path.Paths.AlliancePath;
import java.util.List;

public class AutoRoutines {

  private static final double LEFT_TRENCH_CENTER = FieldInfo.width().in(Meters) - 0.639445;

  /** Offset ahead of a path's start for cleanup drive targets. */
  private static final double CLEANUP_OVERSHOOT_M = 0.25;

  private final AutoCommands autoCommands;
  private final Superstructure superstructure;
  private final IntakeCoordinator intakeCoordinator;

  // Pre-generated blue/red path pairs
  private final AlliancePath leftToMiddle = AlliancePath.of(Paths.LEFT_TO_MIDDLE);
  private final AlliancePath leftToMiddleCleanup = AlliancePath.of(Paths.LEFT_TO_MIDDLE_CLEANUP);
  private final AlliancePath rightToMiddle = AlliancePath.of(Paths.RIGHT_TO_MIDDLE);
  private final AlliancePath rightToMiddleCleanup = AlliancePath.of(Paths.RIGHT_TO_MIDDLE_CLEANUP);
  private final AlliancePath leftToMiddleFeed = AlliancePath.of(Paths.LEFT_TO_MIDDLE_FEED);
  private final AlliancePath feedToMiddleCleanup =
      AlliancePath.of(Paths.FEED_CLEANUP_BACK_TO_MIDDLE);

  public AutoRoutines(
      AutoCommands autoCommands,
      Superstructure superstructure,
      IntakeCoordinator intakeCoordinator) {
    this.autoCommands = autoCommands;
    this.superstructure = superstructure;
    this.intakeCoordinator = intakeCoordinator;

    // Precompute SplinePath + VelocityProfile for all path variants (both alliances)
    leftToMiddle.blue().precompute();
    leftToMiddle.red().precompute();
    leftToMiddleCleanup.blue().precompute();
    leftToMiddleCleanup.red().precompute();
    rightToMiddle.blue().precompute();
    rightToMiddle.red().precompute();
    rightToMiddleCleanup.blue().precompute();
    rightToMiddleCleanup.red().precompute();
    leftToMiddleFeed.blue().precompute();
    leftToMiddleFeed.red().precompute();
    feedToMiddleCleanup.blue().precompute();
    feedToMiddleCleanup.red().precompute();

    // Warm up JVM class loading by building a throwaway command chain.
    autoCommands.followPath(leftToMiddle.blue());
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

  // ==================== Auto Routines ====================

  public Command leftSide2Passes() {
    return Commands.sequence(
        resetPose(leftToMiddle),
        followAndIntake(leftToMiddle, 0.15),
        shootFor(2.5),
        superstructure.prerollShooter(30),
        driveToStart(leftToMiddleCleanup, 4.75),
        follow(leftToMiddleCleanup, 0.15),
        shootFor(3.5),
        driveToStart(leftToMiddleCleanup, 4.75),
        follow(leftToMiddleCleanup, 0.15));
  }

  public Command rightSide2Passes() {
    return Commands.sequence(
        resetPose(rightToMiddle),
        followAndIntake(rightToMiddle, 0.15),
        shootFor(2.5),
        superstructure.prerollShooter(30),
        driveToStart(rightToMiddleCleanup, 4.75),
        follow(rightToMiddleCleanup, 0.15),
        shootFor(3.5),
        driveToStart(rightToMiddleCleanup, 4.75),
        follow(rightToMiddleCleanup, 0.15));
  }

  public Command leftSideFeed2Passes() {
    ExtPose feedTarget =
        new ExtPose(FieldFlip.overWidth(new Pose2d(0.88, 0.75, Rotation2d.k180deg)));

    return Commands.sequence(
        resetPose(leftToMiddleFeed),
        autoCommands
            .followPathWithActions(
                leftToMiddleFeed.get(),
                List.of(
                    new AutoCommands.PathAction(1, 0.5, superstructure::feedShoot),
                    new AutoCommands.PathAction(4, 1.0, superstructure::stopShoot)))
            .deadlineFor(intakeCoordinator.deployAndRunAUTO(), superstructure.prerollShooter(30)),
        autoCommands.driveTo(feedTarget).withMaxSpeed(2).deadlineFor(superstructure.shoot()),
        Commands.waitSeconds(3).deadlineFor(superstructure.shoot()),
        superstructure.stopShoot(),
        superstructure.prerollShooter(30),
        driveToStart(feedToMiddleCleanup, 4.75),
        follow(feedToMiddleCleanup, 0.15),
        superstructure.shoot());
  }

  public Command leftAutoFeed(double midlineX) {
    return Commands.sequence(
        autoCommands.leftAutoSetup(),
        autoCommands
            .driveTo(
                () -> new ExtPose(midlineX, LEFT_TRENCH_CENTER, Rotation2d.fromDegrees(-90)).get())
            .withWaypoint(1)
            .withMaxSpeed(5)
            .deadlineFor(autoCommands.runWhenPastX(6.0, intakeCoordinator.deployAndRunAUTO())),
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
            superstructure.shoot()));
  }

  // ==================== Reusable Steps ====================

  /** Reset pose to the starting position of a path. */
  private Command resetPose(AlliancePath path) {
    return autoCommands.resetPose(() -> path.get().getStartingPose());
  }

  /** Follow a path while running intake and fixed-shooting at the endpoint. */
  private Command followAndIntake(AlliancePath path, double tolerance) {
    var data = path.get();
    return autoCommands
        .followPath(data)
        .withCompletionTolerance(tolerance)
        .deadlineFor(
            intakeCoordinator.deployAndRunAUTO(),
            superstructure.fixedShoot(() -> data.getTargetPose()));
  }

  /** Follow a path with a completion tolerance. */
  private Command follow(AlliancePath path, double tolerance) {
    return autoCommands.followPath(path.get()).withCompletionTolerance(tolerance);
  }

  /** Shoot for the given duration with turret tracking, then stop. */
  private Command shootFor(double seconds) {
    return new WaitCommand(seconds)
        .deadlineFor(superstructure.shoot(), superstructure.turretTrackHub())
        .andThen(superstructure.stopShoot());
  }

  /** Drive to a point slightly ahead of a path's start pose (cleanup approach point). */
  private Command driveToStart(AlliancePath path, double waypointSpeed) {
    Pose2d start = path.get().getStartingPose();
    Pose2d target =
        new Pose2d(
            start
                .getTranslation()
                .plus(new Translation2d(CLEANUP_OVERSHOOT_M, start.getRotation())),
            start.getRotation());
    return autoCommands.driveTo(() -> target).withWaypoint(waypointSpeed);
  }
}
