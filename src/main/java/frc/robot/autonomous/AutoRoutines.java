package frc.robot.autonomous;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveToPoint;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.intake.IntakeCoordinator;
import frc.robot.utils.FieldFlip;
import frc.robot.utils.FieldInfo;
import frc.robot.utils.geometry.ExtPose;
import frc.robot.utils.path.PathData;
import frc.robot.utils.path.Paths;
import java.util.List;
import java.util.function.Supplier;

public class AutoRoutines {

  private static final double LEFT_TRENCH_CENTER = FieldInfo.width().in(Meters) - 0.639445;

  private final AutoCommands autoCommands;
  private final AutoRoutineFactory autoFactory;
  private final Superstructure superstructure;
  private final IntakeCoordinator intakeCoordinator;

  public AutoRoutines(
      AutoCommands autoCommands,
      Superstructure superstructure,
      IntakeCoordinator intakeCoordinator) {
    this.autoCommands = autoCommands;
    this.autoFactory = new AutoRoutineFactory(autoCommands);
    this.superstructure = superstructure;
    this.intakeCoordinator = intakeCoordinator;

    // Warm up JVM class loading by building a throwaway command chain.
    // Forces all command framework classes to load during robot init, not first
    // auto.
    autoCommands.followPath(Paths.LEFT_TO_MIDDLE);
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

  public Command leftSide2Passes() {
    return side2Passes(Paths.LEFT_TO_MIDDLE, Paths.LEFT_TO_MIDDLE_CLEANUP);
  }

  public Command rightSide2Passes() {
    return side2Passes(Paths.RIGHT_TO_MIDDLE, Paths.RIGHT_TO_MIDDLE_CLEANUP);
  }

  public Command rightSide2PassesShallow() {
    return side2Passes(Paths.RIGHT_TO_MIDDLE, Paths.RIGHT_TO_MIDDLE_CLEANUP_SHALLOW);
  }

  public Command antiPoofLeft() {
    return side2Passes(Paths.SHORT_PATH_LEFT, Paths.LEFT_TO_MIDDLE_CLEANUP);
  }

  public Command antiPoofRight() {
    return side2Passes(Paths.SHORT_PATH_RIGHT, Paths.RIGHT_TO_MIDDLE_CLEANUP);
  }

  public Command leftSideFeed2Passes() {
    return sideFeed2Passes(
        Paths.LEFT_TO_MIDDLE_FEED,
        new Pose2d(0.88, 0.75, Rotation2d.k180deg),
        Paths.FEED_CLEANUP_BACK_TO_MIDDLE);
  }

  private Command side2Passes(PathData mainPath, PathData cleanupPath) {
    return buildSide2Passes(mainPath, cleanupPath);
  }

  private Command sideFeed2Passes(PathData mainPath, Pose2d redTargetPose, PathData cleanupPath) {
    return buildFeedSide2Passes(mainPath, feedTargetPose(redTargetPose), cleanupPath);
  }

  private Command buildSide2Passes(PathData mainPath, PathData cleanupPath) {
    Supplier<Pose2d> cleanupDriveTarget = cleanupDriveTarget(cleanupPath);

    return Commands.sequence(
        autoFactory.resetPoseToStart(mainPath),
        autoFactory
            .path(mainPath, command -> command.withCompletionTolerance(0.15))
            .withTimeout(8)
            .deadlineFor(
                intakeCoordinator.deployAndRunAUTO(),
                superstructure.fixedShoot(() -> Paths.forAlliance(mainPath).getTargetPose())),
        // new WaitCommand(0.5).deadlineFor(superstructure.recoverHopper()),
        new WaitCommand(4).deadlineFor(superstructure.shoot(), superstructure.turretTrackHub()),
        superstructure.stopShoot(),
        superstructure.prerollShooter(30),
        autoCommands.driveTo(cleanupDriveTarget).withWaypoint(4.75),
        autoFactory
            .path(cleanupPath, command -> command.withCompletionTolerance(0.15))
            .withTimeout(8),
        // new WaitCommand(0.5).deadlineFor(superstructure.recoverHopper()),
        new WaitCommand(4).deadlineFor(superstructure.shoot(), superstructure.turretTrackHub()),
        superstructure.stopShoot(),
        autoCommands.driveTo(cleanupDriveTarget).withWaypoint(4.75),
        autoFactory.path(cleanupPath, command -> command.withCompletionTolerance(0.15)));
  }

  private Command buildFeedSide2Passes(
      PathData mainPath, Supplier<Pose2d> feedTargetPose, PathData cleanupPath) {
    Supplier<Pose2d> cleanupDriveTarget = cleanupDriveTarget(cleanupPath);

    return Commands.sequence(
        autoFactory.resetPoseToStart(mainPath),
        autoFactory
            .pathWithActions(
                mainPath,
                List.of(
                    new AutoCommands.PathAction(1, 0.5, superstructure::feedShoot),
                    new AutoCommands.PathAction(4, 1.0, superstructure::stopShoot)))
            .deadlineFor(intakeCoordinator.deployAndRunAUTO(), superstructure.prerollShooter(30)),
        autoCommands.driveTo(feedTargetPose).withMaxSpeed(2).deadlineFor(superstructure.shoot()),
        Commands.waitSeconds(3).deadlineFor(superstructure.shoot()),
        superstructure.stopShoot(),
        superstructure.prerollShooter(30),
        autoCommands.driveTo(cleanupDriveTarget).withWaypoint(4.75),
        autoFactory.path(cleanupPath, command -> command.withCompletionTolerance(0.15)),
        superstructure.shoot());
  }

  public Command leftAutoFeed(double midlineX) {
    return Commands.sequence(
        autoFactory.resetPoseToStart(Paths.LEFT_TO_MIDDLE_TO_DEPOT),
        // Drive to midline, left of balls
        autoFactory
            .path(Paths.LEFT_TO_MIDDLE_TO_DEPOT, command -> command.withCompletionTolerance(0.25))
            .withTimeout(6.7)
            .deadlineFor(
                intakeCoordinator.deployAndRunAUTO(),
                superstructure.fixedShoot(
                    () -> Paths.forAlliance(Paths.LEFT_TO_MIDDLE_TO_DEPOT).getTargetPose()))
            .withTimeout(7.2),
        Commands.waitSeconds(3)
            .deadlineFor(superstructure.turretTrackHub().alongWith(superstructure.shoot())),
        superstructure.stopShoot(),
        // Clean up and shoot
        Commands.deadline(
                Commands.sequence(
                    autoCommands
                        .driveTo(new ExtPose(0.76, 7.0, Rotation2d.fromDegrees(-120)))
                        .withMaxSpeed(2.5)
                        .withWaypoint(0.5),
                    autoCommands
                        .driveTo(new ExtPose(0.76, 5.284, Rotation2d.fromDegrees(-120)))
                        .withMaxSpeed(1)
                        .withWaypoint(0.75),
                    autoCommands
                        .driveTo(new ExtPose(2.5, LEFT_TRENCH_CENTER, Rotation2d.fromDegrees(0)))
                        .withMaxSpeed(2.5)
                        .withWaypoint(1)))
            .deadlineFor(
                superstructure.fixedShoot(
                    new ExtPose(2.5, LEFT_TRENCH_CENTER, Rotation2d.fromDegrees(0)))),
        Commands.waitSeconds(5)
            .deadlineFor(superstructure.turretTrackHub().alongWith(superstructure.shoot())),
        autoCommands
            .driveTo(new ExtPose(6, LEFT_TRENCH_CENTER, Rotation2d.kZero))
            .withMaxSpeed(3.5));
  }

  public Command leftDepotCenter() {
    return Commands.sequence(
        autoCommands.resetPose(() -> new ExtPose(3.573, 7.76, Rotation2d.fromDegrees(-180)).get()),
        intakeCoordinator.deployAndRunAUTO(),
        Commands.deadline(
                Commands.sequence(
                    autoCommands
                        .driveTo(new ExtPose(0.76, 7.0, Rotation2d.fromDegrees(-120)))
                        .withMaxSpeed(2.5)
                        .withWaypoint(0.5),
                    autoCommands
                        .driveTo(new ExtPose(0.76, 5.284, Rotation2d.fromDegrees(-120)))
                        .withMaxSpeed(1)))
            .deadlineFor(
                superstructure.fixedShoot(new ExtPose(0.76, 5.284, Rotation2d.fromDegrees(-120)))),
        Commands.waitSeconds(12)
            .deadlineFor(superstructure.turretTrackHub().alongWith(superstructure.shoot())),
        superstructure.stopShoot(),
        autoCommands
            .driveTo(new ExtPose(3.573, LEFT_TRENCH_CENTER, Rotation2d.fromDegrees(-180)))
            .withMaxSpeed(3.5)
            .withEndTargetSpeed(1),
        autoCommands.driveTo(new ExtPose(8.25, LEFT_TRENCH_CENTER, Rotation2d.fromDegrees(-180))));
  }

  public Command leftDepotCenterMiddlePass() {
    DriveToPoint driveToStartOfCleanup =
        autoCommands.driveTo(
            () -> Paths.forAlliance(Paths.LEFT_SIDE_AFTER_DEPOT_CLEANUP).getStartingPose());

    return Commands.sequence(
        autoCommands.resetPose(() -> new ExtPose(3.573, 4.135, Rotation2d.fromDegrees(-180)).get()),
        intakeCoordinator.deployAndRunAUTO(),
        Commands.deadline(
                Commands.sequence(
                    autoCommands
                        .driveTo(new ExtPose(0.76, 7.0, Rotation2d.fromDegrees(-120)))
                        .withMaxSpeed(2.5)
                        .withWaypoint(0.5),
                    autoCommands
                        .driveTo(new ExtPose(0.76, 5.284, Rotation2d.fromDegrees(-120)))
                        .withMaxSpeed(1)))
            .deadlineFor(
                superstructure.fixedShoot(new ExtPose(0.76, 5.284, Rotation2d.fromDegrees(-120)))),
        Commands.waitSeconds(7)
            .deadlineFor(superstructure.turretTrackHub().alongWith(superstructure.shoot())),
        superstructure.stopShoot(),
        driveToStartOfCleanup.withMaxSpeed(3.5).withEndTargetSpeed(1),
        autoFactory.path(Paths.LEFT_SIDE_AFTER_DEPOT_CLEANUP),
        superstructure.shoot());
  }

  private Supplier<Pose2d> cleanupDriveTarget(PathData cleanupPath) {
    return () -> {
      Pose2d start = Paths.forAlliance(cleanupPath).getStartingPose();
      return new Pose2d(
          start.getTranslation().plus(new Translation2d(0.25, start.getRotation())),
          start.getRotation());
    };
  }

  private Supplier<Pose2d> feedTargetPose(Pose2d redTargetPose) {
    ExtPose redPose = new ExtPose(redTargetPose);
    ExtPose bluePose = new ExtPose(FieldFlip.overWidth(redTargetPose));
    return () -> FieldInfo.shouldFlip() ? redPose.get() : bluePose.get();
  }
}
