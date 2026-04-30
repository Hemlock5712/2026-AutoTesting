package frc.robot.autonomous;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.intake.IntakeCoordinator;
import frc.robot.utils.FieldFlip;
import frc.robot.utils.FieldInfo;
import frc.robot.utils.geometry.ExtPose;
import frc.robot.utils.path.PathData;
import frc.robot.utils.path.Paths;
import frc.robot.utils.path.Paths.AlliancePath;
import java.util.List;

public class AutoRoutines {

  private static final double LEFT_TRENCH_CENTER = FieldInfo.width().in(Meters) - 0.639445;

  private final AutoCommands autoCommands;
  private final Superstructure superstructure;
  private final IntakeCoordinator intakeCoordinator;

  // Pre-generated blue/red path pairs
  private final AlliancePath leftToMiddle = AlliancePath.of(Paths.LEFT_TO_MIDDLE);
  private final AlliancePath leftToMiddleCleanup = AlliancePath.of(Paths.LEFT_TO_MIDDLE_CLEANUP);
  private final AlliancePath rightToMiddle = AlliancePath.of(Paths.RIGHT_TO_MIDDLE);
  private final AlliancePath rightToMiddleCleanup = AlliancePath.of(Paths.RIGHT_TO_MIDDLE_CLEANUP);
  private final AlliancePath rightToMiddleCleanupShallow =
      AlliancePath.of(Paths.RIGHT_TO_MIDDLE_CLEANUP_SHALLOW);
  private final AlliancePath leftToMiddleFeed = AlliancePath.of(Paths.LEFT_TO_MIDDLE_FEED);
  private final AlliancePath feedToMiddleCleanup =
      AlliancePath.of(Paths.FEED_CLEANUP_BACK_TO_MIDDLE);

  private final AlliancePath leftToMiddleDepot = AlliancePath.of(Paths.LEFT_TO_MIDDLE_TO_DEPOT);

  public AutoRoutines(
      AutoCommands autoCommands,
      Superstructure superstructure,
      IntakeCoordinator intakeCoordinator) {
    this.autoCommands = autoCommands;
    this.superstructure = superstructure;
    this.intakeCoordinator = intakeCoordinator;

    // Precompute SplinePath + VelocityProfile for all path variants (both
    // alliances)
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
    leftToMiddleCleanup.blue().precompute();
    leftToMiddleCleanup.red().precompute();
    leftToMiddleDepot.blue().precompute();
    leftToMiddleDepot.red().precompute();

    // Warm up JVM class loading by building a throwaway command chain.
    // Forces all command framework classes to load during robot init, not first
    // auto.
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

  public Command leftSide2Passes() {
    return side2Passes(leftToMiddle, leftToMiddleCleanup);
  }

  public Command rightSide2Passes() {
    return side2Passes(rightToMiddle, rightToMiddleCleanup);
  }

  public Command rightSide2PassesShallow() {
    return side2Passes(rightToMiddle, rightToMiddleCleanupShallow);
  }

  public Command leftSideFeed2Passes() {
    return sideFeed2Passes(
        leftToMiddleFeed, new Pose2d(0.88, 0.75, Rotation2d.k180deg), feedToMiddleCleanup);
  }

  private boolean isRedAlliance() {
    return DriverStation.getAlliance().map(a -> a == DriverStation.Alliance.Red).orElse(false);
  }

  private Command side2Passes(AlliancePath mainAlliancePath, AlliancePath cleanupAlliancePath) {
    Command blueCmd = buildSide2Passes(mainAlliancePath.blue(), cleanupAlliancePath.blue());
    Command redCmd = buildSide2Passes(mainAlliancePath.red(), cleanupAlliancePath.red());
    return Commands.either(redCmd, blueCmd, this::isRedAlliance);
  }

  private Command sideFeed2Passes(
      AlliancePath mainAlliancePath, Pose2d redTargetPose, AlliancePath cleanupAlliancePath) {
    Command blueCmd =
        buildFeedSide2Passes(
            mainAlliancePath.blue(),
            new ExtPose(FieldFlip.overWidth(redTargetPose)),
            cleanupAlliancePath.blue());
    Command redCmd =
        buildFeedSide2Passes(
            mainAlliancePath.red(), new ExtPose(redTargetPose), cleanupAlliancePath.red());
    return Commands.either(redCmd, blueCmd, this::isRedAlliance);
  }

  private Command buildSide2Passes(PathData mainPath, PathData cleanupPath) {
    Pose2d cleanupStart = cleanupPath.getStartingPose();
    Pose2d cleanupDriveTarget =
        new Pose2d(
            cleanupStart.getTranslation().plus(new Translation2d(0.25, cleanupStart.getRotation())),
            cleanupStart.getRotation());

    return Commands.sequence(
        autoCommands.resetPose(() -> mainPath.getStartingPose()),
        autoCommands
            .followPath(mainPath)
            .withCompletionTolerance(0.15)
            .deadlineFor(
                intakeCoordinator.deployAndRunAUTO(),
                superstructure.fixedShoot(() -> mainPath.getTargetPose())),
        new WaitCommand(2.5).deadlineFor(superstructure.shoot(), superstructure.turretTrackHub()),
        superstructure.stopShoot(),
        superstructure.prerollShooter(30),
        autoCommands.driveTo(() -> cleanupDriveTarget).withWaypoint(4.75),
        autoCommands.followPath(cleanupPath).withCompletionTolerance(0.15),
        new WaitCommand(3.5).deadlineFor(superstructure.shoot(), superstructure.turretTrackHub()),
        superstructure.stopShoot(),
        autoCommands.driveTo(() -> cleanupDriveTarget).withWaypoint(4.75),
        autoCommands.followPath(cleanupPath).withCompletionTolerance(0.15));
  }

  private Command buildFeedSide2Passes(
      PathData mainPath, ExtPose feedTargetPose, PathData cleanupPath) {
    Pose2d cleanupStart = cleanupPath.getStartingPose();
    Pose2d cleanupDriveTarget =
        new Pose2d(
            cleanupStart.getTranslation().plus(new Translation2d(0.25, cleanupStart.getRotation())),
            cleanupStart.getRotation());

    return Commands.sequence(
        autoCommands.resetPose(() -> mainPath.getStartingPose()),
        autoCommands
            .followPathWithActions(
                mainPath,
                List.of(
                    new AutoCommands.PathAction(1, 0.5, superstructure::feedShoot),
                    new AutoCommands.PathAction(4, 1.0, superstructure::stopShoot)))
            .deadlineFor(intakeCoordinator.deployAndRunAUTO(), superstructure.prerollShooter(30)),
        autoCommands.driveTo(feedTargetPose).withMaxSpeed(2).deadlineFor(superstructure.shoot()),
        Commands.waitSeconds(3).deadlineFor(superstructure.shoot()),
        superstructure.stopShoot(),
        superstructure.prerollShooter(30),
        autoCommands.driveTo(() -> cleanupDriveTarget).withWaypoint(4.75),
        autoCommands.followPath(cleanupPath).withCompletionTolerance(0.15),
        superstructure.shoot());
  }

  public Command leftAutoFeed(double midlineX) {

    Command redSide =
        autoCommands
            .followPath(leftToMiddleDepot.red())
            .withCompletionTolerance(0.25)
            .withTimeout(6.7)
            .deadlineFor(
                intakeCoordinator.deployAndRunAUTO(),
                superstructure.fixedShoot(() -> leftToMiddleDepot.red().getTargetPose()));
    Command blueSide =
        autoCommands
            .followPath(leftToMiddleDepot.blue())
            .withCompletionTolerance(0.25)
            .withTimeout(6.7)
            .deadlineFor(
                intakeCoordinator.deployAndRunAUTO(),
                superstructure.fixedShoot(() -> leftToMiddleDepot.blue().getTargetPose()));

    return Commands.sequence(
        autoCommands.resetPose(() -> leftToMiddleDepot.get().getStartingPose()),
        // Drive to midline, left of balls
        Commands.either(redSide, blueSide, this::isRedAlliance).withTimeout(7.2),
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
}
