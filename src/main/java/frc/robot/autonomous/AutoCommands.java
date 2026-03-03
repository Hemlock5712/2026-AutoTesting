package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveToPoint;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.Supplier;

/**
 * Utility class containing reusable command patterns for autonomous routines.
 *
 * <p>This class provides:
 *
 * <ul>
 *   <li>Common command builders (drive, intake, score)
 *   <li>Game piece spawning for simulation
 *   <li>Vision-based game piece detection and intake
 * </ul>
 */
public class AutoCommands {

  // Subsystems
  private final CommandSwerveDrivetrain drivetrain;

  /**
   * Creates AutoCommands with PhotonVision backend.
   *
   * @param drivetrain The swerve drivetrain
   * @param intake The intake subsystem
   * @param elevator The elevator subsystem (unused but kept for compatibility)
   * @param photonGamePiece PhotonVision detector
   * @param superstructure The superstructure
   */
  public AutoCommands(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  // ==================== Drive Commands ====================

  public DriveToPoint driveTo(Supplier<Pose2d> pose) {
    return new DriveToPoint(drivetrain, pose);
  }

  public Command resetPose(Pose2d pose) {
    return drivetrain.runOnce(() -> drivetrain.resetPose(pose));
  }

  /**
   * Drive through multiple poses
   *
   * <p>Intermediate poses control position only (no heading) - robot smoothly rotates toward
   * endpoint. Only the final pose controls the robot's heading.
   *
   * @param poses Poses to drive through
   * @return Command that drives through all poses
   */
  public Command drivePath(double maxSpeed, Pose2d... poses) {
    if (poses.length == 0) {
      return Commands.none();
    }

    if (poses.length == 1) {
      return new DriveToPoint(drivetrain, () -> poses[0]).withMaxSpeed(maxSpeed);
    }

    // Chain DriveToPoint commands - pass through intermediate poses, stop at final
    Command chain =
        new DriveToPoint(drivetrain, () -> poses[0])
            .withWaypointEnding(maxSpeed)
            .withMaxSpeed(maxSpeed);

    for (int i = 1; i < poses.length - 1; i++) {
      final int idx = i;
      chain =
          chain.andThen(
              new DriveToPoint(drivetrain, () -> poses[idx])
                  .withWaypointEnding(maxSpeed)
                  .withMaxSpeed(maxSpeed));
    }

    // Final pose - stop at destination
    final int lastIdx = poses.length - 1;
    chain =
        chain.andThen(new DriveToPoint(drivetrain, () -> poses[lastIdx]).withMaxSpeed(maxSpeed));

    return chain;
  }

  /**
   * Drive through multiple poses
   *
   * <p>Intermediate poses control position only (no heading) - robot smoothly rotates toward
   * endpoint. Only the final pose controls the robot's heading.
   *
   * @param poses Poses to drive through
   * @return Command that drives through all poses
   */
  public Command drivePath(Pose2d... poses) {
    return drivePath(Double.POSITIVE_INFINITY, poses);
  }

  /**
   * Drive to a pose and run a command when within a certain distance of the target.
   *
   * <p>Combines driving to a pose with triggering a secondary command when within range. When the
   * drive completes, all commands are cleaned up automatically.
   *
   * @param targetPose The pose to drive to
   * @param triggerDistance Distance in meters at which to start the triggered command
   * @param commandToRun The command to run when within trigger distance
   * @return Command that drives to pose and triggers the secondary command at the specified
   *     distance
   */
  public Command driveToWithDistanceTrigger(
      Pose2d targetPose, double triggerDistance, Command commandToRun) {
    return Commands.deadline(
        new DriveToPoint(drivetrain, () -> targetPose),
        Commands.sequence(
            Commands.waitUntil(
                () ->
                    drivetrain.getPose().getTranslation().getDistance(targetPose.getTranslation())
                        < triggerDistance),
            commandToRun));
  }
}
