package frc.robot.autonomous;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveToPoint;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.FieldInfo;
import frc.robot.utils.geometry.ExtPose;
import java.util.function.BooleanSupplier;
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

  /**
   * Creates a new {@link PathPlan} builder for this drivetrain.
   *
   * @return A PathPlan builder with lookahead speed planning
   */
  public PathPlan pathPlan() {
    return PathPlan.create(drivetrain);
  }

  public Command resetPose(Supplier<Pose2d> pose) {
    return drivetrain.runOnce(() -> drivetrain.resetPose(pose.get()));
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
        new DriveToPoint(drivetrain, () -> poses[0]).withWaypoint(maxSpeed).withMaxSpeed(maxSpeed);

    for (int i = 1; i < poses.length - 1; i++) {
      final int idx = i;
      chain =
          chain.andThen(
              new DriveToPoint(drivetrain, () -> poses[idx])
                  .withWaypoint(maxSpeed)
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

  // ==================== Time-Triggered Actions ====================

  /**
   * Drive to a pose and trigger a command after a specified time delay.
   *
   * <p>Useful for timed actions during the drive (e.g., start intake after 1 second of driving).
   *
   * @param targetPose Pose to drive to
   * @param delaySeconds Time in seconds before triggering the command
   * @param commandToRun Command to run after the delay
   * @return Command that drives and triggers the action after the delay
   */
  public Command driveToWithTimedTrigger(
      Supplier<Pose2d> targetPose, double delaySeconds, Command commandToRun) {
    return Commands.deadline(
        driveTo(targetPose), Commands.sequence(Commands.waitSeconds(delaySeconds), commandToRun));
  }

  // ==================== Conditional Actions ====================

  /**
   * Drive to a pose and wait for a condition to be true before continuing.
   *
   * <p>The robot will hold position at the target pose until the condition is satisfied. Useful for
   * waiting on sensor feedback, game state changes, or mechanism readiness.
   *
   * @param targetPose Pose to drive to
   * @param condition Condition to wait for (returns true when ready to continue)
   * @return Command that drives and waits for the condition
   */
  public Command driveToAndWaitFor(Supplier<Pose2d> targetPose, BooleanSupplier condition) {
    return Commands.sequence(driveTo(targetPose), Commands.waitUntil(condition));
  }

  /**
   * Drive to a pose and execute different commands based on a condition.
   *
   * <p>Evaluates the condition after arriving at the pose and branches accordingly. Useful for
   * dynamic autonomous decisions (e.g., check if game piece was collected, then score or retry).
   *
   * @param targetPose Pose to drive to
   * @param condition Condition to evaluate (returns true for ifTrue command, false for ifFalse)
   * @param ifTrue Command to run if condition is true
   * @param ifFalse Command to run if condition is false
   * @return Command that drives and conditionally executes an action
   */
  public Command driveToThenBranch(
      Supplier<Pose2d> targetPose, BooleanSupplier condition, Command ifTrue, Command ifFalse) {
    return Commands.sequence(driveTo(targetPose), Commands.either(ifTrue, ifFalse, condition));
  }

  /**
   * Returns a command that waits until the robot passes a given X position, then runs a command.
   * The threshold is specified in blue-alliance coordinates and is automatically flipped for red
   * alliance using FieldInfo.flipX().
   *
   * @param blueAllianceX X position threshold in blue-alliance coordinates
   * @param commandToRun Command to run once the robot passes the threshold
   * @return Command that triggers based on robot X position
   */
  public Command runWhenPastX(double blueAllianceX, Command commandToRun) {
    return Commands.sequence(
        Commands.waitUntil(() -> FieldInfo.flipX(drivetrain.getPose().getX()) > blueAllianceX),
        commandToRun);
  }

  /**
   * Only affects sim, resets pose to starting config on right side
   *
   * @return
   */
  public Command rightAutoSetup() {
    return resetPose(() -> new ExtPose(4.378, 0.639445, Rotation2d.fromDegrees(90)).get());
  }

  public Command rightAutoSetupFaceForward() {
    return resetPose(() -> new ExtPose(4.378, 0.639445, Rotation2d.fromDegrees(00)).get());
  }

  public Command leftAutoSetup() {
    return resetPose(
        () ->
            new ExtPose(4.378, FieldInfo.width().in(Meters) - 0.639445, Rotation2d.fromDegrees(-90))
                .get());
  }
}
