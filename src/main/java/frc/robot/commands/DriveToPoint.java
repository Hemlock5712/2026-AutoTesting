package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.requests.FieldCentric;
import frc.robot.utils.DriveToPointUtils;
import java.util.function.Supplier;

/**
 * Drives the robot to a target {@link Pose2d}, slowing down smoothly as it approaches. Unlike
 * {@link DriveToWithAvoidance}, this does no path planning — point-to-point only. Use it for
 * alignment, station approach, or any short move where you trust the straight line is clear.
 *
 * <p>Plans the target velocity in {@link #execute()} (50 Hz) and writes it into a {@link
 * FieldCentric} request, which applies the acceleration limiter on the 250 Hz fast loop.
 */
public class DriveToPoint extends Command {

  /**
   * Lead-time the brake curve assumes the chassis needs before commanded deceleration shows up at
   * the wheels (~1-2 robot loops). Effective braking distance is offset by {@code currentSpeed *
   * BRAKING_REACTION_TIME}, so the robot starts slowing this much earlier than the formula
   * suggests. Raise if the chassis overshoots stationary targets.
   */
  private static final double BRAKING_REACTION_TIME = 0.04;

  private final Drive drive;
  private final Supplier<Pose2d> goalPose;

  /** Heading must be within this many radians of the goal before {@link #isFinished} returns. */
  private static final double ROTATION_TOLERANCE_RAD = Math.toRadians(2);

  private double positionTolerance = 0.02;

  private double cachedDistance;
  private double cachedAngleError;

  private final FieldCentric request = new FieldCentric();

  public DriveToPoint(Drive drive, Supplier<Pose2d> goalPose) {
    this.drive = drive;
    this.goalPose = goalPose;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    cachedDistance = Double.POSITIVE_INFINITY;
    cachedAngleError = Double.POSITIVE_INFINITY;
    drive.setControl(request);
  }

  @Override
  public void execute() {
    Pose2d currentPose = drive.getPose();
    Translation2d toGoal = goalPose.get().getTranslation().minus(currentPose.getTranslation());
    double distance = toGoal.getNorm();

    double angleError =
        MathUtil.angleModulus(
            goalPose.get().getRotation().minus(currentPose.getRotation()).getRadians());

    cachedDistance = distance;
    cachedAngleError = Math.abs(angleError);

    ChassisSpeeds fieldSpeeds = drive.getFieldSpeeds();
    double currentSpeed = Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
    double currentOmega = fieldSpeeds.omegaRadiansPerSecond;

    double omega = 0.0;
    if (Math.abs(angleError) >= ROTATION_TOLERANCE_RAD) {
      omega =
          DriveToPointUtils.calculateTargetOmega(
              angleError, distance, currentSpeed, currentOmega, BRAKING_REACTION_TIME);
    }

    Translation2d targetLinearVel = new Translation2d();
    if (distance >= positionTolerance) {
      Translation2d currentVelocity =
          new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
      targetLinearVel =
          DriveToPointUtils.calculatePerAxisBrakingVelocity(
              toGoal, currentVelocity, BRAKING_REACTION_TIME, omega, angleError);
    }

    request
        .withVelocityX(targetLinearVel.getX())
        .withVelocityY(targetLinearVel.getY())
        .withRotationalRate(omega);
  }

  @Override
  public void end(boolean interrupted) {
    drive.clearControl();
    drive.runVelocity(new ChassisSpeeds());
  }

  @Override
  public boolean isFinished() {
    return cachedDistance < positionTolerance && cachedAngleError < ROTATION_TOLERANCE_RAD;
  }

  public DriveToPoint withPositionTolerance(double tolerance) {
    this.positionTolerance = tolerance;
    return this;
  }

  public DriveToPoint withPositionTolerance(Distance tolerance) {
    this.positionTolerance = tolerance.in(Meters);
    return this;
  }
}
