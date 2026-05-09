package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utils.DriveToPointUtils;
import java.util.function.Supplier;

/**
 * Drives the robot to a target pose, slowing down smoothly as it approaches.
 *
 * <p>Plans the target velocity at 50 Hz, then uses the 250 Hz fast loop to smoothly accelerate.
 */
public class DriveToPoint extends Command {

  private static final double BRAKING_REACTION_TIME = 0.04;
  private static final double WAYPOINT_TOLERANCE = 0.25;

  private final Drive drive;
  private Supplier<Pose2d> goalPose;

  private double positionTolerance = 0.02;
  private double rotationTolerance = Math.toRadians(2);
  private double maxSpeed = Double.POSITIVE_INFINITY;
  private double endTargetSpeed = 0;
  private boolean isWaypoint = false;

  private double cachedDistance;
  private double cachedAngleError;

  // Set by the main loop, read by the fast loop.
  private volatile double targetVx;
  private volatile double targetVy;
  private volatile double targetOmega;

  private final ChassisSpeeds limitedFieldSpeeds = new ChassisSpeeds();

  public DriveToPoint(Drive drive, Supplier<Pose2d> goalPose) {
    this.drive = drive;
    this.goalPose = goalPose;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    ChassisSpeeds field = drive.getFieldSpeeds();
    limitedFieldSpeeds.vxMetersPerSecond = field.vxMetersPerSecond;
    limitedFieldSpeeds.vyMetersPerSecond = field.vyMetersPerSecond;
    limitedFieldSpeeds.omegaRadiansPerSecond = field.omegaRadiansPerSecond;
    targetVx = field.vxMetersPerSecond;
    targetVy = field.vyMetersPerSecond;
    targetOmega = field.omegaRadiansPerSecond;
    cachedDistance = Double.POSITIVE_INFINITY;
    cachedAngleError = Double.POSITIVE_INFINITY;
    drive.setHighRateController(this::tickHighRate);
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
    if (Math.abs(angleError) >= rotationTolerance) {
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
              toGoal, currentVelocity, BRAKING_REACTION_TIME, omega, angleError, endTargetSpeed);
      double targetSpeed = targetLinearVel.getNorm();
      if (targetSpeed > maxSpeed) {
        targetLinearVel = targetLinearVel.times(maxSpeed / targetSpeed);
      }
    }

    targetVx = targetLinearVel.getX();
    targetVy = targetLinearVel.getY();
    targetOmega = omega;
  }

  private void tickHighRate(double dt) {
    AccelerationLimiter.integrateVelocityInPlace(
        limitedFieldSpeeds, targetVx, targetVy, targetOmega, dt);
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(limitedFieldSpeeds, drive.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    drive.clearHighRateController();
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    if (isWaypoint) return cachedDistance < positionTolerance;
    return cachedDistance < positionTolerance && cachedAngleError < rotationTolerance;
  }

  public DriveToPoint withMaxSpeed(double maxSpeed) {
    this.maxSpeed = maxSpeed;
    return this;
  }

  public DriveToPoint withPositionTolerance(double tolerance) {
    this.positionTolerance = tolerance;
    return this;
  }

  public DriveToPoint withPositionTolerance(Distance tolerance) {
    this.positionTolerance = tolerance.in(Meters);
    return this;
  }

  public DriveToPoint withRotationTolerance(double tolerance) {
    this.rotationTolerance = tolerance;
    return this;
  }

  public DriveToPoint withEndTargetSpeed(double speed) {
    this.endTargetSpeed = speed;
    return this;
  }

  public DriveToPoint withTolerance(double tolerance) {
    this.positionTolerance = tolerance;
    return this;
  }

  public DriveToPoint withWaypointTolerance() {
    this.positionTolerance = WAYPOINT_TOLERANCE;
    return this;
  }

  public DriveToPoint withWaypoint(double targetSpeed) {
    this.endTargetSpeed = targetSpeed;
    this.positionTolerance = WAYPOINT_TOLERANCE;
    this.isWaypoint = true;
    return this;
  }
}
