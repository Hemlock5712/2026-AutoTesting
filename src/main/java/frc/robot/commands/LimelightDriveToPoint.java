package frc.robot.commands;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.DriveToPointUtils;
import frc.robot.utils.LimelightHelpers;
import java.util.function.Supplier;

/**
 * Drives to a position relative to a Limelight-detected target using physics-based motion control.
 *
 * <p>On initialize, captures the target's robot-space pose from the Limelight and converts it to a
 * field-relative goal. Uses getX() and getZ() from the Pose3d (Limelight convention: X=lateral,
 * Z=forward/depth). The offsets represent how far from the target the robot should end up.
 *
 * <p>Heading is locked to a supplier-provided rotation independent of translation.
 */
public class LimelightDriveToPoint extends Command {

  private static final double BRAKING_REACTION_TIME = 0.03; // seconds
  private static final double HEADING_LOCK_REACTION_TIME = 0.03; // seconds
  private static final double HEADING_LOCK_DEADBAND = Math.toRadians(3);

  private final CommandSwerveDrivetrain swerve;
  private final String limelightName;
  private final double targetXOffset;
  private final double targetZOffset;
  private final Supplier<Rotation2d> targetHeading;

  // Configurable via builder methods
  private double positionTolerance = 0.02; // meters
  private double rotationTolerance = Math.toRadians(2); // radians
  private double maxSpeed = Double.POSITIVE_INFINITY;

  // State tracking
  private ChassisSpeeds lastCommandedVelocity = new ChassisSpeeds();
  private double lastTime;
  private double cachedDistance;
  private double cachedAngleError;

  // Field-relative goal captured once in initialize()
  private Pose2d fieldGoal;

  private final SwerveRequest.ApplyFieldSpeeds request =
      new SwerveRequest.ApplyFieldSpeeds()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.Position);

  /**
   * Creates a LimelightDriveToPoint command.
   *
   * @param swerve The swerve drivetrain
   * @param limelightName NetworkTables name of the Limelight to use
   * @param targetXOffset Desired X distance from target (0 = align with target's X)
   * @param targetZOffset Desired Z distance from target (e.g., 0.5 = stop 0.5m from target)
   * @param targetHeading Supplier for the locked heading
   */
  public LimelightDriveToPoint(
      CommandSwerveDrivetrain swerve,
      String limelightName,
      double targetXOffset,
      double targetZOffset,
      Supplier<Rotation2d> targetHeading) {
    this.swerve = swerve;
    this.limelightName = limelightName;
    this.targetXOffset = targetXOffset;
    this.targetZOffset = targetZOffset;
    this.targetHeading = targetHeading;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    lastCommandedVelocity = swerve.getFieldSpeeds();
    lastTime = Utils.getCurrentTimeSeconds();
    cachedDistance = Double.POSITIVE_INFINITY;
    cachedAngleError = Double.POSITIVE_INFINITY;

    // No target visible -- finish immediately
    if (!LimelightHelpers.getTV(limelightName)) {
      fieldGoal = swerve.getPose();
      cachedDistance = 0;
      cachedAngleError = 0;
      return;
    }

    // Capture Limelight target in robot space
    Pose3d targetRobotSpace = LimelightHelpers.getTargetPose3d_RobotSpace(limelightName);
    double limelightX = targetRobotSpace.getTranslation().getX();
    double limelightZ = targetRobotSpace.getTranslation().getZ();

    // Drive distance = measured position - desired offset from target
    double driveZ = limelightZ - targetZOffset;
    double driveX = limelightX - targetXOffset;

    // Map to robot-relative: Z=forward, -X=left (Limelight X-positive is right)
    Translation2d robotRelativeOffset = new Translation2d(driveZ, -driveX);

    // Convert to field-relative goal
    Pose2d currentPose = swerve.getPose();
    Translation2d fieldOffset = robotRelativeOffset.rotateBy(currentPose.getRotation());
    Translation2d fieldGoalTranslation = currentPose.getTranslation().plus(fieldOffset);
    fieldGoal = new Pose2d(fieldGoalTranslation, targetHeading.get());
  }

  @Override
  public void execute() {
    double currentTime = Utils.getCurrentTimeSeconds();
    double dt = currentTime - lastTime;
    lastTime = currentTime;

    Pose2d currentPose = swerve.getPose();
    Translation2d toGoal = fieldGoal.getTranslation().minus(currentPose.getTranslation());
    double distance = toGoal.getNorm();

    // Heading: use supplier for live heading target
    Rotation2d desiredHeading = targetHeading.get();
    double angleError =
        MathUtil.angleModulus(desiredHeading.minus(currentPose.getRotation()).getRadians());

    cachedDistance = distance;
    cachedAngleError = Math.abs(angleError);

    double currentSpeed =
        Math.hypot(
            lastCommandedVelocity.vxMetersPerSecond, lastCommandedVelocity.vyMetersPerSecond);
    double currentOmega = lastCommandedVelocity.omegaRadiansPerSecond;

    // Heading control: locked rotation with deadband (distance=0 disables time-sync)
    double targetOmega = 0.0;
    if (Math.abs(angleError) >= HEADING_LOCK_DEADBAND) {
      targetOmega =
          DriveToPointUtils.calculateTargetOmega(
              angleError, 0.0, currentSpeed, currentOmega, HEADING_LOCK_REACTION_TIME);
    }

    // Translation control: per-axis braking to stop at goal
    Translation2d targetLinearVel = new Translation2d();
    if (distance >= positionTolerance) {
      Translation2d currentVelocity =
          new Translation2d(
              lastCommandedVelocity.vxMetersPerSecond, lastCommandedVelocity.vyMetersPerSecond);

      targetLinearVel =
          DriveToPointUtils.calculatePerAxisBrakingVelocity(
              toGoal, currentVelocity, BRAKING_REACTION_TIME, targetOmega, angleError, 0.0);

      double targetSpeed = targetLinearVel.getNorm();
      if (targetSpeed > maxSpeed) {
        targetLinearVel = targetLinearVel.times(maxSpeed / targetSpeed);
      }
    }

    ChassisSpeeds targetVelocity =
        AccelerationLimiter.normalizeSpeeds(
            new ChassisSpeeds(targetLinearVel.getX(), targetLinearVel.getY(), targetOmega));

    ChassisSpeeds output =
        AccelerationLimiter.integrateVelocity(lastCommandedVelocity, targetVelocity, dt);
    swerve.setControl(request.withSpeeds(output));
    lastCommandedVelocity = output;
  }

  @Override
  public void end(boolean interrupted) {
    swerve.setControl(new SwerveRequest.Idle());
  }

  @Override
  public boolean isFinished() {
    return cachedDistance < positionTolerance && cachedAngleError < rotationTolerance;
  }

  // Builder methods

  public LimelightDriveToPoint withMaxSpeed(double maxSpeed) {
    this.maxSpeed = maxSpeed;
    return this;
  }

  public LimelightDriveToPoint withPositionTolerance(double tolerance) {
    this.positionTolerance = tolerance;
    return this;
  }

  public LimelightDriveToPoint withRotationTolerance(double tolerance) {
    this.rotationTolerance = tolerance;
    return this;
  }
}
