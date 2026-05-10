package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.requests.FieldCentric;
import frc.robot.utils.DriveToPointUtils;
import frc.robot.utils.FieldInfo;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Teleop command that locks one or more axes (X, Y, rotation) to a target value while letting the
 * driver control the others. Decelerates smoothly to a stop on each locked axis.
 *
 * <p>Target velocities are computed at 50 Hz in {@link #execute()} and written into the {@link
 * FieldCentric} request, which applies the acceleration limiter on the 250 Hz fast loop.
 */
public class AxisLockDrive extends Command {

  private static final double BRAKING_REACTION_TIME = 0.04;
  private static final double POSITION_LOCK_TOLERANCE = 0.005;
  private static final double HEADING_LOCK_REACTION_TIME = 0.03;
  private static final double HEADING_LOCK_OMEGA_THRESHOLD = Math.toRadians(10);
  private static final double HEADING_LOCK_DEADBAND = Math.toRadians(3);

  private final Drive drive;
  private final DoubleSupplier velocityXSupplier;
  private final DoubleSupplier velocityYSupplier;
  private final DoubleSupplier rotationalRateSupplier;

  private final DoubleSupplier lockedXTarget;
  private final DoubleSupplier lockedYTarget;
  private final Supplier<Rotation2d> lockedRotationTarget;

  private Rotation2d lockedHeading = Rotation2d.kZero;
  private boolean wasDriverRotating = false;

  private final FieldCentric request = new FieldCentric();

  public AxisLockDrive(
      Drive drive,
      DoubleSupplier velocityX,
      DoubleSupplier velocityY,
      DoubleSupplier rotationalRate,
      DoubleSupplier lockedXTarget,
      DoubleSupplier lockedYTarget,
      Supplier<Rotation2d> lockedRotationTarget) {
    this.drive = drive;
    this.velocityXSupplier = velocityX;
    this.velocityYSupplier = velocityY;
    this.rotationalRateSupplier = rotationalRate;
    this.lockedXTarget = lockedXTarget;
    this.lockedYTarget = lockedYTarget;
    this.lockedRotationTarget = lockedRotationTarget;
    addRequirements(drive);
  }

  public static AxisLockDrive lockY(
      Drive drive,
      DoubleSupplier velocityX,
      DoubleSupplier rotationalRate,
      DoubleSupplier lockedYTarget,
      Supplier<Rotation2d> lockedRotationTarget) {
    return new AxisLockDrive(
        drive, velocityX, () -> 0.0, rotationalRate, null, lockedYTarget, lockedRotationTarget);
  }

  public static AxisLockDrive lockX(
      Drive drive,
      DoubleSupplier velocityY,
      DoubleSupplier rotationalRate,
      DoubleSupplier lockedXTarget,
      Supplier<Rotation2d> lockedRotationTarget) {
    return new AxisLockDrive(
        drive, () -> 0.0, velocityY, rotationalRate, lockedXTarget, null, lockedRotationTarget);
  }

  @Override
  public void initialize() {
    lockedHeading = drive.getRotation();
    wasDriverRotating = false;
    drive.setControl(request);
  }

  @Override
  public void execute() {
    Pose2d currentPose = drive.getPose();
    ChassisSpeeds fieldSpeeds = drive.getFieldSpeeds();

    double[] flippedInputs =
        FieldInfo.flipJoystick(velocityXSupplier.getAsDouble(), velocityYSupplier.getAsDouble());
    double flippedOmega = FieldInfo.flipJoystickRotation(rotationalRateSupplier.getAsDouble());

    double velX =
        lockedXTarget != null
            ? calculateLockedAxisVelocity(
                currentPose.getX(),
                lockedXTarget.getAsDouble(),
                fieldSpeeds.vxMetersPerSecond,
                fieldSpeeds.omegaRadiansPerSecond)
            : flippedInputs[0];
    double velY =
        lockedYTarget != null
            ? calculateLockedAxisVelocity(
                currentPose.getY(),
                lockedYTarget.getAsDouble(),
                fieldSpeeds.vyMetersPerSecond,
                fieldSpeeds.omegaRadiansPerSecond)
            : flippedInputs[1];

    double omega =
        lockedRotationTarget != null
            ? calculateLockedRotationOmega(currentPose.getRotation())
            : calculateHeadingLockedOmega(flippedOmega);

    request.withVelocityX(velX).withVelocityY(velY).withRotationalRate(omega);
  }

  /**
   * Computes the speed needed to slow down and stop at the locked target. The current velocity
   * passed in MUST be along the locked axis (not total chassis speed).
   */
  private static double calculateLockedAxisVelocity(
      double currentPosition,
      double targetPosition,
      double currentAxisVelocity,
      double currentOmega) {
    double distance = Math.abs(targetPosition - currentPosition);
    if (distance < POSITION_LOCK_TOLERANCE) return 0.0;

    double targetSpeed =
        DriveToPointUtils.calculateBrakingTargetSpeed(
            distance, Math.abs(currentAxisVelocity), BRAKING_REACTION_TIME, currentOmega, 0.0, 0.0);
    return Math.copySign(targetSpeed, targetPosition - currentPosition);
  }

  private double calculateLockedRotationOmega(Rotation2d currentRotation) {
    double angleError =
        MathUtil.angleModulus(lockedRotationTarget.get().minus(currentRotation).getRadians());
    if (Math.abs(angleError) < HEADING_LOCK_DEADBAND) return 0.0;

    ChassisSpeeds fieldSpeeds = drive.getFieldSpeeds();
    double currentSpeed = Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
    return DriveToPointUtils.calculateTargetOmega(
        angleError,
        0.0,
        currentSpeed,
        fieldSpeeds.omegaRadiansPerSecond,
        HEADING_LOCK_REACTION_TIME);
  }

  private double calculateHeadingLockedOmega(double requestedOmega) {
    double measuredOmega = drive.getRobotSpeeds().omegaRadiansPerSecond;
    boolean isSpinningFromMomentum =
        wasDriverRotating && Math.abs(measuredOmega) >= HEADING_LOCK_OMEGA_THRESHOLD;

    if (requestedOmega != 0.0 || isSpinningFromMomentum) {
      lockedHeading = drive.getRotation();
    }

    wasDriverRotating =
        requestedOmega != 0.0
            || (wasDriverRotating && Math.abs(measuredOmega) >= HEADING_LOCK_OMEGA_THRESHOLD);

    if (requestedOmega != 0.0) return requestedOmega;

    double angleError =
        MathUtil.angleModulus(lockedHeading.minus(drive.getRotation()).getRadians());
    if (Math.abs(angleError) < HEADING_LOCK_DEADBAND) return 0.0;

    ChassisSpeeds fieldSpeeds = drive.getFieldSpeeds();
    double currentSpeed = Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
    return DriveToPointUtils.calculateTargetOmega(
        angleError,
        0.0,
        currentSpeed,
        fieldSpeeds.omegaRadiansPerSecond,
        HEADING_LOCK_REACTION_TIME);
  }

  @Override
  public void end(boolean interrupted) {
    drive.clearControl();
    drive.runVelocity(new ChassisSpeeds());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
