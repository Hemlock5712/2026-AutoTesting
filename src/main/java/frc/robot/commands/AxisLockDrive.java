package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DrivePhysics;
import frc.robot.utils.FieldInfo;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Teleop assist that locks one of X / Y / heading to a target value while the driver controls the
 * others. Locked-axis output is a feed-forward brake curve {@code v = sqrt(2·μg·distance)} clipped
 * to chassis max; the resulting field-relative {@link ChassisSpeeds} flows through {@link
 * Drive#runVelocity}, where PathPlanner's {@code SwerveSetpointGenerator} enforces per-module
 * slip, torque, and steer-rate limits. The command itself does no PID and no profiling.
 */
public class AxisLockDrive extends Command {

  private static final double POSITION_TOLERANCE_M = 0.01;
  private static final double HEADING_TOLERANCE_RAD = Math.toRadians(1.0);

  private final Drive drive;
  private final DoubleSupplier driverX;
  private final DoubleSupplier driverY;
  private final DoubleSupplier driverOmega;

  private final DoubleSupplier lockedXTarget;
  private final DoubleSupplier lockedYTarget;
  private final Supplier<Rotation2d> lockedRotationTarget;

  public AxisLockDrive(
      Drive drive,
      DoubleSupplier driverX,
      DoubleSupplier driverY,
      DoubleSupplier driverOmega,
      DoubleSupplier lockedXTarget,
      DoubleSupplier lockedYTarget,
      Supplier<Rotation2d> lockedRotationTarget) {
    this.drive = drive;
    this.driverX = driverX;
    this.driverY = driverY;
    this.driverOmega = driverOmega;
    this.lockedXTarget = lockedXTarget;
    this.lockedYTarget = lockedYTarget;
    this.lockedRotationTarget = lockedRotationTarget;
    addRequirements(drive);
  }

  /** Lock the Y axis to {@code lockedYTarget}; driver controls X and rotation. */
  public static AxisLockDrive lockY(
      Drive drive,
      DoubleSupplier driverX,
      DoubleSupplier driverOmega,
      DoubleSupplier lockedYTarget) {
    return new AxisLockDrive(drive, driverX, () -> 0.0, driverOmega, null, lockedYTarget, null);
  }

  /** Lock the X axis to {@code lockedXTarget}; driver controls Y and rotation. */
  public static AxisLockDrive lockX(
      Drive drive,
      DoubleSupplier driverY,
      DoubleSupplier driverOmega,
      DoubleSupplier lockedXTarget) {
    return new AxisLockDrive(drive, () -> 0.0, driverY, driverOmega, lockedXTarget, null, null);
  }

  /** Lock the heading to {@code lockedRotationTarget}; driver controls X and Y. */
  public static AxisLockDrive lockHeading(
      Drive drive,
      DoubleSupplier driverX,
      DoubleSupplier driverY,
      Supplier<Rotation2d> lockedRotationTarget) {
    return new AxisLockDrive(drive, driverX, driverY, () -> 0.0, null, null, lockedRotationTarget);
  }

  @Override
  public void execute() {
    var pose = drive.getPose();
    double[] flipped = FieldInfo.flipJoystick(driverX.getAsDouble(), driverY.getAsDouble());
    double flippedOmega = FieldInfo.flipJoystickRotation(driverOmega.getAsDouble());

    double vxField =
        lockedXTarget != null ? brakeTowardLinear(pose.getX(), lockedXTarget.getAsDouble()) : flipped[0];
    double vyField =
        lockedYTarget != null ? brakeTowardLinear(pose.getY(), lockedYTarget.getAsDouble()) : flipped[1];
    double omega =
        lockedRotationTarget != null
            ? brakeTowardAngular(pose.getRotation(), lockedRotationTarget.get())
            : flippedOmega;

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(vxField, vyField, omega, pose.getRotation()));
  }

  /**
   * Maximum velocity at which the chassis can still stop at the target under {@link
   * DrivePhysics#MAX_FRICTION_ACCEL}, clipped to chassis max. Sign matches the direction to target.
   * Final saturation is enforced per-module downstream by the setpoint generator; this is a clean
   * kinematic ceiling, not a controller.
   */
  private double brakeTowardLinear(double measured, double target) {
    double error = target - measured;
    if (Math.abs(error) < POSITION_TOLERANCE_M) return 0.0;
    double maxV = drive.getMaxLinearSpeedMetersPerSec();
    double magnitude =
        Math.min(maxV, Math.sqrt(2.0 * DrivePhysics.MAX_FRICTION_ACCEL * Math.abs(error)));
    return Math.copySign(magnitude, error);
  }

  private double brakeTowardAngular(Rotation2d measured, Rotation2d target) {
    double error = MathUtil.angleModulus(target.minus(measured).getRadians());
    if (Math.abs(error) < HEADING_TOLERANCE_RAD) return 0.0;
    double maxOmega = drive.getMaxAngularSpeedRadPerSec();
    double maxAlpha = DrivePhysics.MAX_FRICTION_ACCEL / DrivePhysics.DRIVE_BASE_RADIUS;
    double magnitude = Math.min(maxOmega, Math.sqrt(2.0 * maxAlpha * Math.abs(error)));
    return Math.copySign(magnitude, error);
  }

  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisSpeeds());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
