package frc.robot.subsystems.drive.requests;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.SwerveRequest;

/**
 * Robot-relative velocity request. Same shape as {@link FieldCentric}, but vx/vy are interpreted in
 * robot frame so they aren't rotated by heading before being commanded. Per-module slip/torque/
 * steer-rate limiting happens inside {@link Drive#runVelocity}. Mirrors CTRE's {@code
 * SwerveRequest::RobotCentric}.
 */
public class RobotCentric implements SwerveRequest {
  private volatile double velocityX = 0.0;
  private volatile double velocityY = 0.0;
  private volatile double rotationalRate = 0.0;
  private volatile double deadband = 0.0;
  private volatile double rotationalDeadband = 0.0;
  private volatile Translation2d centerOfRotation = Translation2d.kZero;

  /** Robot-frame velocity X (m/s, forward positive). */
  public RobotCentric withVelocityX(double v) {
    this.velocityX = v;
    return this;
  }

  /** Robot-frame velocity Y (m/s, left positive). */
  public RobotCentric withVelocityY(double v) {
    this.velocityY = v;
    return this;
  }

  /** Rotational rate (rad/s, CCW positive). */
  public RobotCentric withRotationalRate(double omega) {
    this.rotationalRate = omega;
    return this;
  }

  /** Per-axis translation deadband (m/s). */
  public RobotCentric withDeadband(double deadband) {
    this.deadband = deadband;
    return this;
  }

  /** Rotational deadband (rad/s). */
  public RobotCentric withRotationalDeadband(double deadband) {
    this.rotationalDeadband = deadband;
    return this;
  }

  /** Pivot point for rotation, robot-frame meters. Defaults to {@link Translation2d#kZero}. */
  public RobotCentric withCenterOfRotation(Translation2d center) {
    this.centerOfRotation = center;
    return this;
  }

  @Override
  public void apply(Drive drive, double dt) {
    double targetVx = MathUtil.applyDeadband(velocityX, deadband);
    double targetVy = MathUtil.applyDeadband(velocityY, deadband);
    double targetOmega = MathUtil.applyDeadband(rotationalRate, rotationalDeadband);

    drive.runVelocity(new ChassisSpeeds(targetVx, targetVy, targetOmega), centerOfRotation);
  }
}
