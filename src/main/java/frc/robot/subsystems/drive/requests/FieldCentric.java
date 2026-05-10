package frc.robot.subsystems.drive.requests;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.commands.AccelerationLimiter;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.SwerveRequest;

/**
 * Field-relative velocity request. The driver supplies vx/vy in field frame and a rotational rate;
 * the request runs them through {@link AccelerationLimiter} at 250 Hz and commands the resulting
 * robot-frame speeds via {@link Drive#runVelocity(ChassisSpeeds)}. Mirrors the ergonomics of CTRE's
 * {@code SwerveRequest::FieldCentric}.
 *
 * <p>Inputs are mutated via the {@code with*} chainable setters. A typical usage from a teleop
 * command:
 *
 * <pre>{@code
 * private final FieldCentric request = new FieldCentric().withDeadband(0.1);
 *
 * @Override public void initialize() {
 *   drive.setControl(request);
 * }
 * @Override public void execute() {
 *   request
 *       .withVelocityX(stickX.getAsDouble())
 *       .withVelocityY(stickY.getAsDouble())
 *       .withRotationalRate(stickOmega.getAsDouble());
 * }
 * @Override public void end(boolean i) { drive.clearControl(); drive.stop(); }
 * }</pre>
 */
public class FieldCentric implements SwerveRequest {
  // Inputs are written from the main thread and read from the 250 Hz Notifier thread, so they
  // must be safely published. Volatile is sufficient for these primitives.
  private volatile double velocityX = 0.0;
  private volatile double velocityY = 0.0;
  private volatile double rotationalRate = 0.0;
  private volatile double deadband = 0.0;
  private volatile double rotationalDeadband = 0.0;
  private volatile Translation2d centerOfRotation = Translation2d.kZero;

  // Persistent acceleration-limiter state. Written and read only by the fast loop, so plain
  // fields are fine — but the ChassisSpeeds object is mutated in place, so it must outlive a
  // single apply call.
  private final ChassisSpeeds limitedFieldSpeeds = new ChassisSpeeds();

  /** Field-frame velocity X (m/s). */
  public FieldCentric withVelocityX(double v) {
    this.velocityX = v;
    return this;
  }

  /** Field-frame velocity Y (m/s). */
  public FieldCentric withVelocityY(double v) {
    this.velocityY = v;
    return this;
  }

  /** Rotational rate (rad/s, CCW positive). */
  public FieldCentric withRotationalRate(double omega) {
    this.rotationalRate = omega;
    return this;
  }

  /** Per-axis translation deadband (m/s). Applied to velocityX and velocityY independently. */
  public FieldCentric withDeadband(double deadband) {
    this.deadband = deadband;
    return this;
  }

  /** Rotational deadband (rad/s). */
  public FieldCentric withRotationalDeadband(double deadband) {
    this.rotationalDeadband = deadband;
    return this;
  }

  /**
   * Pivot point for rotation, in robot-frame meters. Defaults to {@link Translation2d#kZero} (the
   * geometric center). Useful when the robot's center of mass is offset, or to spin around a
   * specific point such as a corner module or a game-piece pickup location.
   */
  public FieldCentric withCenterOfRotation(Translation2d center) {
    this.centerOfRotation = center;
    return this;
  }

  @Override
  public void onActivate(Drive drive) {
    // Seed the limiter from the current robot motion so the first tick doesn't try to ramp from
    // zero and lag the operator's command.
    ChassisSpeeds field = drive.getFieldSpeeds();
    limitedFieldSpeeds.vxMetersPerSecond = field.vxMetersPerSecond;
    limitedFieldSpeeds.vyMetersPerSecond = field.vyMetersPerSecond;
    limitedFieldSpeeds.omegaRadiansPerSecond = field.omegaRadiansPerSecond;
  }

  @Override
  public void apply(Drive drive, double dt) {
    double targetVx = MathUtil.applyDeadband(velocityX, deadband);
    double targetVy = MathUtil.applyDeadband(velocityY, deadband);
    double targetOmega = MathUtil.applyDeadband(rotationalRate, rotationalDeadband);

    Rotation2d heading = drive.getRotation();
    AccelerationLimiter.integrateVelocityInPlace(
        limitedFieldSpeeds, targetVx, targetVy, targetOmega, dt, heading.getRadians());
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(limitedFieldSpeeds, heading), centerOfRotation);
  }
}
