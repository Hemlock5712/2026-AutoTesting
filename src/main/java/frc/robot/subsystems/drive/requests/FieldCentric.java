package frc.robot.subsystems.drive.requests;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.SwerveRequest;
import frc.robot.utils.path.ObstacleAvoidance;
import java.util.function.BooleanSupplier;

/**
 * Field-relative velocity request. The driver supplies vx/vy in field frame and a rotational rate;
 * the request applies optional pose-based avoidance clamping and hands the result to {@link
 * Drive#runVelocity(ChassisSpeeds)}, which runs the per-module slip/torque/steer-rate limiter
 * inside the setpoint generator. Mirrors the ergonomics of CTRE's {@code SwerveRequest::FieldCentric}.
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
  private volatile ObstacleAvoidance obstacleAvoidance = null;
  private volatile BooleanSupplier avoidanceOverride = null;

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

  /** Pose-based avoidance clamp applied to the field-frame target. Null disables. */
  public FieldCentric withObstacleAvoidance(ObstacleAvoidance avoidance) {
    this.obstacleAvoidance = avoidance;
    return this;
  }

  /** Hold-to-bypass gate for the clamp. Null disables. */
  public FieldCentric withAvoidanceOverride(BooleanSupplier override) {
    this.avoidanceOverride = override;
    return this;
  }

  @Override
  public void apply(Drive drive, double dt) {
    double targetVx = MathUtil.applyDeadband(velocityX, deadband);
    double targetVy = MathUtil.applyDeadband(velocityY, deadband);
    double targetOmega = MathUtil.applyDeadband(rotationalRate, rotationalDeadband);

    ObstacleAvoidance avoidance = obstacleAvoidance;
    BooleanSupplier override = avoidanceOverride;
    if (avoidance != null && (override == null || !override.getAsBoolean())) {
      Translation2d clamped = avoidance.clamp(drive.getPose(), targetVx, targetVy);
      targetVx = clamped.getX();
      targetVy = clamped.getY();
    }

    Rotation2d heading = drive.getRotation();
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(targetVx, targetVy, targetOmega, heading),
        centerOfRotation);
  }
}
