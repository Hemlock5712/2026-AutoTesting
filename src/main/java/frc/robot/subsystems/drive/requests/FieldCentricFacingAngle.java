package frc.robot.subsystems.drive.requests;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.commands.AccelerationLimiter;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.SwerveRequest;

/**
 * Field-relative velocity request with a heading lock. The driver supplies vx/vy in field frame and
 * a target heading; the request's PID controller turns the heading error into a rotational rate,
 * which then flows through the same {@link AccelerationLimiter} pipeline as {@link FieldCentric}.
 * Mirrors CTRE's {@code SwerveRequest::FieldCentricFacingAngle}.
 *
 * <p>The internal {@link PIDController} is exposed via {@link #getHeadingController()} so callers
 * can tune gains and tolerance. It's configured for continuous input on {@code [-pi, pi]} so the
 * controller chooses the shortest-path direction when the target wraps around.
 */
public class FieldCentricFacingAngle implements SwerveRequest {
  // Inputs written from main thread, read from fast loop.
  private volatile double velocityX = 0.0;
  private volatile double velocityY = 0.0;
  private volatile double deadband = 0.0;
  private volatile Rotation2d targetDirection = Rotation2d.kZero;
  private volatile Translation2d centerOfRotation = Translation2d.kZero;

  // Read by the fast loop only.
  private final ChassisSpeeds limitedFieldSpeeds = new ChassisSpeeds();

  // PID is touched only by the fast loop after onActivate resets it on the main thread (which
  // happens-before the volatile activeRequest publish in Drive.setControl).
  private final PIDController headingController;

  /** Constructs with a default heading controller (kP = 5.0, kD = 0.0). */
  public FieldCentricFacingAngle() {
    this(new PIDController(5.0, 0.0, 0.0));
  }

  /** Constructs with a caller-supplied heading controller. */
  public FieldCentricFacingAngle(PIDController headingController) {
    this.headingController = headingController;
    this.headingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /** Field-frame velocity X (m/s). */
  public FieldCentricFacingAngle withVelocityX(double v) {
    this.velocityX = v;
    return this;
  }

  /** Field-frame velocity Y (m/s). */
  public FieldCentricFacingAngle withVelocityY(double v) {
    this.velocityY = v;
    return this;
  }

  /** Per-axis translation deadband (m/s). */
  public FieldCentricFacingAngle withDeadband(double deadband) {
    this.deadband = deadband;
    return this;
  }

  /** Heading the chassis should face. */
  public FieldCentricFacingAngle withTargetDirection(Rotation2d direction) {
    this.targetDirection = direction;
    return this;
  }

  /** Pivot point for rotation, robot-frame meters. Defaults to {@link Translation2d#kZero}. */
  public FieldCentricFacingAngle withCenterOfRotation(Translation2d center) {
    this.centerOfRotation = center;
    return this;
  }

  /**
   * Exposes the internal heading PID controller for tuning. Mutate gains/tolerance only on the main
   * thread before this request becomes the active controller (or while it is paused via {@link
   * Drive#clearControl()}) — the 250 Hz fast loop reads {@code calculate()} concurrently with
   * {@code PIDController}'s unsynchronized internal state, so mid-flight gain changes are a data
   * race.
   */
  public PIDController getHeadingController() {
    return headingController;
  }

  @Override
  public void onActivate(Drive drive) {
    ChassisSpeeds field = drive.getFieldSpeeds();
    limitedFieldSpeeds.vxMetersPerSecond = field.vxMetersPerSecond;
    limitedFieldSpeeds.vyMetersPerSecond = field.vyMetersPerSecond;
    limitedFieldSpeeds.omegaRadiansPerSecond = field.omegaRadiansPerSecond;
    headingController.reset();
  }

  @Override
  public void apply(Drive drive, double dt) {
    double targetVx = MathUtil.applyDeadband(velocityX, deadband);
    double targetVy = MathUtil.applyDeadband(velocityY, deadband);

    Rotation2d heading = drive.getRotation();
    double targetOmega =
        headingController.calculate(heading.getRadians(), targetDirection.getRadians());

    AccelerationLimiter.integrateVelocityInPlace(
        limitedFieldSpeeds, targetVx, targetVy, targetOmega, dt, heading.getRadians());
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(limitedFieldSpeeds, heading), centerOfRotation);
  }
}
