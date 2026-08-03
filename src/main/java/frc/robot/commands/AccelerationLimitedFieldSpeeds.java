package frc.robot.commands;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * Custom SwerveRequest that applies physics-based acceleration limiting on the 250Hz odometry
 * thread.
 *
 * <p>Target speeds are set from the command thread via volatile writes. The {@link #apply} method
 * runs on the odometry thread with consistent ~4ms dt, applying motor torque and friction circle
 * limits before delegating to {@link SwerveRequest.ApplyFieldVelocity}.
 */
public class AccelerationLimitedFieldSpeeds implements SwerveRequest {

  private final SwerveRequest.ApplyFieldVelocity innerRequest =
      new SwerveRequest.ApplyFieldVelocity()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.MotionMagicExpo)
          .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);

  // Thread-safe target speeds (written from command thread, read on odometry thread)
  private volatile double targetVx;
  private volatile double targetVy;
  private volatile double targetOmega;

  // Initialization flag (volatile write publishes all prior writes)
  private volatile boolean needsInit = true;

  // Odometry-thread-only state (no synchronization needed, pre-allocated to avoid GC)
  private final ChassisVelocities limited = new ChassisVelocities();
  private final double[] accelResult = new double[3];

  private static final double MIN_DT = 1e-9;

  /**
   * Sets the target field-relative speeds. Called from the command thread.
   *
   * @param vx Field-relative X velocity in m/s
   * @param vy Field-relative Y velocity in m/s
   * @param omega Rotational rate in rad/s
   */
  public void setTargetSpeeds(double vx, double vy, double omega) {
    targetVx = vx;
    targetVy = vy;
    targetOmega = omega;
  }

  /** Signals that the next apply() should reinitialize from current robot speeds. */
  public void requestInit() {
    needsInit = true;
  }

  @Override
  public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modules) {
    double dt = parameters.updatePeriod;
    if (dt < MIN_DT) dt = MIN_DT;

    // On init, seed from current field speeds for smooth transitions
    if (needsInit) {
      ChassisVelocities fieldSpeeds =
          parameters.currentChassisVelocity.toFieldRelative(parameters.currentPose.getRotation());
      limited.vx = fieldSpeeds.vx;
      limited.vy = fieldSpeeds.vy;
      limited.omega = fieldSpeeds.omega;
      needsInit = false;
    }

    // Read targets (volatile reads)
    double tVx = targetVx;
    double tVy = targetVy;
    double tOmega = targetOmega;

    // Compute desired acceleration: (target - current) / dt
    double accelX = (tVx - limited.vx) / dt;
    double accelY = (tVy - limited.vy) / dt;
    double accelOmega = (tOmega - limited.omega) / dt;

    // Apply motor torque limit then friction circle limit
    applyMotorLimit(accelX, accelY, accelOmega, limited.vx, limited.vy, limited.omega, accelResult);
    applyFrictionLimit(
        accelResult[0],
        accelResult[1],
        accelResult[2],
        AccelerationLimiter.MAX_FRICTION_ACCEL,
        accelResult);

    // Integrate: next = current + limitedAccel * dt
    limited.vx += accelResult[0] * dt;
    limited.vy += accelResult[1] * dt;
    limited.omega += accelResult[2] * dt;

    // Normalize to prevent module saturation
    AccelerationLimiter.normalizeSpeedsInPlace(limited);

    // Delegate to inner request for module control
    return innerRequest.withVelocity(limited).apply(parameters, modules);
  }

  // --- Inlined pure math from AccelerationLimiter (thread-safe instance methods) ---

  private static boolean isDecelerating(double accelX, double accelY, double velX, double velY) {
    return accelX * velX + accelY * velY < 0;
  }

  private void applyMotorLimit(
      double accelX,
      double accelY,
      double accelOmega,
      double velX,
      double velY,
      double velOmega,
      double[] result) {
    boolean linearBraking = isDecelerating(accelX, accelY, velX, velY);
    boolean angularBraking = accelOmega * velOmega < 0;

    if (linearBraking && angularBraking) {
      result[0] = accelX;
      result[1] = accelY;
      result[2] = accelOmega;
      return;
    }

    double linearAccelMag = Math.hypot(accelX, accelY);
    double linearContrib = linearBraking ? 0 : linearAccelMag;
    double angularContrib =
        angularBraking ? 0 : Math.abs(accelOmega) * AccelerationLimiter.DRIVE_BASE_RADIUS;
    double combinedAccel = Math.hypot(linearContrib, angularContrib);

    double linearVelMag = Math.hypot(velX, velY);
    double moduleSpeed = linearVelMag + Math.abs(velOmega) * AccelerationLimiter.DRIVE_BASE_RADIUS;
    double maxMotorAccel =
        AccelerationLimiter.MOTOR.getMaxAcceleration(
            moduleSpeed,
            AccelerationLimiter.GEAR_RATIO,
            AccelerationLimiter.WHEEL_RADIUS,
            AccelerationLimiter.ROBOT_MASS,
            AccelerationLimiter.NUM_DRIVE_MOTORS,
            AccelerationLimiter.STATOR_CURRENT_LIMIT);

    if (combinedAccel <= maxMotorAccel) {
      result[0] = accelX;
      result[1] = accelY;
      result[2] = accelOmega;
      return;
    }

    double scale = maxMotorAccel / combinedAccel;
    double linearScale = linearBraking ? 1.0 : scale;
    double angularScale = angularBraking ? 1.0 : scale;
    result[0] = accelX * linearScale;
    result[1] = accelY * linearScale;
    result[2] = accelOmega * angularScale;
  }

  private static void applyFrictionLimit(
      double accelX, double accelY, double accelOmega, double maxAccel, double[] result) {
    double effectiveLimit = Math.min(maxAccel, AccelerationLimiter.MAX_FRICTION_ACCEL);
    double linearMag = Math.hypot(accelX, accelY);
    double angularContribution = Math.abs(accelOmega) * AccelerationLimiter.DRIVE_BASE_RADIUS;
    double combinedAccel = Math.hypot(linearMag, angularContribution);

    if (combinedAccel <= effectiveLimit) {
      result[0] = accelX;
      result[1] = accelY;
      result[2] = accelOmega;
      return;
    }

    double scale = effectiveLimit / combinedAccel;
    result[0] = accelX * scale;
    result[1] = accelY * scale;
    result[2] = accelOmega * scale;
  }
}
