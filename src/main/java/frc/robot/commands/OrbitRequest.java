package frc.robot.commands;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * OrbitRequest - A swerve request that prevents skidding and slipping. Based on FRC Team 1690's
 * implementation.
 *
 * <p>Implements three acceleration limits: 1. Forward Limit - Accounts for reduced motor torque at
 * higher speeds 2. Tilt Limit - Prevents robot tipping during hard acceleration 3. Skid Limit -
 * Prevents wheel slip on carpet
 */
public class OrbitRequest implements SwerveRequest {

  // Driver input velocities (field-centric)
  private double velocityX = 0.0;
  private double velocityY = 0.0;
  private double rotationalRate = 0.0;

  // State tracking for acceleration limiting
  private ChassisSpeeds lastCommandedVelocity = new ChassisSpeeds();
  private double lastTimestamp = -1.0;

  // Delegate to CTRE's ApplyRobotSpeeds for module control
  private final SwerveRequest.ApplyFieldSpeeds applyFieldSpeeds =
      new SwerveRequest.ApplyFieldSpeeds()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.Position)
          .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);

  public OrbitRequest() {}

  @Override
  public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modules) {
    double currentTime = parameters.timestamp;
    double dt = currentTime - lastTimestamp;
    lastTimestamp = currentTime;

    if (dt < 1e-6) {
      // Prevent division by zero or negative time deltas
      return applyFieldSpeeds.withSpeeds(lastCommandedVelocity).apply(parameters, modules);
    }

    // Get driver inputs and normalize to prevent module saturation
    ChassisSpeeds targetVelocity =
        AccelerationLimiter.normalizeSpeeds(
            new ChassisSpeeds(velocityX, velocityY, rotationalRate));

    // Apply physics-based acceleration limiting
    lastCommandedVelocity =
        AccelerationLimiter.integrateVelocity(lastCommandedVelocity, targetVelocity, dt);

    // Send to modules
    return applyFieldSpeeds.withSpeeds(lastCommandedVelocity).apply(parameters, modules);
  }

  /** Sets the desired field-centric velocity in the X direction (forward positive) */
  public OrbitRequest withVelocityX(double velocityX) {
    this.velocityX = velocityX;
    return this;
  }

  /** Sets the desired field-centric velocity in the Y direction (left positive) */
  public OrbitRequest withVelocityY(double velocityY) {
    this.velocityY = velocityY;
    return this;
  }

  /** Sets the desired rotational rate (counter-clockwise positive) */
  public OrbitRequest withRotationalRate(double rotationalRate) {
    this.rotationalRate = rotationalRate;
    return this;
  }

  /** Resets the internal state - call when re-enabling after disable */
  public void reset(ChassisSpeeds currentVelocity) {
    lastTimestamp = Utils.getCurrentTimeSeconds();
    lastCommandedVelocity = currentVelocity;
  }
}
