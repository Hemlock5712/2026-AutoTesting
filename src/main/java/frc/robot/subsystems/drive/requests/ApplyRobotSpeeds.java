package frc.robot.subsystems.drive.requests;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.SwerveRequest;

/**
 * Applies a robot-relative {@link ChassisSpeeds} setpoint each tick. Pure passthrough — no
 * deadband. The per-module limiter inside {@link Drive#runVelocity} still applies. For autonomous
 * controllers (path followers, etc.) that already shape the trajectory and want the chassis to
 * track it directly. Mirrors CTRE's {@code SwerveRequest::ApplyRobotSpeeds}.
 */
public class ApplyRobotSpeeds implements SwerveRequest {
  // Defensive copies of the caller's ChassisSpeeds, so reusing a single instance across ticks
  // can't race the 250 Hz fast loop reading partially-written vx/vy/omega doubles.
  private volatile ChassisSpeeds speeds = new ChassisSpeeds();
  private volatile Translation2d centerOfRotation = Translation2d.kZero;

  /**
   * Sets the target robot-relative speeds. The supplied {@link ChassisSpeeds} is defensively
   * copied, so the caller may safely retain and reuse it.
   */
  public ApplyRobotSpeeds withSpeeds(ChassisSpeeds speeds) {
    this.speeds =
        new ChassisSpeeds(
            speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
    return this;
  }

  /** Pivot point for rotation, robot-frame meters. Defaults to {@link Translation2d#kZero}. */
  public ApplyRobotSpeeds withCenterOfRotation(Translation2d center) {
    this.centerOfRotation = center;
    return this;
  }

  @Override
  public void apply(Drive drive, double dt) {
    drive.runVelocity(speeds, centerOfRotation);
  }
}
