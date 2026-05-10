package frc.robot.subsystems.drive.requests;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.SwerveRequest;

/**
 * Applies a field-relative {@link ChassisSpeeds} setpoint each tick. Identical to {@link
 * ApplyRobotSpeeds} except the speeds are interpreted in field frame and rotated to robot frame
 * using the current heading before being commanded. Mirrors CTRE's {@code
 * SwerveRequest::ApplyFieldSpeeds}.
 */
public class ApplyFieldSpeeds implements SwerveRequest {
  // Defensive copies of the caller's ChassisSpeeds, so reusing a single instance across ticks
  // can't race the 250 Hz fast loop reading partially-written vx/vy/omega doubles.
  private volatile ChassisSpeeds speeds = new ChassisSpeeds();
  private volatile Translation2d centerOfRotation = Translation2d.kZero;

  /**
   * Sets the target field-relative speeds. The supplied {@link ChassisSpeeds} is defensively
   * copied, so the caller may safely retain and reuse it.
   */
  public ApplyFieldSpeeds withSpeeds(ChassisSpeeds speeds) {
    this.speeds =
        new ChassisSpeeds(
            speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
    return this;
  }

  /** Pivot point for rotation, robot-frame meters. Defaults to {@link Translation2d#kZero}. */
  public ApplyFieldSpeeds withCenterOfRotation(Translation2d center) {
    this.centerOfRotation = center;
    return this;
  }

  @Override
  public void apply(Drive drive, double dt) {
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()), centerOfRotation);
  }
}
