package frc.robot.subsystems.drive.requests;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.SwerveRequest;

/**
 * No-op request. Mirrors CTRE's {@code SwerveRequest::Idle} — installing it via {@link
 * Drive#setControl(SwerveRequest)} stops the active controller from issuing new setpoints, but does
 * not actively zero the modules. The modules will continue holding whatever setpoint was last
 * commanded (motor controllers retain their last target).
 *
 * <p>If you actually want the wheels to stop, use {@code drive.stop()} or {@link SwerveDriveBrake}
 * instead. {@code drive.clearControl()} achieves the same thing as installing this; this class
 * exists for symmetry with the CTR API.
 */
public class Idle implements SwerveRequest {
  @Override
  public void apply(Drive drive, double dt) {}
}
