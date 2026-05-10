package frc.robot.subsystems.drive.requests;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.SwerveRequest;

/**
 * X-pattern brake. Wraps {@link Drive#stopWithX()} so it can be installed via {@link
 * Drive#setControl(SwerveRequest)} like any other request.
 *
 * <p>Re-applied every fast-loop tick because {@link
 * edu.wpi.first.math.kinematics.SwerveDriveKinematics#resetHeadings} only affects the next
 * zero-speed module-state computation, so calling once and then never again would let the modules
 * drift back to their default heading on the next setpoint.
 */
public class SwerveDriveBrake implements SwerveRequest {
  @Override
  public void apply(Drive drive, double dt) {
    drive.stopWithX();
  }
}
