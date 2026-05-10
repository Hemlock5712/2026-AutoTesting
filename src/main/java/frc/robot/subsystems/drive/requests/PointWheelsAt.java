package frc.robot.subsystems.drive.requests;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.SwerveRequest;

/**
 * Points all four module wheels at one direction with zero drive velocity. Useful for pre-aiming
 * the wheels to the start of an autonomous path, or for parking the modules in a known orientation.
 * Mirrors CTRE's {@code SwerveRequest::PointWheelsAt}.
 */
public class PointWheelsAt implements SwerveRequest {
  private volatile Rotation2d moduleDirection = Rotation2d.kZero;

  /** Sets the target wheel direction. */
  public PointWheelsAt withModuleDirection(Rotation2d direction) {
    this.moduleDirection = direction;
    return this;
  }

  @Override
  public void apply(Drive drive, double dt) {
    drive.pointWheelsAt(moduleDirection);
  }
}
