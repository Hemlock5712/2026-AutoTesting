package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

/**
 * Standard teleop drive command. The driver gives field-relative velocities, and the {@link
 * AccelerationLimiter} smooths them out so the wheels don't slip.
 *
 * <p>The 50 Hz main loop just stores the latest joystick inputs. The 250 Hz fast loop reads them
 * and applies physics limits.
 */
public class OrbitDrive extends Command {

  private final Drive drive;
  private final DoubleSupplier velocityXSupplier;
  private final DoubleSupplier velocityYSupplier;
  private final DoubleSupplier rotationalRateSupplier;

  // Set by the main loop, read by the fast loop.
  private volatile double targetVx;
  private volatile double targetVy;
  private volatile double targetOmega;

  // Used only by the fast loop.
  private final ChassisSpeeds limitedFieldSpeeds = new ChassisSpeeds();

  public OrbitDrive(
      Drive drive,
      DoubleSupplier velocityX,
      DoubleSupplier velocityY,
      DoubleSupplier rotationalRate) {
    this.drive = drive;
    this.velocityXSupplier = velocityX;
    this.velocityYSupplier = velocityY;
    this.rotationalRateSupplier = rotationalRate;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    ChassisSpeeds field = drive.getFieldSpeeds();
    limitedFieldSpeeds.vxMetersPerSecond = field.vxMetersPerSecond;
    limitedFieldSpeeds.vyMetersPerSecond = field.vyMetersPerSecond;
    limitedFieldSpeeds.omegaRadiansPerSecond = field.omegaRadiansPerSecond;
    targetVx = field.vxMetersPerSecond;
    targetVy = field.vyMetersPerSecond;
    targetOmega = field.omegaRadiansPerSecond;
    drive.setHighRateController(this::tickHighRate);
  }

  @Override
  public void execute() {
    targetVx = velocityXSupplier.getAsDouble();
    targetVy = velocityYSupplier.getAsDouble();
    targetOmega = rotationalRateSupplier.getAsDouble();
  }

  /** Called from the 250 Hz fast loop. dt is seconds since the previous call. */
  private void tickHighRate(double dt) {
    AccelerationLimiter.integrateVelocityInPlace(
        limitedFieldSpeeds, targetVx, targetVy, targetOmega, dt);
    Rotation2d heading = drive.getRotation();
    drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(limitedFieldSpeeds, heading));
  }

  @Override
  public void end(boolean interrupted) {
    drive.clearHighRateController();
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
