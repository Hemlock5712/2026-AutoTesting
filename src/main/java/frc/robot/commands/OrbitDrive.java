package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.DoubleSupplier;

/**
 * Physics-based teleop drive command.
 *
 * <p>Applies acceleration limiting using real motor dyno data and friction coefficients. This
 * prevents wheel slip during aggressive maneuvers while allowing maximum performance.
 *
 * <p>Runs indefinitely until cancelled (typical teleop behavior).
 */
public class OrbitDrive extends Command {

  private final CommandSwerveDrivetrain swerve;
  private final DoubleSupplier velocityXSupplier;
  private final DoubleSupplier velocityYSupplier;
  private final DoubleSupplier rotationalRateSupplier;

  private final AccelerationLimitedFieldSpeeds request = new AccelerationLimitedFieldSpeeds();
  private final ChassisSpeeds targetSpeeds = new ChassisSpeeds();

  /**
   * Creates an OrbitDrive command for teleop control.
   *
   * @param swerve The swerve drivetrain
   * @param velocityX Supplier for field-relative X velocity in m/s
   * @param velocityY Supplier for field-relative Y velocity in m/s
   * @param rotationalRate Supplier for rotational rate in rad/s
   */
  public OrbitDrive(
      CommandSwerveDrivetrain swerve,
      DoubleSupplier velocityX,
      DoubleSupplier velocityY,
      DoubleSupplier rotationalRate) {
    this.swerve = swerve;
    this.velocityXSupplier = velocityX;
    this.velocityYSupplier = velocityY;
    this.rotationalRateSupplier = rotationalRate;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    request.requestInit();
    swerve.setControl(request);
  }

  @Override
  public void execute() {
    double velX = velocityXSupplier.getAsDouble();
    double velY = velocityYSupplier.getAsDouble();
    double omega = rotationalRateSupplier.getAsDouble();

    targetSpeeds.vxMetersPerSecond = velX;
    targetSpeeds.vyMetersPerSecond = velY;
    targetSpeeds.omegaRadiansPerSecond = omega;
    AccelerationLimiter.normalizeSpeedsInPlace(targetSpeeds);
    request.setTargetSpeeds(
        targetSpeeds.vxMetersPerSecond,
        targetSpeeds.vyMetersPerSecond,
        targetSpeeds.omegaRadiansPerSecond);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.setControl(new SwerveRequest.Idle());
  }

  @Override
  public boolean isFinished() {
    return false; // Teleop command runs until cancelled
  }
}
