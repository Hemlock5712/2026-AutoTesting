package frc.robot.commands;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;
import java.util.function.DoubleSupplier;

/**
 * Shoot-mode teleop drive command.
 *
 * <p>Same driver inputs as {@link OrbitDrive} but with acceleration, velocity, and jerk limits
 * applied to keep the robot within the SWM accuracy envelope. The acceleration cap only limits
 * positive acceleration (speeding up); braking magnitude is uncapped. The jerk cap limits rate of
 * change of acceleration in both directions (including braking transitions).
 *
 * <p>Bind to a button so the driver holds it while shooting. On release, the default OrbitDrive
 * resumes with no limits.
 */
public class TurretDrive extends Command {

  public static final double MAX_SHOOT_ACCEL = 6.5;
  public static final double MAX_SHOOT_JERK = 360; // m/s^3

  private final CommandSwerveDrivetrain swerve;
  private final DoubleSupplier velocityXSupplier;
  private final DoubleSupplier velocityYSupplier;
  private final DoubleSupplier rotationalRateSupplier;

  // State tracking between execute cycles
  private ChassisSpeeds lastCommandedVelocity = new ChassisSpeeds();
  private double lastTime;

  private final SwerveRequest.ApplyFieldSpeeds request =
      new SwerveRequest.ApplyFieldSpeeds()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.MotionMagicExpo)
          .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
          .withCenterOfRotation(Superstructure.TURRET_TRANSFORM.getTranslation());

  /**
   * Creates a TurretDrive command for teleop control while shooting.
   *
   * @param swerve The swerve drivetrain
   * @param velocityX Supplier for field-relative X velocity in m/s
   * @param velocityY Supplier for field-relative Y velocity in m/s
   * @param rotationalRate Supplier for rotational rate in rad/s
   */
  public TurretDrive(
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
    lastCommandedVelocity = swerve.getFieldSpeeds();
    lastTime = Utils.getCurrentTimeSeconds();
  }

  @Override
  public void execute() {
    double currentTime = Utils.getCurrentTimeSeconds();
    double dt = currentTime - lastTime;
    lastTime = currentTime;

    // Get driver inputs
    double velX = velocityXSupplier.getAsDouble();
    double velY = velocityYSupplier.getAsDouble();
    double omega = rotationalRateSupplier.getAsDouble();

    // Normalize to prevent module saturation
    ChassisSpeeds targetVelocity =
        AccelerationLimiter.normalizeSpeeds(new ChassisSpeeds(velX, velY, omega));

    // Integrate with shoot-mode acceleration and jerk caps
    lastCommandedVelocity =
        AccelerationLimiter.integrateVelocity(
            lastCommandedVelocity,
            targetVelocity,
            dt,
            MAX_SHOOT_ACCEL,
            MAX_SHOOT_JERK,
            MAX_SHOOT_JERK);

    swerve.setControl(request.withSpeeds(lastCommandedVelocity));
  }

  @Override
  public void end(boolean interrupted) {
    swerve.setControl(new SwerveRequest.Idle());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
