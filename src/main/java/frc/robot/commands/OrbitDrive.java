package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.Set;
import java.util.function.DoubleSupplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * Physics-based teleop drive command.
 *
 * <p>Applies acceleration limiting using real motor dyno data and friction coefficients. This
 * prevents wheel slip during aggressive maneuvers while allowing maximum performance.
 *
 * <p>Runs indefinitely until cancelled (typical teleop behavior).
 */
public class OrbitDrive implements Command {

  private final CommandSwerveDrivetrain swerve;
  private final Set<Mechanism> requirements;
  private final String name;
  private final DoubleSupplier velocityXSupplier;
  private final DoubleSupplier velocityYSupplier;
  private final DoubleSupplier rotationalRateSupplier;

  private final AccelerationLimitedFieldSpeeds request = new AccelerationLimitedFieldSpeeds();
  private final ChassisVelocities targetSpeeds = new ChassisVelocities();

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
    this.requirements = Set.of(swerve.getCommandMechanism());
    this.name = getClass().getSimpleName();
    this.velocityXSupplier = velocityX;
    this.velocityYSupplier = velocityY;
    this.rotationalRateSupplier = rotationalRate;
  }

  @Override
  public void run(Coroutine coroutine) {
    request.requestInit();
    swerve.setControl(request);

    try {
      while (true) {
        double velX = velocityXSupplier.getAsDouble();
        double velY = velocityYSupplier.getAsDouble();
        double omega = rotationalRateSupplier.getAsDouble();

        targetSpeeds.vx = velX;
        targetSpeeds.vy = velY;
        targetSpeeds.omega = omega;
        AccelerationLimiter.normalizeSpeedsInPlace(targetSpeeds);
        request.setTargetSpeeds(targetSpeeds.vx, targetSpeeds.vy, targetSpeeds.omega);
        coroutine.yield();
      }
    } catch (RuntimeException ex) {
      stop();
      throw ex;
    }
  }

  @Override
  public void onCancel() {
    stop();
  }

  private void stop() {
    swerve.setControl(new SwerveRequest.Idle());
  }

  @Override
  public String name() {
    return name;
  }

  @Override
  public Set<Mechanism> requirements() {
    return requirements;
  }
}
