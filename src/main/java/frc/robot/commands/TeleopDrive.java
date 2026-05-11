package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.requests.FieldCentric;
import frc.robot.utils.path.ObstacleAvoidance;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Field-relative teleop drive built on the {@link FieldCentric} {@link
 * frc.robot.subsystems.drive.SwerveRequest}. Acceleration limiting and the 250 Hz pacing live
 * entirely inside {@link FieldCentric}; the command just plumbs joystick suppliers into the
 * request's setters.
 *
 * <p>Joystick deadbanding/rescaling is expected to be done upstream by the supplier (see {@code
 * RobotContainer.rescaleTranslation}), so the request's own deadband is left at the default of
 * zero.
 */
public class TeleopDrive extends Command {
  private final Drive drive;
  private final DoubleSupplier velocityXSupplier;
  private final DoubleSupplier velocityYSupplier;
  private final DoubleSupplier rotationalRateSupplier;
  private final FieldCentric request = new FieldCentric();

  public TeleopDrive(
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

  /** Install the pose-based avoidance clamp. Null disables. */
  public TeleopDrive withObstacleAvoidance(ObstacleAvoidance avoidance) {
    request.withObstacleAvoidance(avoidance);
    return this;
  }

  /** Hold-to-bypass gate for the clamp (climb / defense / contact scoring). */
  public TeleopDrive withAvoidanceOverride(BooleanSupplier override) {
    request.withAvoidanceOverride(override);
    return this;
  }

  @Override
  public void initialize() {
    drive.setControl(request);
  }

  @Override
  public void execute() {
    request
        .withVelocityX(velocityXSupplier.getAsDouble())
        .withVelocityY(velocityYSupplier.getAsDouble())
        .withRotationalRate(rotationalRateSupplier.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    drive.clearControl();
    drive.runVelocity(new ChassisSpeeds());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
