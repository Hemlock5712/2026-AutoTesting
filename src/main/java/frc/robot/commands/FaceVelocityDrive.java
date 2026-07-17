package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.DriveToPointUtils;
import java.util.function.DoubleSupplier;

/**
 * Intake-assist teleop drive command that points the intake at the direction of travel.
 *
 * <p>Driver keeps full translation control while rotation is automatically commanded so the intake
 * side of the robot (robot heading 0) faces the direction the robot is actually moving.
 *
 * <p>The desired heading comes from the <b>measured</b> field-relative chassis speeds rather than
 * the joystick command. While braking, the measured velocity still points along the direction of
 * travel even when the driver pulls the stick the opposite way, so the robot doesn't flip 180
 * degrees while slowing down.
 *
 * <p>The driver can still override rotation with the rotation stick; auto-facing resumes when the
 * stick is released. Below a minimum travel speed no rotation correction is applied, so the robot
 * doesn't spin from velocity measurement noise while nearly stationary.
 */
public class FaceVelocityDrive extends Command {

  // Minimum measured travel speed before we trust the velocity direction (m/s)
  private static final double MIN_SPEED_FOR_HEADING = 0.5;

  // Heading control parameters (matches AxisLockDrive heading lock)
  private static final double HEADING_REACTION_TIME = 0.03;
  private static final double HEADING_DEADBAND = Math.toRadians(3);

  private final CommandSwerveDrivetrain swerve;
  private final DoubleSupplier velocityXSupplier;
  private final DoubleSupplier velocityYSupplier;
  private final DoubleSupplier rotationalRateSupplier;

  private final AccelerationLimitedFieldSpeeds request = new AccelerationLimitedFieldSpeeds();
  private final ChassisSpeeds targetSpeeds = new ChassisSpeeds();

  /**
   * Creates a FaceVelocityDrive command.
   *
   * @param swerve The swerve drivetrain
   * @param velocityX Supplier for field-relative X velocity in m/s
   * @param velocityY Supplier for field-relative Y velocity in m/s
   * @param rotationalRate Supplier for driver rotation override in rad/s (0 = auto-face)
   */
  public FaceVelocityDrive(
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
    double driverOmega = rotationalRateSupplier.getAsDouble();

    double omega = driverOmega != 0.0 ? driverOmega : calculateFaceVelocityOmega();

    targetSpeeds.vxMetersPerSecond = velX;
    targetSpeeds.vyMetersPerSecond = velY;
    targetSpeeds.omegaRadiansPerSecond = omega;
    AccelerationLimiter.normalizeSpeedsInPlace(targetSpeeds);
    request.setTargetSpeeds(
        targetSpeeds.vxMetersPerSecond,
        targetSpeeds.vyMetersPerSecond,
        targetSpeeds.omegaRadiansPerSecond);
  }

  /**
   * Calculates omega to rotate the intake (robot heading 0) toward the measured direction of
   * travel.
   *
   * @return Target omega in rad/s, or 0 when moving too slowly to trust the velocity direction
   */
  private double calculateFaceVelocityOmega() {
    ChassisSpeeds fieldSpeeds = swerve.getFieldSpeeds();
    double travelSpeed = Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);

    if (travelSpeed < MIN_SPEED_FOR_HEADING) {
      return 0.0;
    }

    Rotation2d travelDirection =
        new Rotation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
    double angleError =
        MathUtil.angleModulus(travelDirection.minus(swerve.getRotation()).getRadians());

    // Deadband to avoid oscillation once we're facing the direction of travel
    if (Math.abs(angleError) < HEADING_DEADBAND) {
      return 0.0;
    }

    return DriveToPointUtils.calculateTargetOmega(
        angleError,
        0.0, // Distance=0 disables time-sync for responsive correction
        travelSpeed,
        swerve.getRobotSpeeds().omegaRadiansPerSecond,
        HEADING_REACTION_TIME);
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
