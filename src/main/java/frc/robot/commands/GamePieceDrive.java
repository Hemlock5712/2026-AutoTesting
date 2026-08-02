package frc.robot.commands;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.LimelightHelpers;
import java.util.function.DoubleSupplier;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * Physics-based drive command with vision-assisted game piece tracking.
 *
 * <p>Uses Limelight vision to automatically adjust strafe velocity toward detected game pieces.
 * Vision correction is applied before physics limiting, treating it as part of driver intent.
 *
 * <p>The correction strength scales with forward speed - faster driving means stronger correction.
 */
public class GamePieceDrive extends CommandLifecycleAdapter {

  // Default proportional gain for vision correction
  private static final double DEFAULT_VISION_KP = 0.5;

  private final CommandSwerveDrivetrain swerve;
  private final DoubleSupplier velocityXSupplier;
  private final DoubleSupplier velocityYSupplier;
  private final DoubleSupplier rotationalRateSupplier;
  private final String limelightName;
  private final double visionKp;

  // State tracking between execute cycles
  private ChassisVelocities lastCommandedVelocity = new ChassisVelocities();
  private double lastTime;

  private final SwerveRequest.ApplyFieldVelocity request =
      new SwerveRequest.ApplyFieldVelocity()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.Position)
          .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);

  /**
   * Creates a GamePieceDrive command with default vision gain.
   *
   * @param swerve The swerve drivetrain
   * @param velocityX Supplier for field-relative X velocity in m/s
   * @param velocityY Supplier for field-relative Y velocity in m/s
   * @param rotationalRate Supplier for rotational rate in rad/s
   * @param limelightName NetworkTables name of the Limelight to use
   */
  public GamePieceDrive(
      CommandSwerveDrivetrain swerve,
      DoubleSupplier velocityX,
      DoubleSupplier velocityY,
      DoubleSupplier rotationalRate,
      String limelightName) {
    this(swerve, velocityX, velocityY, rotationalRate, limelightName, DEFAULT_VISION_KP);
  }

  /**
   * Creates a GamePieceDrive command with custom vision gain.
   *
   * @param swerve The swerve drivetrain
   * @param velocityX Supplier for field-relative X velocity in m/s
   * @param velocityY Supplier for field-relative Y velocity in m/s
   * @param rotationalRate Supplier for rotational rate in rad/s
   * @param limelightName NetworkTables name of the Limelight to use
   * @param visionKp Proportional gain for strafe correction (higher = stronger correction)
   */
  public GamePieceDrive(
      CommandSwerveDrivetrain swerve,
      DoubleSupplier velocityX,
      DoubleSupplier velocityY,
      DoubleSupplier rotationalRate,
      String limelightName,
      double visionKp) {
    super(swerve.getCommandMechanism());
    this.swerve = swerve;
    this.velocityXSupplier = velocityX;
    this.velocityYSupplier = velocityY;
    this.rotationalRateSupplier = rotationalRate;
    this.limelightName = limelightName;
    this.visionKp = visionKp;
  }

  @Override
  public void initialize() {
    // Start from current velocity for smooth transitions
    lastCommandedVelocity = swerve.getFieldSpeeds();
    lastTime = Utils.getCurrentTimeSeconds();
  }

  @Override
  public void execute() {
    // Calculate time since last execute
    double currentTime = Utils.getCurrentTimeSeconds();
    double dt = currentTime - lastTime;
    lastTime = currentTime;

    double vx = velocityXSupplier.getAsDouble();
    double vy = velocityYSupplier.getAsDouble();
    double omega = rotationalRateSupplier.getAsDouble();

    // Apply vision correction if target is visible
    if (LimelightHelpers.getTV(limelightName)) {
      // Calculate strafe correction based on target horizontal offset
      // Negative because positive TX means target is to the right, so we strafe left
      double strafeCorrection = -LimelightHelpers.getTX(limelightName) * visionKp;

      // Scale correction by total speed (no correction when stationary)
      ChassisVelocities robotSpeeds = swerve.getRobotSpeeds();
      double speedScale =
          Math.hypot(robotSpeeds.vx, robotSpeeds.vy) / AccelerationLimiter.MAX_VELOCITY;

      // Convert robot-relative correction to field coordinates using primitive math
      double corrScaled = strafeCorrection * speedScale;
      Rotation2d rot = swerve.getPose().getRotation().plus(swerve.getOperatorForwardDirection());
      vx += -corrScaled * rot.getSin();
      vy += corrScaled * rot.getCos();
    }

    // Apply physics-based acceleration limiting (normalizes desired speeds internally)
    AccelerationLimiter.integrateVelocityInPlace(lastCommandedVelocity, vx, vy, omega, dt);

    swerve.setControl(request.withVelocity(lastCommandedVelocity));
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
