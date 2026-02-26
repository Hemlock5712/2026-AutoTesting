package frc.robot.commands;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.DriveToPointUtils;
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

  // State tracking between execute cycles
  private ChassisSpeeds lastCommandedVelocity = new ChassisSpeeds();
  private double lastTime;

  // Heading lock state
  private Rotation2d lockedHeading = Rotation2d.kZero;
  private boolean wasDriverRotating = false; // Tracks if driver recently commanded rotation
  private static final double HEADING_LOCK_REACTION_TIME = 0.03;
  private static final double HEADING_LOCK_OMEGA_THRESHOLD =
      Math.toRadians(10); // Don't lock while spinning fast (10 deg/s)
  private static final double HEADING_LOCK_DEADBAND =
      Math.toRadians(3); // Don't correct errors smaller than 3 degrees

  private final SwerveRequest.ApplyFieldSpeeds request =
      new SwerveRequest.ApplyFieldSpeeds()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.Position)
          .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);

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
    // Start from current velocity for smooth transitions
    lastCommandedVelocity = swerve.getFieldSpeeds();
    lastTime = Utils.getCurrentTimeSeconds();
    lockedHeading = swerve.getRotation();
  }

  @Override
  public void execute() {
    // Calculate time since last execute
    double currentTime = Utils.getCurrentTimeSeconds();
    double dt = currentTime - lastTime;
    lastTime = currentTime;

    // Get driver inputs
    double velX = velocityXSupplier.getAsDouble();
    double velY = velocityYSupplier.getAsDouble();
    double requestedOmega = rotationalRateSupplier.getAsDouble();

    // === HEADING LOCK STATE MACHINE ===
    // Purpose: Maintain heading when driver releases rotation stick
    // State: lockedHeading (target angle), wasDriverRotating (momentum tracking flag)
    // Behavior:
    //   - While driver rotates OR momentum continues (>10 deg/s): track current heading
    //   - Once stopped: correct toward lockedHeading using physics-based omega
    //   - If bumped while stationary: correct immediately (wasDriverRotating is false)
    //   - Deadband (3 deg) prevents oscillation around target
    double measuredOmega = swerve.getRobotSpeeds().omegaRadiansPerSecond;
    boolean isSpinningFromMomentum =
        wasDriverRotating && Math.abs(measuredOmega) >= HEADING_LOCK_OMEGA_THRESHOLD;

    if (requestedOmega != 0.0 || isSpinningFromMomentum) {
      lockedHeading = swerve.getRotation();
    }

    // Update flag: set when driver commands rotation, clear only once they stop AND robot slows
    wasDriverRotating =
        requestedOmega != 0.0
            || (wasDriverRotating && Math.abs(measuredOmega) >= HEADING_LOCK_OMEGA_THRESHOLD);

    // Calculate how far off we are from the locked heading, with deadband to avoid oscillation
    double angleError =
        MathUtil.angleModulus(lockedHeading.minus(swerve.getRotation()).getRadians());
    if (Math.abs(angleError) < HEADING_LOCK_DEADBAND) {
      angleError = 0.0;
    }
    double currentSpeed =
        Math.hypot(
            lastCommandedVelocity.vxMetersPerSecond, lastCommandedVelocity.vyMetersPerSecond);

    // Use driver input if rotating, otherwise calculate correction to maintain locked heading.
    // Distance=0 disables time-sync so we get responsive correction bounded by physics.
    double targetOmega =
        requestedOmega != 0.0
            ? requestedOmega
            : DriveToPointUtils.calculateTargetOmega(
                angleError,
                0.0,
                currentSpeed,
                lastCommandedVelocity.omegaRadiansPerSecond,
                HEADING_LOCK_REACTION_TIME);

    // Normalize to prevent module saturation
    ChassisSpeeds targetVelocity =
        AccelerationLimiter.normalizeSpeeds(new ChassisSpeeds(velX, velY, targetOmega));

    // Apply physics-based acceleration limiting
    lastCommandedVelocity =
        AccelerationLimiter.integrateVelocity(lastCommandedVelocity, targetVelocity, dt);

    swerve.setControl(request.withSpeeds(lastCommandedVelocity));
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
