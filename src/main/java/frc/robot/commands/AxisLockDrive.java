package frc.robot.commands;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.DriveToPointUtils;
import frc.robot.utils.FieldInfo;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Hybrid teleop command that locks field axes to preset coordinates.
 *
 * <p>Supports locking any combination of X, Y, and rotation:
 *
 * <ul>
 *   <li>Lock X: Robot maintains a specific X coordinate while driver controls Y and rotation
 *   <li>Lock Y: Robot maintains a specific Y coordinate while driver controls X and rotation
 *   <li>Lock rotation: Robot maintains a specific heading while driver controls X and Y
 *   <li>Any combination: e.g., lock Y and rotation while driver controls X
 * </ul>
 *
 * <p>Uses physics-based braking curves from DriveToPointUtils to ensure smooth deceleration and
 * zero end speed on locked axes.
 */
public class AxisLockDrive extends Command {

  // Braking reaction time buffer (matches DriveToPoint)
  private static final double BRAKING_REACTION_TIME = 0.06;

  // Position lock tolerance - stop correcting when within this distance (meters)
  private static final double POSITION_LOCK_TOLERANCE = 0.02; // 2 cm

  // Heading lock parameters (matches OrbitDrive)
  private static final double HEADING_LOCK_REACTION_TIME = 0.03;
  private static final double HEADING_LOCK_OMEGA_THRESHOLD = Math.toRadians(10);
  private static final double HEADING_LOCK_DEADBAND = Math.toRadians(3);

  private final CommandSwerveDrivetrain swerve;
  private final DoubleSupplier velocityXSupplier;
  private final DoubleSupplier velocityYSupplier;
  private final DoubleSupplier rotationalRateSupplier;

  // Locked axis targets (null means driver controls that axis)
  private final DoubleSupplier lockedXTarget;
  private final DoubleSupplier lockedYTarget;
  private final Supplier<Rotation2d> lockedRotationTarget;

  // State tracking between execute cycles
  private ChassisSpeeds lastCommandedVelocity = new ChassisSpeeds();
  private double lastTime;

  // Heading lock state (for when rotation is not locked to a preset)
  private Rotation2d lockedHeading = Rotation2d.kZero;
  private boolean wasDriverRotating = false;

  private final SwerveRequest.ApplyFieldSpeeds request =
      new SwerveRequest.ApplyFieldSpeeds()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.Position)
          .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

  /**
   * Creates an AxisLockDrive command with full flexibility.
   *
   * <p>Pass null for any locked target to let the driver control that axis.
   *
   * @param swerve The swerve drivetrain
   * @param velocityX Supplier for X velocity in m/s (used when X is not locked)
   * @param velocityY Supplier for Y velocity in m/s (used when Y is not locked)
   * @param rotationalRate Supplier for rotational rate in rad/s (used when rotation is not locked)
   * @param lockedXTarget Target X coordinate in meters, or null for driver control
   * @param lockedYTarget Target Y coordinate in meters, or null for driver control
   * @param lockedRotationTarget Target rotation, or null for driver control with heading lock
   */
  public AxisLockDrive(
      CommandSwerveDrivetrain swerve,
      DoubleSupplier velocityX,
      DoubleSupplier velocityY,
      DoubleSupplier rotationalRate,
      DoubleSupplier lockedXTarget,
      DoubleSupplier lockedYTarget,
      Supplier<Rotation2d> lockedRotationTarget) {
    this.swerve = swerve;
    this.velocityXSupplier = velocityX;
    this.velocityYSupplier = velocityY;
    this.rotationalRateSupplier = rotationalRate;
    this.lockedXTarget = lockedXTarget;
    this.lockedYTarget = lockedYTarget;
    this.lockedRotationTarget = lockedRotationTarget;
    addRequirements(swerve);
  }

  /**
   * Creates an AxisLockDrive that locks Y and optionally rotation, driver controls X.
   *
   * @param swerve The swerve drivetrain
   * @param velocityX Supplier for X velocity in m/s (driver control)
   * @param rotationalRate Supplier for rotational rate in rad/s
   * @param lockedYTarget Target Y coordinate in meters
   * @param lockedRotationTarget Target rotation, or null for heading lock behavior
   */
  public static AxisLockDrive lockY(
      CommandSwerveDrivetrain swerve,
      DoubleSupplier velocityX,
      DoubleSupplier rotationalRate,
      DoubleSupplier lockedYTarget,
      Supplier<Rotation2d> lockedRotationTarget) {
    return new AxisLockDrive(
        swerve,
        velocityX,
        () -> 0.0, // Y not used when locked
        rotationalRate,
        null, // X is free
        lockedYTarget,
        lockedRotationTarget);
  }

  /**
   * Creates an AxisLockDrive that locks X and optionally rotation, driver controls Y.
   *
   * @param swerve The swerve drivetrain
   * @param velocityY Supplier for Y velocity in m/s (driver control)
   * @param rotationalRate Supplier for rotational rate in rad/s
   * @param lockedXTarget Target X coordinate in meters
   * @param lockedRotationTarget Target rotation, or null for heading lock behavior
   */
  public static AxisLockDrive lockX(
      CommandSwerveDrivetrain swerve,
      DoubleSupplier velocityY,
      DoubleSupplier rotationalRate,
      DoubleSupplier lockedXTarget,
      Supplier<Rotation2d> lockedRotationTarget) {
    return new AxisLockDrive(
        swerve,
        () -> 0.0, // X not used when locked
        velocityY,
        rotationalRate,
        lockedXTarget,
        null, // Y is free
        lockedRotationTarget);
  }

  @Override
  public void initialize() {
    // Start from current velocity for smooth transitions
    lastCommandedVelocity = swerve.getFieldSpeeds();
    lastTime = Utils.getCurrentTimeSeconds();
    lockedHeading = swerve.getRotation();
    wasDriverRotating = false;
  }

  @Override
  public void execute() {
    // Calculate time since last execute
    double currentTime = Utils.getCurrentTimeSeconds();
    double dt = currentTime - lastTime;
    lastTime = currentTime;

    // Get current pose
    Pose2d currentPose = swerve.getPose();

    // Get driver inputs and flip for BlueAlliance perspective
    // This ensures "forward on joystick" = positive field X on both alliances
    double[] flippedInputs =
        FieldInfo.flipJoystick(velocityXSupplier.getAsDouble(), velocityYSupplier.getAsDouble());
    double flippedOmega = FieldInfo.flipJoystickRotation(rotationalRateSupplier.getAsDouble());

    // Calculate X velocity (locked or driver-controlled)
    double velX;
    if (lockedXTarget != null) {
      velX = calculateLockedAxisVelocity(currentPose.getX(), lockedXTarget.getAsDouble());
    } else {
      velX = flippedInputs[0];
    }

    // Calculate Y velocity (locked or driver-controlled)
    double velY;
    if (lockedYTarget != null) {
      velY = calculateLockedAxisVelocity(currentPose.getY(), lockedYTarget.getAsDouble());
    } else {
      velY = flippedInputs[1];
    }

    // Calculate rotation (locked, heading lock, or driver-controlled)
    double targetOmega;
    if (lockedRotationTarget != null) {
      targetOmega = calculateLockedRotationOmega(currentPose.getRotation());
    } else {
      targetOmega = calculateHeadingLockedOmega(flippedOmega);
    }

    // Normalize to prevent module saturation
    ChassisSpeeds targetVelocity =
        AccelerationLimiter.normalizeSpeeds(new ChassisSpeeds(velX, velY, targetOmega));

    // Apply physics-based acceleration limiting
    lastCommandedVelocity =
        AccelerationLimiter.integrateVelocity(lastCommandedVelocity, targetVelocity, dt);

    swerve.setControl(request.withSpeeds(lastCommandedVelocity));
  }

  /**
   * Calculates the velocity needed on a locked axis to reach and stop at the target coordinate.
   *
   * @param currentPosition Current position on the axis in meters
   * @param targetPosition Target position on the axis in meters
   * @return Velocity in m/s (signed toward target)
   */
  private double calculateLockedAxisVelocity(double currentPosition, double targetPosition) {
    double distance = Math.abs(targetPosition - currentPosition);

    // Within tolerance - stop correcting to prevent oscillation
    if (distance < POSITION_LOCK_TOLERANCE) {
      return 0.0;
    }

    double currentOmega = lastCommandedVelocity.omegaRadiansPerSecond;

    // Estimate current speed along this axis
    double currentSpeed =
        Math.hypot(
            lastCommandedVelocity.vxMetersPerSecond, lastCommandedVelocity.vyMetersPerSecond);

    // Use braking curve to calculate target speed
    // angleError=0 and targetEndSpeed=0 since we want to stop at the target
    double targetSpeed =
        DriveToPointUtils.calculateBrakingTargetSpeed(
            distance, currentSpeed, BRAKING_REACTION_TIME, currentOmega, 0.0, 0.0);

    // Apply direction (positive toward target)
    return Math.copySign(targetSpeed, targetPosition - currentPosition);
  }

  /**
   * Calculates omega to reach and hold a locked rotation target.
   *
   * @param currentRotation Current robot rotation
   * @return Target omega in rad/s
   */
  private double calculateLockedRotationOmega(Rotation2d currentRotation) {
    double angleError =
        MathUtil.angleModulus(lockedRotationTarget.get().minus(currentRotation).getRadians());

    // Apply deadband to avoid oscillation
    if (Math.abs(angleError) < HEADING_LOCK_DEADBAND) {
      return 0.0;
    }

    // Calculate correction using physics-based approach
    double currentSpeed =
        Math.hypot(
            lastCommandedVelocity.vxMetersPerSecond, lastCommandedVelocity.vyMetersPerSecond);

    return DriveToPointUtils.calculateTargetOmega(
        angleError,
        0.0, // Distance=0 disables time-sync for responsive correction
        currentSpeed,
        lastCommandedVelocity.omegaRadiansPerSecond,
        HEADING_LOCK_REACTION_TIME);
  }

  /**
   * Calculates omega with heading lock behavior (for when rotation is not locked to a preset).
   *
   * <p>When the driver is rotating, use their input directly. When they stop, automatically
   * maintain the last heading using physics-based correction.
   *
   * @param requestedOmega Driver's requested angular velocity in rad/s
   * @return Target omega in rad/s
   */
  private double calculateHeadingLockedOmega(double requestedOmega) {
    // Track heading while driver rotates or robot is still spinning from intentional momentum
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

    // If driver is rotating, use their input directly
    if (requestedOmega != 0.0) {
      return requestedOmega;
    }

    // Calculate heading error with deadband to avoid oscillation
    double angleError =
        MathUtil.angleModulus(lockedHeading.minus(swerve.getRotation()).getRadians());
    if (Math.abs(angleError) < HEADING_LOCK_DEADBAND) {
      return 0.0;
    }

    // Calculate correction using physics-based approach
    double currentSpeed =
        Math.hypot(
            lastCommandedVelocity.vxMetersPerSecond, lastCommandedVelocity.vyMetersPerSecond);

    return DriveToPointUtils.calculateTargetOmega(
        angleError,
        0.0,
        currentSpeed,
        lastCommandedVelocity.omegaRadiansPerSecond,
        HEADING_LOCK_REACTION_TIME);
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
