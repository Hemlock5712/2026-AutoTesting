package frc.robot.commands;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.DriveToPointUtils;
import java.util.function.Supplier;

/**
 * Drives to a single pose using physics-based motion control.
 *
 * <p>Uses real motor torque curves and friction limits to calculate achievable velocities. Finishes
 * when within position and rotation tolerances.
 */
public class DriveToPoint extends Command {

  // Time buffer for braking calculations (accounts for system latency)
  private static final double BRAKING_REACTION_TIME = 0.03; // seconds

  // Tolerance for waypoint-style endings (larger to prevent oscillation)
  private static final double WAYPOINT_TOLERANCE = 0.15; // meters

  private final CommandSwerveDrivetrain swerve;
  private Supplier<Pose2d> goalPose;

  // Configurable tolerances
  private double positionTolerance = 0.02; // meters
  private double rotationTolerance = Math.toRadians(2); // radians
  private double maxSpeed = Double.POSITIVE_INFINITY;
  private double endTargetSpeed = 0; // m/s (0 = stop at endpoint)
  private double maxEndSpeedX = Double.POSITIVE_INFINITY; // m/s (no constraint)
  private double maxEndSpeedY = Double.POSITIVE_INFINITY; // m/s (no constraint)

  // State tracking between execute cycles
  private ChassisSpeeds lastCommandedVelocity = new ChassisSpeeds();
  private double lastTime;

  // Cached values for isFinished() to avoid redundant calculations
  private double cachedDistance;
  private double cachedAngleError;

  private final SwerveRequest.ApplyFieldSpeeds request =
      new SwerveRequest.ApplyFieldSpeeds()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.Position);

  /**
   * Creates a DriveToPoint command.
   *
   * @param swerve The swerve drivetrain
   * @param goalPose Target pose in field coordinates
   */
  public DriveToPoint(CommandSwerveDrivetrain swerve, Supplier<Pose2d> goalPose) {
    this.swerve = swerve;
    this.goalPose = goalPose;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    // Start from current velocity for smooth transitions
    lastCommandedVelocity = swerve.getFieldSpeeds();
    lastTime = Utils.getCurrentTimeSeconds();

    // Initialize cached values to infinity so isFinished() returns false before first execute()
    cachedDistance = Double.POSITIVE_INFINITY;
    cachedAngleError = Double.POSITIVE_INFINITY;
  }

  @Override
  public void execute() {
    // Calculate time since last execute
    double currentTime = Utils.getCurrentTimeSeconds();
    double dt = currentTime - lastTime;
    lastTime = currentTime;

    Pose2d currentPose = swerve.getPose();
    Translation2d toGoal = goalPose.get().getTranslation().minus(currentPose.getTranslation());
    double distance = toGoal.getNorm();

    // Calculate rotation first (affects friction budget for translation)
    double angleError =
        MathUtil.angleModulus(
            goalPose.get().getRotation().minus(currentPose.getRotation()).getRadians());

    // Cache values for isFinished() to avoid redundant calculations
    cachedDistance = distance;
    cachedAngleError = Math.abs(angleError);

    // Calculate current velocities (needed for omega and translation calculations)
    double currentSpeed =
        Math.hypot(
            lastCommandedVelocity.vxMetersPerSecond, lastCommandedVelocity.vyMetersPerSecond);
    double currentOmega = lastCommandedVelocity.omegaRadiansPerSecond;

    double targetOmega = 0.0;
    if (Math.abs(angleError) >= rotationTolerance) {
      targetOmega =
          DriveToPointUtils.calculateTargetOmega(
              angleError, distance, currentSpeed, currentOmega, BRAKING_REACTION_TIME);
    }

    // Calculate translation velocity with per-axis end speed constraints
    Translation2d targetLinearVel = new Translation2d();
    if (distance >= positionTolerance) {
      Translation2d currentVelocity =
          new Translation2d(
              lastCommandedVelocity.vxMetersPerSecond, lastCommandedVelocity.vyMetersPerSecond);

      targetLinearVel =
          DriveToPointUtils.calculatePerAxisBrakingVelocity(
              toGoal,
              currentVelocity,
              BRAKING_REACTION_TIME,
              targetOmega,
              angleError,
              endTargetSpeed,
              maxEndSpeedX,
              maxEndSpeedY);

      // Apply configured overall speed limit
      double targetSpeed = targetLinearVel.getNorm();
      if (targetSpeed > maxSpeed) {
        targetLinearVel = targetLinearVel.times(maxSpeed / targetSpeed);
      }
    }

    // Normalize to prevent module saturation
    ChassisSpeeds targetVelocity =
        AccelerationLimiter.normalizeSpeeds(
            new ChassisSpeeds(targetLinearVel.getX(), targetLinearVel.getY(), targetOmega));

    // Apply physics-based acceleration limiting
    ChassisSpeeds output =
        AccelerationLimiter.integrateVelocity(lastCommandedVelocity, targetVelocity, dt);
    swerve.setControl(request.withSpeeds(output));

    lastCommandedVelocity = output;
  }

  @Override
  public void end(boolean interrupted) {
    swerve.setControl(new SwerveRequest.Idle());
  }

  @Override
  public boolean isFinished() {
    // Use cached values from execute() to avoid redundant calculations
    return cachedDistance < positionTolerance && cachedAngleError < rotationTolerance;
  }

  // Builder methods for configuration

  /**
   * Sets maximum speed for this command.
   *
   * @param maxSpeed Maximum linear speed in m/s
   * @return This command for chaining
   */
  public DriveToPoint withMaxSpeed(double maxSpeed) {
    this.maxSpeed = maxSpeed;
    return this;
  }

  /**
   * Sets position tolerance for completion check.
   *
   * @param tolerance Position tolerance in meters
   * @return This command for chaining
   */
  public DriveToPoint withPositionTolerance(double tolerance) {
    this.positionTolerance = tolerance;
    return this;
  }

  /**
   * Sets rotation tolerance for completion check.
   *
   * @param tolerance Rotation tolerance in radians
   * @return This command for chaining
   */
  public DriveToPoint withRotationTolerance(double tolerance) {
    this.rotationTolerance = tolerance;
    return this;
  }

  /**
   * Sets target speed at the endpoint instead of stopping.
   *
   * <p>Use this to chain commands where the robot should pass through the endpoint at velocity
   * rather than braking to a stop. The next command in the chain will pick up at the current
   * velocity.
   *
   * @param speed Target speed in m/s (0 = stop, default)
   * @return This command for chaining
   */
  public DriveToPoint withEndTargetSpeed(double speed) {
    this.endTargetSpeed = speed;
    return this;
  }

  /**
   * Configures this command to end like a waypoint - passing through at speed.
   *
   * <p>Sets a looser position tolerance appropriate for chaining commands where the robot should
   * maintain momentum through this point.
   *
   * @param speed Target speed at endpoint in m/s
   * @return This command for chaining
   */
  public DriveToPoint withWaypointEnding(double speed) {
    return withWaypointEnding(speed, WAYPOINT_TOLERANCE);
  }

  /**
   * Configures this command to end like a waypoint with custom tolerance.
   *
   * @param speed Target speed at endpoint in m/s
   * @param tolerance Position tolerance in meters
   * @return This command for chaining
   */
  public DriveToPoint withWaypointEnding(double speed, double tolerance) {
    this.endTargetSpeed = speed;
    this.positionTolerance = tolerance;
    return this;
  }

  public DriveToPoint withTolerance(double tolerance) {
    this.positionTolerance = tolerance;
    return this;
  }

  /**
   * Sets maximum X velocity at the endpoint (field-centric).
   *
   * <p>This is a MAXIMUM constraint - the robot may arrive at or below this speed. Use this to
   * ensure the robot approaches with limited velocity in a specific field direction, such as
   * approaching a field edge with controlled X velocity to prevent overshooting.
   *
   * @param maxSpeedX Maximum X velocity magnitude in m/s (POSITIVE_INFINITY = no constraint)
   * @return This command for chaining
   */
  public DriveToPoint withMaxEndSpeedX(double maxSpeedX) {
    this.maxEndSpeedX = Math.abs(maxSpeedX);
    return this;
  }

  /**
   * Sets maximum Y velocity at the endpoint (field-centric).
   *
   * <p>This is a MAXIMUM constraint - the robot may arrive at or below this speed. Use this to
   * ensure the robot approaches with limited velocity in a specific field direction, such as
   * approaching a field edge with controlled Y velocity to prevent overshooting.
   *
   * @param maxSpeedY Maximum Y velocity magnitude in m/s (POSITIVE_INFINITY = no constraint)
   * @return This command for chaining
   */
  public DriveToPoint withMaxEndSpeedY(double maxSpeedY) {
    this.maxEndSpeedY = Math.abs(maxSpeedY);
    return this;
  }

  /**
   * Sets maximum X and Y velocities at the endpoint (field-centric).
   *
   * <p>These are MAXIMUM constraints - the robot may arrive at or below these speeds. Use this to
   * shape the approach trajectory, ensuring the robot arrives moving in a specific direction.
   *
   * @param maxSpeedX Maximum X velocity magnitude in m/s (POSITIVE_INFINITY = no constraint)
   * @param maxSpeedY Maximum Y velocity magnitude in m/s (POSITIVE_INFINITY = no constraint)
   * @return This command for chaining
   */
  public DriveToPoint withMaxEndSpeeds(double maxSpeedX, double maxSpeedY) {
    this.maxEndSpeedX = Math.abs(maxSpeedX);
    this.maxEndSpeedY = Math.abs(maxSpeedY);
    return this;
  }
}
