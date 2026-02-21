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

  private final CommandSwerveDrivetrain swerve;
  private Supplier<Pose2d> goalPose;

  // Configurable tolerances
  private double positionTolerance = 0.02; // meters
  private double rotationTolerance = Math.toRadians(2); // radians
  private double maxSpeed = Double.POSITIVE_INFINITY;
  private double endTargetSpeed = 0; // m/s (0 = stop at endpoint)

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

    // Calculate translation velocity
    Translation2d targetLinearVel = new Translation2d();
    if (distance >= positionTolerance) {

      double targetSpeed =
          DriveToPointUtils.calculateBrakingTargetSpeed(
              distance,
              currentSpeed,
              BRAKING_REACTION_TIME,
              targetOmega,
              angleError,
              endTargetSpeed);

      // Apply configured speed limit
      targetSpeed = Math.min(targetSpeed, maxSpeed);

      // Direction vector times speed
      targetLinearVel = toGoal.div(distance).times(targetSpeed);
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
    return withWaypointEnding(speed, DriveToPointWaypoints.WAYPOINT_ADVANCEMENT_TOLERANCE);
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
}
