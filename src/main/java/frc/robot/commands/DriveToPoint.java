package frc.robot.commands;

import static org.wpilib.units.Units.Meters;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.DriveToPointUtils;
import java.util.Set;
import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.util.MathUtil;
import org.wpilib.units.measure.Distance;

/**
 * Drives to a single pose using physics-based motion control.
 *
 * <p>Uses real motor torque curves and friction limits to calculate achievable velocities. Finishes
 * when within position and rotation tolerances.
 */
public class DriveToPoint implements Command {

  // Time buffer for braking calculations (accounts for system latency)
  private static final double BRAKING_REACTION_TIME = 0.1; // seconds

  // Tolerance for waypoint-style endings (larger to prevent oscillation)
  private static final double WAYPOINT_TOLERANCE = 0.25; // meters

  private final CommandSwerveDrivetrain swerve;
  private final Set<Mechanism> requirements;
  private final String name;
  private Supplier<Pose2d> goalPose;

  // Configurable tolerances
  private double positionTolerance = 0.02; // meters
  private double rotationTolerance = Math.toRadians(2); // radians
  private double maxSpeed = Double.POSITIVE_INFINITY;
  private double endTargetSpeed = 0; // m/s (0 = stop at endpoint)
  private boolean isWaypoint = false;

  // State tracking between execute cycles
  private ChassisVelocities lastCommandedVelocity = new ChassisVelocities();
  private double lastTime;

  // Cached values for hasReachedGoal() to avoid redundant calculations
  private double cachedDistance;
  private double cachedAngleError;

  private final SwerveRequest.ApplyFieldVelocity request =
      new SwerveRequest.ApplyFieldVelocity()
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
    this.requirements = Set.of(swerve.getCommandMechanism());
    this.name = getClass().getSimpleName();
    this.goalPose = goalPose;
  }

  @Override
  public void run(Coroutine coroutine) {
    try {
      // Start from current velocity for smooth transitions.
      lastCommandedVelocity = swerve.getFieldSpeeds();
      lastTime = Utils.getCurrentTimeSeconds();

      // Initialize cached values so the first control iteration always runs before completion is
      // evaluated (matching the legacy command lifecycle).
      cachedDistance = Double.POSITIVE_INFINITY;
      cachedAngleError = Double.POSITIVE_INFINITY;

      while (true) {
        updateMotionControl();
        if (hasReachedGoal()) {
          break;
        }
        coroutine.yield();
      }
    } catch (RuntimeException ex) {
      // Preserve safe hardware state when scheduler execution fails.
      stopMotion();
      throw ex;
    }
    stopMotion();
  }

  @Override
  public void onCancel() {
    stopMotion();
  }

  @Override
  public String name() {
    return name;
  }

  @Override
  public Set<Mechanism> requirements() {
    return requirements;
  }

  private void updateMotionControl() {
    // Calculate time since the last control iteration.
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

    // Cache values for hasReachedGoal() to avoid redundant calculations
    cachedDistance = distance;
    cachedAngleError = Math.abs(angleError);

    // Calculate current velocities (needed for omega and translation calculations)
    double currentSpeed = Math.hypot(lastCommandedVelocity.vx, lastCommandedVelocity.vy);
    double currentOmega = lastCommandedVelocity.omega;

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
          new Translation2d(lastCommandedVelocity.vx, lastCommandedVelocity.vy);

      targetLinearVel =
          DriveToPointUtils.calculatePerAxisBrakingVelocity(
              toGoal,
              currentVelocity,
              BRAKING_REACTION_TIME,
              targetOmega,
              angleError,
              endTargetSpeed);

      // Apply configured overall speed limit
      double targetSpeed = targetLinearVel.getNorm();
      if (targetSpeed > maxSpeed) {
        targetLinearVel = targetLinearVel.times(maxSpeed / targetSpeed);
      }
    }

    // Apply physics-based acceleration limiting (normalizes desired speeds internally)
    AccelerationLimiter.integrateVelocityInPlace(
        lastCommandedVelocity, targetLinearVel.getX(), targetLinearVel.getY(), targetOmega, dt);
    swerve.setControl(request.withVelocity(lastCommandedVelocity));
  }

  private void stopMotion() {
    swerve.setControl(new SwerveRequest.Idle());
  }

  private boolean hasReachedGoal() {
    // Waypoints finish on position only — rotation continues into the next command
    if (isWaypoint) {
      return cachedDistance < positionTolerance;
    }
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

  public DriveToPoint withPositionTolerance(Distance tolerance) {
    this.positionTolerance = tolerance.in(Meters);
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

  public DriveToPoint withTolerance(double tolerance) {
    this.positionTolerance = tolerance;
    return this;
  }

  public DriveToPoint withWaypointTolerance() {
    this.positionTolerance = WAYPOINT_TOLERANCE;
    return this;
  }

  /**
   * Configures this command as a waypoint - passing through at the target speed. Sets looser
   * position tolerance appropriate for chaining commands.
   *
   * @param targetSpeed Target speed at endpoint in m/s (0 = stop with waypoint tolerance)
   * @return This command for chaining
   */
  public DriveToPoint withWaypoint(double targetSpeed) {
    this.endTargetSpeed = targetSpeed;
    this.positionTolerance = WAYPOINT_TOLERANCE;
    this.isWaypoint = true;
    return this;
  }
}
