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
import java.util.ArrayList;
import java.util.List;

/**
 * Drives through a series of waypoints to a final pose.
 *
 * <p>Uses time-to-arrival to smoothly transition between waypoints while maintaining momentum.
 * Finishes when within tolerance of the final pose.
 */
public class DriveToPointWaypoints extends Command {

  // Time buffer for braking calculations (accounts for system latency)
  private static final double BRAKING_REACTION_TIME = 0.03; // seconds

  // Tolerance for advancing through intermediate waypoints (larger to prevent oscillation)
  public static final double WAYPOINT_ADVANCEMENT_TOLERANCE = 0.15; // meters

  private final CommandSwerveDrivetrain swerve;
  private final List<Translation2d> waypoints;
  private final Pose2d endPose;

  // Configurable parameters
  private double positionTolerance = 0.02; // meters (only for final destination)
  private double rotationTolerance = Math.toRadians(2); // radians
  private double maxSpeed = Double.POSITIVE_INFINITY;
  private double endTargetSpeed = 0; // m/s (0 = stop at endpoint)

  // State tracking between execute cycles
  private int currentWaypointIndex;
  private ChassisSpeeds lastCommandedVelocity = new ChassisSpeeds();
  private double lastTime;
  private boolean isAtFinalDestination;

  // Cached values for isFinished() to avoid redundant calculations
  private double cachedDistance;
  private double cachedAngleError;

  // Precomputed segment lengths: segmentLengths[i] = distance from waypoint[i] to waypoint[i+1]
  // Final element is distance from last waypoint to endPose
  private double[] segmentLengths;
  // Sum of distances from current waypoint to end (excludes dynamic distance to current target)
  private double remainingSegmentDistance;

  private final SwerveRequest.ApplyFieldSpeeds request =
      new SwerveRequest.ApplyFieldSpeeds()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.Position);

  /**
   * Creates a waypoint command.
   *
   * @param swerve The swerve drivetrain
   * @param waypoints Intermediate waypoints to pass through
   * @param endPose Final destination pose (position and rotation)
   */
  public DriveToPointWaypoints(
      CommandSwerveDrivetrain swerve, List<Translation2d> waypoints, Pose2d endPose) {
    this.swerve = swerve;
    this.waypoints = new ArrayList<>(waypoints);
    this.endPose = endPose;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    lastCommandedVelocity = swerve.getFieldSpeeds();
    currentWaypointIndex = 0;
    lastTime = Utils.getCurrentTimeSeconds();
    isAtFinalDestination = false;

    // Initialize cached values to infinity so isFinished() returns false before first execute()
    cachedDistance = Double.POSITIVE_INFINITY;
    cachedAngleError = Double.POSITIVE_INFINITY;

    // Precompute segment lengths between consecutive waypoints and to endPose
    segmentLengths = new double[waypoints.size()];
    remainingSegmentDistance = 0;

    // For each waypoint, calculate the distance to the next point in the path.
    // If there's another waypoint after this one, measure to that waypoint.
    // Otherwise, measure to the final endPose (this is the last segment).
    for (int i = 0; i < waypoints.size(); i++) {
      Translation2d to =
          (i + 1 < waypoints.size()) ? waypoints.get(i + 1) : endPose.getTranslation();
      segmentLengths[i] = to.minus(waypoints.get(i)).getNorm();
      remainingSegmentDistance += segmentLengths[i];
    }
  }

  @Override
  public void execute() {
    double currentTime = Utils.getCurrentTimeSeconds();
    double dt = currentTime - lastTime;
    lastTime = currentTime;

    Pose2d currentPose = swerve.getPose();
    Translation2d currentPosition = currentPose.getTranslation();
    double currentSpeed =
        Math.hypot(
            lastCommandedVelocity.vxMetersPerSecond, lastCommandedVelocity.vyMetersPerSecond);
    isAtFinalDestination = currentWaypointIndex >= waypoints.size();

    // Determine current target
    Translation2d currentTarget = getCurrentTarget();
    Translation2d toTarget = currentTarget.minus(currentPosition);
    double distance = toTarget.getNorm();

    // Check if we've reached the current waypoint (use larger tolerance to prevent oscillation)
    if (!isAtFinalDestination && distance < WAYPOINT_ADVANCEMENT_TOLERANCE) {
      remainingSegmentDistance -= segmentLengths[currentWaypointIndex];
      currentWaypointIndex++;
      isAtFinalDestination = currentWaypointIndex >= waypoints.size();
      currentTarget = getCurrentTarget();
      toTarget = currentTarget.minus(currentPosition);
      distance = toTarget.getNorm();
    }

    // Calculate rotation (coordinate with translation progress)
    double distanceToEnd = distance + remainingSegmentDistance;
    double angleError =
        MathUtil.angleModulus(endPose.getRotation().minus(currentPose.getRotation()).getRadians());

    // Cache values for isFinished() to avoid redundant calculations
    cachedDistance = distance;
    cachedAngleError = Math.abs(angleError);

    double currentOmega = lastCommandedVelocity.omegaRadiansPerSecond;

    double targetOmega = 0.0;
    if (Math.abs(angleError) >= rotationTolerance) {
      targetOmega =
          DriveToPointUtils.calculateTargetOmega(
              angleError, distanceToEnd, currentSpeed, currentOmega, BRAKING_REACTION_TIME);
    }

    // Calculate translation velocity
    Translation2d targetLinearVel = new Translation2d();
    if (distance >= positionTolerance) {
      // Calculate waypoint end speed based on turn angle
      // - Final destination: use configured endTargetSpeed
      // - Intermediate waypoints: scale by turn sharpness (cosine scaling)
      double waypointEndSpeed = endTargetSpeed;
      if (!isAtFinalDestination) {
        Translation2d nextTarget = getNextTarget();
        double turnAngle = calculateTurnAngle(currentPosition, currentTarget, nextTarget);
        // Cosine scaling: straight = full speed, 90° ≈ 70%, 180° = 0
        waypointEndSpeed = AccelerationLimiter.MAX_VELOCITY * Math.cos(turnAngle / 2.0);

        // Cap based on remaining distance - ensure we can brake to 0 after waypoint
        if (remainingSegmentDistance > 0) {
          double maxSpeedFromDistance =
              Math.sqrt(2.0 * AccelerationLimiter.MAX_FRICTION_ACCEL * remainingSegmentDistance);
          waypointEndSpeed = Math.min(waypointEndSpeed, maxSpeedFromDistance);
        }
      }

      double targetSpeed =
          DriveToPointUtils.calculateBrakingTargetSpeed(
              distance,
              currentSpeed,
              BRAKING_REACTION_TIME,
              targetOmega,
              angleError,
              waypointEndSpeed);

      // Apply configured speed limit
      targetSpeed = Math.min(targetSpeed, maxSpeed);

      targetLinearVel = toTarget.div(distance).times(targetSpeed);
    }

    // Normalize and apply physics limits
    ChassisSpeeds targetVelocity =
        AccelerationLimiter.normalizeSpeeds(
            new ChassisSpeeds(targetLinearVel.getX(), targetLinearVel.getY(), targetOmega));

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
    return isAtFinalDestination
        && cachedDistance < positionTolerance
        && cachedAngleError < rotationTolerance;
  }

  /** Gets the current target translation based on waypoint progress. */
  private Translation2d getCurrentTarget() {
    return isAtFinalDestination ? endPose.getTranslation() : waypoints.get(currentWaypointIndex);
  }

  /** Gets the next target after the current waypoint (for turn angle calculation). */
  private Translation2d getNextTarget() {
    int nextIndex = currentWaypointIndex + 1;
    if (nextIndex < waypoints.size()) {
      return waypoints.get(nextIndex);
    } else if (!isAtFinalDestination) {
      return endPose.getTranslation(); // Last waypoint leads to endPose
    }
    return null; // Already at final destination
  }

  /**
   * Calculates the turn angle at a waypoint.
   *
   * @param robotPos Current robot position
   * @param waypoint The waypoint we're approaching
   * @param nextTarget The target after the waypoint
   * @return Turn angle in radians (0 = straight, π/2 = 90°, π = U-turn)
   */
  private double calculateTurnAngle(
      Translation2d robotPos, Translation2d waypoint, Translation2d nextTarget) {
    if (nextTarget == null) return 0; // No turn at final destination

    Translation2d incoming = waypoint.minus(robotPos);
    Translation2d outgoing = nextTarget.minus(waypoint);

    // Calculate angle between vectors using atan2
    double inAngle = Math.atan2(incoming.getY(), incoming.getX());
    double outAngle = Math.atan2(outgoing.getY(), outgoing.getX());
    return Math.abs(MathUtil.angleModulus(outAngle - inAngle));
  }

  /**
   * Sets maximum speed for this command.
   *
   * @param maxSpeed Maximum linear speed in m/s
   * @return This command for chaining
   */
  public DriveToPointWaypoints withMaxSpeed(double maxSpeed) {
    this.maxSpeed = maxSpeed;
    return this;
  }

  /**
   * Sets position tolerance for completion check.
   *
   * @param tolerance Position tolerance in meters
   * @return This command for chaining
   */
  public DriveToPointWaypoints withPositionTolerance(double tolerance) {
    this.positionTolerance = tolerance;
    return this;
  }

  /**
   * Sets rotation tolerance for completion check.
   *
   * @param tolerance Rotation tolerance in radians
   * @return This command for chaining
   */
  public DriveToPointWaypoints withRotationTolerance(double tolerance) {
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
  public DriveToPointWaypoints withEndTargetSpeed(double speed) {
    this.endTargetSpeed = speed;
    return this;
  }
}
