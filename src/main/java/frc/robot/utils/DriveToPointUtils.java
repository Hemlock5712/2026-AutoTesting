package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.commands.AccelerationLimiter;

/**
 * Shared physics calculations for drive-to-point commands.
 *
 * <p>Contains methods for calculating target angular velocity and braking speeds. Used by both
 * DriveToPoint and DriveToPointWaypoints commands.
 */
public final class DriveToPointUtils {

  // Small value to prevent division by zero in physics calculations
  private static final double EPSILON = 1e-9;

  // Precomputed constants for performance (avoid division every cycle)
  private static final double HARDWARE_MAX_OMEGA =
      AccelerationLimiter.MAX_VELOCITY / AccelerationLimiter.DRIVE_BASE_RADIUS;
  private static final double MAX_ANGULAR_DECEL =
      AccelerationLimiter.MAX_FRICTION_ACCEL / AccelerationLimiter.DRIVE_BASE_RADIUS;
  private static final double STOPPING_OMEGA_FACTOR =
      2.0 * AccelerationLimiter.MAX_FRICTION_ACCEL / AccelerationLimiter.DRIVE_BASE_RADIUS;
  private static final double MAX_FRICTION_ACCEL_SQ =
      AccelerationLimiter.MAX_FRICTION_ACCEL * AccelerationLimiter.MAX_FRICTION_ACCEL;

  private DriveToPointUtils() {}

  /**
   * Calculates available linear acceleration after angular deceleration consumes its share.
   *
   * <p>Uses Pythagorean constraint: total acceleration² = linear² + angular². Since angular
   * deceleration is needed to stop rotation at the target angle, the remaining friction budget is
   * available for linear braking.
   *
   * @param targetOmega Planned angular velocity in rad/s
   * @param angleError Remaining angle error in radians
   * @return Available linear acceleration in m/s²
   */
  public static double calculateAvailableLinearAccel(double targetOmega, double angleError) {
    double absAngleError = Math.abs(angleError);

    // Calculate angular deceleration needed to stop rotation at target angle
    // Using kinematic equation: alpha = omega² / (2 * theta), capped at physical max
    double angularDecel =
        absAngleError > EPSILON
            ? Math.min((targetOmega * targetOmega) / (2.0 * absAngleError), MAX_ANGULAR_DECEL)
            : 0.0;

    // Convert angular to linear contribution and compute remaining budget
    double angularAccelContribution = angularDecel * AccelerationLimiter.DRIVE_BASE_RADIUS;
    return Math.sqrt(
        Math.max(0, MAX_FRICTION_ACCEL_SQ - angularAccelContribution * angularAccelContribution));
  }

  /**
   * Calculates target angular velocity using time-synchronized approach.
   *
   * <p>Key insight: omega is calculated so rotation finishes when translation finishes. When far
   * away, translationTime is large, so omega is small (prioritizes translation). When close,
   * translationTime is small, so omega increases to finish rotation on time.
   *
   * <p>Applies three constraints and returns the minimum:
   *
   * <ul>
   *   <li>Stopping constraint: max omega that allows stopping at target angle
   *   <li>Time-synchronized: omega to finish rotation when translation completes
   *   <li>Hardware limit: physical maximum of the drivetrain
   * </ul>
   *
   * @param angleError Signed angle error in radians (positive = counterclockwise)
   * @param distance Distance to target in meters (use remaining path distance for waypoints)
   * @param currentSpeed Current linear speed in m/s
   * @param currentOmega Current angular velocity in rad/s
   * @param reactionBuffer Time buffer in seconds (rotation finishes early by this amount)
   * @return Target angular velocity in rad/s (signed to match angleError direction)
   */
  public static double calculateTargetOmega(
      double angleError,
      double distance,
      double currentSpeed,
      double currentOmega,
      double reactionBuffer) {

    double absAngle = Math.abs(angleError);

    // No rotation needed if angle error is negligible
    if (absAngle < EPSILON) {
      return 0.0;
    }

    // LIMIT 1: How fast can we spin and still stop at the target angle?
    // Shrink angle by how far we'll rotate during reaction delay
    double bufferedAngle = Math.max(0, absAngle - Math.abs(currentOmega) * reactionBuffer);
    double maxStoppingOmega = Math.sqrt(STOPPING_OMEGA_FACTOR * bufferedAngle);

    // LIMIT 2: How fast to finish rotating when driving finishes?
    // Shrink distance by how far we'll travel during reaction delay
    double effectiveDistance = Math.max(0, distance - currentSpeed * reactionBuffer);

    // Estimate available linear acceleration (accounting for friction shared with rotation)
    // Use current omega to estimate - this creates a feedback loop that converges
    double availableLinearAccel = calculateAvailableLinearAccel(currentOmega, angleError);

    // Estimate how long driving will take (pick the longer/safer estimate)
    double brakingTime =
        availableLinearAccel > EPSILON
            ? Math.sqrt(2.0 * effectiveDistance / availableLinearAccel)
            : Double.POSITIVE_INFINITY;
    double cruiseTime = currentSpeed > 0 ? effectiveDistance / currentSpeed : brakingTime;
    double driveTime = Math.max(brakingTime, cruiseTime);

    // Rotation speed needed to finish in that time
    double timeBasedOmega = driveTime > 0 ? 2.0 * absAngle / driveTime : HARDWARE_MAX_OMEGA;

    // Use the smallest limit, with correct +/- direction
    return Math.copySign(min(maxStoppingOmega, timeBasedOmega, HARDWARE_MAX_OMEGA), angleError);
  }

  /**
   * Calculates per-axis target velocity for braking with independent X/Y end speed constraints.
   *
   * <p>This method calculates braking speeds for each axis independently, allowing the robot to
   * approach a target with controlled velocity in each field-centric direction. This is useful for
   * approaching field edges or scoring positions where you want to limit velocity in one direction.
   *
   * @param toGoal Vector from current position to goal (field-centric)
   * @param currentVelocity Current velocity for reaction time buffering
   * @param brakingReactionTime Expected delay before braking begins
   * @param targetOmega Planned angular velocity (reduces available braking force)
   * @param angleError Remaining angle error (determines rotation deceleration needs)
   * @param endTargetSpeed Scalar end speed (projected onto axes based on direction)
   * @param maxEndSpeedX Maximum X velocity at endpoint (POSITIVE_INFINITY = no constraint)
   * @param maxEndSpeedY Maximum Y velocity at endpoint (POSITIVE_INFINITY = no constraint)
   * @return Target velocity vector that satisfies per-axis constraints
   */
  public static Translation2d calculatePerAxisBrakingVelocity(
      Translation2d toGoal,
      Translation2d currentVelocity,
      double brakingReactionTime,
      double targetOmega,
      double angleError,
      double endTargetSpeed,
      double maxEndSpeedX,
      double maxEndSpeedY) {

    double distance = toGoal.getNorm();
    if (distance < EPSILON) {
      return new Translation2d();
    }

    // Calculate available linear acceleration after angular deceleration
    double availableLinearAccel = calculateAvailableLinearAccel(targetOmega, angleError);

    // Get displacement components
    double toGoalX = toGoal.getX();
    double toGoalY = toGoal.getY();
    double distanceX = Math.abs(toGoalX);
    double distanceY = Math.abs(toGoalY);

    // Calculate effective end speed constraint for each axis
    // Project scalar endTargetSpeed onto each axis, then apply per-axis max constraint
    double dirX = toGoalX / distance;
    double dirY = toGoalY / distance;
    double projectedEndSpeedX = endTargetSpeed * Math.abs(dirX);
    double projectedEndSpeedY = endTargetSpeed * Math.abs(dirY);
    double effectiveEndSpeedX = Math.min(projectedEndSpeedX, maxEndSpeedX);
    double effectiveEndSpeedY = Math.min(projectedEndSpeedY, maxEndSpeedY);

    // Scale acceleration per-axis so total stays within friction circle
    // When distanceX = distanceY (45°), each axis gets availableLinearAccel / sqrt(2)
    // This ensures hypot(accelX, accelY) = availableLinearAccel
    double accelX = availableLinearAccel * Math.abs(dirX);
    double accelY = availableLinearAccel * Math.abs(dirY);

    // Calculate target speed for each axis using shared braking kinematics
    double currentSpeedX = Math.abs(currentVelocity.getX());
    double currentSpeedY = Math.abs(currentVelocity.getY());
    double targetSpeedX =
        calculateAxisBrakingSpeed(
            distanceX, currentSpeedX, brakingReactionTime, accelX, effectiveEndSpeedX);
    double targetSpeedY =
        calculateAxisBrakingSpeed(
            distanceY, currentSpeedY, brakingReactionTime, accelY, effectiveEndSpeedY);

    // Build velocity vector with correct signs (toward goal)
    return new Translation2d(
        Math.copySign(targetSpeedX, toGoalX), Math.copySign(targetSpeedY, toGoalY));
  }

  /**
   * Calculates target speed for braking to a target speed at the destination.
   *
   * <p>This method determines how fast we can be traveling while still being able to reach the
   * target speed at the destination. It accounts for:
   *
   * <ul>
   *   <li>Distance remaining to target
   *   <li>Friction budget shared between rotation and translation
   *   <li>System reaction time before braking begins
   *   <li>Desired end speed (0 = stop, or higher for pass-through)
   * </ul>
   *
   * @param distance Distance to target in meters
   * @param currentSpeed Current linear speed in m/s
   * @param brakingReactionTime Expected delay before braking begins (accounts for system latency)
   * @param targetOmega Planned angular velocity (reduces available braking force)
   * @param angleError Remaining angle error (determines rotation deceleration needs)
   * @param targetEndSpeed Target speed at destination in m/s (0 = stop)
   * @return Target speed in m/s that allows reaching targetEndSpeed at the destination
   */
  public static double calculateBrakingTargetSpeed(
      double distance,
      double currentSpeed,
      double brakingReactionTime,
      double targetOmega,
      double angleError,
      double targetEndSpeed) {

    double availableLinearAccel = calculateAvailableLinearAccel(targetOmega, angleError);
    return calculateAxisBrakingSpeed(
        distance, currentSpeed, brakingReactionTime, availableLinearAccel, targetEndSpeed);
  }

  /**
   * Core braking calculation for a single axis.
   *
   * <p>Uses kinematic equation v² = v_end² + 2*a*d to calculate the speed needed to reach endSpeed
   * after traveling distance. Applies reaction time buffering when decelerating.
   *
   * @param distance Distance to target
   * @param currentSpeed Current speed along this axis (absolute value)
   * @param reactionTime Expected delay before braking begins
   * @param availableAccel Available acceleration for this axis
   * @param endSpeed Target speed at destination
   * @return Target speed that allows reaching endSpeed at destination
   */
  private static double calculateAxisBrakingSpeed(
      double distance,
      double currentSpeed,
      double reactionTime,
      double availableAccel,
      double endSpeed) {

    double bufferedDistance = Math.max(0, distance - currentSpeed * reactionTime);
    double endSpeedSq = endSpeed * endSpeed;
    double bufferedTargetSpeed = Math.sqrt(endSpeedSq + 2.0 * availableAccel * bufferedDistance);

    // Use buffered if decelerating, full distance if accelerating
    if (bufferedTargetSpeed < currentSpeed) {
      return Math.min(bufferedTargetSpeed, AccelerationLimiter.MAX_VELOCITY);
    } else {
      return Math.min(
          Math.sqrt(endSpeedSq + 2.0 * availableAccel * distance),
          AccelerationLimiter.MAX_VELOCITY);
    }
  }

  /**
   * Returns the minimum of three values.
   *
   * @param a First value
   * @param b Second value
   * @param c Third value
   * @return The smallest of the three values
   */
  private static double min(double a, double b, double c) {
    return Math.min(a, Math.min(b, c));
  }
}
