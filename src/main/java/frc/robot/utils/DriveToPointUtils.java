package frc.robot.utils;

import frc.robot.commands.AccelerationLimiter;

/**
 * Shared physics calculations for drive-to-point commands.
 *
 * <p>Contains methods for calculating target angular velocity and braking speeds. Used by the
 * DriveToPoint command.
 */
public final class DriveToPointUtils {

  // Small angle threshold to prevent division by zero (essentially zero radians)
  private static final double ANGLE_EPSILON = 1e-9;

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
    if (absAngle < ANGLE_EPSILON) {
      return 0.0;
    }

    // LIMIT 1: How fast can we spin and still stop at the target angle?
    // Shrink angle by how far we'll rotate during reaction delay
    double bufferedAngle = Math.max(0, absAngle - Math.abs(currentOmega) * reactionBuffer);
    double maxStoppingOmega = Math.sqrt(STOPPING_OMEGA_FACTOR * bufferedAngle);

    // LIMIT 2: How fast to finish rotating when driving finishes?
    // Shrink distance by how far we'll travel during reaction delay
    double effectiveDistance = Math.max(0, distance - currentSpeed * reactionBuffer);

    // Estimate how long driving will take (pick the longer/safer estimate)
    double brakingTime =
        Math.sqrt(2.0 * effectiveDistance / AccelerationLimiter.MAX_FRICTION_ACCEL);
    double cruiseTime = currentSpeed > 0 ? effectiveDistance / currentSpeed : brakingTime;
    double driveTime = Math.max(brakingTime, cruiseTime);

    // Rotation speed needed to finish in that time
    double timeBasedOmega = 2.0 * absAngle / driveTime;

    // Use the smallest limit, with correct +/- direction
    return Math.copySign(min(maxStoppingOmega, timeBasedOmega, HARDWARE_MAX_OMEGA), angleError);
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

    double absAngleError = Math.abs(angleError);

    // Calculate angular deceleration needed to stop rotation at target angle
    // Using kinematic equation: alpha = omega^2 / (2 * theta)
    // Capped at the physical maximum (using full friction budget for rotation)
    double angularDecel =
        absAngleError > ANGLE_EPSILON
            ? Math.min((targetOmega * targetOmega) / (2.0 * absAngleError), MAX_ANGULAR_DECEL)
            : 0.0;

    // Calculate how much friction budget is used by angular deceleration
    // Convert angular to equivalent linear: a_linear = alpha * radius
    double angularAccelContribution = angularDecel * AccelerationLimiter.DRIVE_BASE_RADIUS;

    // Remaining friction budget for linear braking (Pythagorean: sqrt(max^2 - angular^2))
    double availableLinearAccel =
        Math.sqrt(
            Math.max(
                0, MAX_FRICTION_ACCEL_SQ - angularAccelContribution * angularAccelContribution));

    // Account for reaction time: we'll travel some distance before braking starts
    // Reduce effective distance by how far we'll travel during reaction time
    double bufferedDistance = Math.max(0, distance - currentSpeed * brakingReactionTime);

    // Calculate target speed using kinematic equation: v² = v_target² + 2 * a * d
    // This gives the speed needed to reach targetEndSpeed after traveling distance d
    double targetEndSpeedSq = targetEndSpeed * targetEndSpeed;
    double bufferedTargetSpeed =
        Math.min(
            Math.sqrt(targetEndSpeedSq + 2.0 * availableLinearAccel * bufferedDistance),
            AccelerationLimiter.MAX_VELOCITY);

    // If we need to slow down, use the conservative buffered value
    // Otherwise, we can use the full distance (we're accelerating, not braking)
    if (bufferedTargetSpeed < currentSpeed) {
      return bufferedTargetSpeed;
    } else {
      return Math.min(
          Math.sqrt(targetEndSpeedSq + 2.0 * availableLinearAccel * distance),
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
  public static double min(double a, double b, double c) {
    return Math.min(a, Math.min(b, c));
  }
}
