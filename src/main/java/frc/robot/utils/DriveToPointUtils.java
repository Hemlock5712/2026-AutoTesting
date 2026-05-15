package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.lib.dynamics.AccelerationLimiter;

/**
 * Math helpers for "drive to a target" commands. Calculates target rotation speed and how fast to
 * be going while still being able to brake to the destination.
 */
public final class DriveToPointUtils {

  // Used to avoid divide-by-zero.
  private static final double EPSILON = 1e-9;

  // Precomputed once at class load to save time per loop.
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
   * How much linear acceleration is left over after rotation uses its share of friction. Uses the
   * Pythagorean rule: linear² + angular² = total². Whatever rotation needs to stop on time, linear
   * gets the rest.
   */
  public static double calculateAvailableLinearAccel(double targetOmega, double angleError) {
    double absAngleError = Math.abs(angleError);

    // How fast we need to slow down our rotation to stop at the target angle.
    // From physics: alpha = omega² / (2 * theta).
    double angularDecel =
        absAngleError > EPSILON
            ? Math.min((targetOmega * targetOmega) / (2.0 * absAngleError), MAX_ANGULAR_DECEL)
            : 0.0;

    // Convert that to a wheel-edge acceleration and subtract from total budget.
    double angularAccelContribution = angularDecel * AccelerationLimiter.DRIVE_BASE_RADIUS;
    return Math.sqrt(
        Math.max(0, MAX_FRICTION_ACCEL_SQ - angularAccelContribution * angularAccelContribution));
  }

  /**
   * Picks a rotation speed so the robot finishes rotating right when it arrives. Far from the
   * target, rotation is slow (we have time). Close to the target, rotation speeds up to finish on
   * time. Capped by the hardware max and by what we can stop in time.
   *
   * @param angleError Angle remaining (positive = need to turn left)
   * @param distance Distance to the target (m)
   * @param currentSpeed Current speed (m/s)
   * @param currentOmega Current rotation speed (rad/s)
   * @param reactionBuffer Time cushion - finish rotating this many seconds early
   * @return Rotation speed in rad/s, signed to match angleError
   */
  public static double calculateTargetOmega(
      double angleError,
      double distance,
      double currentSpeed,
      double currentOmega,
      double reactionBuffer) {

    double absAngle = Math.abs(angleError);

    // Already at the right angle.
    if (absAngle < EPSILON) {
      return 0.0;
    }

    // Limit 1: max speed we can spin and still stop on time.
    double bufferedAngle = Math.max(0, absAngle - Math.abs(currentOmega) * reactionBuffer);
    double maxStoppingOmega = Math.sqrt(STOPPING_OMEGA_FACTOR * bufferedAngle);

    // Limit 2: speed needed to finish rotating exactly when we arrive.
    double effectiveDistance = Math.max(0, distance - currentSpeed * reactionBuffer);

    // How much linear accel is left after rotation takes its share.
    double availableLinearAccel = calculateAvailableLinearAccel(currentOmega, angleError);

    // Estimate driving time - use whichever is longer (braking vs cruising).
    double brakingTime =
        availableLinearAccel > EPSILON
            ? Math.sqrt(2.0 * effectiveDistance / availableLinearAccel)
            : Double.POSITIVE_INFINITY;
    double cruiseTime = currentSpeed > 0 ? effectiveDistance / currentSpeed : brakingTime;
    double driveTime = Math.max(brakingTime, cruiseTime);

    // Rotation speed = angle / time.
    double timeBasedOmega = driveTime > 0 ? 2.0 * absAngle / driveTime : HARDWARE_MAX_OMEGA;

    // Take the smallest limit, with the right sign.
    return Math.copySign(min(maxStoppingOmega, timeBasedOmega, HARDWARE_MAX_OMEGA), angleError);
  }

  /**
   * Picks a target velocity that will brake the robot to a stop at the goal. Calculates each axis
   * (x, y) independently and shares the friction budget between them based on which axis needs
   * more.
   */
  public static Translation2d calculatePerAxisBrakingVelocity(
      Translation2d toGoal,
      Translation2d currentVelocity,
      double brakingReactionTime,
      double targetOmega,
      double angleError) {

    double distance = toGoal.getNorm();
    if (distance < EPSILON) {
      return new Translation2d();
    }

    // How much linear accel we have to work with.
    double availableLinearAccel = calculateAvailableLinearAccel(targetOmega, angleError);

    double toGoalX = toGoal.getX();
    double toGoalY = toGoal.getY();
    double distanceX = Math.abs(toGoalX);
    double distanceY = Math.abs(toGoalY);
    double dirX = toGoalX / distance;
    double dirY = toGoalY / distance;

    double currentSpeedX = Math.abs(currentVelocity.getX());
    double currentSpeedY = Math.abs(currentVelocity.getY());

    // Share friction budget proportionally to how much each axis actually needs.

    // Step 1: optimistic targets assuming the full budget on each axis.
    double optTargetX =
        calculateAxisBrakingSpeed(
            distanceX, currentSpeedX, brakingReactionTime, availableLinearAccel);
    double optTargetY =
        calculateAxisBrakingSpeed(
            distanceY, currentSpeedY, brakingReactionTime, availableLinearAccel);

    // Step 2: measure how much velocity each axis needs to change.
    double demandX = Math.abs(optTargetX - currentSpeedX);
    double demandY = Math.abs(optTargetY - currentSpeedY);
    double totalDemand = Math.hypot(demandX, demandY);

    // Step 3: split the friction budget by demand.
    double accelX;
    double accelY;
    if (totalDemand < EPSILON) {
      // No demand - just split by direction to the goal.
      accelX = availableLinearAccel * Math.abs(dirX);
      accelY = availableLinearAccel * Math.abs(dirY);
    } else {
      accelX = availableLinearAccel * (demandX / totalDemand);
      accelY = availableLinearAccel * (demandY / totalDemand);
    }

    // Step 4: redo the target speeds with the split budget.
    double targetSpeedX =
        calculateAxisBrakingSpeed(distanceX, currentSpeedX, brakingReactionTime, accelX);
    double targetSpeedY =
        calculateAxisBrakingSpeed(distanceY, currentSpeedY, brakingReactionTime, accelY);

    // Final velocity vector pointing toward the goal.
    return new Translation2d(
        Math.copySign(targetSpeedX, toGoalX), Math.copySign(targetSpeedY, toGoalY));
  }

  /**
   * How fast we can go right now and still brake to a stop by the time we reach the target.
   * Accounts for the friction budget shared with rotation, plus a small lag time before braking
   * actually starts.
   */
  public static double calculateBrakingTargetSpeed(
      double distance,
      double currentSpeed,
      double brakingReactionTime,
      double targetOmega,
      double angleError) {

    double availableLinearAccel = calculateAvailableLinearAccel(targetOmega, angleError);
    return calculateAxisBrakingSpeed(
        distance, currentSpeed, brakingReactionTime, availableLinearAccel);
  }

  /**
   * Braking math for one axis. Uses {@code v² = 2*a*d} to find the max speed that lets us stop
   * after traveling distance.
   */
  private static double calculateAxisBrakingSpeed(
      double distance, double currentSpeed, double reactionTime, double availableAccel) {

    double bufferedDistance = Math.max(0, distance - currentSpeed * reactionTime);
    double bufferedTargetSpeed = Math.sqrt(2.0 * availableAccel * bufferedDistance);

    // If we're slowing down, use the buffered distance. If speeding up, use full distance.
    if (bufferedTargetSpeed < currentSpeed) {
      return Math.min(bufferedTargetSpeed, AccelerationLimiter.MAX_VELOCITY);
    } else {
      return Math.min(Math.sqrt(2.0 * availableAccel * distance), AccelerationLimiter.MAX_VELOCITY);
    }
  }

  /** Smallest of three values. */
  private static double min(double a, double b, double c) {
    return Math.min(a, Math.min(b, c));
  }
}
