package frc.robot.utils.path;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Decides what direction the robot should face during path following.
 *
 * <p>By default {@link frc.robot.commands.FollowPath} uses the path's own heading. Pass a lambda
 * to {@link frc.robot.commands.FollowPath#withRotationSupplier} to override:
 *
 * <pre>{@code
 * // Face a fixed point on the field
 * follower.withRotationSupplier((pose, s, tangent) -> {
 *   Translation2d toGoal = GOAL.minus(pose.getTranslation());
 *   return Math.atan2(toGoal.getY(), toGoal.getX());
 * });
 * }</pre>
 */
@FunctionalInterface
public interface RotationSupplier {

  /**
   * Returns the heading the robot should face right now, in radians.
   *
   * @param robotPose Where the robot is and which way it's facing
   * @param pathS How far along the path the robot is (m)
   * @param pathTangent Unit vector pointing along the path at the robot's location
   */
  double getTargetHeading(Pose2d robotPose, double pathS, Translation2d pathTangent);
}
