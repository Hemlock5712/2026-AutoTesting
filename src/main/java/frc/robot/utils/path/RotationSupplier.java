package frc.robot.utils.path;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Supplies the desired target heading during path following.
 *
 * <p>Unlike a simple {@code DoubleSupplier}, this interface receives path context (robot pose,
 * arc-length position, and path tangent) so rotation strategies can be path-aware.
 *
 * <p>For Choreo trajectories, the default heading comes from the trajectory itself via {@link
 * FollowablePath#getHeading}. This interface is used to override that default when needed.
 */
@FunctionalInterface
public interface RotationSupplier {

  /**
   * Returns the desired target heading for the robot.
   *
   * @param robotPose Current robot pose (position + heading)
   * @param pathS Current arc-length position on the path (meters)
   * @param pathTangent Unit tangent vector of the path at the projected point
   * @return Target heading in radians
   */
  double getTargetHeading(Pose2d robotPose, double pathS, Translation2d pathTangent);

  /** Aligns the robot heading with the path tangent direction (face the direction of travel). */
  static RotationSupplier faceForward() {
    return (robotPose, pathS, pathTangent) -> Math.atan2(pathTangent.getY(), pathTangent.getX());
  }

  /** Aims the robot at a fixed field point (e.g., the goal) while driving the path. */
  static RotationSupplier facePoint(Translation2d target) {
    return (robotPose, pathS, pathTangent) -> {
      Translation2d toTarget = target.minus(robotPose.getTranslation());
      return Math.atan2(toTarget.getY(), toTarget.getX());
    };
  }

  /** Maintains a fixed heading throughout the path. */
  static RotationSupplier holdHeading(Rotation2d heading) {
    return (robotPose, pathS, pathTangent) -> heading.getRadians();
  }
}
