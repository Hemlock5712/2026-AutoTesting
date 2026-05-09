package frc.robot.utils.path;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Decides what direction the robot should face during path following.
 *
 * <p>By default the path's own heading is used. This interface lets you override that with custom
 * logic (face a target, hold a heading, etc.).
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

  /** Robot faces the direction of travel. */
  static RotationSupplier faceForward() {
    return (robotPose, pathS, pathTangent) -> Math.atan2(pathTangent.getY(), pathTangent.getX());
  }

  /** Robot aims at a fixed point on the field (e.g., the goal) the whole time. */
  static RotationSupplier facePoint(Translation2d target) {
    return (robotPose, pathS, pathTangent) -> {
      Translation2d toTarget = target.minus(robotPose.getTranslation());
      return Math.atan2(toTarget.getY(), toTarget.getX());
    };
  }

  /** Robot keeps the same heading throughout the path. */
  static RotationSupplier holdHeading(Rotation2d heading) {
    return (robotPose, pathS, pathTangent) -> heading.getRadians();
  }
}
