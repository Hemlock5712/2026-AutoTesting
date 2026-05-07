package frc.robot.utils.path;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Contract between a path source and a path-following controller.
 *
 * <p>All queries are parameterized by arc-length {@code s} (meters along the path). Implementations
 * include {@link ArcLengthTrajectory} (re-parameterized Choreo trajectories) and any future dynamic
 * path generators (A*, RRT*, etc.).
 *
 * <p>Both {@link frc.robot.commands.FollowPath} and future MPC controllers consume this interface.
 */
public interface FollowablePath {

  /**
   * Returns the position at arc-length {@code s} along the path.
   *
   * @param s Arc length in meters, clamped to [0, totalLength]
   * @return Position on the path
   */
  Translation2d getPoint(double s);

  /**
   * Returns the unit tangent vector at arc-length {@code s}.
   *
   * @param s Arc length in meters, clamped to [0, totalLength]
   * @return Unit tangent vector (direction of travel)
   */
  Translation2d getTangent(double s);

  /**
   * Returns the signed curvature at arc-length {@code s}.
   *
   * @param s Arc length in meters, clamped to [0, totalLength]
   * @return Signed curvature in 1/meters (positive = turning left)
   */
  double getCurvature(double s);

  /**
   * Returns the target speed magnitude at arc-length {@code s}.
   *
   * <p>For Choreo trajectories this comes from the pre-optimized velocity profile. For dynamic
   * paths this may be computed from curvature constraints.
   *
   * @param s Arc length in meters, clamped to [0, totalLength]
   * @return Target speed in m/s
   */
  double getVelocity(double s);

  /**
   * Returns the target robot heading at arc-length {@code s}.
   *
   * <p>For Choreo trajectories this comes from the optimized heading profile. Heading can be
   * overridden via {@link frc.robot.commands.FollowPath#withRotationSupplier}.
   *
   * @param s Arc length in meters, clamped to [0, totalLength]
   * @return Target heading
   */
  Rotation2d getHeading(double s);

  /**
   * Returns the total arc length of the path in meters.
   *
   * @return Total path length
   */
  double getTotalLength();

  /**
   * Projects a point onto the path within a bounded arc-length range.
   *
   * <p>The bounded search prevents the projection from jumping to distant segments when the robot
   * is hit or at path crossings.
   *
   * @param point The point to project onto the path
   * @param sMin Minimum arc length to search (clamped to 0)
   * @param sMax Maximum arc length to search (clamped to totalLength)
   * @return Projection result with arc length, closest point, cross-track error, and tangent
   */
  ProjectionResult getClosestPointInRange(Translation2d point, double sMin, double sMax);
}
