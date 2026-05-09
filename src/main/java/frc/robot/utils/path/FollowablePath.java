package frc.robot.utils.path;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * A path the robot can follow. Indexed by distance s along the path (in meters).
 *
 * <p>Currently implemented by {@link ArcLengthTrajectory} (Choreo trajectories). Used by {@link
 * frc.robot.commands.FollowPath}.
 */
public interface FollowablePath {

  /** Position at distance s along the path. */
  Translation2d getPoint(double s);

  /** Unit vector pointing in the direction of travel at distance s. */
  Translation2d getTangent(double s);

  /** How tightly the path is turning at distance s. Positive = left turn. */
  double getCurvature(double s);

  /** Target speed at distance s, in m/s. */
  double getVelocity(double s);

  /** Target robot heading at distance s. */
  Rotation2d getHeading(double s);

  /** Total path length in meters. */
  double getTotalLength();

  /**
   * Finds the closest point on the path to a given location, only searching between sMin and sMax.
   * Limiting the range prevents the robot from "snapping" to a far-away part of the path if the
   * path crosses itself or the robot gets bumped.
   */
  ProjectionResult getClosestPointInRange(Translation2d point, double sMin, double sMax);
}
