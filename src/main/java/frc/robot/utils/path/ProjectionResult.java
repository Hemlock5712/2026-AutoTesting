package frc.robot.utils.path;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Tells us where the robot is in relation to the path.
 *
 * @param s Distance along the path of the closest point (m)
 * @param point Closest point on the path
 * @param crossTrackError How far off the path the robot is (positive = left of path direction)
 * @param tangent Direction the path points at the closest point
 */
public record ProjectionResult(
    double s, Translation2d point, double crossTrackError, Translation2d tangent) {}
