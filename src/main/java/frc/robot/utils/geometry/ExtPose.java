package frc.robot.utils.geometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

/**
 * A container for {@link Pose2d} objects that dynamically returns flipped variants of the original
 * pose based on the robot's current alliance via {@link ExtPose#get()}.
 */
public final class ExtPose extends ExtGeometry<Pose2d> {

  /**
   * Creates an extended {@link Pose2d}. All parameters are expected to be blue origin relative.
   *
   * @param x The x component of the translational component of the pose.
   * @param y The y component of the translational component of the pose.
   * @param rotation The rotational component of the pose.
   */
  public ExtPose(double x, double y, Rotation2d rotation) {
    this(new Pose2d(x, y, rotation));
  }

  public ExtPose(Distance x, Distance y, Rotation2d rotation) {
    this(new Pose2d(x, y, rotation));
  }

  /**
   * Creates an extended {@link Pose2d}. All parameters are expected to be blue origin relative.
   *
   * @param translation The translational component of the pose.
   * @param rotation The rotational component of the pose.
   */
  public ExtPose(Translation2d translation, Rotation2d rotation) {
    this(new Pose2d(translation, rotation));
  }

  /**
   * Creates an extended {@link Pose2d}.
   *
   * @param pose The blue origin relative pose.
   */
  public ExtPose(Pose2d pose) {
    super(pose);
  }

  @Override
  protected void set(Pose2d newValue) {
    original = newValue;
    overWidth = Flips.overWidth(newValue);
    overLength = Flips.overLength(newValue);
    overDiagonal = Flips.overDiagonal(newValue);
  }
}
