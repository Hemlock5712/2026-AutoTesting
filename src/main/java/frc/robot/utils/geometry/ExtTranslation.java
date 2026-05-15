package frc.robot.utils.geometry;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

/**
 * A container for {@link Translation2d} objects that dynamically returns flipped variants of the
 * original translation based on the robot's current alliance via {@link ExtTranslation#get()}.
 */
public final class ExtTranslation extends ExtGeometry<Translation2d> {

  /**
   * Creates an extended {@link Translation2d}.
   *
   * @param x The blue origin relative x component of the translation.
   * @param y The blue origin relative y component of the translation.
   */
  public ExtTranslation(double x, double y) {
    this(new Translation2d(x, y));
  }

  /**
   * Creates an extended {@link Translation2d}.
   *
   * @param x The blue origin relative x component of the translation.
   * @param y The blue origin relative y component of the translation.
   */
  public ExtTranslation(Distance x, Distance y) {
    this(new Translation2d(x, y));
  }

  /**
   * Creates an extended {@link Translation2d}.
   *
   * @param distance The distance from the origin to the end of the translation.
   * @param angle The blue origin relative angle between the x-axis and the translation vector.
   */
  public ExtTranslation(double distance, Rotation2d angle) {
    this(new Translation2d(distance, angle));
  }

  /**
   * Creates an extended {@link Translation2d}.
   *
   * @param distance The distance from the origin to the end of the translation.
   * @param angle The blue origin relative angle between the x-axis and the translation vector.
   */
  public ExtTranslation(Distance distance, Rotation2d angle) {
    this(new Translation2d(distance.in(Meters), angle));
  }

  /**
   * Creates an extended {@link Translation2d}.
   *
   * @param translation The blue origin relative translation.
   */
  public ExtTranslation(Translation2d translation) {
    super(translation);
  }

  @Override
  protected void set(Translation2d newValue) {
    original = newValue;
    overWidth = Flips.overWidth(newValue);
    overLength = Flips.overLength(newValue);
    overDiagonal = Flips.overDiagonal(newValue);
  }
}
