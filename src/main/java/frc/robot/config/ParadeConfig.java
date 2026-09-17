package frc.robot.config;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

/** Central configuration for operating the robot on the parade trailer. */
public final class ParadeConfig {
  /** Set false to restore the normal competition drive, vision, and Hub configuration. */
  public static final boolean ENABLED = true;

  /** Limelight pipeline containing the parade-specific two-tag field map. */
  public static final int LIMELIGHT_PIPELINE = 2;

  /** Conservative motion limits for the small trailer. */
  public static final double MAX_LINEAR_SPEED_METERS_PER_SECOND = 1.0;

  public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.toRadians(90.0);
  public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 1.0;

  /** Both the AprilTags and the top of the modified Hub are 24 inches lower. */
  public static final double HUB_HEIGHT_OFFSET_METERS = Inches.of(24.0).in(Meters);

  /**
   * MegaTag2 consumes the supplied gyro heading and therefore cannot independently correct a gyro
   * that is turning with the trailer. Parade mode always uses MegaTag1 so visible tags can update
   * the estimated heading.
   */
  public static final boolean FORCE_MEGATAG1 = true;

  /** Five times more heading trust than the competition vision profile. */
  public static final double VISION_ROTATION_STD_DEV_COEFFICIENT = 0.3;

  private ParadeConfig() {}
}
