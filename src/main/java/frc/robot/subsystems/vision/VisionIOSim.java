package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

/**
 * Fake vision IO used in sim. Always reports the robot at a fixed point on the field.
 *
 * <p>We use a fixed pose (not "truth + noise") to avoid feedback - if we read drive.getPose() and
 * fed it back, vision would just confirm whatever the estimator already thinks. With a fixed
 * reference, you can clearly see how the estimator weights vision vs odometry by tweaking the
 * std-dev values.
 */
public class VisionIOSim implements VisionIO {

  /** The fake "observed" robot pose. */
  private static final double FIXED_POSE_X = 8.0;

  private static final double FIXED_POSE_Y = 4.0;
  private static final double FIXED_POSE_THETA_RAD = 0.0;

  private static final int SYNTHETIC_TAG_COUNT = 2;
  private static final double SYNTHETIC_AVG_TAG_DISTANCE_M = 2.0;
  // Pretend to be a MegaTag2 measurement so std-dev tuning behaves the same in sim and real.
  private static final boolean SYNTHETIC_IS_MEGATAG2 = true;

  private final String name;

  public VisionIOSim(String name) {
    this.name = name;
  }

  @Override
  public String getName() {
    return name;
  }

  @Override
  public void updateInputs(VisionInputsAutoLogged inputs) {
    inputs.newFrame = true;
    inputs.hasObservation = true;
    inputs.latestPose =
        new Pose2d(
            new Translation2d(FIXED_POSE_X, FIXED_POSE_Y), new Rotation2d(FIXED_POSE_THETA_RAD));
    inputs.latestTimestampSeconds = Timer.getFPGATimestamp();
    inputs.tagCount = SYNTHETIC_TAG_COUNT;
    inputs.avgTagDistance = SYNTHETIC_AVG_TAG_DISTANCE_M;
    inputs.maxAmbiguity = 0.05;
    inputs.isMegaTag2 = SYNTHETIC_IS_MEGATAG2;
  }
}
