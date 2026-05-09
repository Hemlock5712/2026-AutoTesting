package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;

/**
 * Sim vision IO that reports the simulated robot's true pose every tick. Backed by maple-sim's
 * physics ground truth, so this fills the role a real Limelight would: an external pose source.
 *
 * <p>Std-dev tuning still has a measurable effect because we report a fixed synthetic tag count and
 * distance; tweaking those values in {@code Vision.computeVisionStdDev} changes how aggressively
 * the estimator pulls toward this measurement.
 */
public class VisionIOSim implements VisionIO {

  private static final int SYNTHETIC_TAG_COUNT = 2;
  private static final double SYNTHETIC_AVG_TAG_DISTANCE_M = 2.0;
  // Pretend to be a MegaTag2 measurement so std-dev tuning behaves the same in sim and real.
  private static final boolean SYNTHETIC_IS_MEGATAG2 = true;

  private final String name;
  private final Supplier<Pose2d> truthPoseSupplier;

  public VisionIOSim(String name, Supplier<Pose2d> truthPoseSupplier) {
    this.name = name;
    this.truthPoseSupplier = truthPoseSupplier;
  }

  @Override
  public String getName() {
    return name;
  }

  @Override
  public void updateInputs(VisionInputsAutoLogged inputs) {
    inputs.newFrame = true;
    inputs.hasObservation = true;
    inputs.latestPose = truthPoseSupplier.get();
    inputs.latestTimestampSeconds = Timer.getFPGATimestamp();
    inputs.tagCount = SYNTHETIC_TAG_COUNT;
    inputs.avgTagDistance = SYNTHETIC_AVG_TAG_DISTANCE_M;
    inputs.maxAmbiguity = 0.05;
    inputs.isMegaTag2 = SYNTHETIC_IS_MEGATAG2;
  }
}
