package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.PoseEstimate;
import frc.robot.utils.LimelightHelpers.RawFiducial;

/**
 * Dumb passthrough for a Limelight. Picks the right estimator method (MT1 for multi-tag, MT2 for
 * single-tag) and surfaces raw metadata. {@link Vision} owns all quality filtering.
 */
public class VisionIOLimelight implements VisionIO {

  private final String name;
  private double lastHeartbeat = -1.0;

  public VisionIOLimelight(String name) {
    this.name = name;
  }

  @Override
  public String getName() {
    return name;
  }

  @Override
  public void setRobotOrientation(double yawDegrees, double yawRateDegPerSec) {
    // Don't flush yet - Vision.periodic flushes once after all cameras have written, which is
    // way faster than flushing per-camera.
    LimelightHelpers.SetRobotOrientation_NoFlush(name, yawDegrees, yawRateDegPerSec, 0, 0, 0, 0);
  }

  @Override
  public void updateInputs(VisionInputsAutoLogged inputs) {
    double hb = LimelightHelpers.getHeartbeat(name);
    inputs.newFrame = hb != lastHeartbeat && hb > 0;
    lastHeartbeat = hb;

    inputs.hasObservation = false;

    PoseEstimate pe = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
    if (!LimelightHelpers.validPoseEstimate(pe)) return;

    // Single-tag MT1 is too unstable to use directly — fall back to the gyro-locked MT2 solve.
    // Skip while disabled: gyro orientation hasn't been pushed yet at boot, so MT2 has no
    // reference and would return garbage.
    if (pe.tagCount == 1 && !DriverStation.isDisabled()) {
      PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
      if (!LimelightHelpers.validPoseEstimate(mt2)) return;
      pe = mt2;
    }

    inputs.hasObservation = true;
    inputs.latestPose = pe.pose;
    inputs.latestTimestampSeconds = pe.timestampSeconds;
    inputs.tagCount = pe.tagCount;
    inputs.tagIds = tagIds(pe);
    inputs.avgTagDistance = pe.avgTagDist;
    inputs.maxAmbiguity = maxAmbiguity(pe);
    inputs.isMegaTag2 = pe.isMegaTag2;
  }

  private static int[] tagIds(PoseEstimate pe) {
    if (pe == null || pe.rawFiducials == null) return new int[0];
    int[] ids = new int[pe.rawFiducials.length];
    for (int i = 0; i < pe.rawFiducials.length; i++) ids[i] = pe.rawFiducials[i].id;
    return ids;
  }

  private static double maxAmbiguity(PoseEstimate pe) {
    if (pe == null || pe.rawFiducials == null) return 0.0;
    double max = 0.0;
    for (RawFiducial f : pe.rawFiducials) {
      if (f.ambiguity > max) max = f.ambiguity;
    }
    return max;
  }
}
