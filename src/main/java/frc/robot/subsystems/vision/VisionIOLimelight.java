package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.utils.FieldInfo;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.PoseEstimate;
import frc.robot.utils.LimelightHelpers.RawFiducial;

/**
 * Reads pose estimates from a Limelight camera. Throws out bad observations (too ambiguous, tags
 * too far, robot off the field, robot spinning too fast).
 */
public class VisionIOLimelight implements VisionIO {

  private static final double MAX_AMBIGUITY = 0.3;
  private static final double FIELD_BORDER_MARGIN_METERS = 0.5;
  private static final double MAX_TAG_DISTANCE_METERS = 5.5;
  private static final double MAX_ANGULAR_VELOCITY_MT1_DEG_PER_SEC = 360;
  private static final double MAX_YAW_RATE_FOR_MT2_RETRY_DEG_PER_SEC = 200;

  private final String name;
  private double cachedYawRateDegPerSec = 0.0;

  // The Limelight's heartbeat counter. We use it to detect when a new frame arrives.
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
    cachedYawRateDegPerSec = yawRateDegPerSec;
    // Don't flush yet - Vision.periodic flushes once after all cameras have written, which is
    // way faster than flushing per-camera.
    LimelightHelpers.SetRobotOrientation_NoFlush(name, yawDegrees, yawRateDegPerSec, 0, 0, 0, 0);
  }

  @Override
  public void updateInputs(VisionInputsAutoLogged inputs) {
    double hb = LimelightHelpers.getHeartbeat(name);
    inputs.newFrame = hb != lastHeartbeat && hb > 0;
    lastHeartbeat = hb;

    PoseEstimate pe = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
    inputs.hasObservation = false;

    if (!LimelightHelpers.validPoseEstimate(pe)) return;

    if (maxAmbiguity(pe) > MAX_AMBIGUITY) return;
    if (pe.avgTagDist > MAX_TAG_DISTANCE_METERS) return;

    if (pe.tagCount == 1 && !DriverStation.isDisabled()) {
      if (Math.abs(cachedYawRateDegPerSec) > MAX_YAW_RATE_FOR_MT2_RETRY_DEG_PER_SEC) return;
      PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
      if (!LimelightHelpers.validPoseEstimate(mt2)) return;
      // Re-check the same filters on the MegaTag2 result.
      if (maxAmbiguity(mt2) > MAX_AMBIGUITY) return;
      if (mt2.avgTagDist > MAX_TAG_DISTANCE_METERS) return;
      pe = mt2;
    }

    if (!pe.isMegaTag2 && Math.abs(cachedYawRateDegPerSec) > MAX_ANGULAR_VELOCITY_MT1_DEG_PER_SEC)
      return;
    if (!isPoseOnField(pe.pose)) return;

    inputs.hasObservation = true;
    inputs.latestPose = pe.pose;
    inputs.latestTimestampSeconds = pe.timestampSeconds;
    inputs.tagCount = pe.tagCount;
    inputs.avgTagDistance = pe.avgTagDist;
    inputs.maxAmbiguity = maxAmbiguity(pe);
    inputs.isMegaTag2 = pe.isMegaTag2;
  }

  private static double maxAmbiguity(PoseEstimate pe) {
    double max = 0.0;
    if (pe == null || pe.rawFiducials == null) return max;
    for (RawFiducial f : pe.rawFiducials) {
      if (f.ambiguity > max) max = f.ambiguity;
    }
    return max;
  }

  private static boolean isPoseOnField(Pose2d pose) {
    return pose.getX() >= -FIELD_BORDER_MARGIN_METERS
        && pose.getX() <= FieldInfo.lengthMeters() + FIELD_BORDER_MARGIN_METERS
        && pose.getY() >= -FIELD_BORDER_MARGIN_METERS
        && pose.getY() <= FieldInfo.widthMeters() + FIELD_BORDER_MARGIN_METERS;
  }
}
