package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.utils.FieldInfo;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Reads pose estimates from a PhotonVision coprocessor camera. Same filtering and selection logic
 * as {@link VisionIOLimelight}:
 *
 * <ul>
 *   <li>Multi-tag observations use coprocessor PNP (analogous to MegaTag1).
 *   <li>Single-tag observations use PNP_DISTANCE_TRIG_SOLVE from PhotonVision PR #1767, which is
 *       the single-tag analog of MegaTag2: it fuses our gyro heading with the tag's distance so the
 *       rotation comes from the gyro and only translation is solved visually.
 * </ul>
 */
public class VisionIOPhotonVision implements VisionIO {

  private static final double MAX_AMBIGUITY = 0.3;
  private static final double FIELD_BORDER_MARGIN_METERS = 0.5;
  private static final double MAX_TAG_DISTANCE_METERS = 5.5;
  private static final double MAX_ANGULAR_VELOCITY_MULTITAG_DEG_PER_SEC = 360;
  private static final double MAX_YAW_RATE_FOR_TRIG_SOLVE_DEG_PER_SEC = 200;

  private final String name;
  private final PhotonCamera camera;
  private final PhotonPoseEstimator poseEstimator;
  private double cachedYawRateDegPerSec = 0.0;

  public VisionIOPhotonVision(String name, Transform3d robotToCamera) {
    this.name = name;
    this.camera = new PhotonCamera(name);
    this.poseEstimator = new PhotonPoseEstimator(FieldInfo.aprilTags(), robotToCamera);
  }

  @Override
  public String getName() {
    return name;
  }

  /** Exposed so {@link VisionIOPhotonVisionSim} can register a sim feed for this camera. */
  protected PhotonCamera getCamera() {
    return camera;
  }

  @Override
  public void setRobotOrientation(double yawDegrees, double yawRateDegPerSec) {
    cachedYawRateDegPerSec = yawRateDegPerSec;
    // PNP_DISTANCE_TRIG_SOLVE consumes this for single-tag pose solving.
    poseEstimator.addHeadingData(Timer.getFPGATimestamp(), Rotation2d.fromDegrees(yawDegrees));
  }

  @Override
  public void updateInputs(VisionInputsAutoLogged inputs) {
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    inputs.newFrame = !results.isEmpty();
    inputs.hasObservation = false;

    EstimatedRobotPose bestEstimate = null;
    boolean bestIsTrigSolve = false;
    double bestAvgDistance = 0.0;
    double bestMaxAmbiguity = 0.0;

    for (PhotonPipelineResult result : results) {
      if (!result.hasTargets()) continue;

      Optional<EstimatedRobotPose> estimateOpt;
      boolean isTrigSolve;
      if (result.getMultiTagResult().isPresent()) {
        estimateOpt = poseEstimator.estimateCoprocMultiTagPose(result);
        isTrigSolve = false;
      } else if (!DriverStation.isDisabled()) {
        // Single-tag: use trig-solve (MT2 analog) — needs gyro heading + reasonable yaw rate.
        if (Math.abs(cachedYawRateDegPerSec) > MAX_YAW_RATE_FOR_TRIG_SOLVE_DEG_PER_SEC) continue;
        estimateOpt = poseEstimator.estimatePnpDistanceTrigSolvePose(result);
        isTrigSolve = true;
      } else {
        // Disabled: fall back to lowest-ambiguity single-tag solve so the estimator stays usable
        // before match start (no gyro heading data has been pushed yet).
        estimateOpt = poseEstimator.estimateLowestAmbiguityPose(result);
        isTrigSolve = false;
      }

      if (estimateOpt.isEmpty()) continue;
      EstimatedRobotPose estimate = estimateOpt.get();

      double avgDistance = averageTagDistance(estimate);
      if (avgDistance > MAX_TAG_DISTANCE_METERS) continue;

      double maxAmbiguity = maxAmbiguity(estimate.targetsUsed);
      if (maxAmbiguity > MAX_AMBIGUITY) continue;

      if (!isTrigSolve
          && Math.abs(cachedYawRateDegPerSec) > MAX_ANGULAR_VELOCITY_MULTITAG_DEG_PER_SEC) continue;

      if (!isPoseOnField(estimate.estimatedPose.toPose2d())) continue;

      // Keep the latest valid frame in the batch (results are time-ordered oldest→newest).
      bestEstimate = estimate;
      bestIsTrigSolve = isTrigSolve;
      bestAvgDistance = avgDistance;
      bestMaxAmbiguity = maxAmbiguity;
    }

    if (bestEstimate == null) return;

    inputs.hasObservation = true;
    inputs.latestPose = bestEstimate.estimatedPose.toPose2d();
    inputs.latestTimestampSeconds = bestEstimate.timestampSeconds;
    inputs.tagCount = bestEstimate.targetsUsed.size();
    inputs.avgTagDistance = bestAvgDistance;
    inputs.maxAmbiguity = bestMaxAmbiguity;
    // Reuse the existing flag — same semantic: rotation came from our gyro, so the estimator
    // should treat its theta component as untrustworthy.
    inputs.isMegaTag2 = bestIsTrigSolve;
  }

  private static double maxAmbiguity(List<PhotonTrackedTarget> targets) {
    double max = 0.0;
    for (PhotonTrackedTarget t : targets) {
      if (t.getPoseAmbiguity() > max) max = t.getPoseAmbiguity();
    }
    return max;
  }

  private double averageTagDistance(EstimatedRobotPose estimate) {
    if (estimate.targetsUsed.isEmpty()) return 0.0;
    Pose3d cameraPose = estimate.estimatedPose.plus(poseEstimator.getRobotToCameraTransform());
    double sum = 0.0;
    int counted = 0;
    for (PhotonTrackedTarget t : estimate.targetsUsed) {
      Optional<Pose3d> tagPoseOpt = poseEstimator.getFieldTags().getTagPose(t.getFiducialId());
      if (tagPoseOpt.isEmpty()) continue;
      sum += cameraPose.getTranslation().getDistance(tagPoseOpt.get().getTranslation());
      counted++;
    }
    return counted == 0 ? 0.0 : sum / counted;
  }

  private static boolean isPoseOnField(Pose2d pose) {
    return pose.getX() >= -FIELD_BORDER_MARGIN_METERS
        && pose.getX() <= FieldInfo.lengthMeters() + FIELD_BORDER_MARGIN_METERS
        && pose.getY() >= -FIELD_BORDER_MARGIN_METERS
        && pose.getY() <= FieldInfo.widthMeters() + FIELD_BORDER_MARGIN_METERS;
  }
}
