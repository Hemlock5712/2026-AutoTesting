package frc.robot.subsystems.vision;

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
 * Dumb passthrough for a PhotonVision coprocessor camera. Picks the right estimator method
 * (multi-tag PNP, single-tag trig-solve, or lowest-ambiguity fallback while disabled) and
 * surfaces raw metadata. {@link Vision} owns all quality filtering.
 *
 * <p>PNP = Perspective-N-Point, the geometric algorithm that recovers a camera's pose from N
 * known 3D points seen as 2D image coordinates. "Multi-tag PNP" pools points from several
 * AprilTags for a strong solve; "trig-solve" uses a single tag plus our gyro heading (the
 * MegaTag2 analog: rotation comes from the gyro, only translation is solved visually).
 *
 * <p>{@code isMegaTag2 = true} signals "rotation came from our gyro" — the trig-solve case.
 */
public class VisionIOPhotonVision implements VisionIO {

  private final String name;
  private final PhotonCamera camera;
  private final PhotonPoseEstimator poseEstimator;

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

    for (PhotonPipelineResult result : results) {
      if (!result.hasTargets()) continue;

      Optional<EstimatedRobotPose> estimateOpt;
      boolean isTrigSolve;
      if (result.getMultiTagResult().isPresent()) {
        estimateOpt = poseEstimator.estimateCoprocMultiTagPose(result);
        isTrigSolve = false;
      } else if (!DriverStation.isDisabled()) {
        // Single-tag: use trig-solve (MT2 analog) — needs gyro heading data buffered.
        estimateOpt = poseEstimator.estimatePnpDistanceTrigSolvePose(result);
        isTrigSolve = true;
      } else {
        // Disabled: no gyro heading has been buffered yet, so trig-solve has nothing to fuse.
        estimateOpt = poseEstimator.estimateLowestAmbiguityPose(result);
        isTrigSolve = false;
      }

      if (estimateOpt.isEmpty()) continue;
      // Keep the latest valid frame in the batch (results are time-ordered oldest→newest).
      bestEstimate = estimateOpt.get();
      bestIsTrigSolve = isTrigSolve;
    }

    if (bestEstimate == null) return;

    inputs.hasObservation = true;
    inputs.latestPose = bestEstimate.estimatedPose.toPose2d();
    inputs.latestTimestampSeconds = bestEstimate.timestampSeconds;
    inputs.tagCount = bestEstimate.targetsUsed.size();
    inputs.tagIds = tagIds(bestEstimate.targetsUsed);
    inputs.avgTagDistance = averageTagDistance(bestEstimate);
    inputs.maxAmbiguity = maxAmbiguity(bestEstimate.targetsUsed);
    inputs.isMegaTag2 = bestIsTrigSolve;
  }

  private static int[] tagIds(List<PhotonTrackedTarget> targets) {
    int[] ids = new int[targets.size()];
    for (int i = 0; i < ids.length; i++) ids[i] = targets.get(i).getFiducialId();
    return ids;
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
}
