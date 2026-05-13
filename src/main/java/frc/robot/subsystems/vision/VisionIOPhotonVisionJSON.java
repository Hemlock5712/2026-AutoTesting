package frc.robot.subsystems.vision;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.utils.FieldInfo;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Base64;
import java.util.List;
import java.util.Optional;
import java.util.function.LongSupplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Drop-in replacement for {@link VisionIOPhotonVision} that reads pre-recorded pipeline results
 * from a PhotonVision JSON-export file instead of a live coprocessor camera. Used during AKit
 * replay to swap in re-tuned vision data over the match wpilog's drive/sensor stream — see {@code
 * docs/replay.md} in the PhotonVision repo for the recording / export workflow.
 *
 * <p><b>Time alignment.</b> The JSON file's {@code capture_ns} values are in the coprocessor's
 * local {@code wpi::nt::Now} basis; the {@code tss_offset_at_record_ns} header field carries the
 * record-time TSS offset (= delta to the robot's FPGA clock). We add it once at construction so the
 * cursor advances against {@code Timer.getFPGATimestamp()} — same time base AKit's replay driver
 * uses to step the loop.
 *
 * <p><b>Refusal contract.</b> Construction throws if the header reports TSS as inactive or unknown
 * at record time: without a valid offset there's no way to align the JSON with an AKit wpilog, and
 * silently producing wrong poses is worse than refusing to load.
 *
 * <p>Filter and estimation logic (multi-tag vs trig-solve vs lowest-ambiguity fallback, distance /
 * ambiguity / yaw-rate / field-bounds gates) mirrors {@link VisionIOPhotonVision} so a JSON-
 * sourced session produces the same pose stream the live coprocessor would have given the same
 * underlying frames.
 */
public class VisionIOPhotonVisionJSON implements VisionIO {

  // --- Keep in lockstep with VisionIOPhotonVision's gate constants. ---
  private static final double MAX_AMBIGUITY = 0.3;
  private static final double FIELD_BORDER_MARGIN_METERS = 0.5;
  private static final double MAX_TAG_DISTANCE_METERS = 5.5;
  private static final double MAX_ANGULAR_VELOCITY_MULTITAG_DEG_PER_SEC = 360;
  private static final double MAX_YAW_RATE_FOR_TRIG_SOLVE_DEG_PER_SEC = 200;

  private final String name;
  private final PhotonPoseEstimator poseEstimator;
  private final List<JsonEntry> entries;
  private final LongSupplier nowFpgaNsSupplier;
  private int cursor = 0;
  private double cachedYawRateDegPerSec = 0.0;

  /**
   * @param jsonPath path to a {@code <pipeline-hash>.jsonl} file produced by PhotonVision's {@code
   *     JsonResultExporter}.
   */
  public VisionIOPhotonVisionJSON(String name, Transform3d robotToCamera, Path jsonPath)
      throws IOException {
    // Use AdvantageKit's logger timestamp (microseconds) so REPLAY mode advances the cursor in
    // the wpilog's simulated time base, not wall-clock. setUseTiming(false) makes
    // Timer.getFPGATimestamp() return wall-clock time which doesn't line up with capture_ns.
    this(name, robotToCamera, jsonPath, () -> Logger.getTimestamp() * 1_000L);
  }

  /** Package-private clock-injecting ctor for tests. */
  VisionIOPhotonVisionJSON(
      String name, Transform3d robotToCamera, Path jsonPath, LongSupplier nowFpgaNsSupplier)
      throws IOException {
    this.name = name;
    this.poseEstimator = new PhotonPoseEstimator(FieldInfo.aprilTags(), robotToCamera);
    this.entries = parseJsonl(jsonPath);
    this.nowFpgaNsSupplier = nowFpgaNsSupplier;
  }

  @Override
  public String getName() {
    return name;
  }

  @Override
  public void setRobotOrientation(double yawDegrees, double yawRateDegPerSec) {
    cachedYawRateDegPerSec = yawRateDegPerSec;
    poseEstimator.addHeadingData(Timer.getFPGATimestamp(), Rotation2d.fromDegrees(yawDegrees));
  }

  @Override
  public void updateInputs(VisionInputsAutoLogged inputs) {
    long nowFpgaNs = nowFpgaNsSupplier.getAsLong();

    List<PhotonPipelineResult> drained = new ArrayList<>();
    while (cursor < entries.size() && entries.get(cursor).captureNsAligned <= nowFpgaNs) {
      drained.add(decode(entries.get(cursor)));
      cursor++;
    }
    inputs.newFrame = !drained.isEmpty();
    inputs.hasObservation = false;

    EstimatedRobotPose bestEstimate = null;
    boolean bestIsTrigSolve = false;
    double bestAvgDistance = 0.0;
    double bestMaxAmbiguity = 0.0;

    for (PhotonPipelineResult result : drained) {
      if (!result.hasTargets()) continue;

      Optional<EstimatedRobotPose> estimateOpt;
      boolean isTrigSolve;
      if (result.getMultiTagResult().isPresent()) {
        estimateOpt = poseEstimator.estimateCoprocMultiTagPose(result);
        isTrigSolve = false;
      } else if (!DriverStation.isDisabled()) {
        if (Math.abs(cachedYawRateDegPerSec) > MAX_YAW_RATE_FOR_TRIG_SOLVE_DEG_PER_SEC) continue;
        estimateOpt = poseEstimator.estimatePnpDistanceTrigSolvePose(result);
        isTrigSolve = true;
      } else {
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
    inputs.isMegaTag2 = bestIsTrigSolve;
  }

  // -------- JSON parsing --------

  private record JsonEntry(long captureNsAligned, long seq, String packetB64) {}

  /**
   * Parse the JSON header + result lines, refusing the file if the header reports TSS as unknown /
   * inactive (we'd have no offset to align with the wpilog).
   */
  private static List<JsonEntry> parseJsonl(Path jsonPath) throws IOException {
    List<String> lines = Files.readAllLines(jsonPath);
    if (lines.isEmpty()) throw new IOException("JSON replay file is empty: " + jsonPath);

    ObjectMapper mapper = new ObjectMapper();
    JsonNode header = mapper.readTree(lines.get(0));
    int schemaVersion = header.path("schema_version").asInt(-1);
    if (schemaVersion != 1) {
      throw new IOException(
          "Unsupported JSON replay schema_version " + schemaVersion + " in " + jsonPath);
    }
    JsonNode tssActiveNode = header.get("tss_active_at_record");
    JsonNode tssOffsetNode = header.get("tss_offset_at_record_ns");
    if (tssActiveNode == null || tssActiveNode.isNull() || !tssActiveNode.asBoolean()) {
      throw new IOException(
          "JSON replay refused: tss_active_at_record is not true in "
              + jsonPath
              + " — capture timestamps can't be aligned with the wpilog");
    }
    if (tssOffsetNode == null || tssOffsetNode.isNull()) {
      throw new IOException("JSON replay refused: tss_offset_at_record_ns missing in " + jsonPath);
    }
    long tssOffsetNs = tssOffsetNode.asLong();

    List<JsonEntry> entries = new ArrayList<>(lines.size());
    for (int i = 1; i < lines.size(); i++) {
      JsonNode line = mapper.readTree(lines.get(i));
      long captureNs = line.get("capture_ns").asLong();
      long seq = line.get("seq").asLong();
      String b64 = line.get("packet_b64").asText();
      entries.add(new JsonEntry(captureNs + tssOffsetNs, seq, b64));
    }
    // Defensive: exporter writes entries in capture_ns order, but a hand-edited or corrupted file
    // could violate that. We rely on monotonic order for the cursor advance.
    entries.sort((a, b) -> Long.compare(a.captureNsAligned, b.captureNsAligned));
    return entries;
  }

  private static PhotonPipelineResult decode(JsonEntry entry) {
    byte[] bytes = Base64.getDecoder().decode(entry.packetB64);
    return PhotonPipelineResult.photonStruct.unpack(new Packet(bytes));
  }

  // -------- Filter helpers (copies of VisionIOPhotonVision's privates) --------

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
