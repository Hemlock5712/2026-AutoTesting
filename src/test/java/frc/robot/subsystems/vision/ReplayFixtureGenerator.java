package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.utils.FieldInfo;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Base64;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.PnpResult;
import org.photonvision.targeting.TargetCorner;

/**
 * Generates a synthetic PhotonVision JSON-export fixture under {@code replay-fixtures/} so an
 * AdvantageKit REPLAY session can be driven against a known vision pose stream without needing a
 * real coprocessor recording.
 *
 * <p>Run with: {@code ./gradlew test --tests frc.robot.subsystems.vision.ReplayFixtureGenerator}.
 *
 * <p>Output layout matches what {@code JsonResultExporter} produces in PhotonVision:
 *
 * <pre>{@code
 * replay-fixtures/photon-fl-demo/
 *   results/
 *     synthetic.jsonl
 * }</pre>
 *
 * <p>The fixture stages 25 frames at 100ms intervals starting at {@code capture_ns = 3_000_000_000}
 * (i.e. t = 3s of the wpilog timeline). Each frame's multi-tag PNP result places the camera ~2m in
 * front of tag 7, so the implied robot pose lands cleanly inside the field bounds and the user's
 * filter (avg distance &lt; 5.5m, ambiguity &lt; 0.3, on-field) accepts every entry.
 */
public class ReplayFixtureGenerator {

  private static final Path FIXTURE_DIR = Path.of("replay-fixtures", "photon-fl-demo", "results");

  /**
   * Two fixtures with different "tunings": tuneA places the camera 2m from tag 7, tuneB places it
   * 2.5m from tag 8. AdvantageScope renders both as distinct dots — the visual proof that two
   * PhotonVision configurations can be replayed in parallel against the same wpilog.
   */
  private enum Variant {
    TUNE_A("tuneA.jsonl", 7, 2.0),
    TUNE_B("tuneB.jsonl", 8, 2.5);

    final String filename;
    final int tagId;
    final double cameraDistanceMeters;

    Variant(String filename, int tagId, double cameraDistanceMeters) {
      this.filename = filename;
      this.tagId = tagId;
      this.cameraDistanceMeters = cameraDistanceMeters;
    }
  }

  private static final String CAMERA_NAME = "photon-fl";
  private static final String RECORDING_NAME = "demo-recording";

  // Match VisionConstants.PHOTON_CAMERA_TRANSFORMS[0] (photon-fl): front-left, 30° yaw out.
  private static final Transform3d ROBOT_TO_CAMERA =
      new Transform3d(
          new Translation3d(0.254, 0.254, 0.229),
          new Rotation3d(0.0, Math.toRadians(-15.0), Math.toRadians(30.0)));

  // Synthetic frame schedule: 25 frames at 100ms intervals starting at t = 3s.
  private static final int FRAME_COUNT = 25;
  private static final long FIRST_CAPTURE_NS = 3_000_000_000L;
  private static final long FRAME_PERIOD_NS = 100_000_000L; // 10 fps

  private static final ObjectMapper JSON = new ObjectMapper();

  @BeforeAll
  static void pinLayout() {
    // Match the layout production REPLAY mode lazy-initializes to (FieldInfo.kDefaultField).
    // If this drifts from production, tag positions will disagree and the filter will reject
    // every fixture entry as "off-field" or "too far from tag."
    FieldInfo.setLayout(AprilTagFields.kDefaultField);
  }

  @Test
  public void writesBothTuningFixtures() throws IOException {
    Files.createDirectories(FIXTURE_DIR);
    for (Variant v : Variant.values()) {
      Path jsonlPath = FIXTURE_DIR.resolve(v.filename);

      Pose3d tagPose =
          FieldInfo.aprilTags()
              .getTagPose(v.tagId)
              .orElseThrow(
                  () ->
                      new IllegalStateException(
                          "Tag " + v.tagId + " missing from field layout for " + v.name()));

      // Camera staged v.cameraDistanceMeters along the tag's outward normal, so the camera looks
      // straight back at the tag's face. The "best" Transform3d in MultiTargetPNPResult is the
      // field-to-camera transform — PhotonPoseEstimator.estimateCoprocMultiTagPose multiplies
      // by robotToCamera.inverse() to land on a robot pose in field frame.
      Transform3d tagFaceOffset =
          new Transform3d(new Translation3d(v.cameraDistanceMeters, 0.0, 0.0), Rotation3d.kZero);
      Pose3d cameraInField = tagPose.transformBy(tagFaceOffset);
      Transform3d fieldToCamera =
          new Transform3d(cameraInField.getTranslation(), cameraInField.getRotation());

      // Sanity: implied robot pose stays on-field. If this assertion fails the fixture would be
      // filter-rejected at replay time, masking real wiring bugs as "no vision updates."
      Pose3d impliedRobot = cameraInField.plus(ROBOT_TO_CAMERA.inverse());
      assertTrue(
          impliedRobot.getX() > 0.0
              && impliedRobot.getX() < FieldInfo.lengthMeters()
              && impliedRobot.getY() > 0.0
              && impliedRobot.getY() < FieldInfo.widthMeters(),
          v.name() + " implied robot pose " + impliedRobot.toPose2d() + " must be on field");

      var lines = new ArrayList<String>();
      lines.add(headerLine(v));
      for (int i = 0; i < FRAME_COUNT; i++) {
        lines.add(resultLine(i, v.tagId, fieldToCamera));
      }

      Files.writeString(jsonlPath, String.join("\n", lines) + "\n", StandardCharsets.UTF_8);
      List<String> readBack = Files.readAllLines(jsonlPath);
      assertEquals(FRAME_COUNT + 1, readBack.size(), v.name() + ": header + N result lines");
      System.out.println(
          "Wrote "
              + v.name()
              + " fixture: "
              + jsonlPath.toAbsolutePath()
              + " (implied robot "
              + impliedRobot.toPose2d()
              + ")");
    }
  }

  private static String headerLine(Variant v) {
    ObjectNode header = JSON.createObjectNode();
    header.put("schema_version", 1);
    header.put("camera_unique_name", CAMERA_NAME);
    header.put("recording_name", RECORDING_NAME);
    header.put("pipeline_type", "AprilTag");
    // Distinct hashes per tuning so a downstream consumer that groups by hash sees two streams.
    header.put("pipeline_hash", v.name().toLowerCase());
    header.put("tss_active_at_record", true);
    header.put("tss_offset_at_record_ns", 0L);
    return header.toString();
  }

  private static String resultLine(int seq, int targetTagId, Transform3d fieldToCamera) {
    long captureNs = FIRST_CAPTURE_NS + seq * FRAME_PERIOD_NS;
    long captureMicros = captureNs / 1_000L;

    // One target referencing the real tag so averageTagDistance can resolve a field pose for it.
    var corners =
        List.of(
            new TargetCorner(0, 0),
            new TargetCorner(10, 0),
            new TargetCorner(10, 10),
            new TargetCorner(0, 10));
    var target =
        new PhotonTrackedTarget(
            0.0,
            0.0,
            10.0,
            0.0,
            targetTagId,
            -1,
            -1f,
            new Transform3d(new Translation3d(2.0, 0.0, 0.0), Rotation3d.kZero),
            new Transform3d(new Translation3d(2.0, 0.0, 0.0), Rotation3d.kZero),
            0.05,
            corners,
            corners);

    var multitag =
        Optional.of(
            new MultiTargetPNPResult(
                new PnpResult(fieldToCamera, 0.05), List.of((short) targetTagId)));

    var result =
        new PhotonPipelineResult(
            seq, captureMicros, captureMicros + 1000L, 0L, List.of(target), multitag);

    var packet = new Packet(1024);
    PhotonPipelineResult.photonStruct.pack(packet, result);
    String b64 = Base64.getEncoder().encodeToString(packet.getWrittenDataCopy());

    ObjectNode line = JSON.createObjectNode();
    line.put("capture_ns", captureNs);
    line.put("seq", seq);
    line.put("packet_b64", b64);
    return line.toString();
  }
}
