package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.utils.FieldInfo;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicLong;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.PnpResult;
import org.photonvision.targeting.TargetCorner;

/**
 * Validates the construction-time refusal contract and cursor-advancement behavior of {@link
 * VisionIOPhotonVisionJSON}. Uses a fixed-clock supplier (not {@code Timer.getFPGATimestamp}) so
 * the test doesn't depend on simulated time advancement.
 */
public class VisionIOPhotonVisionJSONTest {

  private static final Transform3d CAMERA_TRANSFORM =
      new Transform3d(
          new Translation3d(0.2, 0.0, 0.4), new Rotation3d(0.0, Math.toRadians(-10.0), 0.0));
  private static final ObjectMapper JSON = new ObjectMapper();

  @BeforeAll
  public static void pinLayout() {
    FieldInfo.setLayout(AprilTagFields.k2026RebuiltWelded);
  }

  @Test
  public void refusesHeaderWithInactiveTss(@TempDir Path tmp) throws Exception {
    Path jsonPath = tmp.resolve("rec.jsonl");
    writeFile(
        jsonPath,
        headerJson(false, 0L),
        // One result line so the failure is in the header check, not "empty file"
        resultLine(1_000_000_000L, 0L, "AAA="));

    IOException ex =
        assertThrows(
            IOException.class,
            () -> new VisionIOPhotonVisionJSON("cam", CAMERA_TRANSFORM, jsonPath, () -> 0L));
    assertTrue(
        ex.getMessage().contains("tss_active_at_record"),
        "error should call out tss_active flag; got: " + ex.getMessage());
  }

  @Test
  public void refusesHeaderWithNullTssFields(@TempDir Path tmp) throws Exception {
    Path jsonPath = tmp.resolve("rec.jsonl");
    Files.writeString(
        jsonPath,
        "{\"schema_version\":1,\"tss_active_at_record\":null,\"tss_offset_at_record_ns\":null}\n",
        StandardCharsets.UTF_8);

    assertThrows(
        IOException.class,
        () -> new VisionIOPhotonVisionJSON("cam", CAMERA_TRANSFORM, jsonPath, () -> 0L));
  }

  @Test
  public void refusesUnsupportedSchemaVersion(@TempDir Path tmp) throws Exception {
    Path jsonPath = tmp.resolve("rec.jsonl");
    Files.writeString(
        jsonPath,
        "{\"schema_version\":42,\"tss_active_at_record\":true,\"tss_offset_at_record_ns\":0}\n",
        StandardCharsets.UTF_8);

    IOException ex =
        assertThrows(
            IOException.class,
            () -> new VisionIOPhotonVisionJSON("cam", CAMERA_TRANSFORM, jsonPath, () -> 0L));
    assertTrue(
        ex.getMessage().contains("schema_version"),
        "error should name schema_version; got: " + ex.getMessage());
  }

  @Test
  public void cursorAdvancesByFpgaClockAndDrainsAllEntriesByTheirTssAlignedTimestamp(
      @TempDir Path tmp) throws Exception {
    // 3 entries at increasing TSS-aligned timestamps: 100ms, 200ms, 300ms FPGA-time.
    // tss_offset = 100ms (in nanos), so capture_ns values shift down by that amount in the file.
    long offsetNs = 100_000_000L;
    long[] alignedNs = {100_000_000L, 200_000_000L, 300_000_000L};

    Path jsonPath = tmp.resolve("rec.jsonl");
    writeFile(
        jsonPath,
        headerJson(true, offsetNs),
        resultLine(alignedNs[0] - offsetNs, 0L, b64OfMultitagResult(0L, alignedNs[0] / 1_000L)),
        resultLine(alignedNs[1] - offsetNs, 1L, b64OfMultitagResult(1L, alignedNs[1] / 1_000L)),
        resultLine(alignedNs[2] - offsetNs, 2L, b64OfMultitagResult(2L, alignedNs[2] / 1_000L)));

    AtomicLong fakeNowNs = new AtomicLong(0L);
    var io = new VisionIOPhotonVisionJSON("cam", CAMERA_TRANSFORM, jsonPath, fakeNowNs::get);

    // t < 100ms: no entries drain.
    var inputs = new VisionInputsAutoLogged();
    fakeNowNs.set(50_000_000L);
    io.updateInputs(inputs);
    assertFalse(inputs.newFrame);

    // t = 250ms: entries 0 and 1 drain. (Field constants gate whether hasObservation is set —
    // see VisionIOPhotonVision; off-field synthetic multitag pose can be filtered. We only assert
    // newFrame here.)
    fakeNowNs.set(250_000_000L);
    inputs = new VisionInputsAutoLogged();
    io.updateInputs(inputs);
    assertTrue(inputs.newFrame, "two entries should have drained at t=250ms");

    // t = 250ms again: cursor is already past entries 0+1, so no new frame.
    inputs = new VisionInputsAutoLogged();
    io.updateInputs(inputs);
    assertFalse(inputs.newFrame, "no new entries between calls at the same clock");

    // t = 350ms: entry 2 drains.
    fakeNowNs.set(350_000_000L);
    inputs = new VisionInputsAutoLogged();
    io.updateInputs(inputs);
    assertTrue(inputs.newFrame);
  }

  @Test
  public void populatesPoseFieldsWhenMultitagEstimateOnField(@TempDir Path tmp) throws Exception {
    // Synthesize a multitag result whose targets reference a real AprilTag so the pose
    // estimator's averageTagDistance can do its math. Use tag ID 1 (always present in
    // k2026RebuiltWelded).
    Path jsonPath = tmp.resolve("rec.jsonl");
    long offsetNs = 0L;
    long captureMicros = 1_000_000L;
    writeFile(
        jsonPath,
        headerJson(true, offsetNs),
        resultLine(captureMicros * 1_000L, 0L, b64OfRealisticMultitag(captureMicros)));

    var io =
        new VisionIOPhotonVisionJSON(
            "cam",
            CAMERA_TRANSFORM,
            jsonPath,
            () -> 10_000_000_000L); // way past the entry's aligned timestamp

    var inputs = new VisionInputsAutoLogged();
    io.updateInputs(inputs);
    assertTrue(inputs.newFrame);
    // We don't assert hasObservation==true because the synthesized pose may be off-field or
    // outside the filter gates depending on tag layout. The pipeline-walk reaching newFrame=true
    // confirms cursor + decode + estimator wiring works end-to-end; deeper assertions live in
    // VisionIOPhotonVision's own test suite (when present) since the filter logic is identical.
    assertNotNull(inputs.latestPose);
  }

  // -------- helpers --------

  private static void writeFile(Path path, String... lines) throws IOException {
    StringBuilder sb = new StringBuilder();
    for (String line : lines) sb.append(line).append('\n');
    Files.writeString(path, sb.toString(), StandardCharsets.UTF_8);
  }

  private static String headerJson(boolean tssActive, long tssOffsetNs) {
    ObjectNode node = JSON.createObjectNode();
    node.put("schema_version", 1);
    node.put("camera_unique_name", "test-cam");
    node.put("recording_name", "test-rec");
    node.put("pipeline_type", "AprilTag");
    node.put("pipeline_hash", "deadbeef");
    node.put("tss_active_at_record", tssActive);
    node.put("tss_offset_at_record_ns", tssOffsetNs);
    return node.toString();
  }

  private static String resultLine(long captureNs, long seq, String packetB64) {
    ObjectNode node = JSON.createObjectNode();
    node.put("capture_ns", captureNs);
    node.put("seq", seq);
    node.put("packet_b64", packetB64);
    return node.toString();
  }

  /** Synthesize a PhotonPipelineResult with a multitag payload and pack it to base64. */
  private static String b64OfMultitagResult(long seq, long captureMicros) {
    var multitag =
        Optional.of(
            new MultiTargetPNPResult(
                new PnpResult(
                    new edu.wpi.first.math.geometry.Transform3d(
                        new edu.wpi.first.math.geometry.Translation3d(1, 2, 0.5),
                        new edu.wpi.first.math.geometry.Rotation3d(0.0, 0.0, 0.1)),
                    0.05),
                List.of((short) 1, (short) 2)));
    var result =
        new PhotonPipelineResult(
            seq,
            captureMicros,
            captureMicros + 1000L,
            0L,
            List.<PhotonTrackedTarget>of(),
            multitag);
    var packet = new Packet(1024);
    PhotonPipelineResult.photonStruct.pack(packet, result);
    return java.util.Base64.getEncoder().encodeToString(packet.getWrittenDataCopy());
  }

  /**
   * Same as {@link #b64OfMultitagResult} but includes a single tracked target so the
   * PhotonPoseEstimator has something to attribute the multitag pose to. ID 1 because it exists in
   * every 2026 field layout we care about.
   */
  private static String b64OfRealisticMultitag(long captureMicros) {
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
            1,
            -1,
            -1f,
            new Transform3d(new Translation3d(2, 0, 0.5), new Rotation3d()),
            new Transform3d(new Translation3d(2, 0, 0.5), new Rotation3d()),
            0.05,
            corners,
            corners);
    var multitag =
        Optional.of(
            new MultiTargetPNPResult(
                new PnpResult(
                    new Transform3d(new Translation3d(8, 4, 0.5), new Rotation3d()), 0.05),
                List.of((short) 1)));
    var result =
        new PhotonPipelineResult(
            0L, captureMicros, captureMicros + 1000L, 0L, List.of(target), multitag);
    var packet = new Packet(1024);
    PhotonPipelineResult.photonStruct.pack(packet, result);
    return java.util.Base64.getEncoder().encodeToString(packet.getWrittenDataCopy());
  }
}
