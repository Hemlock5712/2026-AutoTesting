package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import frc.robot.utils.FieldInfo;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import org.junit.jupiter.api.Test;

/**
 * Opens the replay log produced by running the robot in REPLAY mode with {@code
 * -Preplay.vision.json.photon-fl=...} and reports per-entry record counts under {@code
 * /AdvantageKit/RealOutputs/Vision/photon-fl/*}. The JSON-IO path should produce at least one
 * {@code latestPose} update; if it doesn't, our wiring is silently no-op.
 */
public class ReplayLogVerifier {

  private static final Path REPLAY_LOG = Path.of("logs", "akit_26-05-12_22-35-26_replay.wpilog");

  @Test
  public void fixtureFilePassesFilterWhenFedThroughIoDirectly() throws IOException {
    // Direct unit test: load the fixture file with the real IO + clock advanced past every entry.
    // If hasObservation stays false here, the fixture itself doesn't pass the filter chain
    // (kDefaultField tag mismatch, off-field implied pose, etc.). If it's true here but kZero in
    // REPLAY mode, the issue is REPLAY-environment-specific (FPGA clock, FieldInfo, etc.).
    FieldInfo.setLayout(edu.wpi.first.apriltag.AprilTagFields.kDefaultField);
    var io =
        new VisionIOPhotonVisionJSON(
            "photon-fl",
            VisionConstants.PHOTON_CAMERA_TRANSFORMS[0],
            Path.of("replay-fixtures", "photon-fl-demo", "results", "synthetic.jsonl"),
            () -> 10_000_000_000L); // way past every fixture capture
    var inputs = new VisionInputsAutoLogged();
    io.updateInputs(inputs);
    System.out.println(
        "Direct fixture run: newFrame="
            + inputs.newFrame
            + " hasObservation="
            + inputs.hasObservation
            + " latestPose="
            + inputs.latestPose
            + " tagCount="
            + inputs.tagCount);
    assertTrue(inputs.newFrame, "newFrame must be true with clock past all entries");
    assertTrue(
        inputs.hasObservation,
        "hasObservation must be true — if false, the fixture is filter-rejected");
  }

  @Test
  public void replayLogContainsPhotonFlVisionUpdates() throws IOException {
    DataLogReader reader = new DataLogReader(REPLAY_LOG.toString());
    assertTrue(reader.isValid(), "replay log header must parse");

    Map<Integer, String> entryName = new HashMap<>();
    Map<Integer, String> entryType = new HashMap<>();
    Map<String, Integer> recordCount = new TreeMap<>();
    List<Pose2d> latestPoseValues = new ArrayList<>();
    Integer latestPoseEntry = null;

    for (DataLogRecord record : reader) {
      if (record.isStart()) {
        var start = record.getStartData();
        entryName.put(start.entry, start.name);
        entryType.put(start.entry, start.type);
        if (start.name.endsWith("/Vision/photon-fl-jsonreplay/LatestPose")) {
          latestPoseEntry = start.entry;
        }
      } else if (record.isControl()) {
        // ignore other control records
      } else {
        String name = entryName.get(record.getEntry());
        if (name != null
            && (name.contains("Vision/photon-fl/")
                || name.contains("Vision/photon-fl-jsonreplay/"))) {
          recordCount.merge(name, 1, Integer::sum);
        }
        if (latestPoseEntry != null && record.getEntry() == latestPoseEntry) {
          // Pose2d wpilog struct: 3 doubles (x, y, theta) = 24 bytes raw, little-endian.
          var buf =
              java.nio.ByteBuffer.wrap(record.getRaw()).order(java.nio.ByteOrder.LITTLE_ENDIAN);
          double x = buf.getDouble(0);
          double y = buf.getDouble(8);
          double theta = buf.getDouble(16);
          latestPoseValues.add(new Pose2d(x, y, new edu.wpi.first.math.geometry.Rotation2d(theta)));
        }
      }
    }

    System.out.println("=== /Vision/photon-fl/* entries in " + REPLAY_LOG + " ===");
    recordCount.forEach((name, count) -> System.out.println("  " + count + "\t" + name));
    System.out.println("LatestPose values observed: " + latestPoseValues);

    // Dump ALL entry names that contain "jsonreplay" or "Vision" to find where our IO landed.
    System.out.println("=== All Vision-related entries ===");
    entryName.values().stream()
        .filter(n -> n.contains("Vision") || n.contains("jsonreplay"))
        .sorted()
        .forEach(n -> System.out.println("  " + n));

    // AKit's REPLAY mode doesn't write Inputs entries for names that weren't in the source
    // log, so /Vision/<channel>/LatestPose never lands in the output wpilog. The proof that
    // each JSON channel ran AND its observation was absorbed by the drive's pose estimator is
    // that Vision.java's recordOutput() fired — XYStdDev only gets written when hasObservation
    // is true and drive.addVisionMeasurement is called.
    var absorbedChannels =
        entryName.values().stream()
            .filter(n -> n.matches(".*/Vision/photon-fl-jsonreplay[^/]*/XYStdDev"))
            .sorted()
            .toList();
    System.out.println("JSON channels absorbed by drive: " + absorbedChannels);
    assertTrue(
        !absorbedChannels.isEmpty(),
        "No photon-fl-jsonreplay* XYStdDev entries — JSON IO never made it past the filter. Seen"
            + " names: "
            + entryName.values());
  }
}
