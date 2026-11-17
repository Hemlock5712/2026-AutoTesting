package frc.team5712.path.wpilib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.struct.Struct;
import frc.team5712.path.core.Pose;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/**
 * WPILib 2026 stand-in for 2027's {@code TelemetryTable}: each value goes to NetworkTables under
 * {@code /Telemetry/<table>/<key>} (where the editor's live view looks) and into the AdvantageKit
 * log under {@code Telemetry/<table>/<key>}. Publishers are made on first use and kept.
 */
final class TelemetryTable {
  private static final NetworkTable ROOT = NetworkTableInstance.getDefault().getTable("Telemetry");

  private final NetworkTable table;
  private final String logPrefix;
  private final Map<String, AutoCloseable> publishers = new HashMap<>();

  TelemetryTable(String name) {
    this.table = ROOT.getSubTable(name);
    this.logPrefix = "Telemetry/" + name + "/";
  }

  @SuppressWarnings("unchecked")
  private <P extends AutoCloseable> P publisher(String key, java.util.function.Supplier<P> make) {
    return (P) publishers.computeIfAbsent(key, k -> make.get());
  }

  void log(String key, double value) {
    this.<DoublePublisher>publisher(key, () -> table.getDoubleTopic(key).publish()).set(value);
    Logger.recordOutput(logPrefix + key, value);
  }

  void log(String key, long value) {
    this.<IntegerPublisher>publisher(key, () -> table.getIntegerTopic(key).publish()).set(value);
    Logger.recordOutput(logPrefix + key, value);
  }

  void log(String key, boolean value) {
    this.<BooleanPublisher>publisher(key, () -> table.getBooleanTopic(key).publish()).set(value);
    Logger.recordOutput(logPrefix + key, value);
  }

  void log(String key, String value) {
    this.<StringPublisher>publisher(key, () -> table.getStringTopic(key).publish()).set(value);
    Logger.recordOutput(logPrefix + key, value);
  }

  void log(String key, double[] value) {
    this.<DoubleArrayPublisher>publisher(key, () -> table.getDoubleArrayTopic(key).publish())
        .set(value);
    Logger.recordOutput(logPrefix + key, value);
  }

  void log(String key, String[] value) {
    this.<StringArrayPublisher>publisher(key, () -> table.getStringArrayTopic(key).publish())
        .set(value);
    Logger.recordOutput(logPrefix + key, value);
  }

  /**
   * A pose: NetworkTables gets the centre frame (the editor's), the log the 2026 corner frame (so
   * it lines up with the robot's pose in AdvantageScope).
   */
  void logPose(String key, Pose pose) {
    this.<StructPublisher<Pose2d>>publisher(
            key, () -> table.getStructTopic(key, Pose2d.struct).publish())
        .set(Conversions.toCenterPose2d(pose));
    Logger.recordOutput(logPrefix + key, Conversions.toPose2d(pose));
  }

  <T> void log(String key, T value, Struct<T> struct) {
    this.<StructPublisher<T>>publisher(key, () -> table.getStructTopic(key, struct).publish())
        .set(value);
    Logger.recordOutput(logPrefix + key, struct, value);
  }
}
