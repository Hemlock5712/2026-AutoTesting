package frc.robot.utils;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.List;
import java.util.Optional;

/**
 * Game-agnostic field state: AprilTag layout, dimensions, alliance flip helpers. Year-specific
 * geometry (Hub, Tower, etc.) lives in season-prefixed files like {@code Field2026Constants}.
 */
public final class FieldInfo {

  public enum SymmetryType {
    MIRROR,
    ROTATE
  }

  private static AprilTagFieldLayout layout;
  private static SymmetryType symmetryType;

  // Cached to avoid Optional + autoboxing allocations every 50Hz cycle.
  private static Boolean cachedShouldFlip = null;
  private static final double[] FLIP_JOYSTICK_RESULT = new double[2];

  static {
    setLayout(AprilTagFields.kDefaultField);
  }

  private FieldInfo() {}

  public static void setLayout(AprilTagFields field) {
    try {
      layout = AprilTagFieldLayout.loadField(field);
      symmetryType =
          switch (field) {
            case k2026RebuiltAndymark -> SymmetryType.ROTATE;
            case k2026RebuiltWelded -> SymmetryType.ROTATE;
            case k2025ReefscapeWelded -> SymmetryType.ROTATE;
            case k2025ReefscapeAndyMark -> SymmetryType.ROTATE;
            case k2024Crescendo -> SymmetryType.MIRROR;
            case k2023ChargedUp -> SymmetryType.MIRROR;
            case k2022RapidReact -> SymmetryType.ROTATE;
          };
    } catch (Exception e) {
      layout = new AprilTagFieldLayout(List.of(), 0.0, 0.0);
      symmetryType = SymmetryType.MIRROR;
      DriverStation.reportError(
          "Failed to load layout from field \"" + field.name() + "\": " + e.getMessage(), true);
    }
  }

  public static void setLayout(String resourcePath, SymmetryType symmetryType) {
    try {
      layout = AprilTagFieldLayout.loadFromResource(resourcePath);
      FieldInfo.symmetryType = symmetryType;
    } catch (Exception e) {
      layout = new AprilTagFieldLayout(List.of(), 0.0, 0.0);
      symmetryType = SymmetryType.MIRROR;
      DriverStation.reportError(
          "[AprilTags] Unable to load layout from resource \""
              + resourcePath
              + "\": "
              + e.getMessage(),
          true);
    }
  }

  public static void setLayout(AprilTagFieldLayout layout, SymmetryType symmetryType) {
    FieldInfo.layout = layout;
    FieldInfo.symmetryType = symmetryType;
  }

  public static Distance length() {
    return Meters.of(layout.getFieldLength());
  }

  /** Field length in meters, zero-allocation. */
  public static double lengthMeters() {
    return layout.getFieldLength();
  }

  public static Distance width() {
    return Meters.of(layout.getFieldWidth());
  }

  /** Field width in meters, zero-allocation. */
  public static double widthMeters() {
    return layout.getFieldWidth();
  }

  public static SymmetryType symmetryType() {
    return symmetryType;
  }

  public static AprilTagFieldLayout aprilTags() {
    return layout;
  }

  // ==================== Alliance flip ====================

  /** Clears the cached alliance flip so the next {@link #shouldFlip()} re-reads from DS. */
  public static void resetAllianceCache() {
    cachedShouldFlip = null;
  }

  /** True if coordinates should be flipped (red alliance). Cached after the first DS read. */
  public static boolean shouldFlip() {
    if (cachedShouldFlip != null) return cachedShouldFlip;
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      cachedShouldFlip = alliance.get() == Alliance.Red;
      return cachedShouldFlip;
    }
    return false;
  }

  /**
   * Flips joystick X/Y for blue-alliance-perspective driving. Returns a reused [x, y] array — do
   * not retain across calls.
   */
  public static double[] flipJoystick(double x, double y) {
    double sign = shouldFlip() ? -1.0 : 1.0;
    FLIP_JOYSTICK_RESULT[0] = sign * x;
    FLIP_JOYSTICK_RESULT[1] = sign * y;
    return FLIP_JOYSTICK_RESULT;
  }

  public static double flipJoystickRotation(double omega) {
    return shouldFlip() ? -omega : omega;
  }

  public static Pose2d flip(Pose2d pose) {
    if (!shouldFlip()) return pose;
    return switch (symmetryType) {
      case MIRROR ->
          new Pose2d(
              layout.getFieldLength() - pose.getX(),
              pose.getY(),
              new Rotation2d(Math.PI - pose.getRotation().getRadians()));
      case ROTATE ->
          new Pose2d(
              layout.getFieldLength() - pose.getX(),
              layout.getFieldWidth() - pose.getY(),
              pose.getRotation().rotateBy(Rotation2d.k180deg));
    };
  }

  public static Translation2d flip(Translation2d t) {
    if (!shouldFlip()) return t;
    return switch (symmetryType) {
      case MIRROR -> new Translation2d(layout.getFieldLength() - t.getX(), t.getY());
      case ROTATE ->
          new Translation2d(layout.getFieldLength() - t.getX(), layout.getFieldWidth() - t.getY());
    };
  }

  public static Rotation2d flip(Rotation2d r) {
    if (!shouldFlip()) return r;
    return switch (symmetryType) {
      case MIRROR -> new Rotation2d(Math.PI - r.getRadians());
      case ROTATE -> r.rotateBy(Rotation2d.k180deg);
    };
  }

  /** Always flipped for both MIRROR and ROTATE. */
  public static double flipX(double x) {
    return shouldFlip() ? layout.getFieldLength() - x : x;
  }

  public static double flipX(Distance x) {
    return flipX(x.in(Meters));
  }

  /** Only flipped for ROTATE symmetry. */
  public static double flipY(double y) {
    return shouldFlip() && symmetryType == SymmetryType.ROTATE ? layout.getFieldWidth() - y : y;
  }

  public static double flipY(Distance y) {
    return flipY(y.in(Meters));
  }
}
