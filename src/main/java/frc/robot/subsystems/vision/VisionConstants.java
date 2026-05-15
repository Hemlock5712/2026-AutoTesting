package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.Set;

/**
 * Camera names, robot-to-camera transforms, and per-tag trust.
 *
 * <p>Real robot: one or more Limelights — names must match each camera's NetworkTables entry, set
 * in the Limelight web UI.
 *
 * <p>Sim: PhotonVision sim cameras with the same names+transforms — Limelight has no Java sim, so
 * PhotonVision stands in. Add or remove entries to match your robot.
 */
public final class VisionConstants {

  public static final String[] LIMELIGHT_NAMES = {"limelight"};

  public static final String[] PHOTON_CAMERA_NAMES = {"photon-front"};

  /**
   * Robot-to-camera transforms for sim cameras. Index-aligned with {@link #PHOTON_CAMERA_NAMES}.
   * <strong>Tune to your CAD</strong> before trusting trig-solve distances.
   */
  public static final Transform3d[] PHOTON_CAMERA_TRANSFORMS = {
    new Transform3d(
        new Translation3d(0.25, 0.0, 0.23), new Rotation3d(0.0, Math.toRadians(-15.0), 0.0))
  };

  /**
   * AprilTag IDs we trust at full noise. Tags not in this set get a 1.5× noise scalar (AOS-style
   * per-tag deweight). For 2026 Rebuilt Reefscape: reef branches (6-11 red, 17-22 blue) and
   * processor faces (3 red, 16 blue). Perimeter/station tags (coral stations, barge) are deweighted
   * — they're farther from scoring action and less precisely surveyed in practice.
   */
  public static final Set<Integer> TRUSTED_TAG_IDS =
      Set.of(3, 6, 7, 8, 9, 10, 11, 16, 17, 18, 19, 20, 21, 22);

  /** Noise multiplier applied when ANY tag used in a frame is outside {@link #TRUSTED_TAG_IDS}. */
  public static final double DEWEIGHTED_TAG_NOISE_SCALAR = 1.5;

  private VisionConstants() {}
}
