package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/** Camera names and robot-to-camera transforms for all vision IO impls. */
public final class VisionConstants {

  /** Real-robot Limelights. Names must match each camera's NetworkTables entry. */
  public static final String[] LIMELIGHT_NAMES = {
    "limelight-br", "limelight-bl", "limelight-fl", "limelight-fr", "limelight-mm"
  };

  /**
   * Sim PhotonVision camera names. Distinct from {@link #LIMELIGHT_NAMES} so a replay log only ever
   * references one set.
   */
  public static final String[] PHOTON_CAMERA_NAMES = {
    "photon-fl", "photon-fr", "photon-bl", "photon-br"
  };

  /**
   * Set to {@code true} once {@link #PHOTON_CAMERA_TRANSFORMS} has been measured against CAD. While
   * this is {@code false}, an Alert fires on the dashboard so an untuned sim setup can't quietly
   * ship to a real-robot calibration session. Trig-solve distances are nonsense without correct
   * camera extrinsics, so this gate exists to catch the "forgot to tune" footgun.
   */
  public static final boolean PHOTON_TRANSFORMS_TUNED = false;

  /**
   * Robot-to-camera transforms for the PhotonVision sim cameras. Placeholder corner mounts: 10 in
   * from center, 9 in up, pitched 15° up, yawed toward the matching corner. Tune to CAD before
   * trusting trig-solve distances, then flip {@link #PHOTON_TRANSFORMS_TUNED} to silence the
   * placeholder Alert. Index-aligned with {@link #PHOTON_CAMERA_NAMES}.
   */
  public static final Transform3d[] PHOTON_CAMERA_TRANSFORMS = {
    new Transform3d(
        new Translation3d(0.254, 0.254, 0.229),
        new Rotation3d(0.0, Math.toRadians(-15.0), Math.toRadians(30.0))),
    new Transform3d(
        new Translation3d(0.254, -0.254, 0.229),
        new Rotation3d(0.0, Math.toRadians(-15.0), Math.toRadians(-30.0))),
    new Transform3d(
        new Translation3d(-0.254, 0.254, 0.229),
        new Rotation3d(0.0, Math.toRadians(-15.0), Math.toRadians(150.0))),
    new Transform3d(
        new Translation3d(-0.254, -0.254, 0.229),
        new Rotation3d(0.0, Math.toRadians(-15.0), Math.toRadians(-150.0)))
  };

  private VisionConstants() {}
}
