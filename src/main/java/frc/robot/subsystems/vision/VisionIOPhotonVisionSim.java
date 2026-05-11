package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.utils.FieldInfo;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/**
 * Simulated PhotonVision camera. Same processing pipeline as {@link VisionIOPhotonVision} — the sim
 * renders AprilTag detections into a {@link PhotonCameraSim} which publishes through NetworkTables
 * to the parent class's {@link org.photonvision.PhotonCamera}, so the trig-solve / multi-tag
 * selection and filtering logic is shared verbatim.
 *
 * <p>The {@link VisionSystemSim} is a singleton because PhotonVision only needs one simulated world
 * regardless of how many cameras observe it. Call {@link #update(Pose2d)} once per loop from sim
 * code with the ground-truth chassis pose.
 */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {

  private static final VisionSystemSim SYSTEM_SIM = createSystemSim();

  // Loaded only when this class is referenced, which only happens in SIM (RobotContainer's
  // REAL/REPLAY branches don't touch it). Keeps the placeholder-transform warning off the
  // real-robot dashboard where it has no meaning.
  private static final Alert PLACEHOLDER_TRANSFORMS_ALERT =
      new Alert(
          "VisionConstants.PHOTON_CAMERA_TRANSFORMS are placeholders; tune to CAD before "
              + "trusting trig-solve distances, then set PHOTON_TRANSFORMS_TUNED = true.",
          AlertType.kWarning);

  static {
    PLACEHOLDER_TRANSFORMS_ALERT.set(!VisionConstants.PHOTON_TRANSFORMS_TUNED);
  }

  private static VisionSystemSim createSystemSim() {
    VisionSystemSim sim = new VisionSystemSim("main");
    sim.addAprilTags(FieldInfo.aprilTags());
    return sim;
  }

  /** Tick the shared sim world. Call once per loop with the simulated chassis pose. */
  public static void update(Pose2d truthPose) {
    SYSTEM_SIM.update(truthPose);
  }

  public VisionIOPhotonVisionSim(String name, Transform3d robotToCamera) {
    super(name, robotToCamera);
    SimCameraProperties props = new SimCameraProperties();
    // Coprocessor-class camera: 1280x720, 70° diag FOV, 40 FPS, ~35 ms latency. Tuned to match
    // the std-dev assumptions in Vision.periodic so the sim feels like the real cameras.
    props.setCalibration(1280, 720, Rotation2d.fromDegrees(70));
    props.setCalibError(0.25, 0.08);
    props.setFPS(40);
    props.setAvgLatencyMs(35);
    props.setLatencyStdDevMs(5);

    PhotonCameraSim cameraSim = new PhotonCameraSim(getCamera(), props);
    SYSTEM_SIM.addCamera(cameraSim, robotToCamera);
  }
}
