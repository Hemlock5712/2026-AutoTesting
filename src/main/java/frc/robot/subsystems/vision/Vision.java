package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;

/**
 * Reads AprilTag observations from one or more cameras and feeds them to the pose estimator.
 *
 * <p>Each frame, every camera's observation is fed in with a "trust score" (standard deviation)
 * that depends on how far away the tags are and how many we see. Closer tags and more tags = more
 * trusted.
 */
public class Vision extends SubsystemBase {

  // --- Trust factors (how much we trust the vision pose). Lower = more trusted. ---
  private static final double XY_STD_DEV_COEFFICIENT = 0.333;
  private static final double ROTATION_STD_DEV_COEFFICIENT = 1.5;
  private static final double MEGATAG2_ROTATION_STD_DEV = Double.POSITIVE_INFINITY;
  private static final double MAX_EFFECTIVE_TAG_COUNT = 2.5;

  /** Camera is marked disconnected if no frames arrive for this long. */
  private static final double DISCONNECT_TIMEOUT_S = 0.5;

  private final Drive drive;
  private final VisionIO[] ios;
  private final VisionInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;
  private final double[] lastFreshFrameTime;

  public Vision(Drive drive, VisionIO... ios) {
    super("Vision");
    this.drive = drive;
    this.ios = ios;
    this.inputs = new VisionInputsAutoLogged[ios.length];
    this.disconnectedAlerts = new Alert[ios.length];
    this.lastFreshFrameTime = new double[ios.length];
    Arrays.fill(lastFreshFrameTime, Double.NEGATIVE_INFINITY);
    for (int i = 0; i < ios.length; i++) {
      this.inputs[i] = new VisionInputsAutoLogged();
      this.disconnectedAlerts[i] =
          new Alert("Vision: " + ios[i].getName() + " disconnected.", AlertType.kWarning);
    }
  }

  @Override
  public void periodic() {
    long t0 = System.nanoTime();

    ChassisSpeeds robotSpeeds = drive.getRobotSpeeds();
    double yawDegrees = drive.getRotation().getDegrees();
    double yawRateDegPerSec = Math.toDegrees(robotSpeeds.omegaRadiansPerSecond);

    for (VisionIO io : ios) {
      io.setRobotOrientation(yawDegrees, yawRateDegPerSec);
    }
    // One NetworkTables flush after queueing all cameras (faster than flushing per-camera).
    NetworkTableInstance.getDefault().flush();

    double now = Timer.getFPGATimestamp();
    for (int i = 0; i < ios.length; i++) {
      ios[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/" + ios[i].getName(), inputs[i]);

      if (inputs[i].newFrame) {
        lastFreshFrameTime[i] = now;
      }
      disconnectedAlerts[i].set(now - lastFreshFrameTime[i] > DISCONNECT_TIMEOUT_S);

      if (!inputs[i].hasObservation) continue;

      double distanceFactor = Math.pow(inputs[i].avgTagDistance, 1.2);
      double effectiveTags = Math.min(MAX_EFFECTIVE_TAG_COUNT, inputs[i].tagCount);
      double tagFactor = effectiveTags * effectiveTags;
      double xyStdDev = XY_STD_DEV_COEFFICIENT * distanceFactor / tagFactor;
      double thetaStdDev =
          inputs[i].isMegaTag2
              ? MEGATAG2_ROTATION_STD_DEV
              : ROTATION_STD_DEV_COEFFICIENT * distanceFactor / tagFactor;

      drive.addVisionMeasurement(
          inputs[i].latestPose,
          inputs[i].latestTimestampSeconds,
          VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));

      Logger.recordOutput("Vision/" + ios[i].getName() + "/XYStdDev", xyStdDev);
      Logger.recordOutput("Vision/" + ios[i].getName() + "/ThetaStdDev", thetaStdDev);
    }

    Logger.recordOutput("Timing/VisionMs", (System.nanoTime() - t0) / 1.0e6);
  }
}
