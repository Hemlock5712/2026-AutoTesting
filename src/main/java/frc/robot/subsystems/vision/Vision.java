package frc.robot.subsystems.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utils.FieldInfo;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;

/**
 * Reads AprilTag observations from one or more cameras and feeds them to the pose estimator.
 *
 * <p>All quality filtering lives here — the IO impls are dumb passthroughs that pick the right
 * estimator method for their source (LL: MT1 vs MT2; PV: multi-tag vs trig-solve) and surface raw
 * metadata. This class enforces the gates and computes the per-observation std-dev.
 *
 * <p>Rejection rules ported from AOS swerve_localizer (see {@code frc/vision/swerve_localizer/
 * localizer.cc} in github.com/RealtimeRoboticsGroup/aos): ambiguity ratio, distance, speed-in-
 * auto, and implied yaw error. We skip AOS's raw-pose-error and distortion gates because PV/LL
 * don't expose those signals.
 */
public class Vision extends SubsystemBase {

  /**
   * Reason a frame was thrown out. Logged as per-camera counters under {@code
   * Vision/<cam>/Rejected/}.
   */
  private enum RejectionReason {
    AMBIGUOUS,
    TOO_FAR,
    OFF_FIELD,
    ROBOT_TOO_FAST,
    IMPLIED_YAW_ERROR,
    IMAGE_FROM_FUTURE
  }

  /** Tolerance for "image from future" check. Anything beyond this is a clock-sync glitch. */
  private static final double FUTURE_TIMESTAMP_TOLERANCE_S = 0.05;

  // --- AOS-style hard gates (reject the observation entirely) ---
  /** AOS {@code HIGH_POSE_ERROR_RATIO}: PV ambiguity is the same best/alt reprojection ratio. */
  private static final double MAX_AMBIGUITY = 0.4;

  /** AOS {@code HIGH_DISTANCE_TO_TARGET}. */
  private static final double MAX_TAG_DISTANCE_METERS = 5.0;

  /** AOS {@code ROBOT_TOO_FAST}: reject vision during auto when chassis is moving too fast. */
  private static final double MAX_AUTO_SPEED_MPS = 5.0;

  /** AOS {@code HIGH_IMPLIED_YAW_ERROR}: max disagreement between vision yaw and gyro yaw. */
  private static final double MAX_AUTO_IMPLIED_YAW_ERROR_RAD = Math.toRadians(5.0);

  private static final double MAX_TELEOP_IMPLIED_YAW_ERROR_RAD = Math.toRadians(30.0);

  /** Ours, not in AOS: drop observations placing the robot off the field (tag misidentified). */
  private static final double FIELD_BORDER_MARGIN_METERS = 0.5;

  // --- Std-dev model ported verbatim from AOS (lower = more trusted).
  //
  // AOS base noises in their (heading, distance, skew) polar frame are (0.03, 0.25, 0.15) and
  // get multiplied by 2.0 → (0.06, 0.5, 0.3). They then scale (distance, skew) — but NOT heading
  // — by min(1, d²), and everything by (1 + speed).
  //
  // In our Cartesian (x, y, θ) frame we use AOS's distance-axis value (0.5) for xy and AOS's
  // heading value (0.06) for θ. No tag-count factor: AOS does per-tag sequential updates which
  // give the same multi-tag boost implicitly; WPILib's API only accepts a consolidated pose so
  // we'd have to fake it. Skipping for now to keep the port exact — re-add if log review shows
  // multi-tag observations need extra trust.
  private static final double XY_STD_DEV_BASE = 0.5;
  private static final double THETA_STD_DEV_BASE = 0.06;
  private static final double MEGATAG2_ROTATION_STD_DEV = Double.POSITIVE_INFINITY;

  /** Camera is marked disconnected if no frames arrive for this long. */
  private static final double DISCONNECT_TIMEOUT_S = 0.5;

  private static final Pose2d[] EMPTY_POSE = new Pose2d[0];
  private static final Pose3d[] EMPTY_TAGS = new Pose3d[0];

  private final Drive drive;
  private final VisionIO[] ios;
  private final VisionInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;
  private final double[] lastFreshFrameTime;

  /** Per-camera, per-reason cumulative rejection counts. Logged each cycle. */
  private final long[][] rejectionCounts;

  public Vision(Drive drive, VisionIO... ios) {
    super("Vision");
    this.drive = drive;
    this.ios = ios;
    this.inputs = new VisionInputsAutoLogged[ios.length];
    this.disconnectedAlerts = new Alert[ios.length];
    this.lastFreshFrameTime = new double[ios.length];
    this.rejectionCounts = new long[ios.length][RejectionReason.values().length];
    Arrays.fill(lastFreshFrameTime, Double.NEGATIVE_INFINITY);
    for (int i = 0; i < ios.length; i++) {
      this.inputs[i] = new VisionInputsAutoLogged();
      this.disconnectedAlerts[i] =
          new Alert("Vision: " + ios[i].getName() + " disconnected.", AlertType.kWarning);
    }
  }

  private void reject(int camIdx, RejectionReason reason) {
    rejectionCounts[camIdx][reason.ordinal()]++;
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
      String camPrefix = "Vision/" + ios[i].getName();

      if (inputs[i].newFrame) {
        lastFreshFrameTime[i] = now;
      }
      disconnectedAlerts[i].set(now - lastFreshFrameTime[i] > DISCONNECT_TIMEOUT_S);

      // No new frame this cycle: leave the last-published values in place so the visualization
      // doesn't flicker between camera frames (PV sim is 40 FPS, robot loop is 50 Hz).
      if (!inputs[i].hasObservation) continue;

      Pose3d[] tags = tagPoses(inputs[i].tagIds);
      double obsTs = inputs[i].latestTimestampSeconds;
      Pose2d robotAtObs = drive.samplePoseAt(obsTs).orElse(drive.getPose());

      // --- AOS-style rejection gates ---
      if (obsTs > now + FUTURE_TIMESTAMP_TOLERANCE_S) {
        reject(i, RejectionReason.IMAGE_FROM_FUTURE);
        publishOutcome(camPrefix, null, inputs[i].latestPose, EMPTY_TAGS, tags, obsTs, robotAtObs);
        continue;
      }
      // Per-target ambiguity is only meaningful for single-tag observations; PhotonVision's
      // multi-tag PNP doesn't populate it reliably, so a multi-tag solve can falsely report 1.0.
      if (inputs[i].tagCount <= 1 && inputs[i].maxAmbiguity > MAX_AMBIGUITY) {
        reject(i, RejectionReason.AMBIGUOUS);
        publishOutcome(camPrefix, null, inputs[i].latestPose, EMPTY_TAGS, tags, obsTs, robotAtObs);
        continue;
      }
      if (inputs[i].avgTagDistance > MAX_TAG_DISTANCE_METERS) {
        reject(i, RejectionReason.TOO_FAR);
        publishOutcome(camPrefix, null, inputs[i].latestPose, EMPTY_TAGS, tags, obsTs, robotAtObs);
        continue;
      }
      if (!isPoseOnField(inputs[i].latestPose)) {
        reject(i, RejectionReason.OFF_FIELD);
        publishOutcome(camPrefix, null, inputs[i].latestPose, EMPTY_TAGS, tags, obsTs, robotAtObs);
        continue;
      }

      double robotSpeed = drive.translationSpeed();
      boolean isAuto = DriverStation.isAutonomous();
      if (isAuto && robotSpeed > MAX_AUTO_SPEED_MPS) {
        reject(i, RejectionReason.ROBOT_TOO_FAST);
        publishOutcome(camPrefix, null, inputs[i].latestPose, EMPTY_TAGS, tags, obsTs, robotAtObs);
        continue;
      }

      // Implied-yaw-error gate. Only meaningful for non-gyro-locked solves — MT2/trig-solve set
      // the rotation from the gyro, so the comparison would always be ~0.
      if (!inputs[i].isMegaTag2) {
        double yawError =
            Math.abs(
                MathUtil.angleModulus(
                    inputs[i].latestPose.getRotation().minus(drive.getRotation()).getRadians()));
        double yawLimit =
            isAuto ? MAX_AUTO_IMPLIED_YAW_ERROR_RAD : MAX_TELEOP_IMPLIED_YAW_ERROR_RAD;
        if (yawError > yawLimit) {
          reject(i, RejectionReason.IMPLIED_YAW_ERROR);
          publishOutcome(
              camPrefix, null, inputs[i].latestPose, EMPTY_TAGS, tags, obsTs, robotAtObs);
          continue;
        }
      }

      // --- Std-dev model (AOS verbatim).
      //   xy = base_xy * min(1, d²) * (1 + speed) * deweight
      //   θ  = base_θ  *               (1 + speed) * deweight   (no distance scalar on heading)
      double distanceScalar = Math.min(1.0, Math.pow(inputs[i].avgTagDistance, 2.0));
      double speedScalar = 1.0 + robotSpeed;
      double deweightScalar =
          anyDeweightedTag(inputs[i].tagIds) ? VisionConstants.DEWEIGHTED_TAG_NOISE_SCALAR : 1.0;
      double xyStdDev = XY_STD_DEV_BASE * distanceScalar * speedScalar * deweightScalar;
      double thetaStdDev =
          inputs[i].isMegaTag2
              ? MEGATAG2_ROTATION_STD_DEV
              : THETA_STD_DEV_BASE * speedScalar * deweightScalar;

      drive.addVisionMeasurement(
          inputs[i].latestPose,
          inputs[i].latestTimestampSeconds,
          VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));

      Logger.recordOutput(camPrefix + "/XYStdDev", xyStdDev);
      Logger.recordOutput(camPrefix + "/ThetaStdDev", thetaStdDev);
      publishOutcome(camPrefix, inputs[i].latestPose, null, tags, EMPTY_TAGS, obsTs, robotAtObs);
    }

    // Publish rejection counters once per cycle so each gate is observable in the log.
    for (int i = 0; i < ios.length; i++) {
      String prefix = "Vision/" + ios[i].getName() + "/Rejected/";
      for (RejectionReason r : RejectionReason.values()) {
        Logger.recordOutput(prefix + r.name(), rejectionCounts[i][r.ordinal()]);
      }
    }

    Logger.recordOutput("Timing/VisionMs", (System.nanoTime() - t0) / 1.0e6);
  }

  private static boolean anyDeweightedTag(int[] tagIds) {
    for (int id : tagIds) {
      if (!VisionConstants.TRUSTED_TAG_IDS.contains(id)) return true;
    }
    return false;
  }

  /**
   * Publish the accept/reject outcome (pose + tag poses) for one camera in one cycle, along with
   * the observation FPGA timestamp and the robot pose sampled at that timestamp.
   */
  private static void publishOutcome(
      String camPrefix,
      Pose2d accepted,
      Pose2d rejected,
      Pose3d[] accTags,
      Pose3d[] rejTags,
      double observationTimestamp,
      Pose2d robotPoseAtObservation) {
    Logger.recordOutput(
        camPrefix + "/AcceptedPose", accepted == null ? EMPTY_POSE : new Pose2d[] {accepted});
    Logger.recordOutput(
        camPrefix + "/RejectedPose", rejected == null ? EMPTY_POSE : new Pose2d[] {rejected});
    Logger.recordOutput(camPrefix + "/AcceptedTagPoses", accTags);
    Logger.recordOutput(camPrefix + "/RejectedTagPoses", rejTags);
    Logger.recordOutput(camPrefix + "/LastObservationTimestamp", observationTimestamp);
    Logger.recordOutput(camPrefix + "/LastObservationRobotPose", robotPoseAtObservation);
  }

  private static Pose3d[] tagPoses(int[] tagIds) {
    if (tagIds == null || tagIds.length == 0) return new Pose3d[0];
    Pose3d[] poses = new Pose3d[tagIds.length];
    int n = 0;
    for (int id : tagIds) {
      var tagPose = FieldInfo.aprilTags().getTagPose(id);
      if (tagPose.isPresent()) poses[n++] = tagPose.get();
    }
    return n == tagIds.length ? poses : Arrays.copyOf(poses, n);
  }

  private static boolean isPoseOnField(Pose2d pose) {
    return pose.getX() >= -FIELD_BORDER_MARGIN_METERS
        && pose.getX() <= FieldInfo.lengthMeters() + FIELD_BORDER_MARGIN_METERS
        && pose.getY() >= -FIELD_BORDER_MARGIN_METERS
        && pose.getY() <= FieldInfo.widthMeters() + FIELD_BORDER_MARGIN_METERS;
  }
}
