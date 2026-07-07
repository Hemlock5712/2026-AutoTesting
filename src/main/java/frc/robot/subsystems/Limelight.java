// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.FieldInfo;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.PoseEstimate;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class Limelight extends SubsystemBase {

  // --- Standard Deviation Formula ---
  // Formula: coefficient * pow(avgTagDist, 1.2) / pow(tagCount, 2.0)
  //
  // Coefficients are scaled to match 6328 Mechanical Advantage's trust ratio
  // while using WPILib default odometry std devs [0.1, 0.1, 0.1].
  // MA uses [0.003, 0.003, 0.002] for odometry with coefficients [0.01, 0.03].
  // Scaling: XY = 0.01 * (0.1/0.003) ≈ 0.333, Theta = 0.03 * (0.1/0.002) = 1.5

  private static final double XY_STD_DEV_COEFFICIENT = 0.333;
  private static final double ROTATION_STD_DEV_COEFFICIENT = 1.5;
  private static final double MEGATAG2_ROTATION_STD_DEV = Double.POSITIVE_INFINITY;

  // Cap effective tag count to prevent over-trusting 3+ tags.
  // tagCount^2 assumes independent measurements, but tags on the same wall are correlated.
  // Real accuracy improvement from 2→3 tags is ~40%, not the 125% the formula gives uncapped.
  private static final double MAX_EFFECTIVE_TAG_COUNT = 2.5;

  // Toggle between synced inverse-variance fusion (true) and simple per-camera measurements (false)
  private static final boolean USE_FUSED_VISION = true;
  private static final int DEFAULT_PIPELINE = 0;

  // --- Rejection Thresholds ---
  private static final double MAX_AMBIGUITY = 0.3;
  private static final double FIELD_BORDER_MARGIN_METERS = 0.5;
  private static final double MAX_ANGULAR_VELOCITY_MT1_DEG_PER_SEC = 360;
  private static final double MAX_ANGULAR_VELOCITY_MT2_DEG_PER_SEC = 200;

  private static final class CameraState {
    final String name;

    CameraState(String name) {
      this.name = name;
    }
  }

  private final CommandSwerveDrivetrain m_drivetrain;
  private final CameraState[] cameras;
  private final int cameraCount;

  // Cached once per cycle, reused in periodic()
  private double cachedOmegaDegPerSec = 0.0;

  // Pre-allocated fusion accumulators — reused every cycle, zero allocations
  private final Matrix<N3, N1> fusedStdDevs = VecBuilder.fill(0, 0, 0);

  // Pre-allocated arrays for timestamp synchronization — reused every cycle
  private final PoseEstimate[] validEstimates;
  private final double[] validXYStdDevs;
  private final double[] validRotStdDevs;

  /**
   * Creates a Limelight subsystem managing multiple cameras.
   *
   * @param cameraNames NetworkTables names for each camera (e.g., "limelight-br").
   * @param drivetrain Swerve drivetrain for pose estimation and gyro data.
   */
  public Limelight(List<String> cameraNames, CommandSwerveDrivetrain drivetrain) {
    super("Limelight");
    m_drivetrain = drivetrain;
    cameraCount = cameraNames.size();
    cameras = new CameraState[cameraCount];
    validEstimates = new PoseEstimate[cameraCount];
    validXYStdDevs = new double[cameraCount];
    validRotStdDevs = new double[cameraCount];
    for (int i = 0; i < cameraCount; i++) {
      cameras[i] = new CameraState(cameraNames.get(i));
      LimelightHelpers.setPipelineIndex(cameras[i].name, DEFAULT_PIPELINE);
    }
    LimelightHelpers.Flush();
  }

  @Override
  public void periodic() {
    long _t = System.nanoTime();
    // Read drivetrain state once for all cameras
    ChassisSpeeds speeds = m_drivetrain.getRobotSpeeds();
    cachedOmegaDegPerSec = Math.toDegrees(speeds.omegaRadiansPerSecond);
    double yawDegrees = m_drivetrain.getPose().getRotation().getDegrees();

    // Set orientation for all cameras, then flush once
    for (int i = 0; i < cameraCount; i++) {
      LimelightHelpers.SetRobotOrientation_NoFlush(
          cameras[i].name, yawDegrees, cachedOmegaDegPerSec, 0, 0, 0, 0);
    }
    LimelightHelpers.Flush();

    // Phase 1: Collect valid poses and compute stddevs
    int validCount = 0;
    for (int i = 0; i < cameraCount; i++) {
      PoseEstimate pe = getValidPoseEstimate(cameras[i]);
      if (pe == null) continue;
      Logger.recordOutput("LL" + cameras[i].name, pe.pose);

      double distanceFactor = Math.pow(pe.avgTagDist, 1.2);
      double effectiveTags = Math.min(MAX_EFFECTIVE_TAG_COUNT, pe.tagCount);
      double tagFactor = effectiveTags * effectiveTags;

      validEstimates[validCount] = pe;
      validXYStdDevs[validCount] = XY_STD_DEV_COEFFICIENT * distanceFactor / tagFactor;
      validRotStdDevs[validCount] =
          pe.isMegaTag2
              ? MEGATAG2_ROTATION_STD_DEV
              : ROTATION_STD_DEV_COEFFICIENT * distanceFactor / tagFactor;
      validCount++;
    }

    if (validCount == 0) {
      Logger.recordOutput("Timing/LimelightMs", (System.nanoTime() - _t) / 1e6);
      return;
    }

    if (USE_FUSED_VISION) {
      addSyncedFusedMeasurement(validCount);
    } else {
      addIndividualMeasurements(validCount);
    }
    Logger.recordOutput("Timing/LimelightMs", (System.nanoTime() - _t) / 1e6);
  }

  /**
   * Syncs all camera poses to the latest timestamp via odometry deltas, then fuses via
   * inverse-variance weighting into a single addVisionMeasurement call.
   */
  private void addSyncedFusedMeasurement(int validCount) {
    // Find reference timestamp (latest)
    double refTimestamp = validEstimates[0].timestampSeconds;
    for (int i = 1; i < validCount; i++) {
      if (validEstimates[i].timestampSeconds > refTimestamp) {
        refTimestamp = validEstimates[i].timestampSeconds;
      }
    }
    double refTimeCurrent = Utils.fpgaToCurrentTime(refTimestamp);
    Optional<Pose2d> odomAtRefOpt = m_drivetrain.samplePoseAt(refTimeCurrent);

    // Inverse-variance weighted fusion of time-synchronized poses
    double sumX = 0, sumY = 0, sumSin = 0, sumCos = 0;
    double sumInvVarXY = 0, sumInvVarTheta = 0;

    for (int i = 0; i < validCount; i++) {
      PoseEstimate pe = validEstimates[i];
      Pose2d visionPose = pe.pose;

      // Sync: project older camera poses forward to the reference timestamp
      if (odomAtRefOpt.isPresent() && Math.abs(pe.timestampSeconds - refTimestamp) > 1e-6) {
        double camTimeCurrent = Utils.fpgaToCurrentTime(pe.timestampSeconds);
        Optional<Pose2d> odomAtCamOpt = m_drivetrain.samplePoseAt(camTimeCurrent);
        if (odomAtCamOpt.isPresent()) {
          Transform2d odomDelta = new Transform2d(odomAtCamOpt.get(), odomAtRefOpt.get());
          visionPose = visionPose.plus(odomDelta);
        }
      }

      double invVarXY = 1.0 / (validXYStdDevs[i] * validXYStdDevs[i]);
      sumX += visionPose.getX() * invVarXY;
      sumY += visionPose.getY() * invVarXY;
      sumInvVarXY += invVarXY;

      if (Double.isFinite(validRotStdDevs[i])) {
        double invVarTheta = 1.0 / (validRotStdDevs[i] * validRotStdDevs[i]);
        double theta = visionPose.getRotation().getRadians();
        sumSin += Math.sin(theta) * invVarTheta;
        sumCos += Math.cos(theta) * invVarTheta;
        sumInvVarTheta += invVarTheta;
      }

      validEstimates[i] = null;
    }

    double fusedX = sumX / sumInvVarXY;
    double fusedY = sumY / sumInvVarXY;
    double fusedXYStdDev = 1.0 / Math.sqrt(sumInvVarXY);

    double fusedThetaStdDev;
    double fusedThetaRad;
    if (sumInvVarTheta > 0) {
      fusedThetaRad = Math.atan2(sumSin, sumCos);
      fusedThetaStdDev = 1.0 / Math.sqrt(sumInvVarTheta);
    } else {
      fusedThetaRad = m_drivetrain.getPose().getRotation().getRadians();
      fusedThetaStdDev = MEGATAG2_ROTATION_STD_DEV;
    }

    fusedStdDevs.set(0, 0, fusedXYStdDev);
    fusedStdDevs.set(1, 0, fusedXYStdDev);
    fusedStdDevs.set(2, 0, fusedThetaStdDev);

    m_drivetrain.addVisionMeasurementCurrentTime(
        new Pose2d(fusedX, fusedY, new Rotation2d(fusedThetaRad)), refTimeCurrent, fusedStdDevs);
  }

  /**
   * Adds each camera's pose individually with its own timestamp and stddevs. Lets the CTRE Kalman
   * filter handle timestamp interpolation per-measurement.
   */
  private void addIndividualMeasurements(int validCount) {
    for (int i = 0; i < validCount; i++) {
      PoseEstimate pe = validEstimates[i];
      fusedStdDevs.set(0, 0, validXYStdDevs[i]);
      fusedStdDevs.set(1, 0, validXYStdDevs[i]);
      fusedStdDevs.set(2, 0, validRotStdDevs[i]);
      m_drivetrain.addVisionMeasurement(pe.pose, pe.timestampSeconds, fusedStdDevs);
      validEstimates[i] = null;
    }
  }

  private PoseEstimate getValidPoseEstimate(CameraState camera) {
    PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(camera.name);

    if (!LimelightHelpers.validPoseEstimate(poseEstimate)) {
      return null;
    }

    // Ambiguity check — null/bounds safe
    if (poseEstimate.rawFiducials[0].ambiguity > MAX_AMBIGUITY) {
      return null;
    }

    if (poseEstimate.avgTagDist > 5.5) {
      return null;
    }

    // Use MegaTag2 for single tag estimates when not disabled
    // (disabled gyro may not be seeded correctly yet)
    if (poseEstimate.tagCount == 1 && !DriverStation.isDisabled()) {
      // Check MT2-specific angular velocity threshold before requesting MT2
      if (isRotatingTooFastForMT2()) {
        return null;
      }
      PoseEstimate megaTag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(camera.name);
      if (!LimelightHelpers.validPoseEstimate(megaTag2)) {
        return null;
      }
      poseEstimate = megaTag2;
    }

    // For multi-tag MT1, check the stricter angular velocity threshold
    if (!poseEstimate.isMegaTag2 && isRotatingTooFastForMT1()) {
      return null;
    }

    // Field bounds check with margin
    if (!isPoseOnField(poseEstimate.pose)) {
      return null;
    }

    return poseEstimate;
  }

  private boolean isPoseOnField(Pose2d pose) {
    return pose.getX() >= -FIELD_BORDER_MARGIN_METERS
        && pose.getX() <= FieldInfo.lengthMeters() + FIELD_BORDER_MARGIN_METERS
        && pose.getY() >= -FIELD_BORDER_MARGIN_METERS
        && pose.getY() <= FieldInfo.widthMeters() + FIELD_BORDER_MARGIN_METERS;
  }

  private boolean isRotatingTooFastForMT1() {
    return Math.abs(cachedOmegaDegPerSec) > MAX_ANGULAR_VELOCITY_MT1_DEG_PER_SEC;
  }

  private boolean isRotatingTooFastForMT2() {
    return Math.abs(cachedOmegaDegPerSec) > MAX_ANGULAR_VELOCITY_MT2_DEG_PER_SEC;
  }
}
