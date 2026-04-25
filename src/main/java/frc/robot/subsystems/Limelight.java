// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.FieldInfo;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.PoseEstimate;

import java.util.List;
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
    for (int i = 0; i < cameraCount; i++) {
      cameras[i] = new CameraState(cameraNames.get(i));
    }
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

    // Collect valid poses and fuse into a single measurement via inverse-variance weighting.
    // This calls addVisionMeasurement at most ONCE, avoiding repeated write-lock contention
    // with the 250Hz odometry thread.
    double sumX = 0, sumY = 0, sumTheta = 0;
    double sumInvVarXY = 0, sumInvVarTheta = 0;
    double latestTimestamp = 0;
    int validCount = 0;

    for (int i = 0; i < cameraCount; i++) {
      CameraState camera = cameras[i];
      PoseEstimate pe = getValidPoseEstimate(camera);
      if (pe == null) continue;

      double distanceFactor = Math.pow(pe.avgTagDist, 1.2);
      double effectiveTags = Math.min(MAX_EFFECTIVE_TAG_COUNT, pe.tagCount);
      double tagFactor = effectiveTags * effectiveTags;

      double xyStdDev = XY_STD_DEV_COEFFICIENT * distanceFactor / tagFactor;
      double rotStdDev =
          pe.isMegaTag2
              ? MEGATAG2_ROTATION_STD_DEV
              : ROTATION_STD_DEV_COEFFICIENT * distanceFactor / tagFactor;

      double invVarXY = 1.0 / (xyStdDev * xyStdDev);
      sumX += pe.pose.getX() * invVarXY;
      sumY += pe.pose.getY() * invVarXY;
      sumInvVarXY += invVarXY;

      if (Double.isFinite(rotStdDev)) {
        double invVarTheta = 1.0 / (rotStdDev * rotStdDev);
        sumTheta += pe.pose.getRotation().getRadians() * invVarTheta;
        sumInvVarTheta += invVarTheta;
      }

      if (pe.timestampSeconds > latestTimestamp) {
        latestTimestamp = pe.timestampSeconds;
      }
      validCount++;
    }

    if (validCount > 0) {
      double fusedX = sumX / sumInvVarXY;
      double fusedY = sumY / sumInvVarXY;
      double fusedXYStdDev = 1.0 / Math.sqrt(sumInvVarXY);

      double fusedThetaStdDev;
      double fusedThetaRad;
      if (sumInvVarTheta > 0) {
        fusedThetaRad = sumTheta / sumInvVarTheta;
        fusedThetaStdDev = 1.0 / Math.sqrt(sumInvVarTheta);
      } else {
        fusedThetaRad = m_drivetrain.getPose().getRotation().getRadians();
        fusedThetaStdDev = MEGATAG2_ROTATION_STD_DEV;
      }

      fusedStdDevs.set(0, 0, fusedXYStdDev);
      fusedStdDevs.set(1, 0, fusedXYStdDev);
      fusedStdDevs.set(2, 0, fusedThetaStdDev);

      m_drivetrain.addVisionMeasurement(
          new Pose2d(fusedX, fusedY, new Rotation2d(fusedThetaRad)),
          latestTimestamp,
          fusedStdDevs);
    }
    Logger.recordOutput("Timing/LimelightMs", (System.nanoTime() - _t) / 1e6);
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
