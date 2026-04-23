// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.FieldInfo;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.PoseEstimate;
import frc.robot.utils.LoopProfiler;
import org.littletonrobotics.junction.Logger;

public class Limelight extends SubsystemBase {

  // --- Standard Deviation Formula ---
  // Formula: coefficient * pow(avgTagDist, 1.2) / pow(tagCount, 2.0) * stdDevFactor
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

  private final String m_limelightName;
  private final CommandSwerveDrivetrain m_drivetrain;
  private final double m_stdDevFactor;
  private PoseEstimate lastPoseEstimate = new PoseEstimate();

  /**
   * Creates a Limelight subsystem.
   *
   * @param limelightName NetworkTables name (e.g., "limelight-front").
   * @param drivetrain Swerve drivetrain for pose estimation and gyro data.
   * @param stdDevFactor Per-camera trust multiplier. 1.0 = normal trust. Higher = less trust. Use
   *     to account for camera quality or mounting position (e.g., 2.0 for a camera with a worse
   *     viewing angle).
   */
  public Limelight(String limelightName, CommandSwerveDrivetrain drivetrain, double stdDevFactor) {
    super(limelightName);
    m_limelightName = limelightName;
    m_drivetrain = drivetrain;
    m_stdDevFactor = stdDevFactor;
  }

  public Limelight(String limelightName, CommandSwerveDrivetrain drivetrain) {
    this(limelightName, drivetrain, 1.0);
  }

  @Override
  public void periodic() {
    LoopProfiler.measure(
        "Subsystems/" + m_limelightName,
        () -> {
          PoseEstimate poseEstimate =
              LoopProfiler.measure(m_limelightName + "/PoseEstimate", this::getValidPoseEstimate);
          if (poseEstimate != null) {
            lastPoseEstimate = poseEstimate;
            LoopProfiler.measure(
                m_limelightName + "/AddVisionMeasurement",
                () -> addVisionMeasurement(poseEstimate));
          }

          LoopProfiler.measure(
              m_limelightName + "/Logging",
              () -> {
                Logger.recordOutput(m_limelightName + "/Pose", lastPoseEstimate.pose);
                Logger.recordOutput(
                    m_limelightName + "/TimestampSeconds", lastPoseEstimate.timestampSeconds);
                Logger.recordOutput(m_limelightName + "/AvgTagDist", lastPoseEstimate.avgTagDist);
                Logger.recordOutput(m_limelightName + "/TagCount", lastPoseEstimate.tagCount);
              });
          // Logger.recordOutput(m_limelightName + "/TagFilter", lastPoseEstimate.)
        });
  }

  public void updateRobotOrientationNoFlush() {
    LoopProfiler.measure(
        m_limelightName + "/SetOrientationNoFlush",
        () ->
            LimelightHelpers.SetRobotOrientation_NoFlush(
                m_limelightName,
                m_drivetrain.getPose().getRotation().getDegrees(),
                Math.toDegrees(m_drivetrain.getRobotSpeeds().omegaRadiansPerSecond),
                0,
                0,
                0,
                0));
  }

  public static void flushOrientationUpdates() {
    LoopProfiler.measure("Limelight/FlushOrientationUpdates", LimelightHelpers::Flush);
  }

  private PoseEstimate getValidPoseEstimate() {
    PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(m_limelightName);

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
      PoseEstimate megaTag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_limelightName);
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
        && pose.getX() <= FieldInfo.length().in(Meter) + FIELD_BORDER_MARGIN_METERS
        && pose.getY() >= -FIELD_BORDER_MARGIN_METERS
        && pose.getY() <= FieldInfo.width().in(Meter) + FIELD_BORDER_MARGIN_METERS;
  }

  private boolean isRotatingTooFastForMT1() {
    double angularVelocityDegPerSec =
        Math.toDegrees(m_drivetrain.getRobotSpeeds().omegaRadiansPerSecond);
    return Math.abs(angularVelocityDegPerSec) > MAX_ANGULAR_VELOCITY_MT1_DEG_PER_SEC;
  }

  private boolean isRotatingTooFastForMT2() {
    double angularVelocityDegPerSec =
        Math.toDegrees(m_drivetrain.getRobotSpeeds().omegaRadiansPerSecond);
    return Math.abs(angularVelocityDegPerSec) > MAX_ANGULAR_VELOCITY_MT2_DEG_PER_SEC;
  }

  private void addVisionMeasurement(PoseEstimate poseEstimate) {
    double distanceFactor = Math.pow(poseEstimate.avgTagDist, 1.2);
    double effectiveTags = Math.min(MAX_EFFECTIVE_TAG_COUNT, poseEstimate.tagCount);
    double tagFactor = Math.pow(effectiveTags, 2.0);

    double xyStdDev = XY_STD_DEV_COEFFICIENT * distanceFactor / tagFactor * m_stdDevFactor;
    double rotationStdDev =
        poseEstimate.isMegaTag2
            ? MEGATAG2_ROTATION_STD_DEV
            : ROTATION_STD_DEV_COEFFICIENT * distanceFactor / tagFactor * m_stdDevFactor;

    m_drivetrain.addVisionMeasurement(
        poseEstimate.pose,
        poseEstimate.timestampSeconds,
        VecBuilder.fill(xyStdDev, xyStdDev, rotationStdDev));
  }

  /**
   * Sets which AprilTag IDs this Limelight will use for pose estimation. Tags not in the list will
   * be ignored. Pass an empty array to clear the filter (accept all).
   *
   * @param validIDs Array of valid AprilTag IDs, or empty array to accept all.
   */
  public void setTagFilter(int[] validIDs) {
    LimelightHelpers.SetFiducialIDFiltersOverride(m_limelightName, validIDs);
  }

  public Pose2d getPose() {
    return lastPoseEstimate.pose;
  }

  public double getTimestampSeconds() {
    return lastPoseEstimate.timestampSeconds;
  }

  public double getAvgTagDist() {
    return lastPoseEstimate.avgTagDist;
  }

  public int getTagCount() {
    return lastPoseEstimate.tagCount;
  }
}
