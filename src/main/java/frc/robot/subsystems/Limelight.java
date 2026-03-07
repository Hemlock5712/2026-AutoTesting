// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.FieldInfo;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.PoseEstimate;

@Logged(strategy = Strategy.OPT_IN)
public class Limelight extends SubsystemBase {

  private static final double XY_STD_DEV_COEFFICIENT = 0.5;
  private static final double ROTATION_STD_DEV_COEFFICIENT = 5.0;
  private static final double MEGATAG2_ROTATION_STD_DEV = 9999;
  private static final double MAX_ANGULAR_VELOCITY_DEG_PER_SEC = 70;

  private final String m_limelightName;
  private final CommandSwerveDrivetrain m_drivetrain;
  private PoseEstimate lastPoseEstimate = new PoseEstimate();

  public Limelight(String limelightName, CommandSwerveDrivetrain drivetrain) {
    m_limelightName = limelightName;
    m_drivetrain = drivetrain;
  }

  @Override
  public void periodic() {
    updateRobotOrientation();

    PoseEstimate poseEstimate = getValidPoseEstimate();
    if (poseEstimate == null) {
      return;
    }

    lastPoseEstimate = poseEstimate;
    addVisionMeasurement(poseEstimate);
  }

  private void updateRobotOrientation() {
    LimelightHelpers.SetRobotOrientation(
        m_limelightName,
        m_drivetrain.getPose().getRotation().getDegrees(),
        Math.toDegrees(m_drivetrain.getRobotSpeeds().omegaRadiansPerSecond),
        0,
        0,
        0,
        0);
  }

  private PoseEstimate getValidPoseEstimate() {
    PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(m_limelightName);

    if (!LimelightHelpers.validPoseEstimate(poseEstimate)) {
      return null;
    }

    // Use MegaTag2 for single tag estimates
    if (poseEstimate.tagCount == 1) {
      PoseEstimate megaTag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_limelightName);
      if (!LimelightHelpers.validPoseEstimate(megaTag2)) {
        return null;
      }
      poseEstimate = megaTag2;
    }

    if (!isPoseOnField(poseEstimate.pose)) {
      return null;
    }

    if (isRotatingTooFast()) {
      return null;
    }

    return poseEstimate;
  }

  private boolean isPoseOnField(Pose2d pose) {
    return pose.getX() >= 0
        && pose.getX() <= FieldInfo.length().in(Meter)
        && pose.getY() >= 0
        && pose.getY() <= FieldInfo.width().in(Meter);
  }

  private boolean isRotatingTooFast() {
    double angularVelocityDegPerSec =
        Math.toDegrees(m_drivetrain.getRobotSpeeds().omegaRadiansPerSecond);
    return Math.abs(angularVelocityDegPerSec) > MAX_ANGULAR_VELOCITY_DEG_PER_SEC;
  }

  private void addVisionMeasurement(PoseEstimate poseEstimate) {
    double distanceSquared = poseEstimate.avgTagDist * poseEstimate.avgTagDist;
    double xyStdDev = XY_STD_DEV_COEFFICIENT * distanceSquared / poseEstimate.tagCount;
    double rotationStdDev =
        poseEstimate.isMegaTag2
            ? MEGATAG2_ROTATION_STD_DEV
            : ROTATION_STD_DEV_COEFFICIENT * distanceSquared / poseEstimate.tagCount;

    m_drivetrain.addVisionMeasurement(
        poseEstimate.pose,
        poseEstimate.timestampSeconds,
        VecBuilder.fill(xyStdDev, xyStdDev, rotationStdDev));
  }

  @Logged
  public Pose2d getPose() {
    return lastPoseEstimate.pose;
  }

  @Logged
  public double getTimestampSeconds() {
    return lastPoseEstimate.timestampSeconds;
  }

  @Logged
  public double getAvgTagDist() {
    return lastPoseEstimate.avgTagDist;
  }

  @Logged
  public int getTagCount() {
    return lastPoseEstimate.tagCount;
  }
}
