// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.PoseEstimate;

@Logged
public class Limelight extends SubsystemBase {

  /** Limelight name. */
  private final String m_limelightName;

  private final CommandSwerveDrivetrain m_drivetrain;

  /** Cached last valid pose estimate from the Limelight. */
  private PoseEstimate lastPoseEstimate = new PoseEstimate();

  /** Creates a new Limelight. */
  public Limelight(String limelightName, CommandSwerveDrivetrain drivetrain) {
    m_limelightName = limelightName;
    m_drivetrain = drivetrain;
  }

  @Override
  public void periodic() {
    // Called once per scheduler run: pull a fresh pose estimate from Limelight
    // using the WPILib (blue alliance) coordinate frame.
    PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(m_limelightName);

    // Validate that the estimate is trustworthy (e.g., sufficient targets, ambiguity, etc.).
    boolean valid = LimelightHelpers.validPoseEstimate(poseEstimate);
    if (valid) {
      if (poseEstimate.tagCount == 1 && poseEstimate.rawFiducials[0].ambiguity > 0.7) {
        return;
      }

      // if (poseEstimate.pose.getY() > FieldInfo.length().in(Meter)
      //     || poseEstimate.pose.getY() < 0
      //     || poseEstimate.pose.getX() > FieldInfo.width().in(Meter)
      //     || poseEstimate.pose.getX() < 0) {
      //   return;
      // }

      // if (RadiansPerSecond.of(m_drivetrain.getRobotSpeeds().omegaRadiansPerSecond)
      //         .in(DegreesPerSecond)
      //     > 70) {
      //   return;
      // }
      // Cache the latest valid estimate so it can be accessed elsewhere when needed.
      lastPoseEstimate = poseEstimate;

      // Heuristic measurement noise model:
      // - Uncertainty grows with the square of the average tag distance
      // - Uncertainty decreases as more tags are observed
      // These values inform pose estimators how much to trust this measurement.
      // Lower the value higher the trust. https://www.desmos.com/calculator/2e0cd4c36b
      double xyStandardDev = 0.5 * Math.pow(poseEstimate.avgTagDist, 2.0) / poseEstimate.tagCount;
      double rotationStandardDev =
          5.0 * Math.pow(poseEstimate.avgTagDist, 2.0) / poseEstimate.tagCount;

      // Provide the measurement (pose, timestamp, per-axis std devs) to the drivetrain,
      // typically a pose estimator. X/Y in meters, rotation in radians.
      m_drivetrain.addVisionMeasurement(
          poseEstimate.pose,
          poseEstimate.timestampSeconds,
          VecBuilder.fill(xyStandardDev, xyStandardDev, rotationStandardDev));
    }
  }

  /** Getter for last pose estimate. */
  private PoseEstimate getPoseEstimate() {
    return lastPoseEstimate;
  }

  // Expose latest vision values for Epilogue logging/telemetry.
  /** Logging: latest estimated robot pose from vision. */
  public Pose2d getPose() {
    return getPoseEstimate().pose;
  }

  /** Logging: timestamp (seconds) of the last valid vision estimate. */
  public double getTimestampSeconds() {
    return getPoseEstimate().timestampSeconds;
  }

  /** Logging: average tag distance used in the last estimate (meters). */
  public double getAvgTagDist() {
    return getPoseEstimate().avgTagDist;
  }

  /** Logging: number of tags contributing to the last estimate. */
  public double getTagCount() {
    return getPoseEstimate().tagCount;
  }
}
