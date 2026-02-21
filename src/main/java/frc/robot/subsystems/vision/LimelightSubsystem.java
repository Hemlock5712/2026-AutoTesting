package frc.robot.subsystems.vision;
// package frc.robot.subsystems.vision.apriltags;

// import com.ctre.phoenix6.HootAutoReplay;
// import com.ctre.phoenix6.Utils;
// import edu.wpi.first.epilogue.Logged;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform2d;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.utils.LimelightHelpers;
// import java.util.Optional;

// /** Limelight vision subsystem with HootReplay support and latency compensation. */
// @Logged
// public class LimelightSubsystem extends SubsystemBase {
//   private final String limelightName;
//   private final CommandSwerveDrivetrain drivetrain;

//   // Robot localization (MegaTag)
//   private Pose2d robotPose = new Pose2d();
//   private double robotPoseTimestamp = 0.0;
//   private int tagCount = 0;
//   private double avgTagDistance = 0.0;

//   // Target tracking
//   private boolean targetVisible = false;
//   private long targetId = -1;
//   private Pose2d targetPoseRobotSpace = new Pose2d();
//   private double timestampOffset = 0.0;
//   private double captureLatencyMs = 0.0;
//   private double pipelineLatencyMs = 0.0;

//   private final HootAutoReplay autoReplay;

//   // Processed outputs
//   private Optional<Pose2d> targetPoseFieldSpace = Optional.empty();
//   private boolean latencyCompensationActive = false;
//   private double totalLatencyMs = 0.0;

//   public LimelightSubsystem(String limelightName, CommandSwerveDrivetrain drivetrain) {
//     this.limelightName = limelightName;
//     this.drivetrain = drivetrain;

//     this.autoReplay =
//         new HootAutoReplay()
//             .withStruct(
//                 "Limelight/" + limelightName + "/RobotPose",
//                 Pose2d.struct,
//                 () -> robotPose,
//                 val -> robotPose = val)
//             .withDouble(
//                 "Limelight/" + limelightName + "/RobotPoseTimestamp",
//                 () -> robotPoseTimestamp,
//                 val -> robotPoseTimestamp = val)
//             .withInteger(
//                 "Limelight/" + limelightName + "/TagCount",
//                 () -> tagCount,
//                 val -> tagCount = (int) val)
//             .withDouble(
//                 "Limelight/" + limelightName + "/AvgTagDistance",
//                 () -> avgTagDistance,
//                 val -> avgTagDistance = val)
//             .withBoolean(
//                 "Limelight/" + limelightName + "/TargetVisible",
//                 () -> targetVisible,
//                 val -> targetVisible = val)
//             .withInteger(
//                 "Limelight/" + limelightName + "/TargetID", () -> targetId, val -> targetId =
// val)
//             .withStruct(
//                 "Limelight/" + limelightName + "/TargetPose_RobotSpace",
//                 Pose2d.struct,
//                 () -> targetPoseRobotSpace,
//                 val -> targetPoseRobotSpace = val)
//             .withDouble(
//                 "Limelight/" + limelightName + "/TimestampOffset",
//                 () -> timestampOffset,
//                 val -> timestampOffset = val)
//             .withDouble(
//                 "Limelight/" + limelightName + "/CaptureLatency",
//                 () -> captureLatencyMs,
//                 val -> captureLatencyMs = val)
//             .withDouble(
//                 "Limelight/" + limelightName + "/PipelineLatency",
//                 () -> pipelineLatencyMs,
//                 val -> pipelineLatencyMs = val);
//   }

//   @Override
//   public void periodic() {
//     if (!Utils.isReplay()) {
//       fetchInputs();
//     }
//     autoReplay.update();
//     processInputs();
//   }

//   private void fetchInputs() {
//     LimelightHelpers.PoseEstimate poseEstimate =
//         LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

//     if (poseEstimate != null && poseEstimate.tagCount > 0) {
//       robotPose = poseEstimate.pose;
//       robotPoseTimestamp = poseEstimate.timestampSeconds;
//       tagCount = poseEstimate.tagCount;
//       avgTagDistance = poseEstimate.avgTagDist;
//     } else {
//       robotPose = new Pose2d();
//       robotPoseTimestamp = 0.0;
//       tagCount = 0;
//       avgTagDistance = 0.0;
//     }

//     if (poseEstimate != null
//         && poseEstimate.rawFiducials != null
//         && poseEstimate.rawFiducials.length > 0) {
//       targetVisible = true;
//       targetId = poseEstimate.rawFiducials[0].id;

//       // Get separate latencies for proper replay timestamp reconstruction
//       captureLatencyMs = LimelightHelpers.getLimelightNTDouble(limelightName, "cl");
//       pipelineLatencyMs = LimelightHelpers.getLimelightNTDouble(limelightName, "tl");

//       double[] targetPoseArray = LimelightHelpers.getTargetPose_RobotSpace(limelightName);
//       targetPoseRobotSpace = LimelightHelpers.toPose2D(targetPoseArray);

//       double timestampMicros = LimelightHelpers.getLimelightNTDouble(limelightName, "ts_rio");
//       double timestampSeconds = timestampMicros / 1000000.0;
//       double convertedTimestamp = Utils.fpgaToCurrentTime(timestampSeconds);
//       timestampOffset = convertedTimestamp - Utils.getCurrentTimeSeconds();
//     } else {
//       targetVisible = false;
//       targetId = -1;
//       targetPoseRobotSpace = new Pose2d();
//       timestampOffset = 0.0;
//       captureLatencyMs = 0.0;
//       pipelineLatencyMs = 0.0;
//     }
//   }

//   private void processInputs() {
//     if (!targetVisible) {
//       targetPoseFieldSpace = Optional.empty();
//       latencyCompensationActive = false;
//       totalLatencyMs = 0.0;
//       return;
//     }

//     double timestamp = Utils.getCurrentTimeSeconds() + timestampOffset;
//     totalLatencyMs = captureLatencyMs + pipelineLatencyMs;

//     Optional<Pose2d> historicalPose = drivetrain.samplePoseAt(timestamp);

//     if (historicalPose.isPresent()) {
//       targetPoseFieldSpace =
//           Optional.of(
//               historicalPose
//                   .get()
//                   .plus(
//                       new Transform2d(
//                           targetPoseRobotSpace.getTranslation(),
//                           targetPoseRobotSpace.getRotation())));
//       latencyCompensationActive = true;
//     } else {
//       targetPoseFieldSpace =
//           Optional.of(
//               drivetrain
//                   .getState()
//                   .Pose
//                   .plus(
//                       new Transform2d(
//                           targetPoseRobotSpace.getTranslation(),
//                           targetPoseRobotSpace.getRotation())));
//       latencyCompensationActive = false;
//     }
//   }

//   /** Gets robot pose from MegaTag. */
//   public Optional<Pose2d> getRobotPose() {
//     return tagCount > 0 ? Optional.of(robotPose) : Optional.empty();
//   }

//   /** Gets timestamp for robot pose (latency-adjusted by Limelight). */
//   public double getRobotPoseTimestamp() {
//     return robotPoseTimestamp;
//   }

//   /** Gets number of AprilTags used for robot pose. */
//   public int getTagCount() {
//     return tagCount;
//   }

//   /** Gets average distance to tags. */
//   public double getAvgTagDistance() {
//     return avgTagDistance;
//   }

//   /** Gets target pose in field coordinates with latency compensation. */
//   public Optional<Pose2d> getTargetPoseFieldSpace() {
//     return targetPoseFieldSpace;
//   }

//   /** Gets target pose relative to robot. */
//   public Pose2d getTargetPoseRobotSpace() {
//     return targetPoseRobotSpace;
//   }

//   /** Checks if target is visible. */
//   public boolean isTargetVisible() {
//     return targetVisible;
//   }

//   /** Gets tracked target ID (-1 if none). */
//   public long getTargetId() {
//     return targetId;
//   }

//   /** Gets distance to target from robot's current position. */
//   public Optional<Double> getDistanceToTarget() {
//     if (!targetVisible || targetPoseFieldSpace.isEmpty()) {
//       return Optional.empty();
//     }

//     Pose2d currentPose = drivetrain.getState().Pose;
//     double distance =
//         currentPose.getTranslation().getDistance(targetPoseFieldSpace.get().getTranslation());
//     return Optional.of(distance);
//   }

//   /** Gets angle to target from robot's current position. */
//   public Optional<Rotation2d> getAngleToTarget() {
//     if (!targetVisible || targetPoseFieldSpace.isEmpty()) {
//       return Optional.empty();
//     }

//     Pose2d currentPose = drivetrain.getState().Pose;
//     var toTarget =
// targetPoseFieldSpace.get().getTranslation().minus(currentPose.getTranslation());
//     Rotation2d angle = new Rotation2d(toTarget.getX(), toTarget.getY());
//     return Optional.of(angle);
//   }

//   /** Checks if latency compensation is active. */
//   public boolean isLatencyCompensationActive() {
//     return latencyCompensationActive;
//   }

//   /** Gets total vision latency (capture + pipeline). */
//   public double getTotalVisionLatency() {
//     return totalLatencyMs;
//   }

//   /** Gets capture latency separately. */
//   public double getCaptureLatency() {
//     return captureLatencyMs;
//   }

//   /** Gets pipeline latency separately. */
//   public double getPipelineLatency() {
//     return pipelineLatencyMs;
//   }
// }
