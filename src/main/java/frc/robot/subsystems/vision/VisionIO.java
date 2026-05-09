package frc.robot.subsystems.vision;

/**
 * Interface for one AprilTag camera. Each implementation handles its own filtering of bad
 * observations. {@link Vision} just adds them to the pose estimator.
 */
public interface VisionIO {

  default void updateInputs(VisionInputsAutoLogged inputs) {}

  /** Tells the camera the robot's current heading and how fast it's spinning (for MegaTag2). */
  default void setRobotOrientation(double yawDegrees, double yawRateDegPerSec) {}

  /** Used in log keys and as the Limelight's NetworkTables name. */
  String getName();
}
