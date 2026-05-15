package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

/** Logged vision data for one camera. The IO does the filtering; this just stores the result. */
@AutoLog
public class VisionInputs {
  /**
   * True if the camera produced a new frame this robot loop. Cameras run at ~40 FPS while the robot
   * loop runs at 50 Hz, so this is normally false 1 in 5 ticks - that's not a problem.
   */
  public boolean newFrame = false;

  public boolean hasObservation = false;
  public Pose2d latestPose = Pose2d.kZero;
  public double latestTimestampSeconds = 0.0;
  public int tagCount = 0;

  /** Fiducial IDs of the tags used in this observation. Drives per-tag deweight in Vision. */
  public int[] tagIds = new int[0];

  public double avgTagDistance = 0.0;
  public double maxAmbiguity = 0.0;
  public boolean isMegaTag2 = false;
}
