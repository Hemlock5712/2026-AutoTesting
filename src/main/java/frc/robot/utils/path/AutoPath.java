package frc.robot.utils.path;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import frc.robot.autonomous.ChoreoTraj;
import frc.robot.utils.FieldInfo;

/**
 * List of all auto paths the robot can run.
 *
 * <p>Each entry loads a Choreo trajectory and prepares blue and red alliance versions ahead of
 * time. The names come from {@link ChoreoTraj}, which is auto-generated from the Choreo files.
 *
 * <p>Usage:
 *
 * <pre>{@code
 * autoCommands.followPath(AutoPath.NEW_PATH);
 * }</pre>
 */
public enum AutoPath {
  NEW_PATH(ChoreoTraj.NewPath),
  ;

  private final ChoreoTraj choreoTraj;
  private final Trajectory<SwerveSample> trajectory;
  private final ArcLengthTrajectory blue;
  private final ArcLengthTrajectory red;

  AutoPath(ChoreoTraj choreoTraj) {
    this.choreoTraj = choreoTraj;
    this.trajectory =
        Choreo.<SwerveSample>loadTrajectory(choreoTraj.name())
            .orElseThrow(
                () ->
                    new RuntimeException(
                        "Failed to load Choreo trajectory: "
                            + choreoTraj.name()
                            + ". Ensure the .traj file exists in deploy/choreo/"));
    this.blue = ArcLengthTrajectory.fromChoreo(trajectory);
    this.red = ArcLengthTrajectory.fromChoreo(trajectory.flipped());
  }

  /** Returns the path for our current alliance (flipped on red). */
  public ArcLengthTrajectory get() {
    return FieldInfo.shouldFlip() ? red : blue;
  }

  /** Raw Choreo trajectory (for reading event markers). */
  public Trajectory<SwerveSample> trajectory() {
    return trajectory;
  }

  /** Choreo metadata (start/end poses, total time, etc.). */
  public ChoreoTraj choreoTraj() {
    return choreoTraj;
  }
}
