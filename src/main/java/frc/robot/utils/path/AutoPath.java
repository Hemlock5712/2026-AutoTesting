package frc.robot.utils.path;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import frc.robot.utils.FieldInfo;

/**
 * Type-safe enum of pre-planned autonomous paths.
 *
 * <p>Each entry wraps a Choreo trajectory re-parameterized by arc-length for distance-based
 * following. Blue and red alliance variants are pre-computed at load time.
 *
 * <p>Add entries here as you design trajectories in Choreo. If a trajectory name doesn't match a
 * deployed .traj file, construction will throw at startup (fail-fast).
 *
 * <p>When Choreo code generation is set up, replace the string names with {@code ChoreoTraj.X}
 * constants for compile-time safety.
 *
 * <p>Usage:
 *
 * <pre>{@code
 * autoCommands.followPath(AutoPath.EXAMPLE_PATH);
 * }</pre>
 */
public enum AutoPath {
// Add paths here as they are created in Choreo:
// EXAMPLE_PATH("ExamplePath"),
;

  private final ArcLengthTrajectory blue;
  private final ArcLengthTrajectory red;

  AutoPath(String choreoName) {
    Trajectory<SwerveSample> traj =
        Choreo.<SwerveSample>loadTrajectory(choreoName)
            .orElseThrow(
                () ->
                    new RuntimeException(
                        "Failed to load Choreo trajectory: "
                            + choreoName
                            + ". Ensure the .traj file exists in deploy/choreo/"));
    this.blue = ArcLengthTrajectory.fromChoreo(traj);
    this.red = ArcLengthTrajectory.fromChoreo(traj.flipped());
  }

  /**
   * Returns the alliance-appropriate arc-length trajectory.
   *
   * @return Blue trajectory on blue alliance, flipped trajectory on red alliance
   */
  public ArcLengthTrajectory get() {
    return FieldInfo.shouldFlip() ? red : blue;
  }
}
