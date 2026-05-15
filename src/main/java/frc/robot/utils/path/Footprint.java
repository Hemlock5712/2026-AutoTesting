package frc.robot.utils.path;

/**
 * Robot footprint in the robot frame, consumed by the avoidance clamp. {@code offsetX}/{@code
 * offsetY} shift the bounding-box center off the chassis pivot — useful when an intake extends past
 * the front of the bumpers.
 *
 * <p>If you want the footprint to change at runtime (e.g. an intake deploying), construct a new
 * {@code Footprint} on the writing thread and publish it through a {@code volatile} reference the
 * 250 Hz reader picks up. Don't mutate a single instance — records are immutable.
 */
public record Footprint(double halfX, double halfY, double offsetX, double offsetY) {

  /** Fixed-size footprint centered on the chassis pivot. */
  public static Footprint fixed(double halfX, double halfY) {
    return new Footprint(halfX, halfY, 0.0, 0.0);
  }
}
