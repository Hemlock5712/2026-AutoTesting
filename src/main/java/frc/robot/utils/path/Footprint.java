package frc.robot.utils.path;

/**
 * Robot footprint in the robot frame, queried each clamp tick. {@link #offsetX}/{@link #offsetY}
 * shift the bounding-box center off the chassis pivot — useful when an intake extends past the
 * front. Implementations may compute values fresh each call; the getters fire on the 250 Hz fast
 * loop, so keep them cheap.
 *
 * <p><b>Thread safety:</b> getters are invoked from the drive's 250 Hz Notifier thread, while any
 * subsystem state they read (e.g. an intake's "deployed" flag) is typically written on the 50 Hz
 * main thread. Implementations MUST publish that state safely — either {@code volatile} primitives,
 * an atomic reference, or values written under a lock the reader also takes. Plain non-volatile
 * fields will silently tear or stale.
 */
public interface Footprint {

  double halfX();

  double halfY();

  default double offsetX() {
    return 0.0;
  }

  default double offsetY() {
    return 0.0;
  }

  /** Fixed-size footprint centered on the robot. */
  static Footprint fixed(double halfX, double halfY) {
    return new Footprint() {
      @Override
      public double halfX() {
        return halfX;
      }

      @Override
      public double halfY() {
        return halfY;
      }
    };
  }
}
