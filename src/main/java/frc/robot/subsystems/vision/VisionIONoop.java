package frc.robot.subsystems.vision;

/**
 * No-op vision IO. Used in SIM for cameras that aren't synthesized and in REPLAY for every camera —
 * the inputs come from the log, the IO just needs to expose the right name for AKit log keys.
 */
public final class VisionIONoop implements VisionIO {

  private final String name;

  public VisionIONoop(String name) {
    this.name = name;
  }

  @Override
  public String getName() {
    return name;
  }
}
