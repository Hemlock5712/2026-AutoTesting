package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {

  public enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  public static Mode getMode() {
    if (!RobotBase.isReal() && System.getProperty("frc.replay.input") != null) {
      return Mode.REPLAY;
    }
    return RobotBase.isReal() ? Mode.REAL : Mode.SIM;
  }

  public static final double LOOP_PERIOD_SECONDS = 0.02;

  private Constants() {}
}
