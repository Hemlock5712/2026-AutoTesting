package frc.robot.subsystems.examples.arm;

import org.littletonrobotics.junction.AutoLog;

/** Example IO interface for a single-jointed arm with closed-loop position control. */
public interface ArmIO {

  @AutoLog
  class ArmIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double setpointRad = 0.0;
  }

  default void updateInputs(ArmIOInputs inputs) {}

  /** Drive to the given angle (radians, 0 = horizontal forward). */
  default void setPosition(double rad) {}

  /** Open-loop voltage for tuning or manual jog. */
  default void setVoltage(double volts) {}

  default void stop() {}
}
