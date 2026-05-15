package frc.robot.subsystems.examples.flywheel;

import org.littletonrobotics.junction.AutoLog;

/**
 * Example IO interface. Copy this pattern for every new subsystem: one {@code XxxIO} interface, one
 * {@code XxxIOInputs} class with {@link AutoLog}, and one impl per hardware target ({@code
 * XxxIOTalonFX}, {@code XxxIOSim}, …). AdvantageKit auto-generates {@code
 * FlywheelIOInputsAutoLogged}.
 */
public interface FlywheelIO {

  @AutoLog
  class FlywheelIOInputs {
    public double velocityRPM = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  /** Pull fresh sensor values into {@code inputs}. Called every loop by the subsystem. */
  default void updateInputs(FlywheelIOInputs inputs) {}

  /** Set the target velocity in RPM. */
  default void setVelocityRPM(double rpm) {}

  /** Open-loop voltage for testing. Bypasses closed-loop control. */
  default void setVoltage(double volts) {}

  /** Stop the motor (open-loop 0 V). */
  default void stop() {}
}
