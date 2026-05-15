package frc.robot.subsystems.examples.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Example subsystem demonstrating the AdvantageKit IO pattern with closed-loop velocity control.
 *
 * <p>Wire in {@code RobotContainer}:
 *
 * <pre>{@code
 * flywheel = switch (Constants.getMode()) {
 *   case REAL -> new Flywheel(new FlywheelIOTalonFX(20));
 *   case SIM  -> new Flywheel(new FlywheelIOSim());
 *   case REPLAY -> new Flywheel(new FlywheelIO() {});
 * };
 *
 * driver.x().whileTrue(flywheel.runAtRPM(3000));
 * }</pre>
 */
public class Flywheel extends SubsystemBase {

  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
  }

  /** Returns true once the measured velocity is within {@code tolerance} RPM of the target. */
  public boolean atSpeed(double targetRPM, double toleranceRPM) {
    return Math.abs(inputs.velocityRPM - targetRPM) < toleranceRPM;
  }

  /** Spin up to {@code rpm} and hold until the command is cancelled. */
  public Command runAtRPM(double rpm) {
    return startEnd(() -> io.setVelocityRPM(rpm), io::stop).withName("Flywheel " + rpm + " RPM");
  }
}
