package frc.robot.subsystems.examples.arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Example arm subsystem with profiled position control. Mirrors the {@link
 * frc.robot.subsystems.examples.flywheel.Flywheel} pattern — copy this for elevators, wrists, or
 * any other position-controlled mechanism.
 *
 * <p>Wire in {@code RobotContainer}:
 *
 * <pre>{@code
 * arm = switch (Constants.getMode()) {
 *   case REAL -> new Arm(new ArmIOTalonFX(21));
 *   case SIM  -> new Arm(new ArmIOSim());
 *   case REPLAY -> new Arm(new ArmIO() {});
 * };
 *
 * driver.y().onTrue(arm.goTo(Arm.Position.HIGH));
 * driver.b().onTrue(arm.goTo(Arm.Position.STOW));
 * }</pre>
 */
public class Arm extends SubsystemBase {

  /** Pre-defined arm angles. Tune to your mechanism. */
  public enum Position {
    STOW(Units.degreesToRadians(0)),
    LOW(Units.degreesToRadians(30)),
    HIGH(Units.degreesToRadians(85));

    public final double rad;

    Position(double rad) {
      this.rad = rad;
    }
  }

  private static final double TOLERANCE_RAD = Units.degreesToRadians(2.0);

  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  public Arm(ArmIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
  }

  public boolean atSetpoint() {
    return Math.abs(inputs.positionRad - inputs.setpointRad) < TOLERANCE_RAD;
  }

  /** Move to a preset and hold there until interrupted. */
  public Command goTo(Position p) {
    return run(() -> io.setPosition(p.rad)).withName("Arm → " + p.name());
  }
}
