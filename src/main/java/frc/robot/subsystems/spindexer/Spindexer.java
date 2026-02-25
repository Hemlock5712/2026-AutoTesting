package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.TalonFXUtil;

@Logged
public class Spindexer extends SubsystemBase {
  protected final TalonFX leader = new TalonFX(51, CANBus.roboRIO());

  protected final TalonFX kicker = new TalonFX(52, CANBus.roboRIO());

  private final VelocityTorqueCurrentFOC velocityOut = new VelocityTorqueCurrentFOC(0);

  Alert motorConfigAlert = new Alert("Turret Motor Configuration Failed", AlertType.kError);

  public Spindexer() {
    TalonFXConfiguration leaderConfig = new TalonFXConfiguration();

    leaderConfig.Slot0.kS = 1.0; // Static friction compensation
    leaderConfig.Slot0.kP = 20; // Proportional gain
    leaderConfig.Slot0.kD = 0; // Derivative gain (damping to reduce overshoot)
    // MotionMagic settings - with SensorToMechanismRatio set, units are mechanism
    // rotations
    // Cruise velocity: max SPINDEXER speed during motion profile (RPS)
    // Acceleration: how quickly the SPINDEXER speeds up/slows down (RPS²)
    leaderConfig.MotionMagic.MotionMagicCruiseVelocity = 30.0; // RPS
    leaderConfig.MotionMagic.MotionMagicAcceleration = 60.0; // RPS²

    boolean success = TalonFXUtil.applyConfigWithRetries(leader, leaderConfig);
    motorConfigAlert.set(!success);
  }

  public void setVelocity(double velocity) {
    leader.setControl(velocityOut.withVelocity(velocity));
  }

  public void setKickerVelocity(double velocity) {
    kicker.setControl(velocityOut.withVelocity(velocity));
  }

  public Command startCommand() {
    return runOnce(() -> setVelocity(12));
  }

  public Command stopCommand() {
    return runOnce(() -> setVelocity(0));
  }

  public Command startKickerCommand() {
    return runOnce(() -> setKickerVelocity(10));
  }

  public Command stopKickerCommand() {
    return runOnce(() -> setKickerVelocity(0));
  }
}
