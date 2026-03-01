package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.TalonFXUtil;

@Logged
public class Spindexer extends SubsystemBase {
  protected final double VELOCITY_TOLERANCE = 0.2;

  protected final TalonFX spindexer = new TalonFX(20, TunerConstants.kCANBus);

  protected final TalonFX kicker = new TalonFX(21, TunerConstants.kCANBus);

  private final VelocityVoltage velocityOut = new VelocityVoltage(0);

  protected TalonFXConfiguration spindexerConfig = new TalonFXConfiguration();

  protected TalonFXConfiguration kickerConfig = new TalonFXConfiguration();

  Alert motorConfigAlert = new Alert("Spindexer Motor Configuration Failed", AlertType.kError);

  Alert kickerMotorConfigAlert = new Alert("Kicker Motor Configuration Failed", AlertType.kError);

  public Spindexer() {
    applyConfigs();
  }

  public void setVelocity(double velocity) {
    spindexer.setControl(velocityOut.withVelocity(velocity));
  }

  public void setKickerVelocity(double velocity) {
    kicker.setControl(velocityOut.withVelocity(velocity));
  }

  public double getVelocity() {
    return spindexer.getVelocity().getValueAsDouble();
  }

  public boolean isAtTarget() {
    return spindexer.getVelocity().isNear(velocityOut.Velocity, VELOCITY_TOLERANCE);
  }

  public Command startCommand() {
    return runOnce(() -> setVelocity(12));
  }

  public Command stopCommand() {
    return runOnce(() -> setVelocity(0));
  }

  public Command startKickerCommand() {
    return runOnce(() -> setKickerVelocity(17.2));
  }

  public Command startKickerVoltageCommand() {
    return runOnce(() -> kicker.setVoltage(12));
  }

  public Command stopKickerCommand() {
    return runOnce(() -> setKickerVelocity(0));
  }

  public void applyConfigs() {
    spindexerConfig.Slot0.kS = 0.42; // Static friction compensation
    spindexerConfig.Slot0.kP = 2.0; // Proportional gain
    spindexerConfig.Slot0.kD = 0.0; // Derivative gain (damping to reduce overshoot)
    spindexerConfig.Slot0.kV = 0.91;
    // MotionMagic settings - with SensorToMechanismRatio set, units are mechanism
    // rotations
    // Cruise velocity: max SPINDEXER speed during motion profile (RPS)
    // Acceleration: how quickly the SPINDEXER speeds up/slows down (RPS²)
    spindexerConfig.MotionMagic.MotionMagicCruiseVelocity = 0; // RPS
    spindexerConfig.MotionMagic.MotionMagicAcceleration = 0; // RPS²

    spindexerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    spindexerConfig.Feedback.SensorToMechanismRatio = 9.0;

    kickerConfig.Slot0.kS = 0.25; // Static friction compensation
    kickerConfig.Slot0.kP = 0.1; // Proportional gain
    kickerConfig.Slot0.kD = 0; // Derivative gain (damping to reduce overshoot)
    // MotionMagic settings - with SensorToMechanismRatio set, units are mechanism
    kickerConfig.Slot0.kV = 0.685;
    // rotations
    // Cruise velocity: max SPINDEXER speed during motion profile (RPS)
    // Acceleration: how quickly the SPINDEXER speeds up/slows down (RPS²)
    kickerConfig.MotionMagic.MotionMagicCruiseVelocity = 0; // RPS
    kickerConfig.MotionMagic.MotionMagicAcceleration = 0; // RPS²

    kickerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    kickerConfig.Feedback.SensorToMechanismRatio = 7.0;

    boolean success = TalonFXUtil.applyConfigWithRetries(spindexer, spindexerConfig);
    motorConfigAlert.set(!success);
    success = TalonFXUtil.applyConfigWithRetries(kicker, kickerConfig);
    kickerMotorConfigAlert.set(!success);
  }
}
