package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.TalonFXUtil;
import org.littletonrobotics.junction.AutoLogOutput;

public class IntakeWheels extends SubsystemBase {

  private final TalonFX wheel = new TalonFX(23, TunerConstants.kCANBus);

  private TalonFXConfiguration wheelConfig = new TalonFXConfiguration();

  Alert motorConfigAlert = new Alert("Intake Wheel Motor Configuration Failed", AlertType.kError);

  VelocityVoltage voltageOut = new VelocityVoltage(0);

  public IntakeWheels() {
    wheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    wheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    wheelConfig.Feedback.SensorToMechanismRatio = 2.33;

    wheelConfig.Slot0.kS = 0.3; // Static friction
    wheelConfig.Slot0.kP = 0.1; // Proportional gain (speed of correction)
    wheelConfig.Slot0.kV = 0.288; // Velocity feedforward

    boolean success = TalonFXUtil.applyConfigWithRetries(wheel, wheelConfig);
    motorConfigAlert.set(!success);

    wheel.optimizeBusUtilization();
  }

  @Override
  public void periodic() {}

  public Command runIntake() {
    return runOnce(() -> wheel.setControl(voltageOut.withVelocity(20)));
  }

  public Command reverseIntake() {
    return runOnce(() -> wheel.setControl(voltageOut.withVelocity(-20)));
  }

  public Command runFast() {
    return runOnce(() -> wheel.setControl(voltageOut.withVelocity(37)));
  }

  public Command stopWheel() {
    return runOnce(() -> wheel.stopMotor());
  }

  @AutoLogOutput
  public double getVelocity() {
    return wheel.getVelocity().getValueAsDouble();
  }

  @AutoLogOutput
  public double getVelocityTarget() {
    return voltageOut.Velocity;
  }
}
