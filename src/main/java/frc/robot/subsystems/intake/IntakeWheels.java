package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.TalonFXUtil;

@Logged(strategy = Strategy.OPT_IN)
public class IntakeWheels extends SubsystemBase {

  private static final double INTAKE_VOLTAGE = 7;

  private final TalonFX wheel = new TalonFX(23, TunerConstants.kCANBus);

  private TalonFXConfiguration wheelConfig = new TalonFXConfiguration();

  Alert motorConfigAlert = new Alert("Intake Wheel Motor Configuration Failed", AlertType.kError);

  public IntakeWheels() {
    wheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    wheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    wheelConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    wheelConfig.Feedback.SensorToMechanismRatio = 2.33;

    boolean success = TalonFXUtil.applyConfigWithRetries(wheel, wheelConfig);
    motorConfigAlert.set(!success);
  }

  @Override
  public void periodic() {}

  public Command runIntake() {
    return runOnce(() -> wheel.setVoltage(INTAKE_VOLTAGE));
  }

  public Command stopWheel() {
    return runOnce(() -> wheel.stopMotor());
  }

  @Logged
  public double getVelocity() {
    return wheel.getVelocity().getValueAsDouble();
  }
}
