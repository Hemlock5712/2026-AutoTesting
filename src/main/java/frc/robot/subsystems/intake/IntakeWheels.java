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

  private final TalonFX wheel = new TalonFX(23, TunerConstants.kCANBus);

  private TalonFXConfiguration configWheel = new TalonFXConfiguration();

  Alert motorConfigAlert = new Alert("Intake Wheel Motor Configuration Failed", AlertType.kError);

  public IntakeWheels() {
    configWheel.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    configWheel.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    configWheel.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    configWheel.Feedback.SensorToMechanismRatio = 2.33;

    boolean success = TalonFXUtil.applyConfigWithRetries(wheel, configWheel);
    motorConfigAlert.set(!success);
  }

  @Override
  public void periodic() {}

  public Command runIntake() {
    return runOnce(() -> wheel.setVoltage(6));
  }

  public Command stopWheel() {
    return runOnce(() -> wheel.stopMotor());
  }

  @Logged
  public double getVelocity() {
    return wheel.getVelocity().getValueAsDouble();
  }
}
