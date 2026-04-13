package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.TalonFXUtil;
import org.littletonrobotics.junction.AutoLogOutput;

public class Hopper extends SubsystemBase {

  protected final TalonFX main = new TalonFX(51, TunerConstants.kCANBus);

  protected final TalonFX side = new TalonFX(50, TunerConstants.kCANBus);

  protected TalonFXConfiguration mainConfig = new TalonFXConfiguration();

  protected TalonFXConfiguration sideConfig = new TalonFXConfiguration();

  Alert motorConfigAlert = new Alert("Main Motor Configuration Failed", AlertType.kError);

  Alert sideMotorConfigAlert = new Alert("Side Motor Configuration Failed", AlertType.kError);

  private final VelocityVoltage mainVelocityOut = new VelocityVoltage(0);
  private final VelocityVoltage sideVelocityOut = new VelocityVoltage(0);

  public Hopper() {
    mainConfig.Feedback.SensorToMechanismRatio = 2.77;

    mainConfig.Slot0.kP = 0.1;
    mainConfig.Slot0.kS = 0.33;
    mainConfig.Slot0.kV = 0.33;

    mainConfig.CurrentLimits.StatorCurrentLimit = 240;
    mainConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    

    mainConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    boolean success = TalonFXUtil.applyConfigWithRetries(main, mainConfig);
    motorConfigAlert.set(!success);

    sideConfig.Slot0.kP = 0.1;
    sideConfig.Slot0.kS = 0.375;
    sideConfig.Slot0.kV = 0.375;

    sideConfig.Feedback.SensorToMechanismRatio = 3.9;

    sideConfig.CurrentLimits.StatorCurrentLimit = 20.0;
    success = TalonFXUtil.applyConfigWithRetries(side, sideConfig);
    sideMotorConfigAlert.set(!success);

    main.optimizeBusUtilization();
    side.optimizeBusUtilization();
  }

  public void setVelocity(AngularVelocity mainVelocity, AngularVelocity sideVelocity) {
    main.setControl(mainVelocityOut.withVelocity(mainVelocity));
    side.setControl(sideVelocityOut.withVelocity(sideVelocity));
  }

  public Command start() {
    return Commands.runOnce(
        () -> setVelocity(RotationsPerSecond.of(20), RotationsPerSecond.of(10)));
  }

  public Command startSlow() {
    return Commands.runOnce(() -> setVelocity(RotationsPerSecond.of(30), RotationsPerSecond.of(5)));
  }

  public Command reverseCommand() {
    return Commands.runOnce(() -> setVelocity(RotationsPerSecond.of(-5), RotationsPerSecond.of(5)));
  }

  public Command stop() {
    return Commands.runOnce(() -> setVelocity(RotationsPerSecond.of(0), RotationsPerSecond.of(0)));
  }

  @AutoLogOutput
  public double leaderStator() {
    return main.getStatorCurrent().getValueAsDouble();
  }
}
