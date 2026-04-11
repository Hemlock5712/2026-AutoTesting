package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.TalonFXUtil;

public class Hopper extends SubsystemBase {

  protected final TalonFX main = new TalonFX(1000, TunerConstants.kCANBus);

  protected final TalonFX side = new TalonFX(1001, TunerConstants.kCANBus);

  protected TalonFXConfiguration mainConfig = new TalonFXConfiguration();

  protected TalonFXConfiguration sideConfig = new TalonFXConfiguration();

  Alert motorConfigAlert = new Alert("Main Motor Configuration Failed", AlertType.kError);

  Alert sideMotorConfigAlert = new Alert("Side Motor Configuration Failed", AlertType.kError);

  private final VelocityVoltage mainVelocityOut = new VelocityVoltage(0);
  private final VelocityVoltage sideVelocityOut = new VelocityVoltage(0);

  public Hopper() {
    boolean success = TalonFXUtil.applyConfigWithRetries(main, mainConfig);
    motorConfigAlert.set(!success);
    success = TalonFXUtil.applyConfigWithRetries(side, sideConfig);
    sideMotorConfigAlert.set(!success);

    main.optimizeBusUtilization();
    side.optimizeBusUtilization();
  }

  public void setVelocity(double velocity) {
    main.setControl(mainVelocityOut.withVelocity(velocity));
    side.setControl(sideVelocityOut.withVelocity(velocity));
  }

  // TODO: change velocity (idk what it should be)
  public Command start() {
    return Commands.runOnce(() -> setVelocity(50));
  }

  public Command startSlow() {
    return Commands.runOnce(() -> setVelocity(25));
  }

  public Command reverse() {
    return Commands.runOnce(() -> setVelocity(-50));
  }

  public Command stop() {
    return Commands.runOnce(() -> setVelocity(0));
  }
}
