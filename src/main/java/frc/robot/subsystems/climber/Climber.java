package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.TalonFXUtil;

public class Climber extends SubsystemBase {
  protected final TalonFX leader = new TalonFX(51, TunerConstants.kCANBus);

  protected final TalonFX follower = new TalonFX(50, TunerConstants.kCANBus);

  protected TalonFXConfiguration climberConfig = new TalonFXConfiguration();

  Alert motorConfigAlert = new Alert("Climber Motor Configuration Failed", AlertType.kError);

  public Climber() {
    applyConfigs();
  }

  public void applyConfigs() {
    follower.setControl(new Follower(leader.getDeviceID(), MotorAlignmentValue.Aligned));

    boolean success = TalonFXUtil.applyConfigWithRetries(leader, climberConfig);
    motorConfigAlert.set(!success);

    leader.optimizeBusUtilization();
    follower.optimizeBusUtilization();
  }
}
