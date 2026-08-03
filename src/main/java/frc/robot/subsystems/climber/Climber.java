package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.RobotMechanism;
import frc.robot.utils.TalonFXUtil;
import org.wpilib.driverstation.Alert;

public class Climber extends RobotMechanism {
  protected final TalonFX leader = new TalonFX(51, TunerConstants.kCANBus);

  protected final TalonFX follower = new TalonFX(50, TunerConstants.kCANBus);

  protected TalonFXConfiguration climberConfig = new TalonFXConfiguration();

  Alert motorConfigAlert = new Alert("Climber Motor Configuration Failed", Alert.Level.HIGH);

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
