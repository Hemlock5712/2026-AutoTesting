package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANrange;
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

  // Ball detection thresholds (in meters)
  private static final double SIDEWAYS_BALL_THRESHOLD_M = 0.19; // 203mm
  private static final double KICKER_BALL_THRESHOLD_M = 0.100; // 100mm

  protected final TalonFX main = new TalonFX(51, TunerConstants.kCANBus);

  protected final TalonFX side = new TalonFX(50, TunerConstants.kCANBus);

  private final CANrange sidewaysRange = new CANrange(40, new CANBus("turret"));
  private final CANrange kickerRange = new CANrange(41, new CANBus("turret"));

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

    sideConfig.CurrentLimits.StatorCurrentLimit = 120.0;
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
        () -> setVelocity(RotationsPerSecond.of(34), RotationsPerSecond.of(10)));
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

  @AutoLogOutput
  public boolean hasBallInSideways() {
    return sidewaysRange.getDistance().getValueAsDouble() < SIDEWAYS_BALL_THRESHOLD_M;
  }

  @AutoLogOutput
  public boolean hasBallInKicker() {
    return kickerRange.getDistance().getValueAsDouble() < KICKER_BALL_THRESHOLD_M;
  }

  @AutoLogOutput
  public double sidewaysDistance() {
    return sidewaysRange.getDistance().getValueAsDouble();
  }

  @AutoLogOutput
  public double kickerDistance() {
    return kickerRange.getDistance().getValueAsDouble();
  }

  public void setJamRecovery() {
    // Reverse kicker (main), forward sideways (side)
    setVelocity(RotationsPerSecond.of(-20), RotationsPerSecond.of(15));
  }
}
