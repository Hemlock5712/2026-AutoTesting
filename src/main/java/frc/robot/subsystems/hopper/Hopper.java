package frc.robot.subsystems.hopper;

import static org.wpilib.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.RobotMechanism;
import frc.robot.utils.TalonFXUtil;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.driverstation.Alert;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Distance;

public class Hopper extends RobotMechanism {

  // Ball detection thresholds (in meters)
  private static final double SIDEWAYS_BALL_THRESHOLD_M = 0.19; // 203mm
  private static final double KICKER_BALL_THRESHOLD_M = 0.100; // 100mm

  protected final TalonFX main = new TalonFX(51, TunerConstants.kCANBus);

  protected final TalonFX side = new TalonFX(50, TunerConstants.kCANBus);

  private final CANrange sidewaysRange = new CANrange(40, new CANBus("turret"));
  private final CANrange kickerRange = new CANrange(41, new CANBus("turret"));

  // Cached status signals — refreshed once per periodic()
  private final StatusSignal<Distance> sidewaysDistSignal = sidewaysRange.getDistance();
  private final StatusSignal<Distance> kickerDistSignal = kickerRange.getDistance();
  private final StatusSignal<Current> mainStatorSignal = main.getStatorCurrent();
  private final StatusSignal<AngularVelocity> mainVelSignal = main.getVelocity();
  private final StatusSignal<AngularVelocity> sideVelSignal = side.getVelocity();

  protected TalonFXConfiguration mainConfig = new TalonFXConfiguration();

  protected TalonFXConfiguration sideConfig = new TalonFXConfiguration();

  Alert motorConfigAlert = new Alert("Main Motor Configuration Failed", Alert.Level.HIGH);

  Alert sideMotorConfigAlert = new Alert("Side Motor Configuration Failed", Alert.Level.HIGH);

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

    // Set explicit update frequencies before optimizing bus utilization
    mainStatorSignal.setUpdateFrequency(100);
    mainVelSignal.setUpdateFrequency(100);
    sideVelSignal.setUpdateFrequency(100);
    sidewaysDistSignal.setUpdateFrequency(50);
    kickerDistSignal.setUpdateFrequency(50);

    main.optimizeBusUtilization();
    side.optimizeBusUtilization();
    sidewaysRange.optimizeBusUtilization();
    kickerRange.optimizeBusUtilization();
    addPeriodicCallback(this::periodic);
  }

  public void periodic() {
    long _t = System.nanoTime();
    BaseStatusSignal.refreshAll(mainStatorSignal, mainVelSignal, sideVelSignal);
    BaseStatusSignal.refreshAll(sidewaysDistSignal, kickerDistSignal);
    Logger.recordOutput("Timing/HopperMs", (System.nanoTime() - _t) / 1e6);
  }

  public void setVelocity(AngularVelocity mainVelocity, AngularVelocity sideVelocity) {
    main.setControl(mainVelocityOut.withVelocity(mainVelocity));
    side.setControl(sideVelocityOut.withVelocity(sideVelocity));
  }

  /** Primitive overload — velocity in rotations per second. Zero allocations. */
  public void setVelocityRPS(double mainRPS, double sideRPS) {
    main.setControl(mainVelocityOut.withVelocity(mainRPS));
    side.setControl(sideVelocityOut.withVelocity(sideRPS));
  }

  public Command start() {
    return runOnce(() -> setVelocity(RotationsPerSecond.of(34), RotationsPerSecond.of(10)));
  }

  public Command startSlow() {
    return runOnce(() -> setVelocity(RotationsPerSecond.of(30), RotationsPerSecond.of(5)));
  }

  public Command reverseCommand() {
    return runOnce(() -> setVelocity(RotationsPerSecond.of(-30), RotationsPerSecond.of(30)));
  }

  public Command stop() {
    return runOnce(() -> setVelocity(RotationsPerSecond.of(0), RotationsPerSecond.of(0)));
  }

  @AutoLogOutput
  public double leaderStator() {
    return mainStatorSignal.getValueAsDouble();
  }

  @AutoLogOutput
  public boolean hasBallInSideways() {
    return sidewaysDistance() < SIDEWAYS_BALL_THRESHOLD_M;
  }

  @AutoLogOutput
  public boolean hasBallInKicker() {
    return kickerDistance() < KICKER_BALL_THRESHOLD_M;
  }

  @AutoLogOutput
  public double sidewaysDistance() {
    return sidewaysDistSignal.getValueAsDouble();
  }

  @AutoLogOutput
  public double kickerDistance() {
    return kickerDistSignal.getValueAsDouble();
  }

  @AutoLogOutput
  public double getVelocityMainRPS() {
    return mainVelSignal.getValueAsDouble();
  }

  @AutoLogOutput
  public double getVelocitySideRPS() {
    return sideVelSignal.getValueAsDouble();
  }

  public void setJamRecovery() {
    // Reverse kicker (main), forward sideways (side)
    setVelocityRPS(-30, -30);
  }

  public void setJamback() {
    // Reverse kicker (main), forward sideways (side)
    setVelocityRPS(-10, -10.);
  }
}
