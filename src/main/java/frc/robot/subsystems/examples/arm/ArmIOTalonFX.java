package frc.robot.subsystems.examples.arm;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/**
 * Real-robot arm on a single TalonFX with Motion Magic profiling and arm-cosine gravity feed
 * forward.
 */
public class ArmIOTalonFX implements ArmIO {

  /** Motor rotations per arm rotation. Tune to your gearbox. */
  private static final double GEAR_RATIO = 100.0;

  private final TalonFX motor;
  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0.0).withEnableFOC(true);
  private final VoltageOut voltageRequest = new VoltageOut(0.0).withEnableFOC(true);
  private double lastSetpointRad = 0.0;

  public ArmIOTalonFX(int canId) {
    motor = new TalonFX(canId);
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.CurrentLimits.SupplyCurrentLimit = 30.0;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    // Slot 0 — tune these on your robot. Units are rotor rotations.
    cfg.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    cfg.Slot0.kS = 0.1;
    cfg.Slot0.kV = 0.12;
    cfg.Slot0.kG = 0.3;
    cfg.Slot0.kP = 8.0;
    cfg.Slot0.kD = 0.1;
    // Motion Magic constraints (rotor units).
    cfg.MotionMagic.MotionMagicCruiseVelocity = 2.0;
    cfg.MotionMagic.MotionMagicAcceleration = 4.0;
    motor.getConfigurator().apply(cfg);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.positionRad = motor.getPosition().getValueAsDouble() * 2 * Math.PI / GEAR_RATIO;
    inputs.velocityRadPerSec = motor.getVelocity().getValueAsDouble() * 2 * Math.PI / GEAR_RATIO;
    inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps = motor.getSupplyCurrent().getValueAsDouble();
    inputs.setpointRad = lastSetpointRad;
  }

  @Override
  public void setPosition(double rad) {
    lastSetpointRad = rad;
    motor.setControl(positionRequest.withPosition(rad * GEAR_RATIO / (2 * Math.PI)));
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }
}
