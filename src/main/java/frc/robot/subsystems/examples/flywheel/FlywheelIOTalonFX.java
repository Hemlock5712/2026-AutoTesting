package frc.robot.subsystems.examples.flywheel;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Real-robot Kraken/Falcon flywheel. */
public class FlywheelIOTalonFX implements FlywheelIO {

  private static final double GEAR_RATIO = 1.0;

  private final TalonFX motor;
  private final VelocityVoltage velocityRequest =
      new VelocityVoltage(0.0).withSlot(0).withEnableFOC(true);
  private final VoltageOut voltageRequest = new VoltageOut(0.0).withEnableFOC(true);

  public FlywheelIOTalonFX(int canId) {
    motor = new TalonFX(canId);
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    cfg.CurrentLimits.SupplyCurrentLimit = 40.0;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    // Tune these on your robot. Slot 0 is RPS-domain.
    cfg.Slot0.kS = 0.05;
    cfg.Slot0.kV = 0.12;
    cfg.Slot0.kP = 0.15;
    motor.getConfigurator().apply(cfg);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.velocityRPM = motor.getVelocity().getValueAsDouble() * 60.0 / GEAR_RATIO;
    inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps = motor.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void setVelocityRPM(double rpm) {
    motor.setControl(velocityRequest.withVelocity(rpm * GEAR_RATIO / 60.0));
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
