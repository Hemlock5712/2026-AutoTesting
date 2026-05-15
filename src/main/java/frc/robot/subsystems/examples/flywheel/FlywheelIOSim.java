package frc.robot.subsystems.examples.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/** Physics-based sim using WPILib's {@link FlywheelSim}. */
public class FlywheelIOSim implements FlywheelIO {

  private static final double LOOP_PERIOD_SECONDS = 0.02;
  private static final double MOMENT_OF_INERTIA_KG_M2 = 0.004;
  private static final double GEAR_RATIO = 1.0;

  private final FlywheelSim sim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getKrakenX60Foc(1), MOMENT_OF_INERTIA_KG_M2, GEAR_RATIO),
          DCMotor.getKrakenX60Foc(1));

  // Sim gains intentionally differ from real — closer to dyno-derived ideal.
  private final PIDController pid = new PIDController(0.05, 0.0, 0.0);
  private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.0, 0.0019);

  private boolean closedLoop = false;
  private double targetRPM = 0.0;
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    if (closedLoop) {
      double currentRPM = sim.getAngularVelocityRPM();
      appliedVolts =
          MathUtil.clamp(pid.calculate(currentRPM, targetRPM) + ff.calculate(targetRPM), -12, 12);
    }
    sim.setInputVoltage(appliedVolts);
    sim.update(LOOP_PERIOD_SECONDS);

    inputs.velocityRPM = sim.getAngularVelocityRPM();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setVelocityRPM(double rpm) {
    closedLoop = true;
    targetRPM = rpm;
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    appliedVolts = MathUtil.clamp(volts, -12, 12);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }
}
