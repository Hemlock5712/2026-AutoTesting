package frc.robot.subsystems.examples.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Physics-based sim using WPILib's {@link SingleJointedArmSim}. */
public class ArmIOSim implements ArmIO {

  private static final double LOOP_PERIOD_S = 0.02;
  private static final double GEAR_RATIO = 100.0;
  private static final double ARM_LENGTH_M = 0.5;
  private static final double ARM_MASS_KG = 2.0;
  private static final double MIN_RAD = Units.degreesToRadians(-15);
  private static final double MAX_RAD = Units.degreesToRadians(95);

  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(1),
          GEAR_RATIO,
          SingleJointedArmSim.estimateMOI(ARM_LENGTH_M, ARM_MASS_KG),
          ARM_LENGTH_M,
          MIN_RAD,
          MAX_RAD,
          true,
          0.0);

  private final ProfiledPIDController pid =
      new ProfiledPIDController(50.0, 0.0, 1.0, new TrapezoidProfile.Constraints(2 * Math.PI, 8.0));
  private final ArmFeedforward ff = new ArmFeedforward(0.0, 0.3, 0.12);

  private boolean closedLoop = false;
  private double setpointRad = 0.0;
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    if (closedLoop) {
      double measured = sim.getAngleRads();
      double pidOut = pid.calculate(measured, setpointRad);
      double ffOut = ff.calculate(pid.getSetpoint().position, pid.getSetpoint().velocity);
      appliedVolts = MathUtil.clamp(pidOut + ffOut, -12, 12);
    }
    sim.setInputVoltage(appliedVolts);
    sim.update(LOOP_PERIOD_S);

    inputs.positionRad = sim.getAngleRads();
    inputs.velocityRadPerSec = sim.getVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = sim.getCurrentDrawAmps();
    inputs.setpointRad = setpointRad;
  }

  @Override
  public void setPosition(double rad) {
    if (!closedLoop) pid.reset(sim.getAngleRads());
    closedLoop = true;
    setpointRad = MathUtil.clamp(rad, MIN_RAD, MAX_RAD);
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
