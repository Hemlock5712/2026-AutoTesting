package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.utils.MechanismUtil;
import frc.robot.utils.TalonFXUtil;

public class IntakeArmSIM extends IntakeArm {

  private static final double GEAR_RATIO = 25.0;
  private static final double ARM_LENGTH = 1.0;
  private static final double ARM_MASS_KG = 5.0;
  private static final double MIN_ANGLE_RAD = Units.rotationsToRadians(-1);
  private static final double MAX_ANGLE_RAD = Units.rotationsToRadians(1);
  private static final double STARTING_ANGLE_RAD = 0.0;
  private static final double SIM_PERIOD_SECONDS = 0.020;
  private static final double ARM_VISUAL_LENGTH = 200.0;

  private final DCMotor dcMotor = DCMotor.getKrakenX60(1);
  private final SingleJointedArmSim armSim;
  private final MechanismUtil.ArmMechanism armMechanism;

  public IntakeArmSIM() {
    super();

    config.Feedback.RotorToSensorRatio = GEAR_RATIO;
    config.Slot0.kG = 0.0;
    config.Slot0.kS = 0.0;
    config.Slot0.kP = 160;
    config.Slot0.kD = 30;
    config.MotionMagic.MotionMagicCruiseVelocity = 1;
    config.MotionMagic.MotionMagicAcceleration = 4;
    TalonFXUtil.applyConfigWithRetries(arm, config);

    armSim =
        new SingleJointedArmSim(
            dcMotor,
            GEAR_RATIO,
            SingleJointedArmSim.estimateMOI(ARM_LENGTH, ARM_MASS_KG),
            ARM_LENGTH,
            MIN_ANGLE_RAD,
            MAX_ANGLE_RAD,
            false,
            STARTING_ANGLE_RAD);

    armMechanism = new MechanismUtil.ArmMechanism("Arm", ARM_VISUAL_LENGTH);
    SmartDashboard.putData("Arm Sim", armMechanism.getMechanism());
  }

  @Override
  public void simulationPeriodic() {
    armSim.setInput(arm.getMotorVoltage().getValueAsDouble());
    armSim.update(SIM_PERIOD_SECONDS);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));

    double encoderPosition = Radians.of(armSim.getAngleRads()).in(Rotations);
    double encoderVelocity =
        RadiansPerSecond.of(armSim.getVelocityRadPerSec()).in(RotationsPerSecond);

    armEncoder.getSimState().setRawPosition(encoderPosition);
    armEncoder.getSimState().setVelocity(encoderVelocity);

    double motorPosition = encoderPosition * GEAR_RATIO;
    double motorVelocity = encoderVelocity * GEAR_RATIO;
    arm.getSimState().setRawRotorPosition(motorPosition);
    arm.getSimState().setRotorVelocity(motorVelocity);

    updateVisualization();

    Robot.telemetry().log("Arm Sim Current (A)", armSim.getCurrentDrawAmps());
  }

  private void updateVisualization() {
    double currentAngleDeg = getPosition().in(Degrees);
    armMechanism.update(currentAngleDeg, isAtTarget());
  }
}
