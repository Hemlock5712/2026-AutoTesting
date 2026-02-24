package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.utils.MechanismUtil;

/**
 * Simulation implementation of the turret subsystem.
 *
 * <p>This class simulates a turret mechanism (rotating base for aiming) and provides visual
 * feedback through SmartDashboard. It uses WPILib's DCMotorSim for physics simulation and
 * Mechanism2d for visualization.
 */
public class TurretSIM extends Turret {

  // ==================== Physical Constants ====================

  /**
   * Moment of inertia of the turret in kg⋅m² Calculated for 10 lb mass on 10" diameter ring: I = m
   * * r² = 4.536 kg * (0.127 m)² ≈ 0.073 kg⋅m²
   */
  private static final double MOI = 0.073;

  /** Simulation update period in seconds (20ms = standard robot loop) */
  private static final double SIM_PERIOD_SECONDS = 0.020;

  /** Visual length of the turret arm in pixels */
  private static final double TURRET_ARM_LENGTH = 0.1;

  // ==================== Sim-only Control Tuning ====================
  // Sim dynamics differ from real hardware (friction/backlash/latency), so tune
  // separately.
  private static final double SIM_KS = 0.3;
  private static final double SIM_KP = 1024.0;
  private static final double SIM_KD = 0.4;
  private static final double SIM_CRUISE_RPS = 100.0;
  private static final double SIM_ACCEL_RPS2 = 300.0;

  // ==================== Simulation Components ====================

  /** DC motor model (Kraken X44 FOC) - using 1 motor as per hardware config */
  private final DCMotor dcMotor = DCMotor.getKrakenX44Foc(1);

  /** Physics simulation of the turret mechanism */
  private final DCMotorSim motorSim;

  /** Mechanism visualization helper */
  private final MechanismUtil.TurretMechanism turretMechanism;

  /**
   * Constructs a new TurretSIM instance.
   *
   * <p>Initializes the physics simulation and creates the visual representation of the turret
   * mechanism on SmartDashboard.
   */
  public TurretSIM() {
    super();

    // Sim-only sign correction: flip feedback frame so ctrlPos matches mechanism
    // motion.
    FeedbackConfigs simFeedback = new FeedbackConfigs();
    simFeedback.SensorToMechanismRatio = -GEAR_RATIO;
    leader.getConfigurator().apply(simFeedback);

    // Sim-only PID + Motion Magic tuning.
    Slot0Configs simSlot0 = new Slot0Configs();
    simSlot0.kS = SIM_KS;
    simSlot0.kP = SIM_KP;
    simSlot0.kD = SIM_KD;
    leader.getConfigurator().apply(simSlot0);

    MotionMagicConfigs simMotionMagic = new MotionMagicConfigs();
    simMotionMagic.MotionMagicCruiseVelocity = SIM_CRUISE_RPS;
    simMotionMagic.MotionMagicAcceleration = SIM_ACCEL_RPS2;
    leader.getConfigurator().apply(simMotionMagic);

    // Create the linear system for physics simulation
    LinearSystem<N2, N1, N2> linearSystem =
        LinearSystemId.createDCMotorSystem(dcMotor, MOI, GEAR_RATIO);
    motorSim = new DCMotorSim(linearSystem, dcMotor);

    // Create the mechanism visualization
    turretMechanism = new MechanismUtil.TurretMechanism("Turret", TURRET_ARM_LENGTH);

    // Publish the mechanism visualization to SmartDashboard
    SmartDashboard.putData("Turret Sim", turretMechanism.getMechanism());
  }

  /**
   * Updates the turret simulation each periodic cycle.
   *
   * <p>This method performs the following tasks:
   *
   * <ul>
   *   <li>Feeds the motor voltage into the physics simulation
   *   <li>Steps the simulation forward by one period
   *   <li>Simulates battery voltage sag from current draw
   *   <li>Updates the motor encoder simulation values
   *   <li>Animates the visual mechanism display
   *   <li>Publishes telemetry data to SmartDashboard
   * </ul>
   */
  @Override
  public void simulationPeriodic() {
    motorSim.setInput(leader.getMotorVoltage().getValueAsDouble());

    // Step the simulation forward by one robot loop period
    motorSim.update(SIM_PERIOD_SECONDS);

    // Simulate battery voltage sag based on current draw
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(motorSim.getCurrentDrawAmps()));

    // Use direct mechanism position from plant.
    double encoderPosition = motorSim.getAngularPositionRotations();
    double encoderVelocity =
        RadiansPerSecond.of(motorSim.getAngularVelocityRadPerSec()).in(RotationsPerSecond);

    // Update the TalonFX sim state using CTRE's standard rotor conversion.
    double motorPosition = encoderPosition * GEAR_RATIO;
    double motorVelocity = encoderVelocity * GEAR_RATIO;
    leader.getSimState().setRawRotorPosition(motorPosition);
    leader.getSimState().setRotorVelocity(motorVelocity);

    // Publish sim-specific telemetry
    Robot.telemetry().log("Turret Sim/Current (A)", motorSim.getCurrentDrawAmps());
    Robot.telemetry().log("Turret Sim/Position (deg)", motorSim.getAngularPosition());
    Robot.telemetry()
        .log("Turret Sim/Velocity (deg/s)", Math.toDegrees(motorSim.getAngularVelocityRadPerSec()));
  }
}
