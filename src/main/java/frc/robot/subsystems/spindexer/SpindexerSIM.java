package frc.robot.subsystems.spindexer;

import edu.wpi.first.epilogue.Logged;
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
import frc.robot.utils.TalonFXUtil;

/**
 * Simulation implementation of the spindexer subsystem.
 *
 * <p>This class simulates a spindexer mechanism (rotating mass for shooting projectiles) and
 * provides visual feedback through SmartDashboard. It uses WPILib's SpindexerSim for physics
 * simulation and Mechanism2d for visualization.
 */
@Logged
public class SpindexerSIM extends Spindexer {

  // ==================== Physical Constants ====================

  /** Gear ratio between motor and spindexer (motor rotations : spindexer rotations) */
  private static final double GEAR_RATIO = 1.0;

  /** Conversion factor from radians to rotations (1 / 2π) */
  private static final double RAD_TO_ROTATIONS = 1.0 / (2.0 * Math.PI);

  /** Moment of inertia of the spindexer in kg⋅m² */
  private static final double FLYWHEEL_MOI = 0.01;

  /** Simulation update period in seconds (20ms = standard robot loop) */
  private static final double SIM_PERIOD_SECONDS = 0.020;

  /** Visual radius of the spindexer in pixels */
  private static final double FLYWHEEL_RADIUS = 80.0;

  // ==================== Simulation Components ====================

  /** DC motor model (Kraken X60) - using 2 motors as per hardware config */
  private final DCMotor dcMotor = DCMotor.getKrakenX60(1);

  /** Physics simulation of the spindexer mechanism */
  private final DCMotorSim spindexerSim;

  /** Mechanism visualization helper */
  private final MechanismUtil.FlywheelMechanism spindexerMechanism;

  /**
   * Constructs a new FlywheelSIM instance.
   *
   * <p>Initializes the physics simulation and creates the visual representation of the spindexer
   * mechanism on SmartDashboard.
   */
  public SpindexerSIM() {
    super();

    // Configure gear ratio for simulation (direct drive, but set for consistency)
    spindexerConfig.Feedback.RotorToSensorRatio = GEAR_RATIO;
    spindexerConfig.Slot0.kS = 0.0; // Static gain (feedforward)
    spindexerConfig.Slot0.kV = 0.12; // Velocity gain (12V / 100 RPS ≈ 0.12)
    spindexerConfig.Slot0.kP = 100; // Proportional gain (tune this!)
    spindexerConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0; // Max velocity (RPS)
    spindexerConfig.MotionMagic.MotionMagicAcceleration = 400.0; // Max acceleration (RPS²)
    TalonFXUtil.applyConfigWithRetries(spindexer, spindexerConfig);

    LinearSystem<N2, N1, N2> linearSystem =
        LinearSystemId.createDCMotorSystem(
            dcMotor, FLYWHEEL_MOI, GEAR_RATIO); // Direct drive (1:1 ratio)
    // Initialize the physics simulation (no gravity for spindexers)
    spindexerSim = new DCMotorSim(linearSystem, dcMotor);

    // Create the mechanism visualization
    spindexerMechanism = new MechanismUtil.FlywheelMechanism("Spindexer", FLYWHEEL_RADIUS);

    // Publish the mechanism visualization to SmartDashboard
    SmartDashboard.putData("Spindexer Sim", spindexerMechanism.getMechanism());
  }

  /**
   * Updates the spindexer simulation each periodic cycle.
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
    // Feed the motor voltage from the controller into the physics simulation
    spindexerSim.setInput(spindexer.getMotorVoltage().getValueAsDouble());

    // Step the simulation forward by one robot loop period
    spindexerSim.update(SIM_PERIOD_SECONDS);

    // Simulate battery voltage sag based on current draw
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(spindexerSim.getCurrentDrawAmps()));

    // Get current spindexer velocity in radians per second
    double velocityRadPerSec = spindexerSim.getAngularVelocityRadPerSec();

    // Convert spindexer velocity to motor velocity (accounting for gear ratio)
    double motorVelocity = velocityRadPerSec * RAD_TO_ROTATIONS * GEAR_RATIO;

    // Update the simulated motor encoder velocity
    spindexer.getSimState().setRotorVelocity(motorVelocity);

    // Use the actual position from physics simulation (more accurate than integration)
    double spindexerPositionRad = spindexerSim.getAngularPositionRad();
    double motorPosition = spindexerPositionRad * RAD_TO_ROTATIONS * GEAR_RATIO;
    spindexer.getSimState().setRawRotorPosition(motorPosition);

    // Animate the visual representation
    updateVisualization(velocityRadPerSec);

    // Publish sim-specific telemetry (other values are auto-logged from base class)
    Robot.telemetry().log("Spindexer Sim/Current (A)", spindexerSim.getCurrentDrawAmps());
  }

  /**
   * Updates the visual representation of the spindexer.
   *
   * <p>Delegates to the SpindexerMechanism utility to update the spokes rotation and color based on
   * current velocity and state.
   *
   * @param velocityRadPerSec The current angular velocity in radians per second
   */
  private void updateVisualization(double velocityRadPerSec) {
    // Update the mechanism visualization
    spindexerMechanism.update(velocityRadPerSec, SIM_PERIOD_SECONDS, isAtTarget());
  }
}
