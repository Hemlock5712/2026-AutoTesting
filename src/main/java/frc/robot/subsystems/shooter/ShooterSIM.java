package frc.robot.subsystems.shooter;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.MechanismUtil;
import frc.robot.utils.TalonFXUtil;
import org.littletonrobotics.junction.Logger;

/**
 * Simulation implementation of the flywheel subsystem.
 *
 * <p>This class simulates a flywheel mechanism (rotating mass for shooting projectiles) and
 * provides visual feedback through SmartDashboard. It uses WPILib's shooterSim for physics
 * simulation and Mechanism2d for visualization.
 */
public class ShooterSIM extends Shooter {

  // ==================== Physical Constants ====================

  /** Gear ratio between motor and flywheel (motor rotations : flywheel rotations) */
  private static final double GEAR_RATIO = 2.0;

  /** Conversion factor from radians to rotations (1 / 2π) */
  private static final double RAD_TO_ROTATIONS = 1.0 / (2.0 * Math.PI);

  /** Moment of inertia of the flywheel in kg⋅m² */
  private static final double FLYWHEEL_MOI = 0.01;

  /** Simulation update period in seconds (20ms = standard robot loop) */
  private static final double SIM_PERIOD_SECONDS = 0.020;

  /** Visual radius of the flywheel in pixels */
  private static final double FLYWHEEL_RADIUS = 80.0;

  // ==================== Simulation Components ====================

  /** DC motor model (Kraken X60) - using 2 motors as per hardware config */
  private final DCMotor dcMotor = DCMotor.getKrakenX60(2);

  /** Physics simulation of the flywheel mechanism */
  private final DCMotorSim shooterSim;

  /** Mechanism visualization helper */
  private final MechanismUtil.FlywheelMechanism flywheelMechanism;

  /**
   * Constructs a new shooterSim instance.
   *
   * <p>Initializes the physics simulation and creates the visual representation of the flywheel
   * mechanism on SmartDashboard.
   */
  public ShooterSIM() {
    super();

    // Configure gear ratio for simulation (direct drive, but set for consistency)
    config.Feedback.RotorToSensorRatio = GEAR_RATIO;
    config.Slot0.kS = 0.0; // Static gain (feedforward)
    config.Slot0.kV = 0.12; // Velocity gain (12V / 100 RPS ≈ 0.12)
    config.Slot0.kP = 0.1; // Proportional gain (tune this!)
    config.MotionMagic.MotionMagicCruiseVelocity = 100.0; // Max velocity (RPS)
    config.MotionMagic.MotionMagicAcceleration = 400.0; // Max acceleration (RPS²)
    TalonFXUtil.applyConfigWithRetries(flywheel, config);

    LinearSystem<N2, N1, N2> linearSystem =
        LinearSystemId.createDCMotorSystem(
            dcMotor, FLYWHEEL_MOI, GEAR_RATIO); // Direct drive (1:1 ratio)
    // Initialize the physics simulation (no gravity for flywheels)
    shooterSim = new DCMotorSim(linearSystem, dcMotor);

    // Create the mechanism visualization
    flywheelMechanism = new MechanismUtil.FlywheelMechanism("Flywheel", FLYWHEEL_RADIUS);

    // Publish the mechanism visualization to SmartDashboard
    SmartDashboard.putData("Flywheel Sim", flywheelMechanism.getMechanism());
  }

  /**
   * Updates the flywheel simulation each periodic cycle.
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
    shooterSim.setInput(flywheel.getMotorVoltage().getValueAsDouble());

    // Step the simulation forward by one robot loop period
    shooterSim.update(SIM_PERIOD_SECONDS);

    // Simulate battery voltage sag based on current draw
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(shooterSim.getCurrentDrawAmps()));

    // Get current flywheel velocity in radians per second
    double velocityRadPerSec = shooterSim.getAngularVelocityRadPerSec();

    // Convert flywheel velocity to motor velocity (accounting for gear ratio)
    double motorVelocity = velocityRadPerSec * RAD_TO_ROTATIONS * GEAR_RATIO;

    // Update the simulated motor encoder velocity
    flywheel.getSimState().setRotorVelocity(motorVelocity);

    // Use the actual position from physics simulation (more accurate than integration)
    double flywheelPositionRad = shooterSim.getAngularPositionRad();
    double motorPosition = flywheelPositionRad * RAD_TO_ROTATIONS * GEAR_RATIO;
    flywheel.getSimState().setRawRotorPosition(motorPosition);

    // Animate the visual representation
    updateVisualization(velocityRadPerSec);

    // Publish sim-specific telemetry (other values are auto-logged from base class)
    Logger.recordOutput("Flywheel Sim/Current (A)", shooterSim.getCurrentDrawAmps());
  }

  /**
   * Updates the visual representation of the flywheel.
   *
   * <p>Delegates to the FlywheelMechanism utility to update the spokes rotation and color based on
   * current velocity and state.
   *
   * @param velocityRadPerSec The current angular velocity in radians per second
   */
  private void updateVisualization(double velocityRadPerSec) {
    // Update the mechanism visualization
    flywheelMechanism.update(velocityRadPerSec, SIM_PERIOD_SECONDS, flywheelIsAtTarget());
  }
}
