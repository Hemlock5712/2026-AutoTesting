package frc.robot.subsystems.turret;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.signals.InvertedValue;
import frc.robot.subsystems.Superstructure;
import frc.robot.utils.MechanismUtil;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.simulation.BatterySim;
import org.wpilib.simulation.RoboRioSim;
import org.wpilib.simulation.SingleJointedArmSim;
import org.wpilib.smartdashboard.SmartDashboard;

/**
 * Simulation implementation of the turret subsystem.
 *
 * <p>This class simulates a turret mechanism (rotating base for aiming) and provides visual
 * feedback through SmartDashboard. Uses SingleJointedArmSim for physics with built-in position
 * limits at +/-180 degrees.
 */
public class TurretSIM extends Turret {

  // ==================== Physical Constants ====================

  /**
   * Moment of inertia of the turret in kg*m^2. Calculated for 10 lb mass on 10" diameter ring: I =
   * m * r^2 = 4.536 kg * (0.127 m)^2 = 0.073 kg*m^2
   */
  private static final double MOI = 0.073;

  /** Arm length for MOI calculation (not critical since no gravity) */
  private static final double ARM_LENGTH = 0.25;

  /** Minimum angle in radians (-180 degrees) */
  private static final double MIN_ANGLE_RAD = -Math.PI;

  /** Maximum angle in radians (+180 degrees) */
  private static final double MAX_ANGLE_RAD = Math.PI;

  /** Simulation update period in seconds (20ms = standard robot loop) */
  private static final double SIM_PERIOD_SECONDS = 0.020;

  /** Visual length of the turret arm in pixels */
  private static final double TURRET_ARM_LENGTH = 0.1;

  // Offset from turret pivot to shooter mechanism (for 3D visualization)
  private static final double SHOOTER_X_OFFSET = 0.111203; // meters
  private static final double SHOOTER_Z_OFFSET = 0.05698; // meters
  private static final double SHOOTER_PITCH_RAD = Degrees.of(75).in(Radians);

  // ==================== Sim-only Control Tuning ====================
  private static final double SIM_KS = 0.1;
  private static final double SIM_KP = 32.0;
  private static final double SIM_KD = 1;
  private static final double SIM_CRUISE_RPS = 100.0;
  private static final double SIM_ACCEL_RPS2 = 300.0;

  // ==================== Simulation Components ====================

  /** DC motor model (Kraken X44 FOC) */
  private final DCMotor dcMotor = DCMotor.getKrakenX44Foc(1);

  /** Physics simulation using SingleJointedArmSim (handles position limits) */
  private final SingleJointedArmSim turretSim;

  /** Mechanism visualization helper */
  private final MechanismUtil.TurretMechanism turretMechanism;

  @AutoLogOutput(key = "TurretSIM/TurretMechanism3D")
  public Pose3d[] turretPose = new Pose3d[] {Superstructure.TURRET_HOLE_CENTER, new Pose3d()};

  /**
   * Constructs a new TurretSIM instance.
   *
   * <p>Initializes the physics simulation and creates the visual representation of the turret
   * mechanism on SmartDashboard.
   */
  public TurretSIM() {
    super();

    // Override motor direction for simulation (real hardware uses
    // Clockwise_Positive)
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Sim-only PID + Motion Magic tuning
    config.Slot0.kS = SIM_KS;
    config.Slot0.kP = SIM_KP;
    config.Slot0.kD = SIM_KD;
    config.MotionMagic.MotionMagicCruiseVelocity = SIM_CRUISE_RPS;
    config.MotionMagic.MotionMagicAcceleration = SIM_ACCEL_RPS2;

    leader.getConfigurator().apply(config);

    // Initialize the physics simulation with position limits
    // Using SingleJointedArmSim because it has built-in min/max angle support
    turretSim =
        new SingleJointedArmSim(
            dcMotor,
            DualEncoderCRT.MOTOR_TO_MECHANISM_RATIO,
            MOI,
            ARM_LENGTH,
            MIN_ANGLE_RAD, // -180 degrees
            MAX_ANGLE_RAD, // +180 degrees
            false, // No gravity (horizontal turret rotation)
            0.0); // Starting angle

    // Create the mechanism visualization
    turretMechanism = new MechanismUtil.TurretMechanism("Turret", TURRET_ARM_LENGTH);

    // Publish the mechanism visualization to SmartDashboard
    SmartDashboard.putData("Turret Sim", turretMechanism.getMechanism());
    addPeriodicCallback(this::simulationPeriodic);
  }

  public void simulationPeriodic() {
    // Feed motor voltage into physics simulation
    turretSim.setInput(leader.getMotorVoltage().getValueAsDouble());

    // Step the simulation forward
    turretSim.update(SIM_PERIOD_SECONDS);

    // Simulate battery voltage sag based on current draw
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(turretSim.getCurrentDraw()));

    // Get position and velocity from simulation (in rotations for TalonFX)
    double mechanismPosition = Radians.of(turretSim.getAngle()).in(Rotations);
    double mechanismVelocity = RadiansPerSecond.of(turretSim.getVelocity()).in(RotationsPerSecond);

    // Update TalonFX sim state (convert to rotor units)
    double rotorPosition = mechanismPosition * DualEncoderCRT.MOTOR_TO_MECHANISM_RATIO;
    double rotorVelocity = mechanismVelocity * DualEncoderCRT.MOTOR_TO_MECHANISM_RATIO;
    leader.getSimState().setRawRotorPosition(rotorPosition);
    leader.getSimState().setRotorVelocity(rotorVelocity);

    // Simulate encoder 1 (21 rotations per mechanism rotation)
    double encoder1Position = (mechanismPosition * DualEncoderCRT.ENCODER_1_MECHANISM_RATIO) % 1.0;
    if (encoder1Position < 0) encoder1Position += 1.0;
    encoder1.getSimState().setRawPosition(encoder1Position);

    // Simulate encoder 2 (22 rotations per mechanism rotation)
    double encoder2Position = (mechanismPosition * DualEncoderCRT.ENCODER_2_MECHANISM_RATIO) % 1.0;
    if (encoder2Position < 0) encoder2Position += 1.0;
    encoder2.getSimState().setRawPosition(encoder2Position);

    // Publish telemetry
    Logger.recordOutput("Turret Sim/Current (A)", turretSim.getCurrentDraw());
    Logger.recordOutput("Turret Sim/Position (deg)", Math.toDegrees(turretSim.getAngle()));
    Logger.recordOutput("Turret Sim/Velocity (deg/s)", Math.toDegrees(turretSim.getVelocity()));

    // Turret base - rotates around Z-axis
    turretPose[0] =
        Superstructure.TURRET_HOLE_CENTER.transformBy(
            new Transform3d(
                Translation3d.kZero, new Rotation3d(0, 0, getTargetAngle().in(Radians))));

    // Shooter position - offset from base with fixed pitch
    turretPose[1] =
        turretPose[0].transformBy(
            new Transform3d(
                SHOOTER_X_OFFSET, 0.0, SHOOTER_Z_OFFSET, new Rotation3d(0, -SHOOTER_PITCH_RAD, 0)));
  }
}
