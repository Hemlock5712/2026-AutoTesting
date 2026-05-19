package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.generated.TunerConstants;

/**
 * Whole-robot physics constants that don't live in {@link TunerConstants} (which Phoenix Tuner X
 * regenerates and only knows about per-module values). Also owns the {@link RobotConfig} factory
 * shared by the PathPlanner planner and the runtime {@code SwerveSetpointGenerator}, plus the
 * motor-curve helper used by the path velocity profiler.
 *
 * <p>Per-module values (gear ratios, wheel radius, kSpeedAt12Volts, locations, slip current) come
 * directly from {@code TunerConstants.FrontLeft.*} — read those at the call site rather than
 * re-aliasing them here.
 */
public final class DrivePhysics {

  /** Mass including bumpers and battery. */
  public static final double ROBOT_MASS_KG = 60.0;

  /** Moment of inertia about the vertical axis. Rough estimate; refine after CAD/measurement. */
  public static final double ROBOT_MOI_KG_M2 = 6.0;

  /** Carpet-on-tread friction coefficient. ~1.0 nominal; 1.1 leaves a small margin. */
  public static final double WHEEL_COF = 1.1;

  /** Translational acceleration cap from friction: μ·g (m/s²). */
  public static final double MAX_FRICTION_ACCEL = WHEEL_COF * 9.81;

  /** Center-to-corner distance assuming a symmetric chassis. */
  public static final double DRIVE_BASE_RADIUS =
      Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY);

  /**
   * Buffer added on top of the robot half-extent when inflating obstacles for PathPlanner's
   * pathfinder. Covers controller lag, weight transfer, pose-staleness between the 50 Hz periodic
   * publish and the 250 Hz fast loop, and modeling error.
   */
  public static final double PATH_INFLATION_MARGIN_M = 0.10;

  /** Drive motor model used by the setpoint generator and the path velocity profiler. */
  private static final DCMotor DRIVE_MOTOR = DCMotor.getKrakenX60Foc(1);

  /**
   * Max steer-mechanism velocity used by {@code SwerveSetpointGenerator}. Derived from the Kraken
   * X60 free speed and the steer reduction, with a 0.95 derate for friction and controller lag.
   * ~22 rad/s on the current TunerConstants. Bumped from 0.8 → 0.95 because the conservative
   * derate was clamping the generator at sharp corners (e.g. the U-turn in NewPath), causing the
   * chassis to nearly stop while the planner expected 0.87 m/s through the turn.
   */
  public static final double MAX_STEER_VELOCITY_RAD_PER_SEC =
      DRIVE_MOTOR.freeSpeedRadPerSec / TunerConstants.FrontLeft.SteerMotorGearRatio * 0.95;

  private DrivePhysics() {}

  /**
   * Max chassis acceleration the drivetrain can produce at the given module speed, considering
   * both the motor's torque curve (drops off near free speed) and the per-motor stator current
   * limit. Used by the path-velocity profiler.
   */
  public static double maxAccelerationAtSpeed(double wheelSpeedMps) {
    double wheelRadius = TunerConstants.FrontLeft.WheelRadius;
    double gearRatio = TunerConstants.FrontLeft.DriveMotorGearRatio;
    double slipCurrent = TunerConstants.FrontLeft.SlipCurrent;
    double motorRadPerSec = (wheelSpeedMps / wheelRadius) * gearRatio;
    double currentAtSpeed = DRIVE_MOTOR.getCurrent(motorRadPerSec, DRIVE_MOTOR.nominalVoltageVolts);
    double effectiveCurrent = Math.min(currentAtSpeed, slipCurrent);
    double torquePerMotor = DRIVE_MOTOR.getTorque(effectiveCurrent);
    double totalForce = 4 * torquePerMotor * gearRatio / wheelRadius;
    return totalForce / ROBOT_MASS_KG;
  }

  /** Builds the {@link RobotConfig} shared by the PP planner and runtime setpoint generator. */
  public static RobotConfig buildRobotConfig() {
    ModuleConfig moduleConfig =
        new ModuleConfig(
            TunerConstants.FrontLeft.WheelRadius,
            TunerConstants.kSpeedAt12Volts.baseUnitMagnitude(),
            WHEEL_COF,
            DRIVE_MOTOR.withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
            TunerConstants.FrontLeft.SlipCurrent,
            1);
    double halfWB = Math.abs(TunerConstants.FrontLeft.LocationX);
    double halfTW = Math.abs(TunerConstants.FrontLeft.LocationY);
    Translation2d[] moduleOffsets = {
      new Translation2d(halfWB, halfTW),
      new Translation2d(halfWB, -halfTW),
      new Translation2d(-halfWB, halfTW),
      new Translation2d(-halfWB, -halfTW),
    };
    return new RobotConfig(ROBOT_MASS_KG, ROBOT_MOI_KG_M2, moduleConfig, moduleOffsets);
  }
}
