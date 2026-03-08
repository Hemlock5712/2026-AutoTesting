package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterSIM;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.spindexer.SpindexerSIM;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretSIM;
import frc.robot.swm.GeneratedCorrectionTable;
import frc.robot.swm.MovingCorrectionTable;
import frc.robot.swm.ShootWhileMovingSolver;
import frc.robot.swm.ShootWhileMovingSolver.ShotSolution;
import frc.robot.swm.StationaryShotTable;
import frc.robot.utils.FieldInfo;
import frc.robot.utils.Tunables;
import frc.robot.utils.Tunables.TunableDouble;
import java.util.function.Supplier;

/**
 * Superstructure - Controls the Arm and Flywheel together.
 *
 * <p>This coordinates:
 *
 * <ul>
 *   <li>Arm - Moves horizontal and vertical to position game pieces
 *   <li>Flywheel - Spins the shooter wheels at the right speed
 * </ul>
 *
 * <p>Instead of controlling the arm and flywheel separately, this gives you simple commands like
 * "score low" or "prepare for shooting" that move both parts together. This makes driving easier
 * and ensures everything moves in sync.
 */
@Logged
public class Superstructure {

  // ==================== Constants ====================

  /** Center position of the turret hole relative to robot center (meters). */
  public static final Pose3d TURRET_HOLE_CENTER =
      new Pose3d(-0.127, 0.13018, 0.3556, Rotation3d.kZero);

  /** 2D transform from robot center to turret position for field calculations. */
  public static final Transform2d TURRET_TRANSFORM =
      new Transform2d(TURRET_HOLE_CENTER.getX(), TURRET_HOLE_CENTER.getY(), Rotation2d.kZero);

  /** Flywheel wheel radius in meters (4-inch wheel). */
  private static final double WHEEL_RADIUS = 0.0508;

  // ==================== Subsystems ====================
  private final Shooter shooter = RobotBase.isSimulation() ? new ShooterSIM() : new Shooter();
  private final Turret turret = RobotBase.isSimulation() ? new TurretSIM() : new Turret();
  private final Spindexer spindexer =
      RobotBase.isSimulation() ? new SpindexerSIM() : new Spindexer();

  private final Supplier<SwerveDriveState> driveState;
  private final ShootWhileMovingSolver swmSolver;
  private final StationaryShotTable stationaryTable;

  private final TunableDouble targetFlywheelVelocity = Tunables.value("Tuning/Flywheel", 26.0);
  private final TunableDouble targetHoodAngle = Tunables.value("Tuning/Hood", 3.0);

  /** Cached SWM solution from update(). */
  private ShotSolution currentSolution;

  // ==================== Targeting Data (calculated once per loop)
  // ====================

  private Translation2d targetPosition = FieldInfo.HUB_POSITION;
  private double distanceToHub = 0;
  private double angleToHub = 0;

  // SWM state
  private double distanceToVirtualTarget = 0;
  private double angleToVirtualTarget = 0;

  private boolean isShooting = false;

  // ==================== Constructor ====================

  public Superstructure(Supplier<SwerveDriveState> driveState) {
    this.driveState = driveState;

    // Initialize SWM solver with calibration data
    // Note: elevation angles converted using (80 - original) formula
    stationaryTable = new StationaryShotTable();
    stationaryTable.addPointRPS(2.0, 30.0, WHEEL_RADIUS, 0.0); // 80 - 80
    stationaryTable.addPointRPS(2.5, 29.0, WHEEL_RADIUS, 4.0); // 80 - 76
    stationaryTable.addPointRPS(3.0, 30.0, WHEEL_RADIUS, 7.0); // 80 - 73
    stationaryTable.addPointRPS(3.5, 32.0, WHEEL_RADIUS, 8.0); // 80 - 72
    stationaryTable.addPointRPS(4.0, 35.0, WHEEL_RADIUS, 9.0); // 80 - 71
    stationaryTable.addPointRPS(4.5, 36.0, WHEEL_RADIUS, 10.5); // 80 - 69.5

    MovingCorrectionTable correctionTable = GeneratedCorrectionTable.create();
    swmSolver = new ShootWhileMovingSolver(stationaryTable, correctionTable);

    // Set turret tracking as default command - uses SWM-aware getters for seamless
    // mode switching
    turret.setDefaultCommand(turret.trackHubCommand(this::getActiveAngle));
    shooter.setDefaultCommand(shooter.runHoodDynamic(this::getActiveElevationDeg));
  }

  // ==================== Periodic ====================

  public void update() {
    // Calculate targeting data once per loop (used by turret tracking and shooter)
    SwerveDriveState state = driveState.get();
    Pose2d robotPose = state.Pose;

    if (FieldInfo.isInAllianceZone(robotPose.getTranslation())) {
      targetPosition = FieldInfo.flip(FieldInfo.HUB_POSITION);
    } else {
      // Compute both feed positions in current-alliance coordinates, then pick the
      // one
      // on the same side of the field (upper vs. lower Y half) as the robot.
      Translation2d feedA = FieldInfo.flip(FieldInfo.LEFT_FEED_POSITION);
      Translation2d feedB = FieldInfo.flip(FieldInfo.RIGHT_FEED_POSITION);
      Translation2d upperFeed = feedA.getY() > feedB.getY() ? feedA : feedB;
      Translation2d lowerFeed = feedA.getY() > feedB.getY() ? feedB : feedA;
      targetPosition =
          robotPose.getY() > FieldInfo.width().baseUnitMagnitude() / 2.0 ? upperFeed : lowerFeed;
    }

    Pose2d turretPose = robotPose.transformBy(TURRET_TRANSFORM);
    Translation2d toTarget = targetPosition.minus(turretPose.getTranslation());

    distanceToHub = toTarget.getNorm();

    Rotation2d angleToTargetField = toTarget.getAngle();
    angleToHub =
        MathUtil.inputModulus(
            angleToTargetField.minus(robotPose.getRotation()).getRotations(), -0.25, 0.75);

    // Compute SWM solution using solver
    currentSolution = swmSolver.solve(turretPose, state.Speeds, targetPosition);

    // Extract values from solution
    distanceToVirtualTarget = currentSolution.distanceM;

    // Convert field-relative azimuth to robot-relative rotations
    Rotation2d solutionAzimuth = new Rotation2d(currentSolution.turretAzimuthRad);
    angleToVirtualTarget =
        MathUtil.inputModulus(
            solutionAzimuth.minus(robotPose.getRotation()).getRotations(), -0.25, 0.75);

    // Calculate virtual target (where SWM is compensating to)
    Translation2d virtualTarget =
        turretPose
            .getTranslation()
            .plus(new Translation2d(currentSolution.distanceM, solutionAzimuth));

    // Telemetry
    Robot.telemetry().log("SWM/FlywheelRPS", currentSolution.getFlywheelRPS(WHEEL_RADIUS));
    Robot.telemetry().log("SWM/ElevationDeg", currentSolution.getElevationDeg());
    Robot.telemetry().log("SWM/TurretAzimuthDeg", currentSolution.getTurretAzimuthDeg());
    Robot.telemetry().log("SWM/VRadial", currentSolution.vRadialMps);
    Robot.telemetry().log("SWM/VTangential", currentSolution.vTangentialMps);
    Robot.telemetry().log("SWM/DistanceDelta", distanceToVirtualTarget - distanceToHub);
    Robot.telemetry().log("SWM/AngleDelta", angleToVirtualTarget - angleToHub);
    Robot.telemetry().log("SWM/IsValid", currentSolution.isValid());
    Robot.telemetry().log("SWM/IsReadyToShoot", currentSolution.isReadyToShoot());
    Robot.telemetry().log("SWM/AzimuthOffsetDeg", Math.toDegrees(currentSolution.azimuthOffsetRad));
    Robot.telemetry()
        .log("SWM/TargetPose", new Pose2d(targetPosition, Rotation2d.kZero), Pose2d.struct);
    Robot.telemetry()
        .log("SWM/VirtualTargetPose", new Pose2d(virtualTarget, Rotation2d.kZero), Pose2d.struct);
  }

  // ==================== Internal Getters ====================

  private double getActiveAngle() {
    if (currentSolution == null || !currentSolution.isValid()) {
      return angleToHub; // Fall back to direct aim at hub
    }
    return angleToVirtualTarget;
  }

  private double getActiveFlywheelRPS() {
    if (currentSolution == null || !currentSolution.isValid()) {
      // Fall back to stationary shot at current distance
      return stationaryTable.getSpeed(distanceToHub) / (2.0 * Math.PI * WHEEL_RADIUS);
    }
    return currentSolution.getFlywheelRPS(WHEEL_RADIUS);
  }

  // ==================== Coordinated Commands ====================

  private double getActiveElevationDeg() {
    if (currentSolution == null || !currentSolution.isValid()) {
      return stationaryTable.getElevationDeg(distanceToHub);
    }
    return currentSolution.getElevationDeg();
  }

  // ==================== Coordinated Commands ====================

  public Command tuningShoot() {
    return shooter
        .runShooterTestMode(
            () -> targetFlywheelVelocity.get(), () -> Degrees.of(targetHoodAngle.get()))
        .alongWith(Commands.runOnce(this::startShooting))
        .alongWith(
            Commands.sequence(
                Commands.waitUntil(() -> shooter.flywheelIsAtTarget()),
                spindexer.startCommand(),
                spindexer.startKickerVoltageCommand()));
  }

  public Command stopShoot() {
    return Commands.sequence(
        Commands.runOnce(this::stopShooting),
        spindexer.stopCommand(),
        spindexer.stopKickerCommand(),
        shooter.stopCommand());
  }

  public Command autoShoot() {
    return shooter
        .runShooterTestMode(
            () -> targetFlywheelVelocity.get(), () -> Degrees.of(targetHoodAngle.get()))
        .alongWith(Commands.runOnce(this::startShooting))
        .alongWith(
            Commands.sequence(
                Commands.waitUntil(() -> shooter.flywheelIsAtTarget()),
                spindexer.startCommand(),
                spindexer.startKickerVoltageCommand()))
        .withTimeout(5);
  }

  // ==================== SWM Commands ====================

  /** Full SWM shooting sequence with velocity compensation. */
  public Command swmShoot() {
    return Commands.parallel(
        shooter
            .runFromSolution(this::getActiveFlywheelRPS, this::getActiveElevationDeg)
            .alongWith(Commands.runOnce(this::startShooting)),
        Commands.sequence(
            Commands.waitUntil(() -> shooter.flywheelIsAtTarget() && turret.isAtTarget()),
            spindexer.startCommand(),
            spindexer.startKickerVoltageCommand()));
  }

  public Command spindexerBack() {
    return spindexer.backCommand();
  }

  public Command spindexerStop() {
    return spindexer.stopCommand();
  }

  public Angle getTargetTurretAngle() {
    return turret.getTargetAngle();
  }

  public Angle getTargetHoodAngle() {
    return shooter.getTargetPosition();
  }

  public AngularVelocity getFlywheelVelocity() {
    return RotationsPerSecond.of(targetFlywheelVelocity.get());
  }

  public void startShooting() {
    isShooting = true;
    turret.setSmoothMotionMagic(false);
  }

  public void stopShooting() {
    isShooting = false;
    turret.setSmoothMotionMagic(true);
  }

  public boolean isShooting() {
    return isShooting;
  }
}
