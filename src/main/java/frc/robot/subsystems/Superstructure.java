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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterLookup;
import frc.robot.subsystems.shooter.ShooterSIM;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.spindexer.SpindexerSIM;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretSIM;
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

  // ==================== Subsystems ====================
  private final Shooter shooter = RobotBase.isSimulation() ? new ShooterSIM() : new Shooter();
  private final Turret turret = RobotBase.isSimulation() ? new TurretSIM() : new Turret();
  private final Spindexer spindexer =
      RobotBase.isSimulation() ? new SpindexerSIM() : new Spindexer();

  private final Supplier<SwerveDriveState> driveState;

  private final TunableDouble targetFlywheelVelocity = Tunables.value("Tuning/Flywheel", 26.0);
  private final TunableDouble targetHoodAngle = Tunables.value("Tuning/Hood", 3.0);

  // ==================== Targeting Data (calculated once per loop)
  // ====================

  private Translation2d targetPosition = FieldInfo.HUB_POSITION;
  private double distanceToHub = 0;
  private double angleToHub = 0;

  // SWM state
  private Translation2d virtualTargetPosition = FieldInfo.HUB_POSITION;
  private Transform2d virtualTargetPositionPose =
      new Transform2d(
          FieldInfo.HUB_POSITION.getX(), FieldInfo.HUB_POSITION.getY(), Rotation2d.kZero);
  private double distanceToVirtualTarget = 0;
  private double angleToVirtualTarget = 0;

  private boolean isShooting = false;

  // ==================== Constructor ====================

  public Superstructure(Supplier<SwerveDriveState> driveState) {
    this.driveState = driveState;
    // Set turret tracking as default command - uses SWM-aware getters for seamless
    // mode switching
    turret.setDefaultCommand(turret.trackHubCommand(this::getActiveAngle));
    shooter.setDefaultCommand(shooter.runHoodDynamic(this::getActiveDistance));
  }

  // ==================== Periodic ====================

  public void update() {
    // Calculate targeting data once per loop (used by turret tracking and shooter)
    SwerveDriveState state = driveState.get();
    Pose2d robotPose = state.Pose;

    if (FieldInfo.getAllianceZone().contains(robotPose.getTranslation())) {
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

    // Calculate SWM targeting values
    virtualTargetPosition = virtualTarget(state);
    virtualTargetPositionPose = new Transform2d(virtualTargetPosition, Rotation2d.kZero);
    Translation2d toVirtualTarget = virtualTargetPosition.minus(turretPose.getTranslation());
    distanceToVirtualTarget = toVirtualTarget.getNorm();
    Rotation2d angleToVirtualTargetField = toVirtualTarget.getAngle();
    angleToVirtualTarget =
        MathUtil.inputModulus(
            angleToVirtualTargetField.minus(robotPose.getRotation()).getRotations(), -0.25, 0.75);

    // Telemetry
    Robot.telemetry()
        .log(
            "SWM/VirtualTarget",
            new Pose2d(virtualTargetPosition, Rotation2d.kZero),
            Pose2d.struct);
    Robot.telemetry().log("SWM/DistanceDelta", distanceToVirtualTarget - distanceToHub);
    Robot.telemetry().log("SWM/AngleDelta", angleToVirtualTarget - angleToHub);
  }

  // ==================== Targeting Getters ====================

  public double getDistanceToHub() {
    return distanceToHub;
  }

  public double getAngleToHub() {
    return angleToHub;
  }

  public Pose2d getTargetPosition() {
    return new Pose2d(targetPosition, new Rotation2d());
  }

  // ==================== SWM-Aware Getters ====================

  /** Returns distance based on SWM mode - virtual target when enabled, real target otherwise. */
  public double getActiveDistance() {
    return distanceToVirtualTarget;
  }

  /** Returns angle based on SWM mode - virtual target when enabled, real target otherwise. */
  public double getActiveAngle() {
    return angleToVirtualTarget;
  }

  // ==================== Coordinated Commands ====================

  public Command shoot() {
    return shooter
        .runDynamic(this::getDistanceToHub)
        .alongWith(Commands.runOnce(() -> isShooting = true))
        .alongWith(
            Commands.sequence(
                Commands.waitUntil(() -> shooter.flywheelIsAtTarget() && turret.isAtTarget()),
                spindexer.startCommand(),
                spindexer.startKickerCommand()));
  }

  public Command tuningShoot() {
    return shooter
        .runShooterTestMode(
            () -> targetFlywheelVelocity.get(), () -> Degrees.of(targetHoodAngle.get()))
        .alongWith(Commands.runOnce(() -> isShooting = true))
        .alongWith(
            Commands.sequence(
                Commands.waitUntil(() -> shooter.flywheelIsAtTarget()),
                spindexer.startCommand(),
                spindexer.startKickerVoltageCommand()));
  }

  public Command stopShoot() {
    return Commands.sequence(
        Commands.runOnce(() -> isShooting = false),
        spindexer.stopCommand(),
        spindexer.stopKickerCommand(),
        shooter.stopCommand());
  }

  // ==================== SWM Commands ====================

  /** Full SWM shooting sequence with velocity compensation. */
  public Command swmShoot() {
    return Commands.parallel(
        shooter.runDynamic(this::getActiveDistance),
        Commands.runOnce(() -> isShooting = true),
        Commands.sequence(
            Commands.waitUntil(() -> shooter.flywheelIsAtTarget() && turret.isAtTarget()),
            spindexer.startCommand(),
            spindexer.CommandRunKickerCommand(10)));
  }

  public Command spinSpinDexerBack() {
    return spindexer.backCommand();
  }

  public Command spinSpinDexerStop() {
    return spindexer.stopCommand();
  }

  private Translation2d virtualTarget(SwerveDriveState state) {
    Translation2d realTarget = getTargetPosition().getTranslation();
    Pose2d turretPose = state.Pose.transformBy(TURRET_TRANSFORM);
    Translation2d robotPosition = turretPose.getTranslation();

    ChassisSpeeds fieldSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, state.Pose.getRotation());
    Translation2d velocity =
        new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);

    Translation2d virtualTarget = realTarget;

    for (int i = 0; i < 5; i++) {
      double dist = robotPosition.getDistance(virtualTarget);
      if (dist < 0.001) break;
      double tof = ShooterLookup.getToFMap().get(dist);
      virtualTarget = realTarget.minus(velocity.times(tof));
    }
    return virtualTarget;
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

  public boolean isShooting() {
    return isShooting;
  }
}
