package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterSIM;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.spindexer.SpindexerSIM;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretSIM;
import frc.robot.utils.FieldInfo;
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

  // ==================== Targeting Data (calculated once per loop) ====================

  private Translation2d targetPosition = FieldInfo.HUB_POSITION;
  private double distanceToHub = 0;
  private double angleToHub = 0;

  // ==================== Constructor ====================

  public Superstructure(Supplier<SwerveDriveState> driveState) {
    this.driveState = driveState;
    // Set turret tracking as default command (can be overridden by other commands)
    // turret.setDefaultCommand(turret.trackHubCommand(() -> angleToHub));
  }

  // ==================== Periodic ====================

  public void update() {
    // Calculate targeting data once per loop (used by turret tracking and shooter)
    SwerveDriveState state = driveState.get();
    Pose2d robotPose = state.Pose;

    if (FieldInfo.getAllianceZone().contains(robotPose.getTranslation())) {
      targetPosition = FieldInfo.flip(FieldInfo.HUB_POSITION);
    } else {
      targetPosition =
          robotPose.getY() > FieldInfo.width().baseUnitMagnitude() / 2.0
              ? FieldInfo.flip(
                  FieldInfo.shouldFlip()
                      ? FieldInfo.LEFT_FEED_POSITION
                      : FieldInfo.RIGHT_FEED_POSITION)
              : FieldInfo.flip(
                  FieldInfo.shouldFlip()
                      ? FieldInfo.RIGHT_FEED_POSITION
                      : FieldInfo.LEFT_FEED_POSITION);
    }

    Pose2d turretPose = robotPose.transformBy(TURRET_TRANSFORM);
    Translation2d hubPosition = FieldInfo.flip(targetPosition);
    Translation2d toTarget = hubPosition.minus(turretPose.getTranslation());

    distanceToHub = toTarget.getNorm();

    Rotation2d angleToTargetField = toTarget.getAngle();
    angleToHub =
        MathUtil.inputModulus(
            angleToTargetField.minus(robotPose.getRotation()).getRotations(), -0.25, 0.75);
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

  // ==================== Coordinated Commands ====================

  public Command beginShoot() {
    return Commands.sequence(
        // shooter.runDynamic(() -> distanceToHub),
        shooter.runVelocity(35),
        new WaitUntilCommand(() -> shooter.flywheelIsAtTarget()),
        spindexer.startCommand(),
        spindexer.startKickerVoltageCommand());
  }

  public Command stopShoot() {
    return Commands.sequence(
        spindexer.stopCommand(), spindexer.stopKickerCommand(), shooter.stopCommand());
  }
}
