package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeSIM;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterSIM;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.spindexer.SpindexerSIM;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretSIM;
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

  // ==================== Subsystems ====================
  private final Shooter shooter = RobotBase.isSimulation() ? new ShooterSIM() : new Shooter();
  private final Turret turret = RobotBase.isSimulation() ? new TurretSIM() : new Turret();
  private final Spindexer spindexer =
      RobotBase.isSimulation() ? new SpindexerSIM() : new Spindexer();
  private final Intake intake = RobotBase.isSimulation() ? new IntakeSIM() : new Intake();

  public boolean isIntakeDeployed = false;

  // ==================== Constructor ====================

  public Superstructure(Supplier<SwerveDriveState> driveState) {
    turret.setDefaultCommand(turret.trackHubCommand(driveState));
  }

  // ==================== Coordinated Commands ====================

  public Command beginShoot() {
    return Commands.sequence(
        spindexer.startCommand(),
        shooter.runVelocity(50),
        Commands.waitUntil(
            () -> shooter.flywheelIsAtTarget()),
        spindexer.startKickerCommand());
  }

  public Command stopShoot() {
    return Commands.sequence(
        spindexer.stopCommand(), spindexer.stopKickerCommand(), shooter.stopCommand());
  }

  public Command startIntake() {
    return Commands.either(
        intake.startIntake(),
        Commands.parallel(
          intake.intakeDown(),
          intake.startIntake()
        ), () -> isIntakeDeployed
      );
  }

  public Command stopIntake() {
    return intake.stopIntake();
  }

  public Command stowIntake() {
    return intake.intakeStowed();
  }
}
