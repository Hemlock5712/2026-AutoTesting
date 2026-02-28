package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelSIM;
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
  private final Flywheel flywheel = RobotBase.isSimulation() ? new FlywheelSIM() : new Flywheel();
  private final Turret turret = RobotBase.isSimulation() ? new TurretSIM() : new Turret();
  private final Spindexer spindexer =
      RobotBase.isSimulation() ? new SpindexerSIM() : new Spindexer();
  private final Supplier<SwerveDriveState> driveState;

  // ==================== Constructor ====================

  public Superstructure(Supplier<SwerveDriveState> driveState) {
    this.driveState = driveState;
    turret.setDefaultCommand(turret.trackHubCommand(driveState));
  }

  // ==================== Coordinated Commands ====================

  public Command beginShoot() {
    return Commands.sequence(
        spindexer.startCommand(),
        flywheel.spinUp(),
        Commands.waitUntil(() -> flywheel.isAtTarget() && turret.isAtTarget()),
        spindexer.startKickerCommand());
  }

  public Command deployIntake() {
    return Commands.none();
  }

  public Command RetractIntake() {
    return Commands.none();
  }
}
