package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.turret.Turret;
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
public class Superstructure extends SubsystemBase {

  // ==================== Subsystems ====================
  private final Flywheel flywheel;
  private final Turret turret;
  private final Supplier<SwerveDriveState> driveState;

  // ==================== Constructor ====================

  public Superstructure(Flywheel flywheel, Turret turret, Supplier<SwerveDriveState> driveState) {
    this.flywheel = flywheel;
    this.turret = turret;
    this.driveState = driveState;
  }

  // ==================== Coordinated Commands ====================

  public Command beginShoot() {
    return flywheel.spinUp();
  }

  public Command aimCommand() {
    return turret.trackHubCommand(driveState);
  }
}
