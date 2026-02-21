package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.jni.SwerveJNI.DriveState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.turret.Turret;

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
  private final Arm arm;
  private final Flywheel flywheel;
  private final Turret turret;
  private final Supplier<SwerveDriveState> driveState;

  // ==================== Constructor ====================

  public Superstructure(Arm arm, Flywheel flywheel, Turret turret, CommandSwerveDrivetrain drivetrain) {
    this.arm = arm;
    this.flywheel = flywheel;
    this.turret = turret;
    this.driveState = () -> drivetrain.getState();
  }

  // ==================== Coordinated Commands ====================

  public Command beginShoot() {
    return flywheel.spinUp();
  }

  public Command aimCommand() {
    return Commands.run(() -> turret.aimCommand(() -> driveState.get().Pose));
  }

}
