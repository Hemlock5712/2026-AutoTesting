package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.spindexer.Spindexer;
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
  private final Spindexer spindexer;
  private final Supplier<SwerveDriveState> driveState;

  @Logged(name = "Turret Mechanism3D")
  public Pose3d[] turretPose = new Pose3d[] {Turret.TURRET_HOLE_CENTER, new Pose3d()};

  // ==================== Constructor ====================

  public Superstructure(
      Flywheel flywheel,
      Turret turret,
      Spindexer spindexer,
      Supplier<SwerveDriveState> driveState) {
    this.flywheel = flywheel;
    this.turret = turret;
    this.spindexer = spindexer;
    this.driveState = driveState;
  }

  @Override
  public void periodic() {
    try {
      turretPose[0] =
          Turret.TURRET_HOLE_CENTER.transformBy(
              new Transform3d(
                  Translation3d.kZero, new Rotation3d(0, 0, turret.getAngle().in(Radians))));

      // TODO: Use actual hood position once subsystem is made
      // Only need to change the Degrees to the actual hood angle
      turretPose[1] =
          turretPose[0]
              .transformBy(new Transform3d(0.111203, 0.0, 0.05698, Rotation3d.kZero))
              .transformBy(
                  new Transform3d(
                      Translation3d.kZero, new Rotation3d(0, -Degrees.of(75).in(Radians), 0)));
    } catch (Exception e) {
      // If this breaks, just ignore it. I just need to make sure it absolutely
      // doesn't break the robot code.
    }
  }

  // ==================== Coordinated Commands ====================

  public Command beginShoot() {
    return Commands.sequence(
        spindexer.startCommand(),
        flywheel.spinUp(),
        Commands.either(
            spindexer.startKickerCommand(), Commands.none(), () -> flywheel.isAtTarget()));
  }

  public Command aimCommand() {
    return idle().alongWith(turret.trackHubCommand(driveState));
  }
}
