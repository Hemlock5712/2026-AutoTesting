package frc.robot.autonomous;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autonomous.AutoCommands.PathAction;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.intake.IntakeCoordinator;
import frc.robot.utils.FieldInfo;
import frc.robot.utils.path.PathData;
import frc.robot.utils.path.Paths;
import java.util.List;

public class AutoRoutines {

  private final AutoCommands autoCommands;
  private final Superstructure superstructure;
  private final IntakeCoordinator intakeCoordinator;

  private static final double RIGHT_TRENCH_CENTER = 0.639445; // Center of right trench
  private static final double LEFT_TRENCH_CENTER = FieldInfo.width().in(Meters) - 0.639445;
  private static final double BUMPERS_ON_LINE =
      4.378; // Under trench, bumpers just barely on the line, starting X

  public AutoRoutines(
      AutoCommands autoCommands,
      Superstructure superstructure,
      IntakeCoordinator intakeCoordinator) {
    this.autoCommands = autoCommands;
    this.superstructure = superstructure;
    this.intakeCoordinator = intakeCoordinator;
  }

  /**
   * Snaps a rotation angle to the nearest 180 degree increment.
   *
   * @param rotation The rotation to snap
   * @return The rotation snapped to the nearest 180 degrees (0 or 180)
   */
  public static Rotation2d snapToNearest180Degrees(Rotation2d rotation) {
    double degrees = rotation.getDegrees();
    double snapped = Math.round(degrees / 180.0) * 180.0;
    return Rotation2d.fromDegrees(snapped);
  }

  /**
   * Runs a path with a series of actions. Always shoots at the end.
   *
   * <p>Actions:
   *
   * <p>- HubShoot: Shoots at the hub - FeedShoot: Shoots in feeding mode - StopShoot: Stops the
   * shooter
   *
   * @param path The path to run
   * @return A command that runs the path with the actions
   */
  public Command autoBuilder(PathData path) {
    PathData pathData = Paths.forAlliance(path);
    return Commands.sequence(
        autoCommands.resetPose(() -> pathData.getStartingPose()),
        intakeCoordinator.downAndRunFast(),
        autoCommands.followPathWithActions(
            pathData,
            List.of(
                new PathAction("HubShoot", 0.5, superstructure::hubShoot),
                new PathAction("FeedShoot", 0.5, superstructure::feedShoot),
                new PathAction("StopShoot", 0.25, superstructure::stopShoot),
                new PathAction("SlowRaiseIntake", 0.5, intakeCoordinator::slowUpAndRun),
                new PathAction("RunIntake", 0.5, intakeCoordinator::downAndRunFast))),
        superstructure.hubShoot());
  }
}
