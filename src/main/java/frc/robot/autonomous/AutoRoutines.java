package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.autonomous.AutoCommands.PathAction;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.intake.IntakeCoordinator;
import frc.robot.utils.path.PathData;
import frc.robot.utils.path.Paths;
import java.util.List;
import java.util.function.BooleanSupplier;

public class AutoRoutines {

  private final AutoCommands autoCommands;
  private final Superstructure superstructure;
  private final IntakeCoordinator intakeCoordinator;

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
    return autoBuilder(path, () -> true);
  }

  public Command autoBuilder(PathData path, BooleanSupplier resetPose) {
    PathData pathData = Paths.forAlliance(path);
    return Commands.sequence(
        Commands.either(
            autoCommands.resetPose(() -> pathData.getStartingPose()), Commands.none(), resetPose),
        intakeCoordinator.deployAndRunAUTO(),
        followPathWithEvents(pathData),
        superstructure.hubShoot());
  }

  private Command followPathWithEvents(PathData path) {
    return autoCommands.followPathWithActions(
        path,
        List.of(
            new PathAction("HubShoot", 0.5, superstructure::hubShoot),
            new PathAction("FeedShoot", 0.5, superstructure::feedShoot),
            new PathAction("StopShoot", 0.25, superstructure::stopShoot),
            new PathAction("SlowRaiseIntake", 0.5, intakeCoordinator::slowUpAndRun),
            new PathAction("RunIntake", 0.5, intakeCoordinator::deployAndRunAUTO)));
  }

  private Command followPathWithEvents(PathData path, double completionTolerance) {
    return autoCommands.followPathWithActions(
        path,
        List.of(
            new PathAction("HubShoot", 0.5, superstructure::hubShoot),
            new PathAction("FeedShoot", 0.5, superstructure::feedShoot),
            new PathAction("StopShoot", 0.25, superstructure::stopShoot),
            new PathAction("SlowRaiseIntake", 0.5, intakeCoordinator::slowUpAndRun),
            new PathAction("RunIntake", 0.5, intakeCoordinator::deployAndRunAUTO)),
        completionTolerance);
  }

  public Command leftSide2Passes() {
    PathData[] cleanupPath = new PathData[1];
    Command[] prebuiltCleanup = new Command[1];
    return Commands.sequence(
        autoCommands.resetPose(() -> Paths.LEFT_TO_MIDDLE.getStartingPose()),
        intakeCoordinator.deployAndRunAUTO(),
        followPathWithEvents(Paths.LEFT_TO_MIDDLE, 0.15),
        // Build cleanup path from robot position while shooting
        Commands.parallel(
            new WaitCommand(5).deadlineFor(superstructure.shoot()),
            Commands.runOnce(
                () -> {
                  cleanupPath[0] =
                      Paths.LEFT_TO_MIDDLE_CLEANUP.withStartingPoint(
                          autoCommands.getRobotTranslation());
                  prebuiltCleanup[0] = followPathWithEvents(cleanupPath[0], 0.15);
                })),
        superstructure.stopShoot(),
        autoCommands.resetPose(() -> cleanupPath[0].getStartingPose()),
        autoCommands.deferCommand(() -> prebuiltCleanup[0]),
        superstructure.shoot());
  }
}
