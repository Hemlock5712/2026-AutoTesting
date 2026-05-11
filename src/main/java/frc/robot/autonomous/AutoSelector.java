package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.PathPlanningDemo;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utils.path.AutoPath;
import java.util.Map;
import java.util.function.Supplier;

/** Owns the dashboard auto chooser and the registered routines. */
public final class AutoSelector {

  private final SendableChooser<Supplier<Command>> chooser = new SendableChooser<>();
  private final AutoCommands autoCommands;

  public AutoSelector(Drive drive, AutoCommands autoCommands) {
    this.autoCommands = autoCommands;
    // "None" is the safe default — explicit no-op auto so a competition where nothing is picked
    // doesn't accidentally start a path-following routine that assumes a specific trajectory file.
    chooser.setDefaultOption("None", Commands::none);
    chooser.addOption("NewPath (PD)", this::newPathAutoPD);
    chooser.addOption("PathPlanningDemo", () -> PathPlanningDemo.create(drive));
    SmartDashboard.putData("Auto Mode", chooser);
  }

  /** Returns the currently-selected routine (or {@link Commands#none()} if nothing is set). */
  public Command getSelected() {
    Supplier<Command> selected = chooser.getSelected();
    return (selected != null) ? selected.get() : Commands.none();
  }

  private Command newPathAutoPD() {
    return Commands.sequence(
        autoCommands.resetPose(AutoPath.NEW_PATH),
        autoCommands.followPathWithActions(
            AutoPath.NEW_PATH.get(),
            autoCommands.actionsFromChoreoEvents(
                AutoPath.NEW_PATH, Map.of("Marker", () -> Commands.print("Marker triggered!")))));
  }
}
