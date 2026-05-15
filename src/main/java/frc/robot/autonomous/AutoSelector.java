package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveToWithAvoidance;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utils.geometry.ExtPose;
import frc.robot.utils.path.AutoPath;
import frc.robot.utils.path.ObstacleField;
import java.util.Map;
import java.util.function.Supplier;

/** Owns the dashboard auto chooser and the registered routines. */
public final class AutoSelector {

  /**
   * Demo start/goal for the runtime-planner auto. Defined in blue-origin coordinates; {@link
   * ExtPose#get()} returns the alliance-correct variant. Replace with your scoring positions.
   *
   * <p>{@link #DEMO_GOAL} is also reused by the driver-A "drive here, avoiding obstacles" binding
   * in {@code RobotContainer} so the two demos point at the same spot.
   */
  private static final ExtPose DEMO_START = new ExtPose(2.5, 1.5, Rotation2d.kZero);

  public static final ExtPose DEMO_GOAL = new ExtPose(14.5, 6.5, Rotation2d.fromDegrees(90));

  private final SendableChooser<Supplier<Command>> chooser = new SendableChooser<>();
  private final AutoCommands autoCommands;

  public AutoSelector(Drive drive, AutoCommands autoCommands, ObstacleField obstacleField) {
    this.autoCommands = autoCommands;
    // "None" is the safe default — explicit no-op auto so a competition where nothing is picked
    // doesn't accidentally start a path-following routine that assumes a specific trajectory file.
    chooser.setDefaultOption("None", Commands::none);
    chooser.addOption("NewPath (PD)", this::newPathAutoPD);
    chooser.addOption(
        "Drive-to-Point Planner Demo",
        () ->
            Commands.sequence(
                Commands.runOnce(() -> drive.resetPose(DEMO_START.get()), drive),
                DriveToWithAvoidance.create(drive, DEMO_GOAL::get, obstacleField)));
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
