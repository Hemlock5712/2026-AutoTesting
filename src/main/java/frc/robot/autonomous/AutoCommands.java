package frc.robot.autonomous;

import choreo.trajectory.EventMarker;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveToPoint;
import frc.robot.commands.FollowPath;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.path.ArcLengthTrajectory;
import frc.robot.utils.path.AutoPath;
import frc.robot.utils.path.FollowablePath;
import frc.robot.utils.path.ProjectionResult;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Reusable command builders for autonomous routines.
 *
 * <p>Paths come from {@link AutoPath} (Choreo trajectories re-parameterized by arc-length). All
 * path following is distance-based — the robot tracks its position on the path, not a clock.
 */
public class AutoCommands {

  private final CommandSwerveDrivetrain drivetrain;

  public AutoCommands(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  public DriveToPoint driveTo(Supplier<Pose2d> pose) {
    return new DriveToPoint(drivetrain, pose);
  }

  public Command resetPose(Supplier<Pose2d> pose) {
    return drivetrain.runOnce(() -> drivetrain.resetPose(pose.get()));
  }

  /** Resets the robot pose to the start of the given path (alliance-aware). */
  public Command resetPose(AutoPath path) {
    return resetPose(
        () -> {
          FollowablePath traj = path.get();
          return new Pose2d(traj.getPoint(0), traj.getHeading(0));
        });
  }

  // ==================== Path Actions ====================

  /**
   * An action to trigger at a specific arc-length position along a path.
   *
   * @param triggerS Arc-length position in meters where the action triggers
   * @param triggerDistance How far before triggerS to fire (meters, subtracted from triggerS)
   * @param command The command to run when triggered
   */
  public record PathAction(double triggerS, double triggerDistance, Supplier<Command> command) {

    /**
     * Creates a PathAction from a Choreo event marker timestamp.
     *
     * @param trajectory The arc-length trajectory (for timestamp→s conversion)
     * @param markerTimestamp The Choreo event marker timestamp in seconds
     * @param triggerDistance How far before the marker to fire (meters)
     * @param command The command to run
     * @return A PathAction with the correct arc-length trigger
     */
    public static PathAction fromMarker(
        ArcLengthTrajectory trajectory,
        double markerTimestamp,
        double triggerDistance,
        Supplier<Command> command) {
      return new PathAction(
          trajectory.getArcLengthAtTimestamp(markerTimestamp), triggerDistance, command);
    }
  }

  /**
   * Reads event markers from a Choreo trajectory and converts them to distance-based PathActions.
   *
   * @param path The AutoPath whose Choreo trajectory contains event markers
   * @param eventMap Maps event marker names to the commands they should trigger
   * @return A list of PathActions, one per matching marker occurrence
   */
  public List<PathAction> actionsFromChoreoEvents(
      AutoPath path, Map<String, Supplier<Command>> eventMap) {
    ArcLengthTrajectory traj = path.get();
    List<PathAction> actions = new ArrayList<>();
    for (var entry : eventMap.entrySet()) {
      for (EventMarker marker : path.trajectory().getEvents(entry.getKey())) {
        actions.add(PathAction.fromMarker(traj, marker.timestamp, 0.0, entry.getValue()));
      }
    }
    return actions;
  }

  record ScheduledPathAction(double triggerS, Supplier<Command> commandSupplier) {}

  static final class ActivePathActionRunner extends Command {
    private final List<ScheduledPathAction> actions;
    private final DoubleSupplier progressSupplier;
    private int nextActionIndex = 0;
    private Command activeCommand;
    private boolean activeCommandInitialized = false;

    ActivePathActionRunner(List<ScheduledPathAction> actions, DoubleSupplier progressSupplier) {
      this.actions = List.copyOf(actions);
      this.progressSupplier = progressSupplier;
    }

    @Override
    public void execute() {
      double progress = progressSupplier.getAsDouble();

      while (nextActionIndex < actions.size()
          && progress >= actions.get(nextActionIndex).triggerS()) {
        scheduleReplacement(actions.get(nextActionIndex).commandSupplier().get());
        nextActionIndex++;
      }

      runActiveCommand();
    }

    private void scheduleReplacement(Command nextCommand) {
      if (activeCommand != null) {
        activeCommand.end(true);
      }
      activeCommand = nextCommand;
      activeCommandInitialized = false;
    }

    private void runActiveCommand() {
      if (activeCommand == null) return;

      if (!activeCommandInitialized) {
        activeCommand.initialize();
        activeCommandInitialized = true;
      }

      activeCommand.execute();
      if (activeCommand.isFinished()) {
        activeCommand.end(false);
        activeCommand = null;
        activeCommandInitialized = false;
      }
    }

    @Override
    public void end(boolean interrupted) {
      if (activeCommand != null) {
        activeCommand.end(true);
        activeCommand = null;
        activeCommandInitialized = false;
      }
    }

    @Override
    public boolean isFinished() {
      return false;
    }
  }

  private static final double ACTION_TRIGGER_PROJECTION_WINDOW = 0.75;

  /**
   * Follow a path with distance-triggered actions and optional alongside commands.
   *
   * @param path The path to follow (from AutoPath or any FollowablePath)
   * @param actions Ordered list of actions to trigger along the path
   * @param alongside Commands that run for the entire path duration
   * @return A command that follows the path with all actions wired
   */
  public Command followPathWithActions(
      FollowablePath path, List<PathAction> actions, Command... alongside) {
    return followPathWithActionsInternal(path, actions, -1, alongside);
  }

  public Command followPathWithActions(
      FollowablePath path,
      List<PathAction> actions,
      double completionTolerance,
      Command... alongside) {
    return followPathWithActionsInternal(path, actions, completionTolerance, alongside);
  }

  private Command followPathWithActionsInternal(
      FollowablePath path,
      List<PathAction> actions,
      double completionTolerance,
      Command... alongside) {
    FollowPath pathCmd = new FollowPath(drivetrain, path);
    if (completionTolerance > 0) {
      pathCmd.withCompletionTolerance(completionTolerance);
    }
    return wrapWithActions(pathCmd, path, actions, alongside);
  }

  private Command wrapWithActions(
      Command pathCmd, FollowablePath path, List<PathAction> actions, Command... alongside) {
    if (actions.isEmpty()) {
      return alongside.length == 0 ? pathCmd : pathCmd.deadlineFor(alongside);
    }

    // Build scheduled actions sorted by trigger arc-length
    List<ScheduledPathAction> scheduled = new ArrayList<>(actions.size());
    for (PathAction action : actions) {
      double triggerS = Math.max(0.0, action.triggerS() - action.triggerDistance());
      scheduled.add(new ScheduledPathAction(triggerS, action.command()));
    }
    scheduled.sort((a, b) -> Double.compare(a.triggerS(), b.triggerS()));
    scheduled = groupScheduledActions(scheduled);

    double[] projectedS = {0.0};
    ActivePathActionRunner actionRunner =
        new ActivePathActionRunner(scheduled, () -> updateProjectedS(path, projectedS));

    Command[] deadlineCommands = new Command[alongside.length + 1];
    deadlineCommands[0] = actionRunner;
    System.arraycopy(alongside, 0, deadlineCommands, 1, alongside.length);

    return pathCmd.deadlineFor(deadlineCommands);
  }

  List<ScheduledPathAction> groupScheduledActions(List<ScheduledPathAction> rawScheduledActions) {
    List<ScheduledPathAction> groupedActions = new ArrayList<>(rawScheduledActions.size());
    int index = 0;
    while (index < rawScheduledActions.size()) {
      ScheduledPathAction action = rawScheduledActions.get(index);
      List<Supplier<Command>> groupedSuppliers = new ArrayList<>();
      groupedSuppliers.add(action.commandSupplier());
      index++;

      while (index < rawScheduledActions.size()) {
        ScheduledPathAction nextAction = rawScheduledActions.get(index);
        if (Double.compare(action.triggerS(), nextAction.triggerS()) != 0) break;
        groupedSuppliers.add(nextAction.commandSupplier());
        index++;
      }

      groupedActions.add(
          new ScheduledPathAction(action.triggerS(), parallelSupplier(groupedSuppliers)));
    }
    return groupedActions;
  }

  private Supplier<Command> parallelSupplier(List<Supplier<Command>> commandSuppliers) {
    List<Supplier<Command>> suppliers = List.copyOf(commandSuppliers);
    return () -> {
      if (suppliers.size() == 1) return suppliers.get(0).get();
      return Commands.parallel(suppliers.stream().map(Supplier::get).toArray(Command[]::new));
    };
  }

  private double updateProjectedS(FollowablePath path, double[] projectedS) {
    double lastProjectedS = projectedS[0];
    ProjectionResult projection =
        path.getClosestPointInRange(
            drivetrain.getPose().getTranslation(),
            Math.max(0.0, lastProjectedS - ACTION_TRIGGER_PROJECTION_WINDOW),
            Math.min(path.getTotalLength(), lastProjectedS + ACTION_TRIGGER_PROJECTION_WINDOW));
    projectedS[0] = Math.max(lastProjectedS, projection.s());
    return projectedS[0];
  }
}
