package frc.robot.autonomous;

import choreo.trajectory.EventMarker;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveToPoint;
import frc.robot.commands.FollowPath;
import frc.robot.subsystems.drive.Drive;
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
 * Helpers for building autonomous routines. Paths come from {@link AutoPath} and are followed by
 * distance (not time) - if the robot stalls, the path waits for it.
 */
public class AutoCommands {

  private final Drive drivetrain;

  public AutoCommands(Drive drivetrain) {
    this.drivetrain = drivetrain;
  }

  public DriveToPoint driveTo(Supplier<Pose2d> pose) {
    return new DriveToPoint(drivetrain, pose);
  }

  public Command resetPose(Supplier<Pose2d> pose) {
    return Commands.runOnce(() -> drivetrain.resetPose(pose.get()), drivetrain);
  }

  /** Resets the robot's pose to the path's starting pose, flipped if we're red alliance. */
  public Command resetPose(AutoPath path) {
    return resetPose(
        () -> {
          FollowablePath traj = path.get();
          return new Pose2d(traj.getPoint(0), traj.getHeading(0));
        });
  }

  // ==================== Path Actions ====================

  /**
   * A command that fires at a certain distance along a path.
   *
   * @param triggerS Where on the path the action fires (meters from start)
   * @param triggerDistance Fire this many meters early (lead time before triggerS)
   * @param command The command to run
   */
  public record PathAction(double triggerS, double triggerDistance, Supplier<Command> command) {

    /** Creates a PathAction from a Choreo event marker, using the marker's timestamp. */
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
   * Pulls all event markers out of a Choreo trajectory and turns them into PathActions. The
   * eventMap maps marker names to the commands that should run when that marker is reached.
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
   * Drives a path while running actions at specific distances along it. Optional alongside commands
   * run for the whole path.
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

    // Sort actions by where they trigger so we fire them in order.
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
