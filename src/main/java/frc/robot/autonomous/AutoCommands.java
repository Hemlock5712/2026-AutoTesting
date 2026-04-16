package frc.robot.autonomous;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveToPoint;
import frc.robot.commands.FollowPath;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.FieldInfo;
import frc.robot.utils.geometry.ExtPose;
import frc.robot.utils.path.PathData;
import frc.robot.utils.path.ProjectionResult;
import frc.robot.utils.path.RotationSupplier;
import frc.robot.utils.path.RotationSuppliers;
import frc.robot.utils.path.SplinePath;
import frc.robot.utils.path.VelocityConstraints;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Utility class containing reusable command patterns for autonomous routines.
 *
 * <p>This class provides:
 *
 * <ul>
 *   <li>Common command builders (drive, intake, score)
 *   <li>Game piece spawning for simulation
 *   <li>Vision-based game piece detection and intake
 * </ul>
 */
public class AutoCommands {

  // Subsystems
  private final CommandSwerveDrivetrain drivetrain;

  /**
   * Creates AutoCommands with PhotonVision backend.
   *
   * @param drivetrain The swerve drivetrain
   * @param intake The intake subsystem
   * @param elevator The elevator subsystem (unused but kept for compatibility)
   * @param photonGamePiece PhotonVision detector
   * @param superstructure The superstructure
   */
  public AutoCommands(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  public DriveToPoint driveTo(Supplier<Pose2d> pose) {
    return new DriveToPoint(drivetrain, pose);
  }

  // ==================== Drive Commands ====================

  /**
   * Follow a spline path with specified constraints.
   *
   * @param path The spline path to follow
   * @param constraints Velocity and acceleration limits
   * @return A FollowPath command
   */
  public FollowPath followPath(SplinePath path, VelocityConstraints constraints) {
    return new FollowPath(drivetrain, path, constraints);
  }

  /**
   * Follow a spline path with default constraints.
   *
   * @param path The spline path to follow
   * @return A FollowPath command
   */
  public FollowPath followPath(SplinePath path) {
    return new FollowPath(drivetrain, path);
  }

  /**
   * Follow a path from a PathData object (e.g., from Paths.java constants).
   *
   * <p>Automatically wires heading waypoints and constraint zones.
   *
   * @param data The path data
   * @return A FollowPath command
   */
  public FollowPath followPath(PathData data) {
    double t0 = Timer.getFPGATimestamp();
    SplinePath path = new SplinePath(data.controlPoints());
    double t1 = Timer.getFPGATimestamp();
    FollowPath cmd =
        new FollowPath(drivetrain, path, data.globalConstraints(), data.constraintZones());
    double t2 = Timer.getFPGATimestamp();

    Logger.recordOutput("PathBench/SplinePathMs", (t1 - t0) * 1000);
    Logger.recordOutput("PathBench/VelocityProfileMs", (t2 - t1) * 1000);
    Logger.recordOutput("PathBench/TotalMs", (t2 - t0) * 1000);
    Logger.recordOutput("PathBench/ControlPoints", data.controlPoints().size());
    Logger.recordOutput("PathBench/PathLengthM", path.getTotalLength());

    if (!data.headingWaypoints().isEmpty()) {
      cmd.withRotationSupplier(
          RotationSuppliers.interpolateAlongPath(path, data.headingWaypoints()));
    }

    return cmd;
  }

  /**
   * Follow a spline path with a rotation supplier.
   *
   * @param path The spline path to follow
   * @param rotation Rotation strategy to use during path following
   * @return A FollowPath command
   */
  public FollowPath followPath(SplinePath path, RotationSupplier rotation) {
    return new FollowPath(drivetrain, path).withRotationSupplier(rotation);
  }

  /**
   * Follow a spline path with constraints and a rotation supplier.
   *
   * @param path The spline path to follow
   * @param constraints Velocity and acceleration limits
   * @param rotation Rotation strategy to use during path following
   * @return A FollowPath command
   */
  public FollowPath followPath(
      SplinePath path, VelocityConstraints constraints, RotationSupplier rotation) {
    return new FollowPath(drivetrain, path, constraints).withRotationSupplier(rotation);
  }

  public Command resetPose(Supplier<Pose2d> pose) {
    return drivetrain.runOnce(() -> drivetrain.resetPose(pose.get()));
  }

  public Command resetTranslation(Supplier<Translation2d> translation) {
    return drivetrain.runOnce(
        () ->
            drivetrain.resetPose(
                new Pose2d(translation.get(), drivetrain.getPose().getRotation())));
  }

  public Translation2d getRobotTranslation() {
    return drivetrain.getPose().getTranslation();
  }

  public Command deferCommand(Supplier<Command> supplier) {
    return Commands.defer(supplier, java.util.Set.of(drivetrain));
  }

  // ==================== Path Actions ====================

  /** An action to trigger at a specific control point or waypoint flag along a path. */
  public record PathAction(
      Integer pointIndex, String flagLabel, double triggerDistance, Supplier<Command> command) {
    public PathAction(int pointIndex, double triggerDistance, Supplier<Command> command) {
      this(pointIndex, null, triggerDistance, command);
    }

    public PathAction(String flagLabel, double triggerDistance, Supplier<Command> command) {
      this(null, flagLabel, triggerDistance, command);
    }
  }

  private record ResolvedPathAction(
      int pointIndex, double triggerDistance, Supplier<Command> command, int insertionOrder) {}

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
      if (activeCommand == null) {
        return;
      }

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

  record ScheduledPathAction(double triggerS, Supplier<Command> commandSupplier) {}

  private static final double ACTION_TRIGGER_PROJECTION_WINDOW = 0.75;

  private void validatePointIndex(PathData pathData, int pointIndex) {
    if (pointIndex < 0 || pointIndex >= pathData.controlPoints().size()) {
      throw new IllegalArgumentException("Invalid waypoint index: " + pointIndex);
    }
  }

  /**
   * Follow a path with distance-triggered actions and optional alongside commands.
   *
   * <p>Each action fires when the robot is within {@code triggerDistance} of the control point at
   * {@code pointIndex}. Once an action is triggered, it stays scheduled until a later action
   * replaces it, the path completes, or the routine is interrupted. Alongside commands run for the
   * entire path duration.
   *
   * @param pathData The path to follow
   * @param actions Ordered list of actions to trigger along the path
   * @param alongside Commands that run for the entire path (e.g., intake)
   * @return A command that follows the path with all actions wired
   */
  public Command followPathWithActions(
      PathData pathData,
      List<PathAction> actions,
      double completionTolerance,
      Command... alongside) {
    return followPathWithActionsInternal(pathData, actions, completionTolerance, alongside);
  }

  public Command followPathWithActions(
      PathData pathData, List<PathAction> actions, Command... alongside) {
    return followPathWithActionsInternal(pathData, actions, -1, alongside);
  }

  private Command followPathWithActionsInternal(
      PathData pathData,
      List<PathAction> actions,
      double completionTolerance,
      Command... alongside) {
    double t0 = Timer.getFPGATimestamp();
    SplinePath path = new SplinePath(pathData.controlPoints());
    double t1 = Timer.getFPGATimestamp();
    FollowPath pathCmd =
        new FollowPath(drivetrain, path, pathData.globalConstraints(), pathData.constraintZones());
    if (completionTolerance > 0) {
      pathCmd.withCompletionTolerance(completionTolerance);
    }
    double t2 = Timer.getFPGATimestamp();

    Logger.recordOutput("PathBench/SplinePathMs", (t1 - t0) * 1000);
    Logger.recordOutput("PathBench/VelocityProfileMs", (t2 - t1) * 1000);
    Logger.recordOutput("PathBench/TotalMs", (t2 - t0) * 1000);
    Logger.recordOutput("PathBench/ControlPoints", pathData.controlPoints().size());
    Logger.recordOutput("PathBench/PathLengthM", path.getTotalLength());

    if (!pathData.headingWaypoints().isEmpty()) {
      pathCmd.withRotationSupplier(
          RotationSuppliers.interpolateAlongPath(path, pathData.headingWaypoints()));
    }

    List<ResolvedPathAction> resolvedActions = resolvePathActions(pathData, actions);

    if (resolvedActions.isEmpty()) {
      return alongside.length == 0 ? pathCmd : pathCmd.deadlineFor(alongside);
    }

    double[] projectedS = {0.0};
    ActivePathActionRunner actionRunner =
        new ActivePathActionRunner(
            buildScheduledPathActions(path, resolvedActions),
            () -> updateProjectedS(path, projectedS));

    Command[] deadlineCommands = new Command[alongside.length + 1];
    deadlineCommands[0] = actionRunner;
    System.arraycopy(alongside, 0, deadlineCommands, 1, alongside.length);

    return pathCmd.deadlineFor(deadlineCommands);
  }

  List<ScheduledPathAction> buildScheduledPathActions(
      SplinePath path, List<ResolvedPathAction> resolvedActions) {
    List<ScheduledPathAction> rawScheduledActions = new ArrayList<>(resolvedActions.size());
    for (ResolvedPathAction action : resolvedActions) {
      double triggerS =
          Math.max(
              0.0,
              path.getArcLengthAtWaypointIndex(action.pointIndex()) - action.triggerDistance());
      rawScheduledActions.add(new ScheduledPathAction(triggerS, action.command()));
    }
    return groupScheduledActions(rawScheduledActions);
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
        if (Double.compare(action.triggerS(), nextAction.triggerS()) != 0) {
          break;
        }
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
      if (suppliers.size() == 1) {
        return suppliers.get(0).get();
      }
      return Commands.parallel(suppliers.stream().map(Supplier::get).toArray(Command[]::new));
    };
  }

  private List<ResolvedPathAction> resolvePathActions(PathData pathData, List<PathAction> actions) {
    List<ResolvedPathAction> resolved = new ArrayList<>();
    int insertionOrder = 0;

    for (PathAction action : actions) {
      if (action.pointIndex() != null) {
        validatePointIndex(pathData, action.pointIndex());
        resolved.add(
            new ResolvedPathAction(
                action.pointIndex(), action.triggerDistance(), action.command(), insertionOrder++));
        continue;
      }

      List<PathData.WaypointFlag> matches =
          pathData.waypointFlags().stream()
              .filter(flag -> action.flagLabel().equals(flag.label()))
              .sorted(Comparator.comparingInt(PathData.WaypointFlag::waypointIndex))
              .toList();

      if (matches.isEmpty()) {
        continue;
      }

      for (PathData.WaypointFlag match : matches) {
        validatePointIndex(pathData, match.waypointIndex());
        resolved.add(
            new ResolvedPathAction(
                match.waypointIndex(),
                action.triggerDistance(),
                action.command(),
                insertionOrder++));
      }
    }

    resolved.sort(
        Comparator.comparingInt(ResolvedPathAction::pointIndex)
            .thenComparingInt(ResolvedPathAction::insertionOrder));
    return resolved;
  }

  private double updateProjectedS(SplinePath path, double[] projectedS) {
    double lastProjectedS = projectedS[0];
    ProjectionResult projection =
        path.getClosestPointInRange(
            drivetrain.getPose().getTranslation(),
            Math.max(0.0, lastProjectedS - ACTION_TRIGGER_PROJECTION_WINDOW),
            Math.min(path.getTotalLength(), lastProjectedS + ACTION_TRIGGER_PROJECTION_WINDOW));
    projectedS[0] = Math.max(lastProjectedS, projection.s());
    return projectedS[0];
  }

  // ==================== Time-Triggered Actions ====================

  /**
   * Returns a command that waits until the robot passes a given X position, then runs a command.
   * The threshold is specified in blue-alliance coordinates and is automatically flipped for red
   * alliance using FieldInfo.flipX().
   *
   * @param blueAllianceX X position threshold in blue-alliance coordinates
   * @param commandToRun Command to run once the robot passes the threshold
   * @return Command that triggers based on robot X position
   */
  public Command runWhenPastX(double blueAllianceX, Command commandToRun) {
    return Commands.sequence(
        Commands.waitUntil(() -> FieldInfo.flipX(drivetrain.getPose().getX()) > blueAllianceX),
        commandToRun);
  }

  public Command leftAutoSetup() {
    return resetPose(
        () ->
            new ExtPose(4.378, FieldInfo.width().in(Meters) - 0.639445, Rotation2d.fromDegrees(-90))
                .get());
  }
}
