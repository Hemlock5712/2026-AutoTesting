package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.FollowPath;
import frc.robot.utils.path.PathData;
import frc.robot.utils.path.Paths.AlliancePath;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

/**
 * Factory for building autonomous path commands from blue-alliance {@link PathData} constants.
 *
 * <p>Use this from auto routines when a path should be selected for the current alliance without
 * hand-writing {@link AlliancePath} fields or {@link Commands#either(Command, Command,
 * BooleanSupplier)} blocks at each call site.
 */
public class AutoRoutineFactory {
  /** Builds a command for one concrete alliance-specific path variant. */
  @FunctionalInterface
  interface PathCommandFactory {
    Command build(PathData path, Consumer<FollowPath> configure);
  }

  /** Builds a command for one concrete alliance-specific path variant with path actions. */
  @FunctionalInterface
  interface PathWithActionsCommandFactory {
    Command build(PathData path, List<AutoCommands.PathAction> actions);
  }

  private final PathCommandFactory pathCommandFactory;
  private final PathWithActionsCommandFactory pathWithActionsCommandFactory;
  private final Function<Supplier<Pose2d>, Command> resetPoseCommandFactory;
  private final BooleanSupplier isRedAlliance;
  private final Map<PathData, AlliancePath> paths = new IdentityHashMap<>();

  /**
   * Creates an auto routine factory backed by the robot's standard autonomous command builders.
   *
   * @param autoCommands the command helper used to build path-following and pose-reset commands
   */
  public AutoRoutineFactory(AutoCommands autoCommands) {
    this(
        (path, configure) -> {
          // Build a fresh FollowPath for this specific alliance variant, then let the routine's
          // small configuration lambda tune that command. This keeps routine call sites concise
          // while still preserving the existing FollowPath builder API.
          FollowPath command = autoCommands.followPath(path);
          configure.accept(command);
          return command;
        },
        autoCommands::followPathWithActions,
        autoCommands::resetPose,
        AutoRoutineFactory::isDriverStationRedAlliance);
  }

  /**
   * Creates a factory with injectable command builders.
   *
   * @param pathCommandFactory builds a command for an already-selected path variant
   * @param pathWithActionsCommandFactory builds a command with actions for an already-selected path
   *     variant
   * @param resetPoseCommandFactory builds a command that resets odometry to a supplied pose
   * @param isRedAlliance returns whether red-alliance path variants should be selected
   */
  AutoRoutineFactory(
      PathCommandFactory pathCommandFactory,
      PathWithActionsCommandFactory pathWithActionsCommandFactory,
      Function<Supplier<Pose2d>, Command> resetPoseCommandFactory,
      BooleanSupplier isRedAlliance) {
    this.pathCommandFactory = pathCommandFactory;
    this.pathWithActionsCommandFactory = pathWithActionsCommandFactory;
    this.resetPoseCommandFactory = resetPoseCommandFactory;
    this.isRedAlliance = isRedAlliance;
  }

  /**
   * Builds an alliance-aware command that follows a path with default settings.
   *
   * @param path the blue-alliance path constant to follow
   * @return a command that follows the red or blue variant when scheduled
   */
  public Command path(PathData path) {
    return path(path, ignored -> {});
  }

  /**
   * Builds an alliance-aware command that follows a path with extra command configuration.
   *
   * @param path the blue-alliance path constant to follow
   * @param configure configuration applied to both red and blue {@link FollowPath} commands
   * @return a command that follows the red or blue variant when scheduled
   */
  public Command path(PathData path, Consumer<FollowPath> configure) {
    AlliancePath alliancePath = alliancePath(path);

    // Construct both branches now, while the selected auto is being built in disabled. WPILib's
    // EitherCommand does not choose between them until the command is initialized, so alliance
    // selection still happens at run time rather than robot startup or chooser registration time.
    Command redCommand = pathCommandFactory.build(alliancePath.red(), configure);
    Command blueCommand = pathCommandFactory.build(alliancePath.blue(), configure);
    return Commands.either(redCommand, blueCommand, isRedAlliance);
  }

  /**
   * Builds an alliance-aware command that follows a path with existing path actions.
   *
   * @param path the blue-alliance path constant to follow
   * @param actions actions to trigger along the selected path
   * @return a command that follows the red or blue variant when scheduled
   */
  public Command pathWithActions(PathData path, List<AutoCommands.PathAction> actions) {
    AlliancePath alliancePath = alliancePath(path);

    // This intentionally preserves the existing AutoCommands.PathAction model. The factory only
    // removes the repeated path-variant selection; it does not reinterpret flags, indexes, or
    // command lifetime rules.
    //
    // Commands that should run alongside the path should be composed around this returned command
    // with normal WPILib helpers like deadlineFor(...). Accepting Command varargs here would tempt
    // callers to pass the same command instances into both red and blue branches.
    Command redCommand = pathWithActionsCommandFactory.build(alliancePath.red(), actions);
    Command blueCommand = pathWithActionsCommandFactory.build(alliancePath.blue(), actions);
    return Commands.either(redCommand, blueCommand, isRedAlliance);
  }

  /**
   * Builds an alliance-aware command that resets odometry to a path's starting pose.
   *
   * @param path the blue-alliance path constant whose start pose should be used
   * @return a command that resets to the red or blue start pose when scheduled
   */
  public Command resetPoseToStart(PathData path) {
    AlliancePath alliancePath = alliancePath(path);

    // The pose supplier intentionally closes over the cached AlliancePath instead of resolving a
    // pose immediately. That keeps the reset command correct if the auto command is built before
    // the Driver Station has reported the final alliance.
    return resetPoseCommandFactory.apply(() -> selectedPath(alliancePath).getStartingPose());
  }

  /**
   * Returns the cached red/blue variants for a blue-alliance path.
   *
   * @param path the blue-alliance path constant
   * @return cached blue and red variants for the path
   */
  AlliancePath alliancePath(PathData path) {
    return paths.computeIfAbsent(
        path,
        bluePath -> {
          // PathData constants are reused by identity from Paths.java, so an IdentityHashMap avoids
          // depending on record structural equality and prevents two different but equal-looking
          // generated paths from sharing a cache entry by accident.
          AlliancePath alliancePath = AlliancePath.of(bluePath);

          // Precompute only when a routine actually uses this path through the factory. Because
          // RobotContainer lazily builds the selected auto supplier, this avoids warming every path
          // declared in AutoRoutines while still preventing first-follow delays for selected paths.
          alliancePath.blue().precompute();
          alliancePath.red().precompute();
          return alliancePath;
        });
  }

  private PathData selectedPath(AlliancePath path) {
    return isRedAlliance.getAsBoolean() ? path.red() : path.blue();
  }

  /** Returns whether the Driver Station currently reports the red alliance. */
  private static boolean isDriverStationRedAlliance() {
    // Unknown alliance falls back to blue. This mirrors the existing auto behavior and keeps local
    // simulation usable, while still re-reading DriverStation when the command is initialized.
    return DriverStation.getAlliance().map(a -> a == DriverStation.Alliance.Red).orElse(false);
  }
}
