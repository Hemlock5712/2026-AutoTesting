package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPCrossTrackHolonomicController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DrivePhysics;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.WeakHashMap;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;

/**
 * Production entry point for PathPlanner-based autonomous paths and on-the-fly pathfinding. On
 * first {@link #configure} it wires {@link AutoBuilder#configureDistanceBased} to {@link
 * Drive#runVelocity} and pushes the static obstacle field into PathPlanner's pathfinder. Sharing
 * one {@link RobotConfig} with the runtime {@code SwerveSetpointGenerator} ({@link
 * DrivePhysics#buildRobotConfig()}) means plan-time and runtime can't disagree on what the wheels
 * can do.
 *
 * <p>Three factories:
 *
 * <ul>
 *   <li>{@link #followPath(Drive, String)} — load a single {@code .path} file from {@code
 *       src/main/deploy/pathplanner/paths/} as a {@link PathPlannerPath} and follow it.
 *   <li>{@link #runAuto(Drive, String)} — run a full {@code .auto} file (sequenced paths +
 *       named-command actions) from {@code src/main/deploy/pathplanner/autos/}.
 *   <li>{@link #pathfindToPose(Drive, Pose2d)} — on-the-fly plan from the current pose to an
 *       arbitrary goal, avoiding the static obstacle field set in {@link #configure}.
 * </ul>
 *
 * <p>Per-tick diagnostics land under the {@code PathPlanner/} log namespace. Path completion status
 * is published as {@code PathPlanner/LastResult} ({@code "finished" | "interrupted"}).
 */
public final class PathPlannerAutos {

  /**
   * Default constraints for {@link #pathfindToPose} — robot max speed and friction-limited
   * accel/decel, angular limits derived from drive-base radius.
   */
  public static final PathConstraints DEFAULT_PATHFIND_CONSTRAINTS = defaultPathfindConstraints();

  private static PathConstraints defaultPathfindConstraints() {
    double maxV = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    return new PathConstraints(
        maxV,
        DrivePhysics.MAX_FRICTION_ACCEL,
        maxV / DrivePhysics.DRIVE_BASE_RADIUS,
        DrivePhysics.MAX_FRICTION_ACCEL / DrivePhysics.DRIVE_BASE_RADIUS);
  }

  /** How close to the goal counts as arrived for the pathfind-to-pose finishing test (meters). */
  private static final double PATHFIND_COMPLETION_TOL_M = 0.10;

  private static boolean configured = false;

  /** RobotConfig built at configure time; reused by the pathfinder command builder. */
  private static RobotConfig configuredRobotConfig = null;

  /**
   * Shared path-following controller. {@link PathfindingCommand} accepts any {@link
   * PathFollowingController}, so on-the-fly pathfinding uses the same cross-track controller (and
   * gains) as auto path-following — only the trajectory-sampling strategy differs (distance for
   * authored paths, time for pathfinder output).
   */
  private static PathFollowingController configuredController = null;

  /** Cache loaded paths so repeated calls don't re-parse the JSON. */
  private static final Map<String, PathPlannerPath> PATH_CACHE = new WeakHashMap<>();

  private PathPlannerAutos() {}

  /**
   * Configure PathPlanner once per JVM: AutoBuilder for path/auto following, plus push the static
   * obstacle field into the on-the-fly pathfinder. Idempotent (guarded by a local flag and {@link
   * AutoBuilder#isConfigured()}); safe to call from multiple call sites. Must be called before
   * {@link #followPath}, {@link #runAuto}, or {@link #pathfindToPose}.
   *
   * @param drive Drive subsystem (provides pose/speeds, consumes ChassisSpeeds)
   * @param obstacles static field obstacles (min/max AABB pairs) to push into PathPlanner's
   *     pathfinder. Pass {@code null} to skip pathfinder setup (path/auto following still works).
   */
  public static synchronized void configure(
      Drive drive, List<Pair<Translation2d, Translation2d>> obstacles) {
    if (configured || AutoBuilder.isConfigured()) {
      configured = true;
      return;
    }
    // Route PathPlanner's internal logging hooks to AdvantageKit so target poses and active path
    // samples land in the same namespace as Drive's logs.
    PathPlannerLogging.setLogCurrentPoseCallback(
        pose -> Logger.recordOutput("PathPlanner/CurrentPose", pose));
    PathPlannerLogging.setLogTargetPoseCallback(
        pose -> Logger.recordOutput("PathPlanner/TargetPose", pose));
    PathPlannerLogging.setLogActivePathCallback(
        poses -> Logger.recordOutput("PathPlanner/ActivePath", poses.toArray(new Pose2d[0])));

    RobotConfig config = DrivePhysics.buildRobotConfig();
    // Wrap the controller so we can log raw target-state fields (heading, fieldSpeeds,
    // linearVelocity, curvature) at every tick. PP doesn't expose these via PathPlannerLogging.
    // Current values track within ~50–100 mm GT-vs-target on the U-path and ~25 mm on a clean
    // straight.
    PPCrossTrackHolonomicController inner =
        new PPCrossTrackHolonomicController(
            new PIDConstants(5.0, 0.0, 1.0), // cross-track PD
            3.0, // along-track P
            new PIDConstants(5.0, 0.0, 0.0), // heading
            0.1); // curvature FF gain
    PathFollowingController loggingController =
        new PathFollowingController() {
          @Override
          public ChassisSpeeds calculateRobotRelativeSpeeds(
              Pose2d currentPose, PathPlannerTrajectoryState targetState) {
            Logger.recordOutput("PathPlanner/Diag/TargetHeadingRad", targetState.heading.getRadians());
            Logger.recordOutput("PathPlanner/Diag/TargetLinearVel", targetState.linearVelocity);
            Logger.recordOutput("PathPlanner/Diag/TargetCurvature", targetState.curvatureRadPerMeter);
            Logger.recordOutput(
                "PathPlanner/Diag/TargetFieldVx", targetState.fieldSpeeds.vxMetersPerSecond);
            Logger.recordOutput(
                "PathPlanner/Diag/TargetFieldVy", targetState.fieldSpeeds.vyMetersPerSecond);
            Logger.recordOutput(
                "PathPlanner/Diag/TargetFieldOmega",
                targetState.fieldSpeeds.omegaRadiansPerSecond);
            return inner.calculateRobotRelativeSpeeds(currentPose, targetState);
          }

          @Override
          public void reset(Pose2d currentPose, ChassisSpeeds currentSpeeds) {
            inner.reset(currentPose, currentSpeeds);
          }

          @Override
          public boolean isHolonomic() {
            return inner.isHolonomic();
          }
        };

    // PP-planned trajectories flow through Drive's SwerveSetpointGenerator (same RobotConfig as
    // the planner), so runtime and plan share one friction/torque model.
    AutoBuilder.configureDistanceBased(
        drive::getPose,
        drive::resetPose,
        drive::getRobotSpeeds,
        (speeds, feedforwards) -> drive.runVelocity(speeds),
        loggingController,
        config,
        () -> false, // alliance flipping handled by caller / PathPlanner path metadata
        drive);

    configuredController = loggingController;
    configuredRobotConfig = config;
    if (obstacles != null) {
      pushObstaclesToPathfinder(obstacles, drive.getPose().getTranslation());
    } else {
      // Still init the pathfinder so a later call to pathfindToPose works against an empty field.
      Pathfinding.ensureInitialized();
    }
    configured = true;
  }

  /**
   * Return a Command that follows the named PathPlanner-native path. The file must live at
   * {@code src/main/deploy/pathplanner/paths/<name>.path}. On load failure, returns a no-op
   * command that prints the error so the robot stays in a safe state.
   *
   * @param drive Drive subsystem
   * @param pathName path file name without extension
   */
  public static Command followPath(Drive drive, String pathName) {
    requireConfigured();
    PathPlannerPath path = PATH_CACHE.get(pathName);
    if (path == null) {
      try {
        path = PathPlannerPath.fromPathFile(pathName);
      } catch (IOException | ParseException | RuntimeException e) {
        return Commands.print("[PathPlannerAutos] Failed to load path \"" + pathName + "\": " + e)
            .ignoringDisable(true);
      }
      PATH_CACHE.put(pathName, path);
    }
    return AutoBuilder.followPath(path)
        .beforeStarting(() -> Logger.recordOutput("PathPlanner/PathName", pathName))
        .finallyDo(
            (interrupted) ->
                Logger.recordOutput(
                    "PathPlanner/LastResult", interrupted ? "interrupted" : "finished"));
  }

  /**
   * Return a Command that runs the named PathPlanner auto file ({@code .auto} under {@code
   * src/main/deploy/pathplanner/autos/}). Handles odometry reset, named commands, and sequencing.
   *
   * @param drive Drive subsystem
   * @param autoName auto file name without extension
   */
  public static Command runAuto(Drive drive, String autoName) {
    requireConfigured();
    Command auto;
    try {
      auto = new PathPlannerAuto(autoName);
    } catch (RuntimeException e) {
      return Commands.print("[PathPlannerAutos] Failed to load auto \"" + autoName + "\": " + e)
          .ignoringDisable(true);
    }
    return auto.beforeStarting(() -> Logger.recordOutput("PathPlanner/AutoName", autoName))
        .finallyDo(
            (interrupted) ->
                Logger.recordOutput(
                    "PathPlanner/LastResult", interrupted ? "interrupted" : "finished"));
  }

  /**
   * On-the-fly plan from the robot's current pose to {@code goal}, avoiding the static obstacles
   * registered in {@link #configure}. Uses {@link #DEFAULT_PATHFIND_CONSTRAINTS}.
   *
   * @param drive Drive subsystem
   * @param goal target pose
   */
  public static Command pathfindToPose(Drive drive, Pose2d goal) {
    return pathfindToPose(drive, goal, DEFAULT_PATHFIND_CONSTRAINTS);
  }

  /**
   * On-the-fly plan from the robot's current pose to {@code goal} with caller-supplied
   * constraints. Constructed directly from {@link PathfindingCommand} because {@link
   * AutoBuilder#configureDistanceBased} opts out of registering a pathfinding command builder;
   * reuses the controller built in {@link #configure} so feedback gains match auto path-following.
   */
  public static Command pathfindToPose(Drive drive, Pose2d goal, PathConstraints constraints) {
    requireConfigured();
    return new PathfindingCommand(
            goal,
            constraints,
            0.0,
            drive::getPose,
            drive::getRobotSpeeds,
            (speeds, feedforwards) -> drive.runVelocity(speeds),
            configuredController,
            configuredRobotConfig,
            drive)
        .beforeStarting(() -> Logger.recordOutput("PathPlanner/PathfindGoal", goal))
        .finallyDo(
            (interrupted) ->
                Logger.recordOutput(
                    "PathPlanner/LastResult", interrupted ? "interrupted" : "finished"));
  }

  /** Tolerance used for finishing the pathfind-to-pose command (meters). */
  public static double pathfindCompletionTolerance() {
    return PATHFIND_COMPLETION_TOL_M;
  }

  private static void requireConfigured() {
    if (!configured && !AutoBuilder.isConfigured()) {
      throw new IllegalStateException(
          "PathPlannerAutos.configure(drive, obstacles) must be called before any path/auto/pathfind"
              + " command is built.");
    }
  }

  /**
   * Inflate each obstacle AABB by {@code (ROBOT_HALF_X + margin, ROBOT_HALF_Y + margin)} and push
   * the result into PathPlanner's pathfinder. The pathfinder treats the robot as a point, so the
   * inflation must cover the full robot half-extent.
   */
  private static void pushObstaclesToPathfinder(
      List<Pair<Translation2d, Translation2d>> obstacles, Translation2d robotPos) {
    Pathfinding.ensureInitialized();
    double inflateX = Drive.ROBOT_HALF_X + DrivePhysics.PATH_INFLATION_MARGIN_M;
    double inflateY = Drive.ROBOT_HALF_Y + DrivePhysics.PATH_INFLATION_MARGIN_M;
    List<Pair<Translation2d, Translation2d>> inflated = new ArrayList<>(obstacles.size());
    for (Pair<Translation2d, Translation2d> box : obstacles) {
      Translation2d min = box.getFirst();
      Translation2d max = box.getSecond();
      inflated.add(
          Pair.of(
              new Translation2d(min.getX() - inflateX, min.getY() - inflateY),
              new Translation2d(max.getX() + inflateX, max.getY() + inflateY)));
    }
    Pathfinding.setDynamicObstacles(inflated, robotPos);
    Logger.recordOutput("PathPlanner/StaticObstacleCount", inflated.size());
  }
}
