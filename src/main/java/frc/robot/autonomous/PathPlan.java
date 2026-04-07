package frc.robot.autonomous;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AccelerationLimiter;
import frc.robot.commands.DriveToPoint;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.FieldInfo;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

/**
 * Fluent builder for waypoint-chain paths with automatic lookahead speed planning.
 *
 * <p>Computes corner speeds from turn geometry, then runs backward and forward kinematic passes to
 * ensure the robot can decelerate and accelerate within physical limits. Works for both autonomous
 * (standing start) and teleop (moving start with live velocity awareness).
 *
 * <p>Accepts {@code Pose2d} or {@code Supplier<Pose2d>} (including {@code ExtPose}) for all pose
 * parameters. Suppliers are evaluated at schedule time inside {@code Commands.defer()}, so alliance
 * flipping via {@code ExtPose} is automatic.
 *
 * <p>Example:
 *
 * <pre>{@code
 * PathPlan.create(drivetrain)
 *     .from(startPose)
 *     .through(midPose1).withMaxSpeed(3.0)
 *     .through(midPose2).withCommand(intake::deployAndRun)
 *     .to(endPose)
 *     .withGlobalMaxSpeed(5.0)
 *     .onPassingX(6.0, coordinator::deployIntake)
 *     .build()
 * }</pre>
 */
public class PathPlan {

  private static final double EPSILON = 1e-6;
  private static final double STATIONARY_SPEED_THRESHOLD = 0.1; // m/s
  private static final double DEFAULT_WAYPOINT_TOLERANCE = 0.25; // meters
  private static final double DEFAULT_FINAL_TOLERANCE = 0.02; // meters

  // Safety margin on the physics-based corner speed calculation. Accounts for control latency,
  // jerk limits, and the fact that the robot can't instantly apply full lateral acceleration.
  private static final double CORNER_SPEED_SAFETY = 0.8;

  private final CommandSwerveDrivetrain drivetrain;
  private Supplier<Pose2d> fromPose;
  private final List<WaypointConfig> waypoints = new ArrayList<>();
  private final List<XTrigger> xTriggers = new ArrayList<>();
  private double globalMaxSpeed = Double.POSITIVE_INFINITY;

  private static class WaypointConfig {
    final Supplier<Pose2d> pose;
    double maxSpeed = Double.POSITIVE_INFINITY;
    double positionTolerance = DEFAULT_WAYPOINT_TOLERANCE;

    /** Fresh commands each time {@link #buildPath} runs (avoids composed-command reuse). */
    final List<Supplier<Command>> commandSuppliers = new ArrayList<>();

    boolean isFinal;

    WaypointConfig(Supplier<Pose2d> pose, boolean isFinal) {
      this.pose = pose;
      this.isFinal = isFinal;
      this.positionTolerance = isFinal ? DEFAULT_FINAL_TOLERANCE : DEFAULT_WAYPOINT_TOLERANCE;
    }
  }

  private static class XTrigger {
    final double blueAllianceX;
    final Supplier<Command> command;

    XTrigger(double blueAllianceX, Supplier<Command> command) {
      this.blueAllianceX = blueAllianceX;
      this.command = command;
    }
  }

  private PathPlan(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  /**
   * Creates a new path plan builder.
   *
   * @param drivetrain The swerve drivetrain
   * @return A new PathPlan builder
   */
  public static PathPlan create(CommandSwerveDrivetrain drivetrain) {
    return new PathPlan(drivetrain);
  }

  // ==================== Starting Pose ====================

  /**
   * Sets the starting pose for geometry calculations (turn angles, distances). Optional — when
   * omitted, the robot's live pose and velocity are used at schedule time.
   *
   * @param pose Starting pose (blue-alliance origin)
   * @return This builder for chaining
   */
  public PathPlan from(Pose2d pose) {
    return from(() -> pose);
  }

  /**
   * @see #from(Pose2d)
   */
  public PathPlan from(Supplier<Pose2d> pose) {
    this.fromPose = pose;
    return this;
  }

  // ==================== Waypoints ====================

  /**
   * Adds an intermediate waypoint. The robot passes through this point without stopping. Use
   * chained methods like {@link #withMaxSpeed} or {@link #withCommand} to configure this waypoint.
   *
   * @param pose Waypoint pose (blue-alliance origin)
   * @return This builder for chaining
   */
  public PathPlan through(Pose2d pose) {
    return through(() -> pose);
  }

  /**
   * @see #through(Pose2d)
   */
  public PathPlan through(Supplier<Pose2d> pose) {
    waypoints.add(new WaypointConfig(pose, false));
    return this;
  }

  /**
   * Sets the final destination. The robot stops here with tight position and heading tolerances.
   *
   * @param pose Destination pose (blue-alliance origin)
   * @return This builder for chaining
   */
  public PathPlan to(Pose2d pose) {
    return to(() -> pose);
  }

  /**
   * @see #to(Pose2d)
   */
  public PathPlan to(Supplier<Pose2d> pose) {
    waypoints.add(new WaypointConfig(pose, true));
    return this;
  }

  // ==================== Per-Waypoint Options ====================
  // These apply to the most recently added waypoint (via through() or to()).

  /**
   * Sets a maximum speed for the segment driving TO the most recent waypoint. Also caps the
   * lookahead-computed corner speed at this waypoint.
   *
   * @param maxSpeed Maximum linear speed in m/s
   * @return This builder for chaining
   */
  public PathPlan withMaxSpeed(double maxSpeed) {
    lastWaypoint().maxSpeed = maxSpeed;
    return this;
  }

  /**
   * Attaches a command to run alongside the segment driving TO the most recent waypoint. The
   * command is interrupted when the robot reaches the waypoint. Multiple commands can be attached
   * by calling this method repeatedly.
   *
   * @param command Command to run in parallel with this segment (invoked once per path build — use
   *     a method reference or lambda that returns a new command, e.g. {@code
   *     superstructure::hubShoot})
   * @return This builder for chaining
   */
  public PathPlan withCommand(Supplier<Command> command) {
    lastWaypoint().commandSuppliers.add(command);
    return this;
  }

  /**
   * Overrides the position tolerance for the most recent waypoint.
   *
   * @param tolerance Position tolerance in meters (default 0.25 for waypoints, 0.02 for final)
   * @return This builder for chaining
   */
  public PathPlan withPositionTolerance(double tolerance) {
    lastWaypoint().positionTolerance = tolerance;
    return this;
  }

  /**
   * @see #withPositionTolerance(double)
   */
  public PathPlan withPositionTolerance(Distance tolerance) {
    return withPositionTolerance(tolerance.in(Meters));
  }

  // ==================== Path-Level Options ====================

  /**
   * Sets a global maximum speed for all segments.
   *
   * @param maxSpeed Maximum linear speed in m/s
   * @return This builder for chaining
   */
  public PathPlan withGlobalMaxSpeed(double maxSpeed) {
    this.globalMaxSpeed = maxSpeed;
    return this;
  }

  /**
   * Triggers a command when the robot passes a given X coordinate. The threshold is in
   * blue-alliance coordinates and is automatically flipped for red alliance. Runs in parallel with
   * the entire path; triggers once.
   *
   * @param blueAllianceX X position threshold (blue-alliance coordinates)
   * @param command Command to run once the robot passes the threshold (invoked once per path build;
   *     prefer a method reference)
   * @return This builder for chaining
   */
  public PathPlan onPassingX(double blueAllianceX, Supplier<Command> command) {
    xTriggers.add(new XTrigger(blueAllianceX, command));
    return this;
  }

  // ==================== Build ====================

  /**
   * Builds the path into a Command. Pose suppliers are evaluated at schedule time (inside {@code
   * Commands.defer}), so alliance flipping via {@code ExtPose} works automatically.
   *
   * <p>The returned command requires the drivetrain subsystem.
   *
   * @return A deferred command that computes lookahead speeds and executes the path
   */
  public Command build() {
    if (waypoints.isEmpty()) {
      return Commands.none();
    }

    final List<WaypointConfig> capturedWaypoints = new ArrayList<>(waypoints);
    final List<XTrigger> capturedTriggers = new ArrayList<>(xTriggers);
    final Supplier<Pose2d> capturedFrom = fromPose;
    final double capturedGlobalMax = globalMaxSpeed;

    return Commands.defer(
        () -> buildPath(capturedWaypoints, capturedTriggers, capturedFrom, capturedGlobalMax),
        Set.of(drivetrain));
  }

  // ==================== Internal ====================

  private Command buildPath(
      List<WaypointConfig> wpConfigs,
      List<XTrigger> triggers,
      Supplier<Pose2d> from,
      double globalMax) {

    int n = wpConfigs.size();
    double maxDecel = AccelerationLimiter.MAX_FRICTION_ACCEL;
    double speedCeiling = Math.min(globalMax, AccelerationLimiter.MAX_VELOCITY);

    // --- 1. Resolve all poses ---
    Pose2d[] poses = new Pose2d[n];
    for (int i = 0; i < n; i++) {
      poses[i] = wpConfigs.get(i).pose.get();
    }

    // --- 2. Determine starting state ---
    Pose2d startPos;
    double startSpeed;

    if (from != null) {
      startPos = from.get();
      startSpeed = 0;
    } else {
      startPos = drivetrain.getPose();
      ChassisSpeeds fieldSpeeds = drivetrain.getFieldSpeeds();
      double vx = fieldSpeeds.vxMetersPerSecond;
      double vy = fieldSpeeds.vyMetersPerSecond;
      double speed = Math.hypot(vx, vy);

      if (speed > STATIONARY_SPEED_THRESHOLD) {
        Translation2d toFirst = poses[0].getTranslation().minus(startPos.getTranslation());
        if (toFirst.getNorm() > EPSILON) {
          double dot = vx * toFirst.getX() + vy * toFirst.getY();
          startSpeed = Math.max(0, dot / toFirst.getNorm());
        } else {
          startSpeed = speed;
        }
      } else {
        startSpeed = 0;
      }
    }

    // --- 3. Compute segment directions and distances ---
    double[] segDist = new double[n];
    Translation2d[] segDir = new Translation2d[n];

    Translation2d prevPoint = startPos.getTranslation();
    for (int i = 0; i < n; i++) {
      Translation2d seg = poses[i].getTranslation().minus(prevPoint);
      segDist[i] = seg.getNorm();
      segDir[i] =
          segDist[i] > EPSILON
              ? seg.div(segDist[i])
              : (i > 0 ? segDir[i - 1] : new Translation2d(1, 0));
      prevPoint = poses[i].getTranslation();
    }

    // --- 4. Compute corner speeds from turn angles ---
    double[] wpSpeeds = new double[n];

    for (int i = 0; i < n; i++) {
      WaypointConfig wp = wpConfigs.get(i);

      if (wp.isFinal) {
        wpSpeeds[i] = 0;
        continue;
      }

      if (i + 1 < n) {
        double dot =
            segDir[i].getX() * segDir[i + 1].getX() + segDir[i].getY() * segDir[i + 1].getY();
        double turnAngle = Math.acos(Math.max(-1, Math.min(1, dot)));
        double sinHalf = Math.sin(turnAngle / 2.0);

        // Physics-based corner speed: max v where the robot can redirect within the
        // waypoint tolerance region. Derived from |Δv| = 2v·sin(θ/2) ≤ a·tolerance/v.
        double cornerSpeed;
        if (sinHalf < EPSILON) {
          cornerSpeed = speedCeiling;
        } else {
          cornerSpeed =
              CORNER_SPEED_SAFETY * Math.sqrt(maxDecel * wp.positionTolerance / (2.0 * sinHalf));
        }

        wpSpeeds[i] = Math.min(cornerSpeed, Math.min(wp.maxSpeed, speedCeiling));
      } else {
        wpSpeeds[i] = Math.min(wp.maxSpeed, speedCeiling);
      }
    }

    // --- 5. Backward pass: deceleration limits ---
    for (int i = n - 2; i >= 0; i--) {
      double reachable =
          Math.sqrt(wpSpeeds[i + 1] * wpSpeeds[i + 1] + 2 * maxDecel * segDist[i + 1]);
      wpSpeeds[i] = Math.min(wpSpeeds[i], reachable);
    }

    // --- 6. Forward pass: acceleration limits ---
    double prevSpeed = startSpeed;
    for (int i = 0; i < n; i++) {
      double reachable = Math.sqrt(prevSpeed * prevSpeed + 2 * maxDecel * segDist[i]);
      wpSpeeds[i] = Math.min(wpSpeeds[i], reachable);
      prevSpeed = wpSpeeds[i];
    }

    // --- 7. Build DriveToPoint chain ---
    Command chain = null;

    for (int i = 0; i < n; i++) {
      WaypointConfig wp = wpConfigs.get(i);
      final Pose2d target = poses[i];

      DriveToPoint dtp = new DriveToPoint(drivetrain, () -> target);

      double segMaxSpeed = Math.min(speedCeiling, wp.maxSpeed);
      dtp.withMaxSpeed(segMaxSpeed);

      if (!wp.isFinal) {
        dtp.withWaypoint(wpSpeeds[i]);
      }
      dtp.withPositionTolerance(wp.positionTolerance);

      Command segment = dtp;
      if (!wp.commandSuppliers.isEmpty()) {
        Command[] parallel =
            wp.commandSuppliers.stream().map(Supplier::get).toArray(Command[]::new);
        segment = dtp.deadlineFor(parallel);
      }

      chain = (chain == null) ? segment : chain.andThen(segment);
    }

    if (chain == null) {
      return Commands.none();
    }

    // --- 8. Attach X-position triggers ---
    if (!triggers.isEmpty()) {
      Command[] triggerCmds = new Command[triggers.size()];
      for (int i = 0; i < triggers.size(); i++) {
        XTrigger t = triggers.get(i);
        triggerCmds[i] =
            Commands.sequence(
                Commands.waitUntil(
                    () -> FieldInfo.flipX(drivetrain.getPose().getX()) > t.blueAllianceX),
                t.command.get());
      }
      chain = chain.deadlineFor(triggerCmds);
    }

    return chain;
  }

  private WaypointConfig lastWaypoint() {
    if (waypoints.isEmpty()) {
      throw new IllegalStateException("No waypoint to configure — call through() or to() first.");
    }
    return waypoints.get(waypoints.size() - 1);
  }
}
