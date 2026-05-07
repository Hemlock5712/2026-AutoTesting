package frc.robot.commands;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.path.FollowablePath;
import frc.robot.utils.path.ProjectionResult;
import frc.robot.utils.path.RotationSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Distance-based path following command for swerve drive.
 *
 * <p>Follows any {@link FollowablePath} using arc-length parameterization with adaptive lookahead
 * and cross-track PD correction. The path parameter tracks the robot's actual position, not a clock
 * — if the robot gets hit or stalls, the path "waits" for the robot.
 *
 * <p>All output is fed through {@link AccelerationLimiter#integrateVelocity} to enforce friction
 * circle, motor torque, and jerk limits.
 */
public class FollowPath extends Command {

  private final CommandSwerveDrivetrain swerve;
  private final FollowablePath path;
  private final double endVelocity;

  // Rotation supplier: null = use path heading, then fall back to hold current heading.
  private RotationSupplier rotationSupplier;

  // Rotation tolerance for isFinished() (radians). Default = don't check heading.
  private double rotationTolerance = Double.POSITIVE_INFINITY;
  private double lastHeadingError;

  // Maximum fraction of friction budget that rotation can consume (0 to 1)
  private double maxRotationBudgetFraction = 0.30;

  // Lookahead parameters: lookahead = k * speed + min, clamped to [min, max]
  private double lookaheadK = 0.15; // seconds
  private double lookaheadMin = 0.15; // meters
  private double lookaheadMax = 1.0; // meters
  private double lookaheadMaxArcAngle = Math.PI / 6; // ~30 degrees

  // Cross-track PD gains + curvature feedforward
  private double crossTrackKp = 3.0; // m/s per meter error
  private double crossTrackKd = 0.5; // m/s per m/s error rate
  private double curvatureFfGain = 0.1; // seconds — converts v²κ (m/s²) to velocity (m/s)

  // Completion criteria
  private double completionTolerance = 0.05; // meters from path end
  private double completionVelocityTolerance = 0.1; // meters per second

  /**
   * Maximum arc-length the projection can move per cycle (meters). Derived from physics: at max FRC
   * speed (5 m/s) with a 20ms loop, the robot moves 0.1m per cycle. 0.5m gives 5x safety margin,
   * handling loop overruns up to 100ms at max speed. This window (1.0m total) is too small to span
   * both segments at a path crossing (minimum separation ~1.6m for a 0.5m-radius turn).
   */
  private static final double PROJECTION_MAX_DELTA = 0.5;

  /** Number of poses to sample for the logged path trajectory. */
  private static final int PATH_LOG_SAMPLES = 50;

  // State tracking between execute cycles
  private ChassisSpeeds lastCommandedVelocity = new ChassisSpeeds();
  private double lastTime;
  private double lastCrossTrackError;
  private double lastProjectedS;
  private boolean referencePathLogged;

  // Cached logging arrays to avoid per-cycle allocations
  private final double[] logEditorTarget = new double[2];
  private final double[] logEditorClosest = new double[2];

  private final SwerveRequest.ApplyFieldSpeeds request =
      new SwerveRequest.ApplyFieldSpeeds()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  /**
   * Creates a FollowPath command from any {@link FollowablePath}.
   *
   * <p>Velocity and heading come from the path itself. For Choreo trajectories, these are the
   * time-optimal values re-parameterized by arc-length.
   *
   * @param swerve The swerve drivetrain
   * @param path The path to follow
   */
  public FollowPath(CommandSwerveDrivetrain swerve, FollowablePath path) {
    this(swerve, path, 0.0);
  }

  /**
   * Creates a FollowPath command with a specified end velocity.
   *
   * @param swerve The swerve drivetrain
   * @param path The path to follow
   * @param endVelocity Target end velocity in m/s (0 = stop at end)
   */
  public FollowPath(CommandSwerveDrivetrain swerve, FollowablePath path, double endVelocity) {
    this.swerve = swerve;
    this.path = path;
    this.endVelocity = endVelocity;
    addRequirements(swerve);
  }

  // ---- Builder methods ----

  public FollowPath withLookahead(double k, double min, double max) {
    this.lookaheadK = k;
    this.lookaheadMin = min;
    this.lookaheadMax = max;
    return this;
  }

  public FollowPath withCrossTrackGains(double kp, double kd) {
    this.crossTrackKp = kp;
    this.crossTrackKd = kd;
    return this;
  }

  public FollowPath withCompletionTolerance(double meters) {
    this.completionTolerance = meters;
    return this;
  }

  /**
   * Overrides the path's heading with a custom rotation supplier. When set, this takes priority
   * over the heading from {@link FollowablePath#getHeading}.
   */
  public FollowPath withRotationSupplier(RotationSupplier supplier) {
    this.rotationSupplier = supplier;
    return this;
  }

  public FollowPath withRotationTolerance(double radians) {
    this.rotationTolerance = radians;
    return this;
  }

  public FollowPath withMaxRotationBudget(double fraction) {
    this.maxRotationBudgetFraction = fraction;
    return this;
  }

  // ---- Command lifecycle ----

  @Override
  public void initialize() {
    lastCommandedVelocity = swerve.getFieldSpeeds();
    lastTime = Utils.getCurrentTimeSeconds();
    lastCrossTrackError = 0;

    // Default heading: use path heading if no override
    if (rotationSupplier == null) {
      rotationSupplier = (robotPose, pathS, pathTangent) -> path.getHeading(pathS).getRadians();
    }
    lastHeadingError = 0;

    lastProjectedS = 0.0;
    referencePathLogged = false;
  }

  @Override
  public void execute() {
    double currentTime = Utils.getCurrentTimeSeconds();
    double dt = currentTime - lastTime;
    lastTime = currentTime;
    if (dt < 1e-6) dt = 0.02;

    if (!referencePathLogged) {
      logReferencePath();
      referencePathLogged = true;
    }

    Pose2d robotPose = swerve.getPose();
    Translation2d robotPos = robotPose.getTranslation();

    // Step 1: Project robot onto path — bounded search around last known position
    ProjectionResult proj =
        path.getClosestPointInRange(
            robotPos, lastProjectedS - PROJECTION_MAX_DELTA, lastProjectedS + PROJECTION_MAX_DELTA);
    double sRobot = proj.s();
    double crossTrackError = proj.crossTrackError();
    Translation2d tangent = proj.tangent();

    // Step 2: Adaptive lookahead — further ahead when moving faster
    double currentSpeed =
        Math.hypot(
            lastCommandedVelocity.vxMetersPerSecond, lastCommandedVelocity.vyMetersPerSecond);
    double lookaheadDist =
        MathUtil.clamp(lookaheadK * currentSpeed + lookaheadMin, lookaheadMin, lookaheadMax);

    // Curvature cap: prevent chord from deviating too far from the arc
    double kappa = Math.abs(path.getCurvature(sRobot));
    if (kappa > 1e-6) {
      lookaheadDist = Math.min(lookaheadDist, lookaheadMaxArcAngle / kappa);
    }
    lookaheadDist = Math.max(lookaheadDist, lookaheadMin);

    double sTarget = Math.min(sRobot + lookaheadDist, path.getTotalLength());

    // Step 3: Get target point and profiled velocity
    // Use max of local and lookahead velocity. This naturally handles arc-length
    // compression at zero-velocity starts (lookahead is past the compressed zone)
    // while preserving Choreo's deceleration profile at the path end.
    Translation2d targetPoint = path.getPoint(sTarget);
    double profiledSpeed = Math.max(path.getVelocity(sRobot), path.getVelocity(sTarget));

    // Step 4: Velocity direction — toward lookahead point
    Translation2d toTarget = targetPoint.minus(robotPos);
    double distToTarget = toTarget.getNorm();
    Translation2d direction;
    if (distToTarget > 1e-6) {
      direction = toTarget.div(distToTarget);
    } else {
      direction = tangent;
    }

    // Step 5: Cross-track PD correction + curvature feedforward
    double crossTrackRate = (dt > 1e-6) ? (crossTrackError - lastCrossTrackError) / dt : 0;
    double correction = crossTrackKp * crossTrackError + crossTrackKd * crossTrackRate;
    double nx = -tangent.getY();
    double ny = tangent.getX();
    double signedKappa = path.getCurvature(sRobot);
    double curvatureFf = curvatureFfGain * profiledSpeed * profiledSpeed * signedKappa;
    double corrScale = -correction + curvatureFf;

    // Step 6: Combine path velocity + correction
    double vx = direction.getX() * profiledSpeed + nx * corrScale;
    double vy = direction.getY() * profiledSpeed + ny * corrScale;

    // Step 7: Heading control with rotation budget allocation
    double targetHeading = rotationSupplier.getTargetHeading(robotPose, sRobot, tangent);
    double headingError =
        MathUtil.angleModulus(targetHeading - robotPose.getRotation().getRadians());
    lastHeadingError = Math.abs(headingError);
    double omega = angleErrorToOmega(headingError);

    if (omega != 0.0) {
      double maxAngularContrib = maxRotationBudgetFraction * AccelerationLimiter.MAX_FRICTION_ACCEL;
      double angularContrib = Math.abs(omega) * AccelerationLimiter.DRIVE_BASE_RADIUS;
      if (angularContrib > maxAngularContrib) {
        omega = Math.copySign(maxAngularContrib / AccelerationLimiter.DRIVE_BASE_RADIUS, omega);
        angularContrib = maxAngularContrib;
      }

      double maxFriction = AccelerationLimiter.MAX_FRICTION_ACCEL;
      double availableFraction =
          Math.sqrt(
              Math.max(0, 1.0 - (angularContrib * angularContrib) / (maxFriction * maxFriction)));
      vx *= availableFraction;
      vy *= availableFraction;
    }

    // Step 8: Acceleration limiter
    AccelerationLimiter.integrateVelocity(
        lastCommandedVelocity,
        lastCommandedVelocity.vxMetersPerSecond,
        lastCommandedVelocity.vyMetersPerSecond,
        lastCommandedVelocity.omegaRadiansPerSecond,
        vx,
        vy,
        omega,
        dt);

    swerve.setControl(request.withSpeeds(lastCommandedVelocity));
    lastCrossTrackError = crossTrackError;
    lastProjectedS = sRobot;

    // Log tracking data
    double progress = (path.getTotalLength() > 0) ? sRobot / path.getTotalLength() : 0;
    Logger.recordOutput("FollowPath/CrossTrackError", crossTrackError);
    Logger.recordOutput("FollowPath/CrossTrackRate", crossTrackRate);
    Logger.recordOutput("FollowPath/ArcLengthS", sRobot);
    Logger.recordOutput("FollowPath/Progress", progress);
    Logger.recordOutput("FollowPath/ProfiledSpeed", profiledSpeed);
    Logger.recordOutput("FollowPath/ActualSpeed", currentSpeed);
    Logger.recordOutput("FollowPath/LookaheadDist", lookaheadDist);
    Logger.recordOutput("FollowPath/Curvature", kappa);
    Logger.recordOutput("FollowPath/Omega", omega);
    Logger.recordOutput("FollowPath/HeadingError", lastHeadingError);
    Logger.recordOutput("FollowPath/RemainingArcLength", path.getTotalLength() - sRobot);

    logEditorTarget[0] = targetPoint.getX();
    logEditorTarget[1] = targetPoint.getY();
    logEditorClosest[0] = proj.point().getX();
    logEditorClosest[1] = proj.point().getY();
    Logger.recordOutput("PathEditor/TargetPoint", logEditorTarget);
    Logger.recordOutput("PathEditor/ClosestPoint", logEditorClosest);
    Logger.recordOutput("PathEditor/CrossTrackError", crossTrackError);
    Logger.recordOutput("PathEditor/Progress", progress);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.setControl(new SwerveRequest.Idle());
    Logger.recordOutput("FollowPath/ReferencePath", new Pose2d[0]);
  }

  private void logReferencePath() {
    Pose2d[] pathPoses = new Pose2d[PATH_LOG_SAMPLES + 1];
    double ds = path.getTotalLength() / PATH_LOG_SAMPLES;

    for (int i = 0; i <= PATH_LOG_SAMPLES; i++) {
      double s = i * ds;
      Translation2d point = path.getPoint(s);
      pathPoses[i] = new Pose2d(point, path.getHeading(s));
    }

    Logger.recordOutput("FollowPath/ReferencePath", pathPoses);
    Logger.recordOutput("FollowPath/TotalLength", path.getTotalLength());
  }

  /** Maximum angular deceleration in rad/s² (friction-limited). */
  private static final double MAX_ANGULAR_DECEL =
      AccelerationLimiter.MAX_FRICTION_ACCEL / AccelerationLimiter.DRIVE_BASE_RADIUS;

  /** Conservative fraction of MAX_ANGULAR_DECEL for stopping-profile planning. */
  private static final double DECEL_BUDGET_FACTOR = 0.25;

  /** Proportional gain for near-target linear taper (replaces hard dead zone). */
  private static final double HEADING_KP = 8.0;

  private double angleErrorToOmega(double headingError) {
    double absError = Math.abs(headingError);
    if (absError < 1e-4) {
      return 0.0;
    }
    double budgetMaxOmega =
        maxRotationBudgetFraction
            * AccelerationLimiter.MAX_FRICTION_ACCEL
            / AccelerationLimiter.DRIVE_BASE_RADIUS;
    double stoppingOmega = Math.sqrt(2.0 * MAX_ANGULAR_DECEL * DECEL_BUDGET_FACTOR * absError);
    double linearOmega = HEADING_KP * absError;
    double omega = Math.min(Math.min(stoppingOmega, linearOmega), budgetMaxOmega);
    return Math.copySign(omega, headingError);
  }

  @Override
  public boolean isFinished() {
    boolean nearEnd = lastProjectedS >= path.getTotalLength() - completionTolerance;
    if (endVelocity > 0) {
      return nearEnd;
    }
    Translation2d tangent = path.getTangent(path.getTotalLength());
    double alongPath =
        lastCommandedVelocity.vxMetersPerSecond * tangent.getX()
            + lastCommandedVelocity.vyMetersPerSecond * tangent.getY();
    boolean headingOk = lastHeadingError <= rotationTolerance;
    return nearEnd && alongPath < completionVelocityTolerance && headingOk;
  }
}
