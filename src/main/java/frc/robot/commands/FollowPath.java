package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utils.path.FollowablePath;
import frc.robot.utils.path.ProjectionResult;
import frc.robot.utils.path.RotationSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Drives along a path, tracking distance (not time).
 *
 * <p>Looks ahead on the path based on speed and uses PD control to fix any sideways drift. Because
 * we track the robot's distance along the path, if the robot gets bumped or stalls, the path just
 * waits - it doesn't fight a clock.
 *
 * <p>Plans the target velocity in {@code execute()} (50 Hz). The acceleration limiter then smoothly
 * accelerates toward that target on the 250 Hz fast loop.
 */
public class FollowPath extends Command {

  private final Drive drive;
  private final FollowablePath path;
  private final double endVelocity;

  private RotationSupplier rotationSupplier;
  private double rotationTolerance = Double.POSITIVE_INFINITY;
  private double lastHeadingError;

  private double maxRotationBudgetFraction = 0.30;

  private double lookaheadK = 0.15;
  private double lookaheadMin = 0.15;
  private double lookaheadMax = 1.0;
  private double lookaheadMaxArcAngle = Math.PI / 6;

  private double crossTrackKp = 3.0;
  private double crossTrackKd = 0.5;
  private double curvatureFfGain = 0.1;

  private double completionTolerance = 0.05;
  private double completionVelocityTolerance = 0.1;

  private static final double PROJECTION_MAX_DELTA = 0.5;
  private static final int PATH_LOG_SAMPLES = 50;

  // Set by the main loop, read by the fast loop.
  private volatile double targetVx;
  private volatile double targetVy;
  private volatile double targetOmega;

  // Used only by the fast loop.
  private final ChassisSpeeds limitedFieldSpeeds = new ChassisSpeeds();

  // Used only by the main loop.
  private double lastTime;
  private double lastCrossTrackError;
  private double lastProjectedS;
  private boolean referencePathLogged;

  // Reusable arrays for logging (avoids allocations).
  private final double[] logEditorTarget = new double[2];
  private final double[] logEditorClosest = new double[2];
  private final double[] tangentAtEnd = new double[2];

  public FollowPath(Drive drive, FollowablePath path) {
    this(drive, path, 0.0);
  }

  public FollowPath(Drive drive, FollowablePath path, double endVelocity) {
    this.drive = drive;
    this.path = path;
    this.endVelocity = endVelocity;
    addRequirements(drive);
  }

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

  @Override
  public void initialize() {
    ChassisSpeeds field = drive.getFieldSpeeds();
    limitedFieldSpeeds.vxMetersPerSecond = field.vxMetersPerSecond;
    limitedFieldSpeeds.vyMetersPerSecond = field.vyMetersPerSecond;
    limitedFieldSpeeds.omegaRadiansPerSecond = field.omegaRadiansPerSecond;
    targetVx = field.vxMetersPerSecond;
    targetVy = field.vyMetersPerSecond;
    targetOmega = field.omegaRadiansPerSecond;

    lastTime = Timer.getFPGATimestamp();
    lastCrossTrackError = 0;

    if (rotationSupplier == null) {
      rotationSupplier = (robotPose, pathS, pathTangent) -> path.getHeading(pathS).getRadians();
    }
    lastHeadingError = 0;
    lastProjectedS = 0.0;
    referencePathLogged = false;

    Translation2d t = path.getTangent(path.getTotalLength());
    tangentAtEnd[0] = t.getX();
    tangentAtEnd[1] = t.getY();

    drive.setHighRateController(this::tickHighRate);
  }

  @Override
  public void execute() {
    double currentTime = Timer.getFPGATimestamp();
    double dt = currentTime - lastTime;
    lastTime = currentTime;
    if (dt < 1e-6) dt = 0.02;

    if (!referencePathLogged) {
      logReferencePath();
      referencePathLogged = true;
    }

    Pose2d robotPose = drive.getPose();
    Translation2d robotPos = robotPose.getTranslation();

    ProjectionResult proj =
        path.getClosestPointInRange(
            robotPos, lastProjectedS - PROJECTION_MAX_DELTA, lastProjectedS + PROJECTION_MAX_DELTA);
    double sRobot = proj.s();
    double crossTrackError = proj.crossTrackError();
    Translation2d tangent = proj.tangent();

    ChassisSpeeds fieldSpeeds = drive.getFieldSpeeds();
    double currentSpeed = Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);

    double lookaheadDist =
        MathUtil.clamp(lookaheadK * currentSpeed + lookaheadMin, lookaheadMin, lookaheadMax);

    double kappa = Math.abs(path.getCurvature(sRobot));
    if (kappa > 1e-6) {
      lookaheadDist = Math.min(lookaheadDist, lookaheadMaxArcAngle / kappa);
    }
    lookaheadDist = Math.max(lookaheadDist, lookaheadMin);

    double sTarget = Math.min(sRobot + lookaheadDist, path.getTotalLength());

    Translation2d targetPoint = path.getPoint(sTarget);
    double profiledSpeed = Math.max(path.getVelocity(sRobot), path.getVelocity(sTarget));

    Translation2d toTarget = targetPoint.minus(robotPos);
    double distToTarget = toTarget.getNorm();
    Translation2d direction = distToTarget > 1e-6 ? toTarget.div(distToTarget) : tangent;

    double crossTrackRate = (dt > 1e-6) ? (crossTrackError - lastCrossTrackError) / dt : 0;
    double correction = crossTrackKp * crossTrackError + crossTrackKd * crossTrackRate;
    double nx = -tangent.getY();
    double ny = tangent.getX();
    double signedKappa = path.getCurvature(sRobot);
    double curvatureFf = curvatureFfGain * profiledSpeed * profiledSpeed * signedKappa;
    double corrScale = -correction + curvatureFf;

    double vx = direction.getX() * profiledSpeed + nx * corrScale;
    double vy = direction.getY() * profiledSpeed + ny * corrScale;

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

    targetVx = vx;
    targetVy = vy;
    targetOmega = omega;

    lastCrossTrackError = crossTrackError;
    lastProjectedS = Math.max(lastProjectedS, sRobot);

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

  private void tickHighRate(double dt) {
    AccelerationLimiter.integrateVelocityInPlace(
        limitedFieldSpeeds, targetVx, targetVy, targetOmega, dt);
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(limitedFieldSpeeds, drive.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    drive.clearHighRateController();
    drive.stop();
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

  private static final double MAX_ANGULAR_DECEL =
      AccelerationLimiter.MAX_FRICTION_ACCEL / AccelerationLimiter.DRIVE_BASE_RADIUS;
  private static final double DECEL_BUDGET_FACTOR = 0.25;
  private static final double HEADING_KP = 8.0;

  private double angleErrorToOmega(double headingError) {
    double absError = Math.abs(headingError);
    if (absError < 1e-4) return 0.0;
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
    if (endVelocity > 0) return nearEnd;
    ChassisSpeeds fs = drive.getFieldSpeeds();
    double alongPath =
        fs.vxMetersPerSecond * tangentAtEnd[0] + fs.vyMetersPerSecond * tangentAtEnd[1];
    boolean headingOk = lastHeadingError <= rotationTolerance;
    return nearEnd && alongPath < completionVelocityTolerance && headingOk;
  }
}
