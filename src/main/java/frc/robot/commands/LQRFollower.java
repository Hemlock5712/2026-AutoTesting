package frc.robot.commands;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.system.Discretization;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.path.FollowablePath;
import frc.robot.utils.path.ProjectionResult;
import org.littletonrobotics.junction.Logger;

/**
 * LQR-based path following controller for A/B testing against {@link FollowPath}.
 *
 * <p>Uses a linearized kinematic model with an infinite-horizon LQR (solved via the discrete
 * algebraic Riccati equation) to compute optimal corrections. This is mathematically equivalent to
 * unconstrained MPC with infinite horizon.
 *
 * <p>State: [cross-track error, cross-track rate, heading error, heading rate]
 *
 * <p>Input: [lateral acceleration correction, angular acceleration correction]
 *
 * <p>Along-track velocity comes from the path's velocity profile (same as FollowPath). The LQR only
 * controls the lateral and rotational corrections.
 *
 * <p>All output is fed through {@link AccelerationLimiter} as a safety net.
 */
public class LQRFollower extends Command {

  private static final double DT = 0.02; // 20ms control loop
  private static final double PROJECTION_MAX_DELTA = 0.5;
  private static final int PATH_LOG_SAMPLES = 50;

  private final CommandSwerveDrivetrain swerve;
  private final FollowablePath path;

  // LQR tuning via Bryson's Rule: weight = 1/(max acceptable value)²
  // Defaults: 5cm cross-track, 5° heading, half-friction-limit effort
  private double qCrossTrack = 400.0; // 1/(0.05 m)²
  private double qCrossTrackRate = 4.0; // 1/(0.5 m/s)²
  private double qHeading = 132.0; // 1/(0.087 rad)² ≈ 1/(5°)²
  private double qHeadingRate = 1.0; // 1/(1.0 rad/s)²
  private double rLateral = 0.04; // 1/(5.0 m/s²)² — half of ~10.8 friction limit
  private double rAngular = 0.01; // 1/(10.0 rad/s²)²

  // State
  private ChassisSpeeds lastCommandedVelocity = new ChassisSpeeds();
  private double lastTime;
  private double lastProjectedS;
  private double lastCrossTrackError;
  private double lastHeadingError;
  private boolean referencePathLogged;

  // Completion
  private double completionTolerance = 0.05;
  private double completionVelocityTolerance = 0.1;

  private final double[] logEditorTarget = new double[2];
  private final double[] logEditorClosest = new double[2];

  private final SwerveRequest.ApplyFieldSpeeds request =
      new SwerveRequest.ApplyFieldSpeeds()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  public LQRFollower(CommandSwerveDrivetrain swerve, FollowablePath path) {
    this.swerve = swerve;
    this.path = path;
    addRequirements(swerve);
  }

  /** Sets Q diagonal (state cost). Higher = more aggressive correction. */
  public LQRFollower withStateCost(
      double crossTrack, double crossTrackRate, double heading, double headingRate) {
    this.qCrossTrack = crossTrack;
    this.qCrossTrackRate = crossTrackRate;
    this.qHeading = heading;
    this.qHeadingRate = headingRate;
    return this;
  }

  /** Sets R diagonal (input cost). Higher = smoother but slower response. */
  public LQRFollower withInputCost(double lateral, double angular) {
    this.rLateral = lateral;
    this.rAngular = angular;
    return this;
  }

  /**
   * Configures Q and R using Bryson's Rule from physical tolerances.
   *
   * <p>Each weight is computed as 1/(tolerance)². This is the standard LQR tuning method — you
   * specify the maximum acceptable error for each state and the acceleration budget for each input,
   * and the math derives optimal gains.
   *
   * @param crossTrackMeters Max acceptable cross-track error (e.g., 0.05 for 5cm)
   * @param headingDegrees Max acceptable heading error in degrees (e.g., 5.0)
   * @param effortFraction Fraction of friction limit to budget for corrections (0.0–1.0, e.g., 0.5)
   * @return this for chaining
   */
  public LQRFollower withTolerances(
      double crossTrackMeters, double headingDegrees, double effortFraction) {
    double headingRad = Math.toRadians(headingDegrees);
    double maxLateralAccel = AccelerationLimiter.MAX_FRICTION_ACCEL * effortFraction;
    double maxAngularAccel = maxLateralAccel / AccelerationLimiter.DRIVE_BASE_RADIUS;

    this.qCrossTrack = 1.0 / (crossTrackMeters * crossTrackMeters);
    this.qCrossTrackRate =
        1.0 / (crossTrackMeters * 10.0 * crossTrackMeters * 10.0); // rate ~ 10x position
    this.qHeading = 1.0 / (headingRad * headingRad);
    this.qHeadingRate = 1.0 / (headingRad * 10.0 * headingRad * 10.0); // rate ~ 10x heading
    this.rLateral = 1.0 / (maxLateralAccel * maxLateralAccel);
    this.rAngular = 1.0 / (maxAngularAccel * maxAngularAccel);
    return this;
  }

  public LQRFollower withCompletionTolerance(double meters) {
    this.completionTolerance = meters;
    return this;
  }

  @Override
  public void initialize() {
    lastCommandedVelocity = swerve.getFieldSpeeds();
    lastTime = Utils.getCurrentTimeSeconds();
    lastProjectedS = 0.0;
    lastCrossTrackError = 0.0;
    lastHeadingError = 0.0;
    referencePathLogged = false;
  }

  @Override
  public void execute() {
    double currentTime = Utils.getCurrentTimeSeconds();
    double dt = currentTime - lastTime;
    lastTime = currentTime;
    if (dt < 1e-6) dt = DT;

    if (!referencePathLogged) {
      logReferencePath();
      referencePathLogged = true;
    }

    Pose2d robotPose = swerve.getPose();
    Translation2d robotPos = robotPose.getTranslation();

    // Project robot onto path
    ProjectionResult proj =
        path.getClosestPointInRange(
            robotPos, lastProjectedS - PROJECTION_MAX_DELTA, lastProjectedS + PROJECTION_MAX_DELTA);
    double sRobot = proj.s();
    double crossTrackError = proj.crossTrackError();
    Translation2d tangent = proj.tangent();

    // Compute state errors
    double crossTrackRate = (dt > 1e-6) ? (crossTrackError - lastCrossTrackError) / dt : 0.0;

    Rotation2d targetHeading = path.getHeading(sRobot);
    double headingError =
        MathUtil.angleModulus(targetHeading.getRadians() - robotPose.getRotation().getRadians());
    double headingRate = (dt > 1e-6) ? (headingError - lastHeadingError) / dt : 0.0;

    // Solve LQR for the linearized lateral+heading dynamics
    // State: [crossTrackError, crossTrackRate, headingError, headingRate]
    // Input: [lateralAccel, angularAccel]
    // Dynamics: x_dot = A*x + B*u where A = [[0,1,0,0],[0,0,0,0],[0,0,0,1],[0,0,0,0]]
    //                                    B = [[0,0],[1,0],[0,0],[0,1]]
    // This is a double integrator for both cross-track and heading.

    // Build Q and R matrices
    Matrix<N4, N4> Q =
        Matrix.eye(Nat.N4())
            .times(0.0)
            .plus(makeQ(qCrossTrack, qCrossTrackRate, qHeading, qHeadingRate));
    Matrix<N2, N2> R = Matrix.eye(Nat.N2()).times(0.0).plus(makeR(rLateral, rAngular));

    // Continuous-time A and B
    Matrix<N4, N4> A = new Matrix<>(Nat.N4(), Nat.N4());
    A.set(0, 1, 1.0); // crossTrackError' = crossTrackRate
    A.set(2, 3, 1.0); // headingError' = headingRate

    Matrix<N4, N2> B = new Matrix<>(Nat.N4(), Nat.N2());
    B.set(1, 0, 1.0); // crossTrackRate' = lateralAccel
    B.set(3, 1, 1.0); // headingRate' = angularAccel

    // Discretize
    var discABPair = Discretization.discretizeAB(A, B, dt);
    Matrix<N4, N4> Ad = discABPair.getFirst();
    Matrix<N4, N2> Bd = discABPair.getSecond();

    // Solve DARE for optimal gain K
    Matrix<N4, N4> P = solveDAREIterative(Ad, Bd, Q, R);
    // K = (R + B^T P B)^{-1} B^T P A
    Matrix<N2, N2> S = R.plus(Bd.transpose().times(P).times(Bd));
    Matrix<N2, N4> K = S.solve(Bd.transpose().times(P).times(Ad));

    // State vector
    Matrix<N4, N1> x = new Matrix<>(Nat.N4(), Nat.N1());
    x.set(0, 0, crossTrackError);
    x.set(1, 0, crossTrackRate);
    x.set(2, 0, headingError);
    x.set(3, 0, headingRate);

    // Optimal input: u = -K * x
    Matrix<N2, N1> u = K.times(x).times(-1.0);
    double lateralAccel = u.get(0, 0);
    double angularAccel = u.get(1, 0);

    // Compute along-track velocity from path profile
    double profiledSpeed = path.getVelocity(sRobot);

    // Build field-relative velocity command
    // Along-track: follow the path tangent at profiled speed
    // Cross-track: integrate the LQR lateral correction
    double currentSpeed =
        Math.hypot(
            lastCommandedVelocity.vxMetersPerSecond, lastCommandedVelocity.vyMetersPerSecond);
    double nx = -tangent.getY();
    double ny = tangent.getX();

    // Lateral velocity correction from LQR (integrate acceleration)
    double lateralCorrection = lateralAccel * dt;
    double desiredVx = tangent.getX() * profiledSpeed + nx * lateralCorrection;
    double desiredVy = tangent.getY() * profiledSpeed + ny * lateralCorrection;
    double omega = lastCommandedVelocity.omegaRadiansPerSecond + angularAccel * dt;

    // Feed through AccelerationLimiter (safety net)
    AccelerationLimiter.integrateVelocity(
        lastCommandedVelocity,
        lastCommandedVelocity.vxMetersPerSecond,
        lastCommandedVelocity.vyMetersPerSecond,
        lastCommandedVelocity.omegaRadiansPerSecond,
        desiredVx,
        desiredVy,
        omega,
        dt);

    swerve.setControl(request.withSpeeds(lastCommandedVelocity));
    lastCrossTrackError = crossTrackError;
    lastHeadingError = headingError;
    lastProjectedS = sRobot;

    // Log tracking data
    double progress = (path.getTotalLength() > 0) ? sRobot / path.getTotalLength() : 0;
    Logger.recordOutput("LQRFollower/CrossTrackError", crossTrackError);
    Logger.recordOutput("LQRFollower/HeadingError", headingError);
    Logger.recordOutput("LQRFollower/ArcLengthS", sRobot);
    Logger.recordOutput("LQRFollower/Progress", progress);
    Logger.recordOutput("LQRFollower/ProfiledSpeed", profiledSpeed);
    Logger.recordOutput("LQRFollower/ActualSpeed", currentSpeed);
    Logger.recordOutput("LQRFollower/LateralAccel", lateralAccel);
    Logger.recordOutput("LQRFollower/AngularAccel", angularAccel);

    Translation2d targetPoint = path.getPoint(Math.min(sRobot + 0.2, path.getTotalLength()));
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
    Logger.recordOutput("LQRFollower/ReferencePath", new Pose2d[0]);
  }

  @Override
  public boolean isFinished() {
    boolean nearEnd = lastProjectedS >= path.getTotalLength() - completionTolerance;
    Translation2d tangent = path.getTangent(path.getTotalLength());
    double alongPath =
        lastCommandedVelocity.vxMetersPerSecond * tangent.getX()
            + lastCommandedVelocity.vyMetersPerSecond * tangent.getY();
    return nearEnd && alongPath < completionVelocityTolerance;
  }

  private void logReferencePath() {
    Pose2d[] pathPoses = new Pose2d[PATH_LOG_SAMPLES + 1];
    double ds = path.getTotalLength() / PATH_LOG_SAMPLES;
    for (int i = 0; i <= PATH_LOG_SAMPLES; i++) {
      double s = i * ds;
      Translation2d point = path.getPoint(s);
      Translation2d tangent = path.getTangent(s);
      pathPoses[i] = new Pose2d(point, new Rotation2d(tangent.getX(), tangent.getY()));
    }
    Logger.recordOutput("LQRFollower/ReferencePath", pathPoses);
    Logger.recordOutput("LQRFollower/TotalLength", path.getTotalLength());
  }

  // ---- LQR math helpers ----

  private static Matrix<N4, N4> makeQ(double q1, double q2, double q3, double q4) {
    Matrix<N4, N4> Q = new Matrix<>(Nat.N4(), Nat.N4());
    Q.set(0, 0, q1);
    Q.set(1, 1, q2);
    Q.set(2, 2, q3);
    Q.set(3, 3, q4);
    return Q;
  }

  private static Matrix<N2, N2> makeR(double r1, double r2) {
    Matrix<N2, N2> R = new Matrix<>(Nat.N2(), Nat.N2());
    R.set(0, 0, r1);
    R.set(1, 1, r2);
    return R;
  }

  /**
   * Solves the Discrete Algebraic Riccati Equation iteratively.
   *
   * <p>P = A^T P A - A^T P B (R + B^T P B)^{-1} B^T P A + Q
   *
   * <p>Iterates until convergence or max iterations. For the double-integrator dynamics this
   * converges in ~20 iterations.
   */
  private static Matrix<N4, N4> solveDAREIterative(
      Matrix<N4, N4> A, Matrix<N4, N2> B, Matrix<N4, N4> Q, Matrix<N2, N2> R) {
    Matrix<N4, N4> P = Q.copy();
    for (int i = 0; i < 50; i++) {
      Matrix<N4, N4> AtP = A.transpose().times(P);
      Matrix<N4, N2> AtPB = AtP.times(B);
      Matrix<N2, N2> S = R.plus(B.transpose().times(P).times(B));
      Matrix<N4, N4> Pnew = AtP.times(A).minus(AtPB.times(S.solve(AtPB.transpose()))).plus(Q);

      // Check convergence
      double maxDiff = 0;
      for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 4; c++) {
          maxDiff = Math.max(maxDiff, Math.abs(Pnew.get(r, c) - P.get(r, c)));
        }
      }
      P = Pnew;
      if (maxDiff < 1e-10) break;
    }
    return P;
  }
}
