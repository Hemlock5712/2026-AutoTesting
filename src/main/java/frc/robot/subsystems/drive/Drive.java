// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
//
// Adapted from the AdvantageKit talonfx_swerve template. PathPlanner integration is removed
// (this project uses Choreo); the rest of the structure is preserved so behavior matches the
// upstream template's sim ↔ replay determinism guarantees.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.commands.AccelerationLimiter;
import frc.robot.generated.TunerConstants;
import frc.robot.simlib.drivesims.COTS;
import frc.robot.simlib.drivesims.configs.DriveTrainSimulationConfig;
import frc.robot.simlib.drivesims.configs.SwerveModuleSimulationConfig;
import frc.robot.utils.FieldInfo;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  // Not included in TunerConstants, so declared here.
  static final double ODOMETRY_FREQUENCY = TunerConstants.kCANBus.isNetworkFD() ? 250.0 : 100.0;
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  // MapleSim physics shares constants with the limiter — one source of truth.
  private static final double ROBOT_MASS_KG = AccelerationLimiter.ROBOT_MASS;
  private static final double WHEEL_COF = AccelerationLimiter.MU_FRICTION;

  private static DriveTrainSimulationConfig mapleSimConfig = null;

  /** Lazily-built MapleSim drivetrain config keyed off TunerConstants. */
  public static DriveTrainSimulationConfig getMapleSimConfig() {
    if (mapleSimConfig != null) return mapleSimConfig;
    return mapleSimConfig =
        DriveTrainSimulationConfig.Default()
            .withRobotMass(Kilograms.of(ROBOT_MASS_KG))
            .withCustomModuleTranslations(getModuleTranslations())
            .withGyro(COTS.ofPigeon2())
            .withSwerveModule(
                new SwerveModuleSimulationConfig(
                    DCMotor.getKrakenX60(1),
                    DCMotor.getKrakenX60(1),
                    TunerConstants.FrontLeft.DriveMotorGearRatio,
                    TunerConstants.FrontLeft.SteerMotorGearRatio,
                    Volts.of(TunerConstants.FrontLeft.DriveFrictionVoltage),
                    Volts.of(TunerConstants.FrontLeft.SteerFrictionVoltage),
                    Meters.of(TunerConstants.FrontLeft.WheelRadius),
                    KilogramSquareMeters.of(TunerConstants.FrontLeft.SteerInertia),
                    WHEEL_COF));
  }

  /**
   * Period for the 250 Hz fast loop. Used by commands that need physics-limit updates more often
   * than the normal 50 Hz robot loop (e.g. the acceleration limiter). Commands plug in via {@link
   * #setControl(SwerveRequest)} and unplug in {@code end()}.
   */
  private static final double HIGH_RATE_PERIOD_S = 0.004;

  static final Lock odometryLock = new ReentrantLock();

  // --- IO + per-cycle inputs ---
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  // --- Pose estimation state ---
  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = Rotation2d.kZero;
  // Most recent raw module positions (real wheel distance + current steer angle). Used to
  // compute per-sample (delta_s, delta_theta) for arc integration.
  private final SwerveModulePosition[] lastModulePositions = newZeroedPositions();
  // Arc-integrated accumulators handed to the estimator. WPILib's kinematics.toTwist2d treats
  // a SwerveModulePosition as a straight-line chord (dx = distance*cos, dy = distance*sin),
  // so re-encoding the arc displacement (dx, dy) as (hypot, atan2) lets the same machinery
  // emit the arc-correct twist without subclassing the estimator.
  private final SwerveModulePosition[] effectiveModulePositions = newZeroedPositions();
  private final SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(
          kinematics, rawGyroRotation, effectiveModulePositions, Pose2d.kZero);
  private Consumer<Pose2d> poseResetListener = null;

  // --- Cached snapshots read by the 250 Hz fast loop without locks ---
  private volatile Pose2d cachedPose = Pose2d.kZero;
  private volatile ChassisSpeeds cachedRobotSpeeds = new ChassisSpeeds();
  private volatile SwerveModuleState[] latestSetpointStates = new SwerveModuleState[0];
  private volatile ChassisSpeeds latestSetpointSpeeds = new ChassisSpeeds();

  // --- Field-escape diagnostic ---
  // Counts periodic ticks where the estimator pose has crossed any field wall. We tried
  // clamping the output here as a safety net, but it bit legitimate near-wall overshoots
  // during path following and introduced cm-scale pose error. Diagnostic only.
  private static final double FIELD_ESCAPE_MARGIN_M = 0.0;
  private long fieldEscapeHits = 0;

  // Tracks DS-disable transitions so we stop the modules once on the rising edge instead of
  // every periodic tick (avoiding a 50 Hz storm of stop() calls racing the 250 Hz hook).
  private boolean lastSeenDisabled = false;
  // Counts arc-integration samples rejected because an input was non-finite (e.g. MapleSim
  // poisoning the steer angle with NaN). Logged so it stays visible if it ever fires on
  // hardware.
  private long arcIntegrateRejections = 0;

  // --- Fast-loop hook ---
  private final Notifier highRateNotifier = new Notifier(this::tickHighRate);
  private volatile DoubleConsumer highRateController = null;
  private double highRateLastTime = 0.0;
  private volatile SwerveRequest activeRequest = null;

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
    modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
    modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

    PhoenixOdometryThread.getInstance().start();

    // Start the 250 Hz fast loop. Does nothing until a command plugs in a callback.
    highRateLastTime = Timer.getFPGATimestamp();
    highRateNotifier.setName("DriveHighRateLoop");
    highRateNotifier.startPeriodic(HIGH_RATE_PERIOD_S);
  }

  @Override
  public void periodic() {
    odometryLock.lock();
    try {
      // Only lock while reading sensor data, so the 250 Hz odometry thread can keep working.
      gyroIO.updateInputs(gyroInputs);
      for (var module : modules) {
        module.updateIo();
      }
    } finally {
      odometryLock.unlock();
    }

    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.postIoPeriodic();
    }

    boolean isDisabled = DriverStation.isDisabled();
    if (isDisabled) {
      // Stop only on the falling enable→disable edge. The 250 Hz hook is gated below in
      // tickHighRate, so once we've stopped the modules they stay stopped — no need to retry
      // every tick (which would race the Notifier and pointlessly hammer setControl).
      if (!lastSeenDisabled) {
        for (var module : modules) {
          module.stop();
        }
      }
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }
    lastSeenDisabled = isDisabled;

    updateOdometry();

    cachedPose = poseEstimator.getEstimatedPosition();
    if (isOutsideField(cachedPose)) fieldEscapeHits++;
    Logger.recordOutput("Drive/FieldEscapeHits", fieldEscapeHits);
    cachedRobotSpeeds = kinematics.toChassisSpeeds(getModuleStates());

    Logger.recordOutput(
        "Drive/Friction/ModuleRatios", AccelerationLimiter.getLastModuleFrictionRatios());
    logLatestSetpoint();

    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.getMode() != Mode.SIM);
  }

  /** Drains all odometry samples accumulated since the last tick into the pose estimator. */
  private void updateOdometry() {
    double[] sampleTimestamps = modules[0].getOdometryTimestamps();
    SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
    for (int i = 0; i < sampleTimestamps.length; i++) {
      for (int m = 0; m < 4; m++) {
        SwerveModulePosition rawCurrent = modules[m].getOdometryPositions()[i];
        moduleDeltas[m] = arcIntegrate(rawCurrent, lastModulePositions[m]);
        effectiveModulePositions[m] =
            new SwerveModulePosition(
                effectiveModulePositions[m].distanceMeters + moduleDeltas[m].distanceMeters,
                moduleDeltas[m].angle);
        lastModulePositions[m] = rawCurrent;
      }
      if (gyroInputs.connected) {
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, effectiveModulePositions);
    }
    Logger.recordOutput("Drive/Odometry/ArcIntegrateRejections", arcIntegrateRejections);
  }

  /**
   * Returns the arc-correct displacement of one module over a sample, encoded as a straight chord
   * (distance + direction) so it can be fed into WPILib's straight-line-assuming
   * kinematics.toTwist2d. Assumes constant module rotational velocity over the sample:
   *
   * <pre>
   *   dx = (delta_s / delta_theta) * (sin theta_2 - sin theta_1)
   *   dy = (delta_s / delta_theta) * (cos theta_1 - cos theta_2)
   * </pre>
   *
   * Falls back to a straight chord when delta_theta is below numerical noise. Returns the chord
   * magnitude as the distance and atan2(dy, dx) as the direction. Rejects samples whose inputs are
   * non-finite (which happens in sim when MapleSim's brownout/LinearFilter feedback poisons the
   * steer angle with NaN — see Issue B in the May 2026 audit) by emitting a zero-displacement
   * sample anchored to the last good angle and bumping the rejection counter.
   */
  private SwerveModulePosition arcIntegrate(
      SwerveModulePosition rawCurrent, SwerveModulePosition rawLast) {
    double deltaS = rawCurrent.distanceMeters - rawLast.distanceMeters;
    double currentRad = rawCurrent.angle.getRadians();
    double lastRad = rawLast.angle.getRadians();
    if (!Double.isFinite(deltaS) || !Double.isFinite(currentRad) || !Double.isFinite(lastRad)) {
      arcIntegrateRejections++;
      return new SwerveModulePosition(0.0, rawLast.angle);
    }
    double deltaTheta = rawCurrent.angle.minus(rawLast.angle).getRadians();
    double dx;
    double dy;
    if (Math.abs(deltaTheta) < 1.0e-6) {
      dx = deltaS * rawLast.angle.getCos();
      dy = deltaS * rawLast.angle.getSin();
    } else {
      double r = deltaS / deltaTheta;
      dx = r * (rawCurrent.angle.getSin() - rawLast.angle.getSin());
      dy = r * (rawLast.angle.getCos() - rawCurrent.angle.getCos());
    }
    double dEff = Math.hypot(dx, dy);
    // Threshold matches WPILib's Rotation2d(x, y) constructor (Math.hypot < 1e-6 → reportError).
    // Any sub-µm displacement just retains the last good angle.
    Rotation2d thetaEff = (dEff < 1.0e-6) ? rawCurrent.angle : new Rotation2d(dx, dy);
    return new SwerveModulePosition(dEff, thetaEff);
  }

  /** Logs the most recent setpoint from the main thread so AKit's logger stays single-threaded. */
  private void logLatestSetpoint() {
    SwerveModuleState[] sp = latestSetpointStates;
    if (sp.length > 0) {
      Logger.recordOutput("SwerveStates/Setpoints", sp);
      Logger.recordOutput("SwerveStates/SetpointsOptimized", sp);
      Logger.recordOutput("SwerveChassisSpeeds/Setpoints", latestSetpointSpeeds);
    }
  }

  // --- Fast loop ---

  private void tickHighRate() {
    var hook = highRateController;
    double now = Timer.getFPGATimestamp();
    double dt = now - highRateLastTime;
    highRateLastTime = now;
    if (hook == null) return;
    // Skip while the DS is disabled — otherwise a still-bound default command keeps issuing
    // closed-loop setpoints on this thread while periodic() (main thread) is calling
    // module.stop(), and the two race on every motor's setControl path.
    if (DriverStation.isDisabled()) return;
    if (dt < 1e-9) dt = HIGH_RATE_PERIOD_S;
    try {
      hook.accept(dt);
    } catch (Throwable t) {
      // A crash in the hook would kill the timer thread silently. Log it and disable instead.
      DriverStation.reportError("DriveHighRate hook threw: " + t.getMessage(), t.getStackTrace());
      highRateController = null;
    }
  }

  /**
   * Install a {@link SwerveRequest} as the active controller. Its {@link SwerveRequest#apply} runs
   * on the 250 Hz fast loop until {@link #clearControl()} is called or another {@code setControl}
   * replaces it. Mutual exclusion across commands is provided by the subsystem requirement on
   * {@link Drive}.
   */
  public void setControl(SwerveRequest request) {
    if (request == activeRequest) return;
    if (request != null) request.onActivate(this);
    activeRequest = request;
    highRateController = this::tickActiveRequest;
  }

  /** Clears the active {@link SwerveRequest}. Call from a command's {@code end()}. */
  public void clearControl() {
    activeRequest = null;
    highRateController = null;
  }

  private void tickActiveRequest(double dt) {
    SwerveRequest req = activeRequest;
    if (req != null) {
      req.apply(this, dt);
    }
  }

  /**
   * Drive at the given robot-relative chassis speeds. Can be called from the main loop or the 250
   * Hz fast loop.
   */
  public void runVelocity(ChassisSpeeds speeds) {
    runVelocity(speeds, Translation2d.kZero);
  }

  /**
   * Drive at the given robot-relative chassis speeds, pivoting around an offset center of rotation
   * (in robot frame, meters). Useful when the robot's CoG is offset, or when you want to spin
   * around a specific point (e.g. a corner module) rather than the geometric center.
   */
  public void runVelocity(ChassisSpeeds speeds, Translation2d centerOfRotation) {
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, Constants.LOOP_PERIOD_SECONDS);
    SwerveModuleState[] setpointStates =
        kinematics.toSwerveModuleStates(discreteSpeeds, centerOfRotation);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Save the latest setpoint so periodic() can log it from the main thread.
    latestSetpointStates = setpointStates;
    latestSetpointSpeeds = discreteSpeeds;
  }

  /** Stop driving but keep wheels pointing where they were. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /** Stop and turn wheels into an X shape so we can't be pushed. */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Point all four module wheels at the given direction with zero drive velocity. */
  public void pointWheelsAt(Rotation2d direction) {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = new SwerveModuleState(0.0, direction);
      modules[i].runSetpoint(states[i]);
    }
    latestSetpointStates = states;
    latestSetpointSpeeds = new ChassisSpeeds();
  }

  @AutoLogOutput(key = "SwerveStates/Measured")
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Robot-relative chassis speeds. Safe to call from any thread. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getRobotSpeeds() {
    return cachedRobotSpeeds;
  }

  /** Field-relative chassis speeds (vx and vy in field frame). */
  public ChassisSpeeds getFieldSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotSpeeds(), getRotation());
  }

  public double translationSpeed() {
    ChassisSpeeds s = getRobotSpeeds();
    return Math.hypot(s.vxMetersPerSecond, s.vyMetersPerSecond);
  }

  public double rotationSpeed() {
    return getRobotSpeeds().omegaRadiansPerSecond;
  }

  /** Estimated pose. Safe to call from any thread. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return cachedPose;
  }

  public Rotation2d getRotation() {
    return cachedPose.getRotation();
  }

  /**
   * Set a hook that fires after every {@link #resetPose} call. Used by sim to keep the physics
   * chassis aligned with the estimator (otherwise auto routines that reset to a path start leave
   * the sim chassis stranded and the controller diverges). Single-slot — calling this again
   * overwrites the previous listener. Pass {@code null} to clear.
   */
  public void onPoseReset(Consumer<Pose2d> listener) {
    poseResetListener = listener;
  }

  /** Resets the robot's estimated pose. */
  public void resetPose(Pose2d pose) {
    // Sync the arc-integration accumulators with the current hardware state so the next sample
    // computes its delta from the current position. The effective accumulator can be zeroed
    // freely (only deltas matter), but its stored angle must match the current wheel direction
    // or the next sample registers a spurious delta_theta.
    SwerveModulePosition[] raw = getModulePositions();
    for (int i = 0; i < 4; i++) {
      lastModulePositions[i] = raw[i];
      effectiveModulePositions[i] = new SwerveModulePosition(0.0, raw[i].angle);
    }
    poseEstimator.resetPosition(rawGyroRotation, effectiveModulePositions, pose);
    if (poseResetListener != null) {
      poseResetListener.accept(pose);
    }
  }

  /** Feeds a vision measurement into the pose estimator. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Looks up the robot's pose at a past timestamp (FPGA seconds). */
  public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
    return poseEstimator.sampleAt(timestampSeconds);
  }

  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  /** True when the pose has crossed any field wall (no margin). Diagnostic only. */
  private static boolean isOutsideField(Pose2d pose) {
    return pose.getX() < FIELD_ESCAPE_MARGIN_M
        || pose.getX() > FieldInfo.lengthMeters() - FIELD_ESCAPE_MARGIN_M
        || pose.getY() < FIELD_ESCAPE_MARGIN_M
        || pose.getY() > FieldInfo.widthMeters() - FIELD_ESCAPE_MARGIN_M;
  }

  private static SwerveModulePosition[] newZeroedPositions() {
    return new SwerveModulePosition[] {
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition()
    };
  }

  /** Returns the (x, y) position of each swerve module, in robot frame. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }
}
