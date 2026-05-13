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
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
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

  /** Standard FRC bumper thickness (3.25 in). */
  public static final double BUMPER_THICKNESS_M = 0.0826;

  /**
   * Half robot length / width including bumpers. Shared by avoidance and path-planning inflation.
   */
  public static final double ROBOT_HALF_X =
      Math.abs(TunerConstants.FrontLeft.LocationX) + BUMPER_THICKNESS_M;

  public static final double ROBOT_HALF_Y =
      Math.abs(TunerConstants.FrontLeft.LocationY) + BUMPER_THICKNESS_M;

  /**
   * Deceleration budget the avoidance clamp's brake curve plans against. Held below {@link
   * AccelerationLimiter#MAX_FRICTION_ACCEL} so the chassis controller still has headroom to
   * actually decelerate in time — the clamp commits to 60% of the friction limit so the remaining
   * 40% covers controller lag, weight transfer, and modeling error.
   */
  public static final double AVOIDANCE_DECEL_BUDGET = 0.6 * AccelerationLimiter.MAX_FRICTION_ACCEL;

  /**
   * Buffer kept between the robot's bounding box and any obstacle edge. Sized to cover the
   * cachedPose staleness budget: pose is published from periodic() at 50 Hz but read on the 250 Hz
   * fast loop, so the clamp can act on a pose up to one 50 Hz cycle (20 ms) stale. At
   * kSpeedAt12Volts (~4.7 m/s) that's ~0.094 m of unobserved motion — 0.10 m gives ~6 mm of slack
   * over the worst case. Don't reduce without re-deriving from the staleness × max-speed product.
   */
  public static final double AVOIDANCE_SAFETY_MARGIN_M = 0.10;

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

  private static final SwerveModuleState[] EMPTY_STATES = new SwerveModuleState[0];

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
  private volatile SwerveModuleState[] latestSetpointStates =
      new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
      };
  private volatile ChassisSpeeds latestSetpointSpeeds = new ChassisSpeeds();
  private volatile double lastRunVelocityTime = -1.0;

  // --- Measured-acceleration tracking (drives AccelerationLimiter weight-transfer) ---
  // With per-module Motion Magic profiling, the chassis-level integrator that previously updated
  // AccelerationLimiter.lastAccel isn't always on the active path (Choreo follower calls
  // runVelocity directly). Estimate field-frame acceleration from the measured chassis-speed
  // delta and feed it back so perModuleAccelCaps's weight-transfer term stays honest.
  private ChassisSpeeds lastMeasuredFieldSpeeds = new ChassisSpeeds();
  private double lastMeasuredFieldSpeedsTime = -1.0;

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
      latestSetpointStates = EMPTY_STATES;
    }
    lastSeenDisabled = isDisabled;

    updateOdometry();

    cachedPose = poseEstimator.getEstimatedPosition();
    cachedRobotSpeeds = kinematics.toChassisSpeeds(getModuleStates());

    // Feed AccelerationLimiter with a measured field-frame acceleration estimate so its
    // weight-transfer term tracks reality even when no caller is running the chassis-level
    // integrator (e.g. Choreo follower calls runVelocity directly).
    ChassisSpeeds measuredFieldSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(cachedRobotSpeeds, getRotation());
    double now = Timer.getFPGATimestamp();
    if (lastMeasuredFieldSpeedsTime > 0) {
      double dt = now - lastMeasuredFieldSpeedsTime;
      if (dt > 1e-6) {
        AccelerationLimiter.setLastAcceleration(
            (measuredFieldSpeeds.vxMetersPerSecond - lastMeasuredFieldSpeeds.vxMetersPerSecond)
                / dt,
            (measuredFieldSpeeds.vyMetersPerSecond - lastMeasuredFieldSpeeds.vyMetersPerSecond)
                / dt,
            (measuredFieldSpeeds.omegaRadiansPerSecond
                    - lastMeasuredFieldSpeeds.omegaRadiansPerSecond)
                / dt);
      }
    }
    lastMeasuredFieldSpeeds = measuredFieldSpeeds;
    lastMeasuredFieldSpeedsTime = now;

    logState();

    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.getMode() != Mode.SIM);
  }

  /**
   * Logs everything a CTRE-style {@code SwerveDriveState} carries, all under one {@code Drive/*}
   * tree. Single source of truth for "what's the drivetrain doing right now" — pose, speeds, module
   * states, targets, and diagnostics. Add new drive-related logs here so they stay in one place
   * instead of leaking into top-level namespaces.
   */
  private void logState() {
    Logger.recordOutput("Drive/Pose", cachedPose);
    Logger.recordOutput("Drive/RawHeading", rawGyroRotation);
    Logger.recordOutput("Drive/Speeds", cachedRobotSpeeds);
    Logger.recordOutput("Drive/FieldSpeeds", getFieldSpeeds());
    Logger.recordOutput("Drive/TranslationSpeedMps", translationSpeed());
    Logger.recordOutput("Drive/RotationSpeedRadPerSec", rotationSpeed());
    Logger.recordOutput("Drive/ModuleStates", getModuleStates());
    Logger.recordOutput("Drive/ModulePositions", getModulePositions());

    Logger.recordOutput("Drive/ModuleTargets", latestSetpointStates);
    Logger.recordOutput("Drive/SetpointSpeeds", latestSetpointSpeeds);

    Logger.recordOutput("Drive/Diagnostics/ArcIntegrateRejections", arcIntegrateRejections);
    Logger.recordOutput(
        "Drive/Diagnostics/FrictionRatios", AccelerationLimiter.getLastModuleFrictionRatios());
  }

  /**
   * SIM-only: logs the MapleSim ground-truth pose alongside the estimator's pose and their
   * difference. Driver-visible answer to "where the robot thinks it is" vs "where the robot
   * actually is" — useful for tuning slip/skid models and (later) vision.
   *
   * <p>Call from {@code Robot#simulationPeriodic} (or wherever the physics tick lives).
   */
  public void updateSimulationGroundTruth(Pose2d truthPose) {
    Pose2d est = cachedPose;
    double dx = est.getX() - truthPose.getX();
    double dy = est.getY() - truthPose.getY();
    Logger.recordOutput("Drive/Sim/GroundTruthPose", truthPose);
    Logger.recordOutput("Drive/Sim/PoseErrorMeters", Math.hypot(dx, dy));
    Logger.recordOutput(
        "Drive/Sim/HeadingErrorRad", est.getRotation().minus(truthPose.getRotation()).getRadians());
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
    double now = Timer.getFPGATimestamp();
    double dt = (lastRunVelocityTime < 0) ? 0.02 : (now - lastRunVelocityTime);
    lastRunVelocityTime = now;
    if (dt < 1e-6) dt = 0.02;

    // 1. Chassis-Level Slip Limiting (Prevent macro-slip)
    // Scale the entire vector back if it exceeds the friction circle, keeping kinematics perfectly
    // locked.
    ChassisSpeeds limitedSpeeds = new ChassisSpeeds();
    AccelerationLimiter.integrateVelocity(
        limitedSpeeds,
        latestSetpointSpeeds.vxMetersPerSecond,
        latestSetpointSpeeds.vyMetersPerSecond,
        latestSetpointSpeeds.omegaRadiansPerSecond,
        speeds.vxMetersPerSecond,
        speeds.vyMetersPerSecond,
        speeds.omegaRadiansPerSecond,
        dt);

    SwerveModuleState[] setpointStates =
        kinematics.toSwerveModuleStates(limitedSpeeds, centerOfRotation);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

    // 2. Motion Magic Interpolation (Smooth out 50Hz/250Hz steps)
    for (int i = 0; i < 4; i++) {
      double deltaV =
          Math.abs(
              setpointStates[i].speedMetersPerSecond
                  - latestSetpointStates[i].speedMetersPerSecond);

      // Compute exact acceleration required to reach the target in dt seconds.
      // We clamp to a minimum of 1.0 m/s^2 so the profiler doesn't freeze on very tiny adjustments,
      // but otherwise it acts perfectly as a linear interpolator bridging the discrete loops!
      double accelCap = Math.max(deltaV / dt, 1.0);

      modules[i].runSetpoint(setpointStates[i], accelCap);
    }

    // Save the latest setpoints for the next loop's interpolation delta, and for logging.
    latestSetpointStates = setpointStates;
    latestSetpointSpeeds = limitedSpeeds;
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
      // Drive setpoint is zero — no slip budget needed; pass infinity to bypass profiling.
      modules[i].runSetpoint(states[i], Double.POSITIVE_INFINITY);
    }
    latestSetpointStates = states;
    latestSetpointSpeeds = new ChassisSpeeds();
  }

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
    // Publish synchronously so the 250 Hz fast loop sees the reset pose before the next
    // periodic() tick. Without this, a fast-loop reader between resetPose() and the next
    // periodic() would see the previous estimator pose (or Pose2d.kZero at boot — which sits on
    // a perimeter obstacle and would cause the avoidance clamp to fire spuriously).
    cachedPose = pose;
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
