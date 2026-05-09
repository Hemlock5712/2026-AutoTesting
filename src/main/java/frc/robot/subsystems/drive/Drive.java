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

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
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
import frc.robot.generated.TunerConstants;
import frc.robot.utils.FieldInfo;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
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

  // Chassis params for MapleSim physics. Tune to the real bot once we have a measurement.
  private static final double ROBOT_MASS_KG = 74.088;
  private static final double WHEEL_COF = 1.2;

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
   * #setHighRateController} and unplug in {@code end()}.
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
  private final List<Consumer<Pose2d>> poseResetListeners = new CopyOnWriteArrayList<>();

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

  // --- Fast-loop hook ---
  private final Notifier highRateNotifier = new Notifier(this::tickHighRate);
  private volatile DoubleConsumer highRateController = null;
  private double highRateLastTime = 0.0;

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

    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

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

    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    updateOdometry();

    cachedPose = poseEstimator.getEstimatedPosition();
    if (isOutsideField(cachedPose)) fieldEscapeHits++;
    Logger.recordOutput("Drive/FieldEscapeHits", fieldEscapeHits);
    cachedRobotSpeeds = kinematics.toChassisSpeeds(getModuleStates());

    logSkidMetrics();
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
   * magnitude as the distance and atan2(dy, dx) as the direction.
   */
  private static SwerveModulePosition arcIntegrate(
      SwerveModulePosition rawCurrent, SwerveModulePosition rawLast) {
    double deltaS = rawCurrent.distanceMeters - rawLast.distanceMeters;
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
    Rotation2d thetaEff = (dEff < 1.0e-9) ? rawCurrent.angle : new Rotation2d(dx, dy);
    return new SwerveModulePosition(dEff, thetaEff);
  }

  /**
   * Skid metrics. Uses gyro-measured omega when available — kinematics-derived omega is itself
   * corrupted by skid, so feeding it back here would mask exactly what we want to detect.
   */
  private void logSkidMetrics() {
    double omega =
        gyroInputs.connected
            ? gyroInputs.yawVelocityRadPerSec
            : cachedRobotSpeeds.omegaRadiansPerSecond;
    SkidDetection.Result skid =
        SkidDetection.compute(getModuleStates(), getModuleTranslations(), omega);
    Logger.recordOutput("Drive/Skid/PerModuleTranslation", skid.perModuleTranslation());
    Logger.recordOutput("Drive/Skid/MeanTranslation", skid.meanTranslation());
    Logger.recordOutput("Drive/Skid/MaxMagnitude", skid.maxMagnitude());
    Logger.recordOutput("Drive/Skid/MinMagnitude", skid.minMagnitude());
    Logger.recordOutput("Drive/Skid/MaxOverMinRatio", skid.maxOverMinRatio());
    Logger.recordOutput("Drive/Skid/MagnitudeStdDev", skid.magnitudeStdDev());
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
   * Plugs in a 250 Hz callback. The callback gets elapsed seconds since the last tick. Replaces any
   * previous callback.
   */
  public void setHighRateController(DoubleConsumer controller) {
    highRateController = controller;
  }

  /** Unplugs the fast-loop callback. Call from a command's {@code end()}. */
  public void clearHighRateController() {
    highRateController = null;
  }

  /**
   * Drive at the given robot-relative chassis speeds. Can be called from the main loop or the 250
   * Hz fast loop.
   */
  public void runVelocity(ChassisSpeeds speeds) {
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, Constants.LOOP_PERIOD_SECONDS);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
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
   * Register a hook that fires every time {@link #resetPose} is called. Used by sim to keep the
   * physics chassis aligned with the estimator (otherwise auto routines that reset to a path start
   * leave the sim chassis stranded and the controller diverges).
   */
  public void onPoseReset(Consumer<Pose2d> listener) {
    poseResetListeners.add(listener);
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
    for (var listener : poseResetListeners) {
      listener.accept(pose);
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
