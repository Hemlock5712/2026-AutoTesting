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
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.generated.TunerConstants;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
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

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = Rotation2d.kZero;
  private SwerveModulePosition[] lastModulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private final SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, Pose2d.kZero);

  // Snapshots so the 250 Hz fast loop can read pose/speeds safely from another thread.
  // Updated each periodic() tick.
  private volatile Pose2d cachedPose = Pose2d.kZero;

  private volatile ChassisSpeeds cachedRobotSpeeds = new ChassisSpeeds();

  /**
   * 250 Hz fast loop. Used by commands that need to apply physics limits more often than the normal
   * 50 Hz robot loop (e.g. the acceleration limiter).
   *
   * <p>Commands plug in their callback in {@code initialize()} via {@link #setHighRateController}
   * and unplug in {@code end()}. Each call gets the time elapsed since the last tick.
   */
  private static final double HIGH_RATE_PERIOD_S = 0.004;

  private final Notifier highRateNotifier = new Notifier(this::tickHighRate);
  private volatile java.util.function.DoubleConsumer highRateController = null;
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

    // Update odometry from all the samples since the last loop (250Hz real, 50Hz sim/replay).
    double[] sampleTimestamps = modules[0].getOdometryTimestamps();
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      if (gyroInputs.connected) {
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    // Update the cached snapshots so the fast loop can read them without locks.
    cachedPose = poseEstimator.getEstimatedPosition();
    cachedRobotSpeeds = kinematics.toChassisSpeeds(getModuleStates());

    // Log the latest setpoint. Logging happens here (the main loop) so AKit's logger stays
    // single-threaded.
    SwerveModuleState[] sp = latestSetpointStates;
    if (sp.length > 0) {
      Logger.recordOutput("SwerveStates/Setpoints", sp);
      Logger.recordOutput("SwerveStates/SetpointsOptimized", sp);
      Logger.recordOutput("SwerveChassisSpeeds/Setpoints", latestSetpointSpeeds);
    }

    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.getMode() != Mode.SIM);
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
  public void setHighRateController(java.util.function.DoubleConsumer controller) {
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

  // Most recent setpoint, written by runVelocity (from any thread) and logged by periodic.
  private volatile SwerveModuleState[] latestSetpointStates = new SwerveModuleState[0];

  private volatile ChassisSpeeds latestSetpointSpeeds = new ChassisSpeeds();

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

  /** Resets the robot's estimated pose. */
  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
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
