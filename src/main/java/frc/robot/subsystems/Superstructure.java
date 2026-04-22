package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AccelerationLimiter;
import frc.robot.commands.JamProtectedShoot;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterLookup;
import frc.robot.subsystems.shooter.ShooterSIM;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretSIM;
import frc.robot.utils.FeedTargetSelector;
import frc.robot.utils.FieldInfo;
import frc.robot.utils.Tunables;
import frc.robot.utils.Tunables.TunableDouble;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Superstructure - Controls the Arm and Flywheel together.
 *
 * <p>This coordinates:
 *
 * <ul>
 *   <li>Arm - Moves horizontal and vertical to position game pieces
 *   <li>Flywheel - Spins the shooter wheels at the right speed
 * </ul>
 *
 * <p>Instead of controlling the arm and flywheel separately, this gives you simple commands like
 * "score low" or "prepare for shooting" that move both parts together. This makes driving easier
 * and ensures everything moves in sync.
 */
public class Superstructure {

  public enum FeedMode {
    AUTO,
    FORCE_LEFT,
    FORCE_RIGHT
  }

  // ==================== Constants ====================

  /** Center position of the turret hole relative to robot center (meters). */
  public static final Pose3d TURRET_HOLE_CENTER =
      new Pose3d(-0.127, 0.13018, 0.3556, Rotation3d.kZero);

  /** 2D transform from robot center to turret position for field calculations. */
  public static final Transform2d TURRET_TRANSFORM =
      new Transform2d(TURRET_HOLE_CENTER.getX(), TURRET_HOLE_CENTER.getY(), Rotation2d.kZero);

  // Air drag parameters for effective TOF calculation (ball properties from
  // BallPhysicsSimulation)
  private static final double AIR_DENSITY = 1.225; // kg/m^3
  private static final double BALL_RADIUS = BallPhysicsSimulation.BALL_DIAMETER_M / 2;
  private static final double CROSS_SECTION = Math.PI * BALL_RADIUS * BALL_RADIUS;
  private static final double K_DRAG =
      0.5 * AIR_DENSITY * BallPhysicsSimulation.DRAG_COEFFICIENT * CROSS_SECTION;

  // ==================== Subsystems ====================
  private final Shooter shooter = RobotBase.isSimulation() ? new ShooterSIM() : new Shooter();

  private final Turret turret = RobotBase.isSimulation() ? new TurretSIM() : new Turret();

  private final Hopper hopper = new Hopper();

  private final Supplier<SwerveDriveState> driveState;

  private final TunableDouble targetFlywheelVelocity = Tunables.value("Tuning/Flywheel", 26.0);
  private final TunableDouble targetHoodAngle = Tunables.value("Tuning/Hood", 3.0);
  private final TunableDouble swmPoseDelay = Tunables.value("SWM/PoseDelay", 0.02);

  // ==================== Targeting Data (calculated once per loop)
  // ====================

  private Translation2d targetPosition = FieldInfo.HUB_POSITION;
  private double distanceToHub = 0;
  private Pose2d turretPose = Pose2d.kZero.transformBy(TURRET_TRANSFORM);

  // SWM state
  private Translation2d virtualTargetPosition = FieldInfo.HUB_POSITION;
  private double distanceToVirtualTarget = 0;
  private double angleToVirtualTarget = 0;

  // SWM feasibility
  private boolean swmSolutionFeasible = true;
  private boolean swmConverged = true;
  private double swmDelay = 0;

  private boolean isHubShot = true;

  public boolean isHubShot() {
    return isHubShot;
  }

  private boolean isShooting = false;
  @AutoLogOutput private boolean isAutoShootEnabled = false;
  private FeedMode teleopFeedMode = FeedMode.AUTO;

  /** When non-null, overrides targetPosition in update(). Blue alliance coordinates. */
  private Translation2d passTargetOverride = null;

  private FeedTargetSelector.FeedSelection feedSelection = null;

  // ==================== Constructor ====================

  public Superstructure(Supplier<SwerveDriveState> driveState) {
    this.driveState = driveState;
    // Set turret tracking as default command - uses SWM-aware getters for seamless
    // mode switching
    turret.setDefaultCommand(turret.trackHubCommand(this::getTurretAngle));
    shooter.setDefaultCommand(shooter.runHoodDynamic(this::getHoodDistance));
  }

  // ==================== Periodic ====================

  public void update() {
    // Calculate targeting data once per loop (used by turret tracking and shooter)
    SwerveDriveState state = driveState.get();
    Pose2d robotPose = state.Pose;
    turretPose = robotPose.transformBy(TURRET_TRANSFORM);

    if (passTargetOverride != null) {
      targetPosition = FieldInfo.flip(passTargetOverride);
      isHubShot = false;
      feedSelection = null;
    } else {
      isHubShot = FieldInfo.flipX(robotPose.getX()) < FieldInfo.ALLIANCE_ZONE_X;
      if (isHubShot) {
        targetPosition = FieldInfo.flip(FieldInfo.HUB_POSITION);
        feedSelection = null;
      } else {
        feedSelection = resolveFeedTarget(robotPose);
        targetPosition = feedSelection.resolvedTarget();
      }
    }

    shooter.setInAllianceZone(isUnderaTrench());

    Translation2d toTarget = targetPosition.minus(turretPose.getTranslation());

    distanceToHub = toTarget.getNorm();

    // Calculate SWM targeting values
    virtualTargetPosition = virtualTarget(state);
    Translation2d toVirtualTarget = virtualTargetPosition.minus(turretPose.getTranslation());
    distanceToVirtualTarget = toVirtualTarget.getNorm();
    Rotation2d angleToVirtualTargetField = toVirtualTarget.getAngle();
    angleToVirtualTarget =
        MathUtil.inputModulus(
            angleToVirtualTargetField.minus(robotPose.getRotation()).getRotations(), -0.25, 0.75);

    logTelemetry(state);
  }

  // ==================== Targeting Getters ====================

  @AutoLogOutput
  public double getDistanceToHub() {
    return distanceToHub;
  }

  @AutoLogOutput
  public Pose2d getTargetPosition() {
    return new Pose2d(targetPosition, new Rotation2d());
  }

  // ==================== SWM-Aware Getters ====================

  @AutoLogOutput
  public double getHoodDistance() {
    return distanceToVirtualTarget;
  }

  @AutoLogOutput
  public double getTurretAngle() {
    return angleToVirtualTarget;
  }

  @AutoLogOutput
  public double getFlywheelDistance() {
    return distanceToVirtualTarget;
  }

  // ==================== Coordinated Commands ====================

  /** Tuning mode: override flywheel/hood with dashboard tunables. */
  public Command tuningShoot() {
    return shooter
        .runShooterTestMode(
            () -> targetFlywheelVelocity.get(), () -> Degrees.of(targetHoodAngle.get()))
        .alongWith(Commands.runOnce(() -> isShooting = true))
        .alongWith(
            Commands.sequence(Commands.waitUntil(() -> shooter.isAtTarget()), hopper.start()));
  }

  /** Core shoot logic: runs shooter, then feeds when ready. */
  private Command shootSequence(Command shooterCommand, BooleanSupplier readyToFeed) {
    return Commands.parallel(
        shooterCommand,
        Commands.runOnce(() -> isShooting = true),
        Commands.either(hopper.start(), hopper.stop(), readyToFeed).repeatedly());
  }

  public Command spinUpShooter() {
    return shooter.runDynamicSWM(this::getFlywheelDistance, this::getHoodDistance);
  }

  public Command prerollShooter(double rps) {
    return Commands.runOnce(() -> shooter.setVelocity(rps), shooter);
  }

  /** Hub shot with SWM compensation and jam protection. */
  public Command hubShoot() {
    return shootSequenceWithJamProtection(
        shooter.runDynamicSWM(this::getFlywheelDistance, this::getHoodDistance), this::isHubReady);
  }

  /** Hub shot with SWM compensation and jam protection. */

  /** Feed shot with jam protection — wider tolerance, uses feed lookup maps. */
  public Command feedShoot() {
    return shootSequenceWithJamProtection(
        shooter.runDynamicFeed(this::getFlywheelDistance, this::getHoodDistance),
        this::isFeedReady);
  }

  /** Manual shooting at fixed distance — fallback when vision is unavailable. */
  public Command shootManual() {
    return Commands.parallel(
        Commands.run(() -> shooter.setForDistance(3.4)),
        turret.trackHubCommand(() -> 0.0),
        Commands.sequence(
            Commands.runOnce(() -> isShooting = true),
            Commands.either(hopper.start(), hopper.stop(), () -> isFeedReady()).repeatedly()));
  }

  private boolean isHubReady() {
    // return true;
    boolean turretTarget = turret.isAtTarget(distanceToVirtualTarget);
    boolean shootTarget = shooter.isAtTarget(distanceToVirtualTarget);

    Logger.recordOutput("SWM/IsTurretTarget", turretTarget);
    Logger.recordOutput("SWM/IsshootTarget", shootTarget);

    return shootTarget && turretTarget && swmSolutionFeasible;
  }

  private boolean isFeedReady() {
    return turret.isNotFlipping()
        && shooter.isAtFeedTarget(distanceToVirtualTarget)
        && swmSolutionFeasible
        && (feedSelection == null || !feedSelection.blocked());
  }

  /** Selects hub shot or feed shot based on field position. */
  public Command shoot() {
    return Commands.either(
        hubShoot(),
        feedShoot(),
        () -> FieldInfo.flipX(driveState.get().Pose.getX()) < FieldInfo.ALLIANCE_ZONE_X);
  }

  /** Core shoot logic with jam protection: runs shooter, then feeds with jam detection. */
  private Command shootSequenceWithJamProtection(
      Command shooterCommand, BooleanSupplier readyToFeed) {
    return Commands.parallel(
        shooterCommand,
        Commands.runOnce(() -> isShooting = true),
        new JamProtectedShoot(hopper, readyToFeed));
  }

  /** Hub shot with SWM compensation and jam protection. */
  public Command jamProtectedHubShoot() {
    return shootSequenceWithJamProtection(
        shooter.runDynamicSWM(this::getFlywheelDistance, this::getHoodDistance), this::isHubReady);
  }

  /** Feed shot with jam protection. */
  public Command jamProtectedFeedShoot() {
    return shootSequenceWithJamProtection(
        shooter.runDynamicFeed(this::getFlywheelDistance, this::getHoodDistance),
        this::isFeedReady);
  }

  /** Selects hub shot or feed shot based on field position, with jam protection. */
  public Command jamProtectedShoot() {
    return Commands.either(
        jamProtectedHubShoot(),
        jamProtectedFeedShoot(),
        () -> FieldInfo.flipX(driveState.get().Pose.getX()) < FieldInfo.ALLIANCE_ZONE_X);
  }

  /**
   * Long-running auto-shoot mode toggled by the driver. Shooting activates only in valid zones and
   * stops when leaving them. Run alongside TurretDrive in RobotContainer.
   */
  public Command autoShootMode() {
    return Commands.sequence(
            Commands.runOnce(() -> isAutoShootEnabled = true),
            Commands.sequence(
                    Commands.waitUntil(this::shouldShoot),
                    shoot().until(() -> !shouldShoot()),
                    stopShoot())
                .repeatedly())
        .finallyDo(
            () -> {
              isAutoShootEnabled = false;
              isShooting = false;
              shooter.stopMotors();
              hopper.setVelocity(RotationsPerSecond.of(0), RotationsPerSecond.of(0));
            });
  }

  public Command stopShoot() {
    return Commands.sequence(
        Commands.runOnce(() -> isShooting = false), hopper.stop(), shooter.stopCommand());
  }

  /**
   * Autonomous pass to a specific field location. Bypasses ALL checks (zone, convergence, speed
   * readiness) and fires immediately. Uses feed lookup tables for long-range passes. SWM
   * compensation still applies so the ball reaches the target while the robot moves.
   *
   * @param blueAllianceTarget Target position in blue alliance coordinates (auto-flipped)
   * @return Command that aims and fires at the target, cleaning up on end/interrupt
   */
  public Command passToLocation(Translation2d blueAllianceTarget) {
    return Commands.parallel(
            shooter.runDynamicFeed(this::getFlywheelDistance, this::getHoodDistance),
            Commands.runOnce(
                () -> {
                  passTargetOverride = blueAllianceTarget;
                  isShooting = true;
                }),
            hopper.start())
        .finallyDo(
            () -> {
              passTargetOverride = null;
              isShooting = false;
            });
  }

  public Command stopSpindexer() {
    return hopper.stop();
  }

  // Hopper commands
  public Command startHopper() {
    return hopper.start();
  }

  public Command startHopperSlow() {
    return hopper.startSlow();
  }

  public Command stopHopper() {
    return hopper.stop();
  }

  public Command recoverHopper() {
    return Commands.runOnce(
        () -> hopper.setVelocity(RotationsPerSecond.of(-30), RotationsPerSecond.of(-30)));
  }

  private Translation2d virtualTarget(SwerveDriveState state) {
    // --- Step 1: Latency compensation ---
    // Our sensor data is slightly old by the time we use it. Predict where the
    // robot will actually be when the ball leaves the shooter by advancing the
    // pose forward in time by "delay" seconds using the current velocity.
    double delay = (Utils.getCurrentTimeSeconds() - state.Timestamp) + swmPoseDelay.get();
    swmDelay = delay;
    Pose2d advancedPose =
        state.Pose.exp(
            new Twist2d(
                state.Speeds.vxMetersPerSecond * delay,
                state.Speeds.vyMetersPerSecond * delay,
                state.Speeds.omegaRadiansPerSecond * delay));
    // Use advanced pose rotation so field speeds are consistent with the
    // turret-offset rotation computed in Step 2.
    ChassisSpeeds fieldSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, advancedPose.getRotation());

    Translation2d realTarget = getTargetPosition().getTranslation();
    // The turret isn't at robot center -- apply the offset to get its real position
    Pose2d turretPose = advancedPose.transformBy(TURRET_TRANSFORM);
    Logger.recordOutput("SWM/TurretPose", turretPose);
    Translation2d robotPosition = turretPose.getTranslation();

    // --- Step 2: Turret velocity on the field ---
    // The ball exits from the turret, not the robot center. state.Speeds reports
    // robot-center velocity, so apply the rigid-body correction: v_turret =
    // v_center + omega x r_{center->turret}.
    Translation2d turretOffsetField =
        TURRET_TRANSFORM.getTranslation().rotateBy(advancedPose.getRotation());
    double omega = fieldSpeeds.omegaRadiansPerSecond;
    Translation2d velocity =
        new Translation2d(
            fieldSpeeds.vxMetersPerSecond - omega * turretOffsetField.getY(),
            fieldSpeeds.vyMetersPerSecond + omega * turretOffsetField.getX());
    Logger.recordOutput("SWM/TurretVelocity", velocity.getNorm());

    // --- Step 2b: Predict velocity at ball-release time ---
    // v_predicted = v_now + a * delay
    // The pose is already advanced by "delay"; advance velocity by the same amount
    // so the virtual target accounts for acceleration, not just constant velocity.
    ChassisSpeeds lastAccel = AccelerationLimiter.getLastAcceleration();
    velocity =
        velocity.plus(
            new Translation2d(lastAccel.vxMetersPerSecond, lastAccel.vyMetersPerSecond)
                .times(delay));

    // --- Step 3: Find the virtual target (where to actually aim) ---
    // Think of it like throwing a ball on a moving train: you aim behind your
    // target so the train's motion carries the ball to the right spot.
    // virtualTarget = where turret aims
    // realTarget = where ball actually lands (the goal)
    // The difference is how far the ball drifts during flight due to our velocity.
    //
    // We iterate because time-of-flight depends on distance to virtualTarget,
    // but virtualTarget depends on time-of-flight. The loop finds the answer
    // where both agree (usually converges in 2-3 iterations).
    double maxRange = isHubShot ? 5.5 : 9.5;
    Translation2d virtualTarget = realTarget;
    Translation2d prev = virtualTarget;
    swmConverged = false;

    for (int i = 0; i < 20; i++) {
      double dist = robotPosition.getDistance(virtualTarget);
      if (dist < 0.001) {
        swmConverged = true;
        break;
      }

      // Clamp distance to lookup table range to prevent extrapolation
      double lookupDist = Math.min(dist, maxRange);
      double tof =
          isHubShot
              ? ShooterLookup.getToFMap().get(lookupDist)
              : ShooterLookup.getFeedTimeMap().get(lookupDist);

      // Decompose velocity into radial (along aim) and tangential (perpendicular)
      Translation2d aim = virtualTarget.minus(robotPosition).div(dist);
      double vRadialMag = velocity.dot(aim);
      Translation2d vRadial = aim.times(vRadialMag);
      Translation2d vTangential = velocity.minus(vRadial);

      double ballRadialSpeed = dist / tof;
      double effectiveRadialSpeed = ballRadialSpeed + vRadialMag;

      // Ball can't outrun the robot — fall back to aiming at real target
      if (effectiveRadialSpeed < 0.5) {
        swmConverged = false;
        break;
      }

      // Radial drift uses raw tof — the inherited radial velocity is a small
      // perturbation on the ball's own airspeed, so drag on it is second-order.
      // Tangential drift needs explicit drag correction (tofEff) because the
      // inherited velocity IS the entire tangential airspeed.
      double vTangentialMag = vTangential.getNorm();
      double vRef =
          Math.sqrt(effectiveRadialSpeed * effectiveRadialSpeed + vTangentialMag * vTangentialMag);
      double beta = K_DRAG * vRef / BallPhysicsSimulation.BALL_MASS_KG;
      double tofEff = (beta > 1e-8) ? (1.0 - Math.exp(-beta * tof)) / beta : tof;

      virtualTarget = realTarget.minus(vRadial.times(tof)).minus(vTangential.times(tofEff));

      if (virtualTarget.getDistance(prev) < 0.001) {
        swmConverged = true;
        break;
      }
      prev = virtualTarget;
    }

    // --- Step 4: Safety check ---
    // Reject if the virtual target is unreasonably close (shooter can't contribute)
    // or beyond our lookup table range (extrapolated values are unreliable).
    double virtDist = robotPosition.getDistance(virtualTarget);
    swmSolutionFeasible = swmConverged && virtDist > 1 && virtDist <= maxRange;

    return virtualTarget;
  }

  @AutoLogOutput
  public Angle getTargetTurretAngle() {
    return turret.getTargetAngle();
  }

  @AutoLogOutput
  public Angle getTargetHoodAngle() {
    return shooter.getTargetPosition();
  }

  @AutoLogOutput
  public AngularVelocity getTargetFlywheel() {
    return shooter.getTargetVelocity();
  }

  @AutoLogOutput
  public boolean isShooting() {
    return isShooting;
  }

  public void setTeleopFeedMode(FeedMode mode) {
    teleopFeedMode = mode;
  }

  public FeedMode getTeleopFeedMode() {
    return teleopFeedMode;
  }

  @AutoLogOutput
  public boolean isInNeutralZone() {
    return FieldInfo.isInNeutralZone(turretPose);
  }

  @AutoLogOutput
  public boolean isUnderaTrench() {
    return FieldInfo.isUnderaTrench(
        turretPose.getTranslation(),
        ChassisSpeeds.fromRobotRelativeSpeeds(
                driveState.get().Speeds, driveState.get().Pose.getRotation())
            .vxMetersPerSecond);
  }

  @AutoLogOutput
  public boolean isInAllianceZone() {
    return FieldInfo.isInAllianceZone(turretPose);
  }

  @AutoLogOutput
  public boolean isInNeutralZoneDeadzone() {
    return FieldInfo.isInNeutralZoneDeadzone(turretPose);
  }

  @AutoLogOutput
  public boolean isUnderTower() {
    return FieldInfo.isUnderTower(turretPose);
  }

  @AutoLogOutput
  public boolean shouldShoot() {
    if (isInAllianceZone() && !isUnderTower()) {
      return true;
    }
    if (isInNeutralZone()) {
      return feedSelection == null || !feedSelection.blocked();
    }
    return false;
  }

  public boolean isAutoShootEnabled() {
    return isAutoShootEnabled;
  }

  private FeedTargetSelector.FeedSelection resolveFeedTarget(Pose2d robotPose) {
    Translation2d leftFeedTarget =
        DriverStation.isAutonomous()
            ? FieldInfo.LEFT_FEED_POSITION_AUTO.get()
            : FieldInfo.LEFT_FEED_POSITION.get();
    Translation2d rightFeedTarget =
        DriverStation.isAutonomous()
            ? FieldInfo.RIGHT_FEED_POSITION_AUTO.get()
            : FieldInfo.RIGHT_FEED_POSITION.get();
    return DriverStation.isAutonomous()
        ? FeedTargetSelector.selectAutoTarget(
            robotPose.getTranslation(), leftFeedTarget, rightFeedTarget)
        : FeedTargetSelector.selectTeleopTarget(
            teleopFeedMode,
            robotPose.getTranslation(),
            turretPose.getTranslation(),
            leftFeedTarget,
            rightFeedTarget,
            FieldInfo.flip(FieldInfo.netLineCenter()));
  }

  private void logTelemetry(SwerveDriveState state) {
    // SWM state
    Logger.recordOutput("SWM/VirtualTarget", new Pose2d(virtualTargetPosition, Rotation2d.kZero));
    Logger.recordOutput("SWM/DistanceDelta", distanceToVirtualTarget - distanceToHub);
    Logger.recordOutput("SWM/VirtualTargetDist", distanceToVirtualTarget);
    Logger.recordOutput("SWM/Feasible", swmSolutionFeasible);
    Logger.recordOutput("SWM/Converged", swmConverged);
    Logger.recordOutput("SWM/Delay", swmDelay);
    Logger.recordOutput(
        "SWM/OdometryAge_ms", (Utils.getCurrentTimeSeconds() - state.Timestamp) * 1000.0);
    Logger.recordOutput("SWM/TotalDelay_ms", swmDelay * 1000.0);

    // Shoot readiness
    boolean shootReady = isShooting && (isHubShot ? isHubReady() : isFeedReady());
    Logger.recordOutput("SWM/ShootReady", shootReady);
    Logger.recordOutput("SWM/IsHubShot", isHubShot);
    Logger.recordOutput("SWM/IsTurretAtTarget", turret.isAtTarget(distanceToHub));
    Logger.recordOutput("SWM/FeedMode", teleopFeedMode.name());

    // Feed selection
    if (feedSelection != null) {
      Logger.recordOutput(
          "SWM/PreferredFeedTarget", new Pose2d(feedSelection.preferredTarget(), Rotation2d.kZero));
      Logger.recordOutput(
          "SWM/ResolvedFeedTarget", new Pose2d(feedSelection.resolvedTarget(), Rotation2d.kZero));
      Logger.recordOutput("SWM/ResolvedFeedOffsetMeters", feedSelection.offset().in(Meters));
      Logger.recordOutput("SWM/FeedPathBlocked", feedSelection.blocked());
      Logger.recordOutput("SWM/FeedHubClearanceMeters", feedSelection.clearance().in(Meters));
      Logger.recordOutput("SWM/FeedResolvedSide", feedSelection.side().name());
    } else {
      Logger.recordOutput("SWM/FeedResolvedSide", passTargetOverride != null ? "OVERRIDE" : "NONE");
      Logger.recordOutput("SWM/FeedPathBlocked", false);
    }

    // Target geometry
    Translation2d hub2d = FieldInfo.flip(FieldInfo.HUB_POSITION);
    Logger.recordOutput("SWM/HubPose2d", new Pose2d(hub2d, Rotation2d.kZero));
    Logger.recordOutput(
        "SWM/HubPose3d",
        new Pose3d(hub2d.getX(), hub2d.getY(), FieldInfo.HUB_HEIGHT.in(Meters), Rotation3d.kZero));
    Logger.recordOutput("SWM/EndGoalPose2d", new Pose2d(targetPosition, Rotation2d.kZero));
    Logger.recordOutput(
        "SWM/EndGoalPose3d",
        new Pose3d(targetPosition.getX(), targetPosition.getY(), 0.0, Rotation3d.kZero));
  }

  public void sethood() {
    shooter.setPosition(Rotations.of(0));
  }
}
