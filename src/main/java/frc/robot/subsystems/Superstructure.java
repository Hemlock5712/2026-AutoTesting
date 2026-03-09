package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterLookup;
import frc.robot.subsystems.shooter.ShooterSIM;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.spindexer.SpindexerSIM;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretSIM;
import frc.robot.utils.FieldInfo;
import frc.robot.utils.Tunables;
import frc.robot.utils.Tunables.TunableDouble;
import java.util.function.Supplier;

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
@Logged(strategy = Strategy.OPT_IN)
public class Superstructure {

  // ==================== Constants ====================

  /** Center position of the turret hole relative to robot center (meters). */
  public static final Pose3d TURRET_HOLE_CENTER =
      new Pose3d(-0.127, 0.13018, 0.3556, Rotation3d.kZero);

  /** 2D transform from robot center to turret position for field calculations. */
  public static final Transform2d TURRET_TRANSFORM =
      new Transform2d(TURRET_HOLE_CENTER.getX(), TURRET_HOLE_CENTER.getY(), Rotation2d.kZero);

  // Air drag parameters for effective TOF calculation
  private static final double AIR_DENSITY = 1.225; // kg/m^3
  private static final double BALL_MASS = 0.2268; // kg
  private static final double BALL_DIAMETER = 0.15; // m
  private static final double BALL_RADIUS = BALL_DIAMETER / 2;
  private static final double DRAG_COEFFICIENT = 0.35; // smooth foam sphere
  private static final double CROSS_SECTION = Math.PI * BALL_RADIUS * BALL_RADIUS;
  private static final double K_DRAG = 0.5 * AIR_DENSITY * DRAG_COEFFICIENT * CROSS_SECTION;
  private static final double FLYWHEEL_RADIUS = 0.0508; // meters (2 inches)

  // ==================== Subsystems ====================
  private final Shooter shooter = RobotBase.isSimulation() ? new ShooterSIM() : new Shooter();
  private final Turret turret = RobotBase.isSimulation() ? new TurretSIM() : new Turret();
  private final Spindexer spindexer =
      RobotBase.isSimulation() ? new SpindexerSIM() : new Spindexer();

  private final Supplier<SwerveDriveState> driveState;

  private final TunableDouble targetFlywheelVelocity = Tunables.value("Tuning/Flywheel", 26.0);
  private final TunableDouble targetHoodAngle = Tunables.value("Tuning/Hood", 3.0);

  // ==================== Targeting Data (calculated once per loop)
  // ====================

  private Translation2d targetPosition = FieldInfo.HUB_POSITION;
  private double distanceToHub = 0;

  // SWM state
  private Translation2d virtualTargetPosition = FieldInfo.HUB_POSITION;
  private double distanceToVirtualTarget = 0;
  private double angleToVirtualTarget = 0;

  // SWM feasibility
  private boolean swmSolutionFeasible = true;
  private boolean swmConverged = true;

  // SWM tunable: advance pose to compensate for processing/communication latency
  private final TunableDouble compDelay = Tunables.value("SWM/CompDelay", 0.03);

  private boolean isShooting = false;

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

    if (FieldInfo.flipX(robotPose.getX()) < FieldInfo.ALLIANCE_ZONE_X) {
      targetPosition = FieldInfo.flip(FieldInfo.HUB_POSITION);
    } else {
      // Compute both feed positions in current-alliance coordinates, then pick the
      // one
      // on the same side of the field (upper vs. lower Y half) as the robot.
      Translation2d feedA = FieldInfo.flip(FieldInfo.LEFT_FEED_POSITION);
      Translation2d feedB = FieldInfo.flip(FieldInfo.RIGHT_FEED_POSITION);
      Translation2d upperFeed = feedA.getY() > feedB.getY() ? feedA : feedB;
      Translation2d lowerFeed = feedA.getY() > feedB.getY() ? feedB : feedA;
      targetPosition =
          robotPose.getY() > FieldInfo.width().baseUnitMagnitude() / 2.0 ? upperFeed : lowerFeed;
    }

    Pose2d turretPose = robotPose.transformBy(TURRET_TRANSFORM);
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

    // Telemetry
    Robot.telemetry()
        .log(
            "SWM/VirtualTarget",
            new Pose2d(virtualTargetPosition, Rotation2d.kZero),
            Pose2d.struct);
    Robot.telemetry().log("SWM/DistanceDelta", distanceToVirtualTarget - distanceToHub);
    Robot.telemetry().log("SWM/VirtualTargetDist", distanceToVirtualTarget);
    Robot.telemetry().log("SWM/Feasible", swmSolutionFeasible);
    Robot.telemetry().log("SWM/Converged", swmConverged);
  }

  // ==================== Targeting Getters ====================

  @Logged
  public double getDistanceToHub() {
    return distanceToHub;
  }

  @Logged
  public Pose2d getTargetPosition() {
    return new Pose2d(targetPosition, new Rotation2d());
  }

  // ==================== SWM-Aware Getters ====================

  @Logged
  public double getHoodDistance() {
    return distanceToVirtualTarget;
  }

  @Logged
  public double getTurretAngle() {
    return angleToVirtualTarget;
  }

  @Logged
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
            Commands.sequence(
                Commands.waitUntil(() -> shooter.flywheelIsAtTarget()),
                spindexer.startCommand(),
                spindexer.startKickerVoltageCommand()));
  }

  /** Shooting sequence with SWM compensation (degrades to static when stationary). */
  public Command shoot() {
    return Commands.parallel(
        shooter.runDynamicSWM(this::getFlywheelDistance, this::getHoodDistance),
        Commands.runOnce(() -> isShooting = true),
        Commands.sequence(
            Commands.waitUntil(
                () ->
                    shooter.flywheelIsAtTarget()
                        && shooter.hoodIsAtTarget()
                        && turret.isAtTarget()
                        && swmSolutionFeasible),
            spindexer.startCommand(),
            spindexer.CommandRunKickerCommand(10)));
  }

  public Command stopShoot() {
    return Commands.sequence(
        Commands.runOnce(() -> isShooting = false),
        spindexer.stopCommand(),
        spindexer.stopKickerCommand(),
        shooter.stopCommand());
  }

  public Command spinSpinDexerBack() {
    return spindexer.backCommand();
  }

  public Command spinSpinDexerStop() {
    return spindexer.stopCommand();
  }

  private Translation2d virtualTarget(SwerveDriveState state) {
    // --- Step 1: Latency compensation ---
    // Our sensor data is slightly old by the time we use it. Predict where the
    // robot will actually be when the ball leaves the shooter by advancing the
    // pose forward in time by "delay" seconds using the current velocity.
    ChassisSpeeds fieldSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, state.Pose.getRotation());
    double delay = compDelay.get();
    Pose2d advancedPose =
        state.Pose.exp(
            new Twist2d(
                state.Speeds.vxMetersPerSecond * delay,
                state.Speeds.vyMetersPerSecond * delay,
                state.Speeds.omegaRadiansPerSecond * delay));

    Translation2d realTarget = getTargetPosition().getTranslation();
    // The turret isn't at robot center -- apply the offset to get its real position
    Pose2d turretPose = advancedPose.transformBy(TURRET_TRANSFORM);
    Translation2d robotPosition = turretPose.getTranslation();

    // --- Step 2: Calculate the turret's total velocity on the field ---
    // The turret moves because (a) the whole robot is translating and (b) the
    // turret is off-center, so robot rotation swings it in a circle (like
    // sitting on a merry-go-round). We need both parts.
    double omega = fieldSpeeds.omegaRadiansPerSecond;

    // Rotate the turret offset from robot frame into field frame
    double cos = advancedPose.getRotation().getCos();
    double sin = advancedPose.getRotation().getSin();
    double fieldOffX = TURRET_HOLE_CENTER.getX() * cos - TURRET_HOLE_CENTER.getY() * sin;
    double fieldOffY = TURRET_HOLE_CENTER.getX() * sin + TURRET_HOLE_CENTER.getY() * cos;

    // Total velocity = robot translation + omega x r (cross product gives the
    // tangential speed from spinning around the offset point)
    Translation2d velocity =
        new Translation2d(
            fieldSpeeds.vxMetersPerSecond + (-omega * fieldOffY),
            fieldSpeeds.vyMetersPerSecond + (omega * fieldOffX));

    // --- Step 3: Find the virtual target (where to actually aim) ---
    // Think of it like throwing a ball on a moving train: you aim behind your
    // target so the train's motion carries the ball to the right spot.
    // virtualTarget = where turret aims
    // realTarget    = where ball actually lands (the goal)
    // The difference is how far the ball drifts during flight due to our velocity.
    //
    // We iterate because time-of-flight depends on distance to virtualTarget,
    // but virtualTarget depends on time-of-flight. The loop finds the answer
    // where both agree (usually converges in 2-3 iterations).
    Translation2d virtualTarget = realTarget;
    Translation2d prev = virtualTarget;
    double robotSpeed = velocity.getNorm();
    swmConverged = false;

    for (int i = 0; i < 20; i++) {
      double dist = robotPosition.getDistance(virtualTarget);
      if (dist < 0.001) {
        swmConverged = true;
        break;
      }

      double tof = ShooterLookup.getToFMap().get(dist);

      // Decompose velocity into radial (along aim) and tangential (perpendicular)
      Translation2d aim = virtualTarget.minus(robotPosition).div(dist);
      double vRadialMag = velocity.getX() * aim.getX() + velocity.getY() * aim.getY();
      Translation2d vRadial = aim.times(vRadialMag);
      Translation2d vTangential = velocity.minus(vRadial);

      // Effective TOF: accounts for drag reducing lateral drift during flight
      double flywheelSpeed =
          ShooterLookup.getFlywheelMap().get(dist) * 2 * Math.PI * FLYWHEEL_RADIUS;
      double vRef = Math.sqrt(flywheelSpeed * flywheelSpeed + robotSpeed * robotSpeed);
      double beta = K_DRAG * vRef / BALL_MASS;
      double tofEff = (beta > 1e-8) ? (1.0 - Math.exp(-beta * tof)) / beta : tof;

      // Radial uses raw TOF; tangential uses effective TOF (drag reduces lateral drift)
      virtualTarget = realTarget.minus(vRadial.times(tof)).minus(vTangential.times(tofEff));

      if (virtualTarget.getDistance(prev) < 0.001) {
        swmConverged = true;
        break;
      }
      prev = virtualTarget;
    }

    // --- Step 4: Safety check ---
    // If we're driving toward the target too fast, the virtual target ends up
    // unreasonably close and the shooter can't meaningfully contribute. Reject it.
    double virtDist = robotPosition.getDistance(virtualTarget);
    swmSolutionFeasible = swmConverged && virtDist > 1;

    return virtualTarget;
  }

  @Logged
  public Angle getTargetTurretAngle() {
    return turret.getTargetAngle();
  }

  @Logged
  public Angle getTargetHoodAngle() {
    return shooter.getTargetPosition();
  }

  @Logged
  public AngularVelocity getTargetFlyhweel() {
    return shooter.getTargetVelocity();
  }

  @Logged
  public boolean isShooting() {
    return isShooting;
  }
}
