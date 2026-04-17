package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.BallTrajectorySimulator;
import frc.robot.utils.FieldInfo;
import frc.robot.utils.Tunables;
import frc.robot.utils.Tunables.TunableDouble;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Simulates multiple ball trajectories in flight and publishes to NetworkTables for visualization.
 *
 * <p>This subsystem only runs in simulation mode - it is only created if RobotBase.isSimulation()
 * is true.
 *
 * <p>Features:
 *
 * <ul>
 *   <li>Tracks multiple balls in flight simultaneously
 *   <li>Configurable shooting rate (balls per second)
 *   <li>Integrates with hopper to check ball availability
 *   <li>Detects goal hits and removes balls
 *   <li>Removes balls when they hit the ground
 * </ul>
 */
public class BallPhysicsSimulation extends SubsystemBase {
  // Ball properties (game-specific, passed to simulator)
  public static final double BALL_MASS_KG = 0.2268; // 0.5 lbs
  public static final double BALL_DIAMETER_M = 0.15; // 150 mm
  public static final double DRAG_COEFFICIENT = 0.5; // Smooth foam sphere

  // Launch geometry defaults
  private static final double LAUNCH_HEIGHT_M = 0.4826; // 19 inches
  private static final double DEFAULT_FLYWHEEL_RADIUS_M = Inches.of(2).in(Meters);
  private static final double DEFAULT_MAX_SIM_TIME = 10.0; // seconds (longer for multiple balls)
  private static final double DEFAULT_SIM_TIMESTEP = 0.01; // seconds

  // Goal region (hub)
  private static final double GOAL_RADIUS_M = 0.4; // Approximate goal radius in meters
  private static final double GOAL_HEIGHT_TOLERANCE_M = 0.3; // Vertical tolerance for goal hit

  private final CommandSwerveDrivetrain drivetrain;
  private final Superstructure superstructure;
  private final BallTrajectorySimulator simulator;

  // Cached tunable values
  private double launchHeight = LAUNCH_HEIGHT_M;
  private double flywheelRadius = DEFAULT_FLYWHEEL_RADIUS_M;
  private double maxSimTime = DEFAULT_MAX_SIM_TIME;
  private double simTimestep = DEFAULT_SIM_TIMESTEP;
  private TunableDouble ballsPerSecondTunable = Tunables.value("BallPhysics/BallsPerSecond", 8.0);

  // Active balls in flight
  private static class BallInFlight {
    final List<Pose3d> trajectory;
    double currentTime;
    int currentIndex;

    BallInFlight(List<Pose3d> fullTrajectory) {
      this.trajectory = new ArrayList<>(fullTrajectory);
      this.currentTime = 0.0;
      this.currentIndex = 0;
    }
  }

  private final List<BallInFlight> activeBalls = new ArrayList<>();
  private double timeSinceLastShot = 0.0;
  private final double robotPeriodicTime = 0.02; // ~50Hz robot periodic

  // Goal position
  private final Translation3d goalPosition;

  // Trajectory visualization - all points visited by any ball
  private final List<Pose3d> ballTrajectory = new ArrayList<>();

  public BallPhysicsSimulation(CommandSwerveDrivetrain drivetrain, Superstructure superstructure) {
    this.drivetrain = drivetrain;
    this.superstructure = superstructure;

    // Create physics simulator with ball properties
    this.simulator = new BallTrajectorySimulator(BALL_MASS_KG, BALL_DIAMETER_M, DRAG_COEFFICIENT);

    // Goal position from FieldConstants
    goalPosition =
        new Translation3d(
            FieldInfo.HUB_POSITION.getX(),
            FieldInfo.HUB_POSITION.getY(),
            FieldInfo.HUB_HEIGHT.in(Meters));
  }

  @Override
  public void simulationPeriodic() {

    // Read tunable values from NetworkTables
    launchHeight = Superstructure.TURRET_HOLE_CENTER.getMeasureZ().in(Meters);
    flywheelRadius = Inches.of(2).in(Meters);
    maxSimTime = 10.0;
    simTimestep = 0.02;
    boolean shootingEnabled = superstructure.isShooting();
    double ballsPerSecond = ballsPerSecondTunable.get();

    // Update active balls (advance time, check for ground/goal hits)
    updateActiveBalls();

    // Shoot new balls if enabled
    if (shootingEnabled && ballsPerSecond > 0.0) {
      timeSinceLastShot += robotPeriodicTime;
      double timeBetweenShots = 1.0 / ballsPerSecond;

      // Check if we should shoot a new ball
      if (timeSinceLastShot >= timeBetweenShots) {
        shootBall();
        timeSinceLastShot = 0.0;
      }
    } else {
      timeSinceLastShot = 0.0;
    }

    // Update trajectory visualization
    updateTrajectoryVisualization();
  }

  /**
   * Updates all active balls, advancing their trajectories and removing ones that hit ground/goal.
   */
  private void updateActiveBalls() {
    Iterator<BallInFlight> iterator = activeBalls.iterator();
    while (iterator.hasNext()) {
      BallInFlight ball = iterator.next();

      // Advance ball time
      ball.currentTime += robotPeriodicTime;

      // Find current position in trajectory based on time
      // Trajectory points are spaced by simTimestep
      int targetIndex = (int) (ball.currentTime / simTimestep);
      if (targetIndex >= ball.trajectory.size()) {
        // Ball has exceeded trajectory length (shouldn't happen, but safety check)
        iterator.remove();
        continue;
      }

      ball.currentIndex = targetIndex;

      // Check if ball hit ground (z <= 0)
      Pose3d currentPose = ball.trajectory.get(ball.currentIndex);
      if (currentPose.getTranslation().getZ() <= 0.0) {
        iterator.remove();
        continue;
      }

      // Check if ball hit goal
      if (isBallInGoal(currentPose.getTranslation())) {
        iterator.remove();
        continue;
      }

      // Check if ball exceeded max time
      if (ball.currentTime >= maxSimTime) {
        iterator.remove();
        continue;
      }
    }
  }

  /** Updates the trajectory visualization to show current positions of all active balls. */
  private void updateTrajectoryVisualization() {
    // Clear previous positions
    ballTrajectory.clear();

    // Add current position of each active ball
    for (BallInFlight ball : activeBalls) {
      if (ball.currentIndex >= 0 && ball.currentIndex < ball.trajectory.size()) {
        ballTrajectory.add(ball.trajectory.get(ball.currentIndex));
      }
    }

    Logger.recordOutput(
        "BallPhysicsSimulation/BallTrajectory", ballTrajectory.toArray(new Pose3d[0]));
  }

  /**
   * Checks if a ball position is within the goal region.
   *
   * @param ballPos Ball position in field coordinates
   * @return true if ball is in goal region
   */
  private boolean isBallInGoal(Translation3d ballPos) {
    // Check horizontal distance to goal center
    double dx = ballPos.getX() - goalPosition.getX();
    double dy = ballPos.getY() - goalPosition.getY();
    double horizontalDist = Math.sqrt(dx * dx + dy * dy);

    if (horizontalDist > GOAL_RADIUS_M) {
      return false;
    }

    // Check vertical position (within tolerance of goal height)
    double dz = Math.abs(ballPos.getZ() - goalPosition.getZ());
    return dz <= GOAL_HEIGHT_TOLERANCE_M;
  }

  /** Shoots a new ball if hopper has balls available. */
  private void shootBall() {
    // Calculate launch conditions
    Translation3d launchPos = getLaunchPosition();
    Translation3d launchVel = getLaunchVelocity();

    // Simulate full trajectory
    List<Pose3d> trajectory = simulator.simulate(launchPos, launchVel, simTimestep, maxSimTime);

    // Create new ball in flight
    BallInFlight newBall = new BallInFlight(trajectory);
    activeBalls.add(newBall);
  }

  /**
   * Calculates the launch position in field coordinates.
   *
   * @return Launch position as Translation3d
   */
  private Translation3d getLaunchPosition() {
    // Launch point is at robot center (0, 0) in X-Y, but at launchHeight above
    // robot center
    // Get robot pose
    var robotPose = drivetrain.getPose();
    var turretPose = robotPose.transformBy(Superstructure.TURRET_TRANSFORM);

    // Transform to field coordinates
    // Since launch is at robot center, X and Y are just the robot's position
    // Height is absolute (above ground), not relative to robot orientation
    double fieldX = turretPose.getX();
    double fieldY = turretPose.getY();
    double fieldZ = launchHeight;

    return new Translation3d(fieldX, fieldY, fieldZ);
  }

  /**
   * Calculates the launch velocity vector in field coordinates.
   *
   * @return Launch velocity as Translation3d (m/s)
   */
  private Translation3d getLaunchVelocity() {
    // Get flywheel speed in rotations per second
    double flywheelRPS = superstructure.getTargetFlywheel().in(RotationsPerSecond);

    // Scale down speed in sim. Scales down more as the flywheel speed is higher.
    // At 50 RPS, the speed is scaled down to 0.65.
    // At 20 RPS, the speed is not scaled down at all (factor 1.0).
    // Solve for scaleDownFactor = m * flywheelRPS + b
    // 0.65 = m * 50 + b
    // 1.0 = m * 20 + b
    // m = (0.65-1.0)/(50-20) = -0.35/30 = -0.011666...
    // b = 1.0 - m * 20 = 1.0 - (-0.011666...)*20 = 1.233333...
    double scaleDownFactor = (-0.35 / 30.0) * flywheelRPS + 1.2333333;
    flywheelRPS *= scaleDownFactor;

    // Convert to linear velocity at flywheel radius
    // v = ω * r = (RPS * 2π) * r
    double flywheelLinearVel = flywheelRPS * 2.0 * Math.PI * flywheelRadius;

    // Get turret angle (relative to robot forward)
    double turretAngleRad = superstructure.getTargetTurretAngle().in(Radians);

    // Calculate launch direction in robot frame
    // Turret angle is yaw (rotation around Z axis)
    // Hood angle is pitch (rotation around Y axis)
    // In robot frame: +X is forward, +Y is left, +Z is up

    // First, apply turret yaw (rotation around Z)
    // This rotates the velocity in the X-Y plane
    double cosYaw = Math.cos(turretAngleRad);
    double sinYaw = Math.sin(turretAngleRad);

    // Then apply hood pitch (rotation around Y)
    double hoodAngleRad = Degrees.of(75).minus(superstructure.getTargetHoodAngle()).in(Radians);
    // This tilts the velocity up/down
    double cosPitch = Math.cos(hoodAngleRad);
    double sinPitch = Math.sin(hoodAngleRad);

    // In robot frame:
    // Base direction is forward (+X) before turret rotation
    // After turret rotation: (cos(yaw), sin(yaw), 0) in X-Y plane
    // After pitch rotation: (cos(yaw)*cos(pitch), sin(yaw)*cos(pitch), sin(pitch))
    double robotVelX = flywheelLinearVel * cosYaw * cosPitch;
    double robotVelY = flywheelLinearVel * sinYaw * cosPitch;
    double robotVelZ = flywheelLinearVel * sinPitch;

    // Get turret velocity (not robot center) and add it to launch velocity.
    // The ball exits from the turret, which has additional tangential velocity
    // when the robot rotates: v_turret = v_center + omega x r_{center->turret}.
    var robotSpeeds = drivetrain.getRobotSpeeds();
    Translation2d turretOffset =
        Superstructure.TURRET_TRANSFORM.getTranslation().rotateBy(drivetrain.getRotation());
    double omega = robotSpeeds.omegaRadiansPerSecond;
    robotVelX += robotSpeeds.vxMetersPerSecond - omega * turretOffset.getY();
    robotVelY += robotSpeeds.vyMetersPerSecond + omega * turretOffset.getX();

    // Transform to field coordinates
    Rotation2d robotRotation = drivetrain.getRotation();
    double cosRobot = robotRotation.getCos();
    double sinRobot = robotRotation.getSin();

    // Rotate velocity vector by robot rotation
    // Field X = robot X * cos(θ) - robot Y * sin(θ)
    // Field Y = robot X * sin(θ) + robot Y * cos(θ)
    double fieldVelX = robotVelX * cosRobot - robotVelY * sinRobot;
    double fieldVelY = robotVelX * sinRobot + robotVelY * cosRobot;
    double fieldVelZ = robotVelZ; // Z doesn't rotate

    return new Translation3d(fieldVelX, fieldVelY, fieldVelZ);
  }

  /** Clears all active balls from the simulation and clears the trajectory visualization. */
  public void clearAllBalls() {
    activeBalls.clear();
    ballTrajectory.clear();
    timeSinceLastShot = 0.0;
  }
}
