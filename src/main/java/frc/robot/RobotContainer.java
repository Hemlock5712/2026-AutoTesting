package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autonomous.AutoSelector;
import frc.robot.commands.PathPlannerAutos;
import frc.robot.commands.TeleopDrive;
import frc.robot.generated.TunerConstants;
import frc.robot.simlib.SimWorldSetup;
import frc.robot.simlib.SimulatedArena;
import frc.robot.simlib.drivesims.SwerveDriveSimulation;
import frc.robot.simlib.motorsims.SimulatedBattery;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DrivePhysics;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIONoop;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.utils.DriverInput;
import frc.robot.utils.FieldInfo;
import frc.robot.utils.path.ObstacleAvoidance;
import frc.robot.utils.path.ObstacleVisualizer;
import java.util.List;

public class RobotContainer {

  // ==================== Constants ====================

  // Mid-field spawn on the y-centerline, ~2.6 m from either Hub edge.
  private static final Pose2d SIM_SPAWN_POSE = new Pose2d(8.27, 4.0, Rotation2d.kZero);

  private static final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(1).in(RadiansPerSecond);

  // -------------------- Subsystems --------------------

  public final Drive drivetrain;
  public final Vision vision;

  // -------------------- Driver controls --------------------

  private final CommandXboxController driver = new CommandXboxController(0);

  // -------------------- World state --------------------

  /** Built once and shared by the avoidance clamp, PathPlanner's pathfinder, and the visualizer. */
  private final List<Pair<Translation2d, Translation2d>> obstacles;

  /** SIM only: dyn4j physics handle for ticking the simulated arena. */
  private final SwerveDriveSimulation driveSimulation;

  // -------------------- Autonomous --------------------

  private final AutoSelector autoSelector;

  // ==================== Construction ====================

  public RobotContainer() {
    obstacles = Field2026Obstacles.build();
    driveSimulation = createDriveSimulation();
    drivetrain = createDrive(driveSimulation);
    vision = createVision();
    if (driveSimulation != null) {
      setupSimWorld(driveSimulation, obstacles);
    }
    // Seed the pose estimator unconditionally so REPLAY reproduces the original run's
    // absolute poses, not just its trajectory shape. In REAL this gets overwritten by
    // autonomousInit's path-start reset; in SIM the sim-world reset callback (registered
    // above by setupSimWorld) snaps the chassis to match.
    drivetrain.resetPose(SIM_SPAWN_POSE);
    // Inflate by the same amount PathPlanner's pathfinder uses (robot half-extent + margin) so
    // the visualized clearance matches what the planner actually sees. For a symmetric chassis
    // ROBOT_HALF_X == ROBOT_HALF_Y so a single scalar captures both axes.
    ObstacleVisualizer.log(
        "World/Obstacles", obstacles, Drive.ROBOT_HALF_X + DrivePhysics.PATH_INFLATION_MARGIN_M);
    // Wire AutoBuilder + push static obstacles into PP's pathfinder. Done once at startup so
    // pathfindToPose works immediately on driver-A press.
    PathPlannerAutos.configure(drivetrain, obstacles);
    autoSelector = new AutoSelector(drivetrain);

    configureBindings();
  }

  // ==================== Button Bindings ====================

  private void configureBindings() {
    drivetrain.setDefaultCommand(buildTeleopDrive());

    // A button: PathPlanner on-the-fly pathfind from current pose to AutoSelector.DEMO_GOAL
    // (alliance-flipped at trigger time by ExtPose.get()), avoiding the static obstacles
    // registered in PathPlannerAutos.configure. STUDENT: replace AutoSelector.DEMO_GOAL with
    // your scoring target (always blue-origin).
    driver
        .a()
        .onTrue(PathPlannerAutos.pathfindToPose(drivetrain, AutoSelector.DEMO_GOAL.get()));
  }

  // ==================== Public hooks called from Robot.java ====================

  public Command getAutonomousCommand() {
    return autoSelector.getSelected();
  }

  /** Tick MapleSim physics. Called from {@link Robot#simulationPeriodic}. No-op outside SIM. */
  public void updateSimulation() {
    if (driveSimulation == null) return;
    SimulatedArena.getInstance().simulationPeriodic();
    Pose2d truthPose = driveSimulation.getSimulatedDriveTrainPose();
    drivetrain.updateSimulationGroundTruth(truthPose);
    // One sim-world update per loop drives all PhotonVision cameras from a consistent pose.
    VisionIOPhotonVisionSim.update(truthPose);
  }

  // ==================== Teleop drive (default command) ====================

  private TeleopDrive buildTeleopDrive() {
    double[] vel = {0, 0};
    TeleopDrive teleop =
        new TeleopDrive(
            drivetrain,
            () -> {
              Translation2d t =
                  DriverInput.rescaleTranslation(driver.getLeftY(), driver.getLeftX());
              double sign = FieldInfo.shouldFlip() ? 1.0 : -1.0;
              vel[0] = sign * t.getX() * MAX_SPEED;
              vel[1] = sign * t.getY() * MAX_SPEED;
              return vel[0];
            },
            () -> vel[1],
            () -> -DriverInput.deadband(driver.getRightX()) * MAX_ANGULAR_RATE);

    // Pose-based safety clamp + right-bumper bypass. Active in every mode (obstacles are built
    // unconditionally so the avoidance brake works on the real robot too).
    teleop.withObstacleAvoidance(
        new ObstacleAvoidance(
            obstacles,
            Drive.ROBOT_HALF_X,
            Drive.ROBOT_HALF_Y,
            0.6 * DrivePhysics.MAX_FRICTION_ACCEL,
            DrivePhysics.PATH_INFLATION_MARGIN_M,
            "Drive/Avoidance"));
    teleop.withAvoidanceOverride(driver.getHID()::getRightBumperButton);
    return teleop;
  }

  // ==================== Subsystem factories (REAL / SIM / REPLAY) ====================

  private static SwerveDriveSimulation createDriveSimulation() {
    if (Constants.getMode() != Constants.Mode.SIM) return null;
    SwerveDriveSimulation sim =
        new SwerveDriveSimulation(Drive.getMapleSimConfig(), SIM_SPAWN_POSE);
    SimulatedArena.getInstance().addDriveTrainSimulation(sim);
    // After module sims register as electrical appliances, neutralize MapleSim's battery sim —
    // its LinearFilter gets poisoned by NaN under heavy current draw and produces a brownout /
    // Rotation2d-zero cascade that corrupts the chassis state.
    SimulatedBattery.disableBatterySim();
    return sim;
  }

  private static Drive createDrive(SwerveDriveSimulation sim) {
    return switch (Constants.getMode()) {
      case REAL ->
          new Drive(
              new GyroIOPigeon2(),
              new ModuleIOTalonFX(TunerConstants.FrontLeft),
              new ModuleIOTalonFX(TunerConstants.FrontRight),
              new ModuleIOTalonFX(TunerConstants.BackLeft),
              new ModuleIOTalonFX(TunerConstants.BackRight));
      case SIM ->
          new Drive(
              new GyroIOSim(sim.getGyroSimulation()),
              new ModuleIOSim(sim.getModules()[0]),
              new ModuleIOSim(sim.getModules()[1]),
              new ModuleIOSim(sim.getModules()[2]),
              new ModuleIOSim(sim.getModules()[3]));
      case REPLAY ->
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
    };
  }

  private Vision createVision() {
    return switch (Constants.getMode()) {
      case REAL -> {
        VisionIO[] ios = new VisionIO[VisionConstants.LIMELIGHT_NAMES.length];
        for (int i = 0; i < ios.length; i++) {
          ios[i] = new VisionIOLimelight(VisionConstants.LIMELIGHT_NAMES[i]);
        }
        yield new Vision(drivetrain, ios);
      }
      case SIM -> {
        // To disable simulated vision (expose pure-odometry drift), swap the inner constructor
        // for `new VisionIONoop(name)`.
        VisionIO[] ios = new VisionIO[VisionConstants.PHOTON_CAMERA_NAMES.length];
        for (int i = 0; i < ios.length; i++) {
          ios[i] =
              new VisionIOPhotonVisionSim(
                  VisionConstants.PHOTON_CAMERA_NAMES[i],
                  VisionConstants.PHOTON_CAMERA_TRANSFORMS[i]);
        }
        yield new Vision(drivetrain, ios);
      }
      case REPLAY -> {
        // Each camera name from REAL and SIM maps to a VisionIONoop so AKit replays the wpilog
        // entries faithfully. Names not in the log produce no observation.
        VisionIO[] ios =
            new VisionIO
                [VisionConstants.LIMELIGHT_NAMES.length
                    + VisionConstants.PHOTON_CAMERA_NAMES.length];
        int idx = 0;
        for (String n : VisionConstants.LIMELIGHT_NAMES) ios[idx++] = new VisionIONoop(n);
        for (String n : VisionConstants.PHOTON_CAMERA_NAMES) ios[idx++] = new VisionIONoop(n);
        yield new Vision(drivetrain, ios);
      }
    };
  }

  /**
   * SIM only: registers the obstacle list with the dyn4j physics sim and wires up the sim chassis
   * to track future estimator resets (including the one in the constructor immediately after this
   * returns). Validates that {@link #SIM_SPAWN_POSE} is in free space — a spawn inside an obstacle
   * would freeze the avoidance clamp and bewilder the student debugging it.
   */
  private void setupSimWorld(
      SwerveDriveSimulation sim, List<Pair<Translation2d, Translation2d>> field) {
    double sx = SIM_SPAWN_POSE.getX();
    double sy = SIM_SPAWN_POSE.getY();
    double spawnClearance = minSignedDistance(field, sx, sy);
    if (spawnClearance < DrivePhysics.PATH_INFLATION_MARGIN_M) {
      DriverStation.reportError(
          "SIM_SPAWN_POSE ("
              + sx
              + ", "
              + sy
              + ") is inside or too close to an obstacle (signedDistance="
              + spawnClearance
              + " m). The avoidance clamp will zero teleop velocities until the robot is moved."
              + " Edit SIM_SPAWN_POSE in RobotContainer.java to a clear area.",
          false);
    }
    SimWorldSetup.addObstacles(SimulatedArena.getInstance(), field);
    drivetrain.onPoseReset(sim::setSimulationWorldPose);
  }

  /** Distance from (x, y) to the nearest obstacle (negative inside). */
  private static double minSignedDistance(
      List<Pair<Translation2d, Translation2d>> obstacles, double x, double y) {
    double best = Double.POSITIVE_INFINITY;
    for (Pair<Translation2d, Translation2d> box : obstacles) {
      Translation2d min = box.getFirst();
      Translation2d max = box.getSecond();
      double outX = Math.max(min.getX() - x, x - max.getX());
      double outY = Math.max(min.getY() - y, y - max.getY());
      double d;
      if (outX > 0.0 || outY > 0.0) {
        d = Math.hypot(Math.max(outX, 0.0), Math.max(outY, 0.0));
      } else {
        // Inside this AABB; distance is negative (the smaller of the per-axis penetrations).
        d = Math.max(outX, outY);
      }
      if (d < best) best = d;
    }
    return best;
  }
}
