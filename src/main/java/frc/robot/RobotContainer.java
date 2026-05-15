package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autonomous.AutoCommands;
import frc.robot.autonomous.AutoSelector;
import frc.robot.commands.DriveToWithAvoidance;
import frc.robot.commands.TeleopDrive;
import frc.robot.generated.TunerConstants;
import frc.robot.simlib.SimWorldSetup;
import frc.robot.simlib.SimulatedArena;
import frc.robot.simlib.drivesims.SwerveDriveSimulation;
import frc.robot.simlib.motorsims.SimulatedBattery;
import frc.robot.subsystems.drive.Drive;
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
import frc.robot.utils.path.Footprint;
import frc.robot.utils.path.ObstacleAvoidance;
import frc.robot.utils.path.ObstacleField;
import frc.robot.utils.path.ObstacleVisualizer;

public class RobotContainer {

  // ==================== Constants ====================

  // Mid-field spawn on the y-centerline, ~2.6 m from either Hub edge.
  private static final Pose2d SIM_SPAWN_POSE = new Pose2d(8.27, 4.0, Rotation2d.kZero);

  private static final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(1).in(RadiansPerSecond);

  // -------------------- Subsystems --------------------

  public final Drive drivetrain;
  public final Vision vision;
  public final AutoCommands autoCommands;

  // -------------------- Driver controls --------------------

  private final CommandXboxController driver = new CommandXboxController(0);

  // -------------------- World state --------------------

  /** Built once and shared by the avoidance clamp, the path planner, and the visualizer. */
  private final ObstacleField obstacleField;

  /** SIM only: dyn4j physics handle for ticking the simulated arena. */
  private final SwerveDriveSimulation driveSimulation;

  // -------------------- Autonomous --------------------

  private final AutoSelector autoSelector;

  // ==================== Construction ====================

  public RobotContainer() {
    obstacleField = Field2026Obstacles.build();
    driveSimulation = createDriveSimulation();
    drivetrain = createDrive(driveSimulation);
    vision = createVision();
    autoCommands = new AutoCommands(drivetrain);
    if (driveSimulation != null) {
      setupSimWorld(driveSimulation, obstacleField);
    }
    // Seed the pose estimator unconditionally so REPLAY reproduces the original run's
    // absolute poses, not just its trajectory shape. In REAL this gets overwritten by
    // autonomousInit's path-start reset; in SIM the sim-world reset callback (registered
    // above by setupSimWorld) snaps the chassis to match.
    drivetrain.resetPose(SIM_SPAWN_POSE);
    ObstacleVisualizer.log(
        "World/Obstacles", obstacleField, Math.hypot(Drive.ROBOT_HALF_X, Drive.ROBOT_HALF_Y));
    autoSelector = new AutoSelector(drivetrain, autoCommands, obstacleField);

    configureBindings();
  }

  // ==================== Button Bindings ====================

  private void configureBindings() {
    drivetrain.setDefaultCommand(buildTeleopDrive());

    // A button: plan a path from the current pose to AutoSelector.DEMO_GOAL (alliance-flipped at
    // trigger time by ExtPose.get()) around the field obstacles, then drive it.
    // STUDENT: replace AutoSelector.DEMO_GOAL with your scoring target (always blue-origin).
    driver
        .a()
        .onTrue(
            DriveToWithAvoidance.create(drivetrain, AutoSelector.DEMO_GOAL::get, obstacleField));
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

    // Pose-based safety clamp + right-bumper bypass. Active in every mode (the obstacle field is
    // built unconditionally so the avoidance brake works on the real robot too).
    teleop.withObstacleAvoidance(
        new ObstacleAvoidance(
            obstacleField,
            Footprint.fixed(Drive.ROBOT_HALF_X, Drive.ROBOT_HALF_Y),
            Drive.AVOIDANCE_DECEL_BUDGET,
            Drive.AVOIDANCE_SAFETY_MARGIN_M,
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
   * SIM only: registers the obstacle field with the dyn4j physics sim and wires up the sim chassis
   * to track future estimator resets (including the one in the constructor immediately after this
   * returns). Validates that {@link #SIM_SPAWN_POSE} is in free space — a spawn inside an obstacle
   * would freeze the avoidance clamp and bewilder the student debugging it.
   */
  private void setupSimWorld(SwerveDriveSimulation sim, ObstacleField field) {
    double spawnClearance = field.signedDistance(SIM_SPAWN_POSE.getX(), SIM_SPAWN_POSE.getY());
    if (spawnClearance < Drive.AVOIDANCE_SAFETY_MARGIN_M) {
      DriverStation.reportError(
          "SIM_SPAWN_POSE ("
              + SIM_SPAWN_POSE.getX()
              + ", "
              + SIM_SPAWN_POSE.getY()
              + ") is inside or too close to an obstacle (signedDistance="
              + spawnClearance
              + " m). The avoidance clamp will zero teleop velocities until the robot is moved."
              + " Edit SIM_SPAWN_POSE in RobotContainer.java to a clear area.",
          false);
    }
    SimWorldSetup.addObstacles(SimulatedArena.getInstance(), field);
    drivetrain.onPoseReset(sim::setSimulationWorldPose);
  }
}
