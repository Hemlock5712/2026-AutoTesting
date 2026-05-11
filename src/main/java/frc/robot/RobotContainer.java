package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autonomous.AutoCommands;
import frc.robot.autonomous.AutoSelector;
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

  // -------------------- SIM-only state --------------------

  private final SwerveDriveSimulation driveSimulation;
  private final ObstacleField simObstacleField;

  // -------------------- Autonomous --------------------

  private final AutoSelector autoSelector;

  // ==================== Construction ====================

  public RobotContainer() {
    driveSimulation = createDriveSimulation();
    drivetrain = createDrive(driveSimulation);
    vision = createVision();
    autoCommands = new AutoCommands(drivetrain);
    simObstacleField = setupSimWorld(driveSimulation);
    autoSelector = new AutoSelector(drivetrain, autoCommands);

    configureBindings();
  }

  // ==================== Button Bindings ====================

  private void configureBindings() {
    drivetrain.setDefaultCommand(buildTeleopDrive());

    // Add new bindings below. Examples:
    //   driver.a().onTrue(...);
    //   driver.b().whileTrue(...);
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

    // Pose-based safety clamp + right-bumper bypass. SIM-only until the real-robot path supplies
    // an obstacle field.
    if (simObstacleField != null) {
      teleop.withObstacleAvoidance(
          new ObstacleAvoidance(
              simObstacleField,
              Footprint.fixed(Drive.ROBOT_HALF_X, Drive.ROBOT_HALF_Y),
              Drive.AVOIDANCE_DECEL_BUDGET,
              Drive.AVOIDANCE_SAFETY_MARGIN_M,
              "Drive/Sim/Avoidance"));
      teleop.withAvoidanceOverride(driver.getHID()::getRightBumperButton);
    }
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
        // Both name sets registered as no-ops so logs from either source replay correctly.
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
   * SIM only: builds the field obstacle set, registers it with the dyn4j sim, logs it for
   * AdvantageScope, and aligns the sim chassis with the spawn pose. Returns the field for the
   * teleop avoidance clamp; {@code null} outside SIM.
   */
  private ObstacleField setupSimWorld(SwerveDriveSimulation sim) {
    if (sim == null) return null;
    ObstacleField field = Field2026Obstacles.build();
    SimWorldSetup.addObstacles(SimulatedArena.getInstance(), field);
    ObstacleVisualizer.log(
        "SimWorld/Obstacles", field, Math.hypot(Drive.ROBOT_HALF_X, Drive.ROBOT_HALF_Y));
    // Keep the sim chassis aligned with any future estimator resets (e.g. auto routines that
    // reset to a path-start pose), then align the estimator with the spawn pose.
    drivetrain.onPoseReset(sim::setSimulationWorldPose);
    drivetrain.resetPose(SIM_SPAWN_POSE);
    return field;
  }
}
