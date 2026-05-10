package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autonomous.AutoCommands;
import frc.robot.commands.PathPlanningDemo;
import frc.robot.commands.TeleopDrive;
import frc.robot.generated.TunerConstants;
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
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIONoop;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.utils.path.AutoPath;
import java.util.Map;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
  private static final double JOYSTICK_DEADBAND = 0.05;

  private static final String[] LIMELIGHT_NAMES = {
    "limelight-br", "limelight-bl", "limelight-fl", "limelight-fr", "limelight-mm"
  };

  private double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double maxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond);

  private final CommandXboxController joystick = new CommandXboxController(0);

  // MapleSim spawn pose — clear of the field perimeter so the rigid-body sim doesn't start
  // wedged against a wall. The estimator is reset to match in the constructor.
  private static final Pose2d SIM_SPAWN_POSE = new Pose2d(8.0, 4.0, Rotation2d.kZero);

  public final Drive drivetrain;
  public final Vision vision;
  public final AutoCommands autoCommands;

  // Non-null only in SIM. Held so Robot.simulationPeriodic can tick the arena and so vision can
  // read ground truth.
  private final SwerveDriveSimulation driveSimulation;

  private final SendableChooser<Supplier<Command>> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    driveSimulation = createDriveSimulation();
    drivetrain = createDrive(driveSimulation);
    vision = createVision(drivetrain, driveSimulation);
    autoCommands = new AutoCommands(drivetrain);

    if (driveSimulation != null) {
      // Keep the sim chassis aligned with any future estimator resets (e.g. auto routines that
      // reset to a path-start pose) — otherwise the sim chassis stays stranded and the
      // controller diverges. Then align the estimator with the spawn pose.
      drivetrain.onPoseReset(driveSimulation::setSimulationWorldPose);
      drivetrain.resetPose(SIM_SPAWN_POSE);
    }

    autoChooser.setDefaultOption("NewPath (PD)", this::newPathAutoPD);
    autoChooser.addOption("PathPlanningDemo", () -> PathPlanningDemo.create(drivetrain));
    SmartDashboard.putData("Auto Mode", autoChooser);

    configureBindings();
  }

  /** Tick MapleSim physics. Called from {@link Robot#simulationPeriodic}. No-op outside SIM. */
  public void updateSimulation() {
    if (driveSimulation == null) return;
    SimulatedArena.getInstance().simulationPeriodic();
    Pose2d truth = driveSimulation.getSimulatedDriveTrainPose();
    Logger.recordOutput("FieldSimulation/RobotPose", truth);
    // Per-tick odometry error for A/B comparing the friction limiter against ground truth.
    Pose2d est = drivetrain.getPose();
    double dx = est.getX() - truth.getX();
    double dy = est.getY() - truth.getY();
    Logger.recordOutput("FieldSimulation/OdomErrorMeters", Math.hypot(dx, dy));
    Logger.recordOutput(
        "FieldSimulation/OdomHeadingErrorRad",
        est.getRotation().minus(truth.getRotation()).getRadians());
  }

  private static SwerveDriveSimulation createDriveSimulation() {
    if (Constants.getMode() != Constants.Mode.SIM) return null;
    SwerveDriveSimulation sim =
        new SwerveDriveSimulation(Drive.getMapleSimConfig(), SIM_SPAWN_POSE);
    SimulatedArena.getInstance().addDriveTrainSimulation(sim);
    // After all module sims have registered as electrical appliances, neutralize MapleSim's
    // battery sim. Its LinearFilter gets poisoned by NaN under heavy current draw and produces
    // a brownout/Rotation2d-zero cascade that corrupts the chassis state. YAGSL's vendored
    // ironmaple snapshot exposes this as a public API.
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

  private static Vision createVision(Drive drive, SwerveDriveSimulation sim) {
    VisionIO[] ios = new VisionIO[LIMELIGHT_NAMES.length];
    switch (Constants.getMode()) {
      case REAL -> {
        for (int i = 0; i < LIMELIGHT_NAMES.length; i++) {
          ios[i] = new VisionIOLimelight(LIMELIGHT_NAMES[i]);
        }
      }
      case SIM -> {
        // One synthetic camera reading the simulated truth pose. Other slots stay no-op so
        // std-dev tuning has a single, measurable input to reason about.
        ios[0] = new VisionIOSim(LIMELIGHT_NAMES[0], sim::getSimulatedDriveTrainPose);
        for (int i = 1; i < LIMELIGHT_NAMES.length; i++) {
          ios[i] = new VisionIONoop(LIMELIGHT_NAMES[i]);
        }
      }
      case REPLAY -> {
        for (int i = 0; i < LIMELIGHT_NAMES.length; i++) {
          ios[i] = new VisionIONoop(LIMELIGHT_NAMES[i]);
        }
      }
    }
    return new Vision(drive, ios);
  }

  private void configureBindings() {
    double[] translationVel = {0, 0};

    drivetrain.setDefaultCommand(
        new TeleopDrive(
            drivetrain,
            () -> {
              double[] scaled = rescaleTranslation(joystick.getLeftY(), joystick.getLeftX());
              translationVel[0] = -scaled[0] * maxSpeed;
              translationVel[1] = -scaled[1] * maxSpeed;
              return translationVel[0];
            },
            () -> translationVel[1],
            () -> -rescaleInputs(joystick.getRightX()) * maxAngularRate));
  }

  public Command getAutonomousCommand() {
    Supplier<Command> selected = autoChooser.getSelected();
    return (selected != null) ? selected.get() : Commands.none();
  }

  public double rescaleInputs(double input) {
    return MathUtil.applyDeadband(input, JOYSTICK_DEADBAND);
  }

  private final double[] scaledTranslation = new double[2];

  public double[] rescaleTranslation(double x, double y) {
    double mag = Math.hypot(x, y);
    if (mag < JOYSTICK_DEADBAND) {
      scaledTranslation[0] = 0;
      scaledTranslation[1] = 0;
      return scaledTranslation;
    }
    double deadbanded = (mag - JOYSTICK_DEADBAND) / (1.0 - JOYSTICK_DEADBAND);
    if (deadbanded > 1.0) deadbanded = 1.0;
    double factor = deadbanded * deadbanded / mag;
    scaledTranslation[0] = x * factor;
    scaledTranslation[1] = y * factor;
    return scaledTranslation;
  }

  private Command newPathAutoPD() {
    return Commands.sequence(
        autoCommands.resetPose(AutoPath.NEW_PATH),
        autoCommands.followPathWithActions(
            AutoPath.NEW_PATH.get(),
            autoCommands.actionsFromChoreoEvents(
                AutoPath.NEW_PATH, Map.of("Marker", () -> Commands.print("Marker triggered!")))));
  }
}
