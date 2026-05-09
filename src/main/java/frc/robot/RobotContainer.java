package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autonomous.AutoCommands;
import frc.robot.commands.OrbitDrive;
import frc.robot.commands.PathPlanningDemo;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
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

public class RobotContainer {
  private static final double JOYSTICK_DEADBAND = 0.05;

  private static final String[] LIMELIGHT_NAMES = {
    "limelight-br", "limelight-bl", "limelight-fl", "limelight-fr", "limelight-mm"
  };

  private double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double maxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond);

  private final CommandXboxController joystick = new CommandXboxController(0);

  public final Drive drivetrain;
  public final Vision vision;
  public final AutoCommands autoCommands;

  private final SendableChooser<Supplier<Command>> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    drivetrain = createDrive();
    vision = createVision(drivetrain);
    autoCommands = new AutoCommands(drivetrain);

    autoChooser.setDefaultOption("NewPath (PD)", this::newPathAutoPD);
    autoChooser.addOption("PathPlanningDemo", () -> PathPlanningDemo.create(drivetrain));
    SmartDashboard.putData("Auto Mode", autoChooser);

    configureBindings();
  }

  private static Drive createDrive() {
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
              new GyroIO() {},
              new ModuleIOSim(TunerConstants.FrontLeft),
              new ModuleIOSim(TunerConstants.FrontRight),
              new ModuleIOSim(TunerConstants.BackLeft),
              new ModuleIOSim(TunerConstants.BackRight));
      case REPLAY ->
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
    };
  }

  private static Vision createVision(Drive drive) {
    VisionIO[] ios = new VisionIO[LIMELIGHT_NAMES.length];
    switch (Constants.getMode()) {
      case REAL -> {
        for (int i = 0; i < LIMELIGHT_NAMES.length; i++) {
          ios[i] = new VisionIOLimelight(LIMELIGHT_NAMES[i]);
        }
      }
      case SIM -> {
        // One synthetic camera so std-dev tuning has a measurable effect in replay.
        ios[0] = new VisionIOSim(LIMELIGHT_NAMES[0]);
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
        new OrbitDrive(
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
