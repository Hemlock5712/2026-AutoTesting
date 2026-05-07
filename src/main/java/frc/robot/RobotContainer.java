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
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.utils.path.AutoPath;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

public class RobotContainer {
  private static final double JOYSTICK_DEADBAND = 0.05;

  private double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double maxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond);

  private final CommandXboxController joystick = new CommandXboxController(0);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  public final AutoCommands autoCommands = new AutoCommands(drivetrain);

  public final Limelight limelight =
      new Limelight(
          List.of("limelight-br", "limelight-bl", "limelight-fl", "limelight-fr", "limelight-mm"),
          drivetrain);

  /* Autonomous mode selector */
  private final SendableChooser<Supplier<Command>> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    autoChooser.setDefaultOption("NewPath (PD)", this::newPathAutoPD);

    SmartDashboard.putData("Auto Mode", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    // Cached translation velocities - computed once per cycle in velocityX supplier
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

    // Add button bindings here
  }

  public Command getAutonomousCommand() {
    Supplier<Command> selected = autoChooser.getSelected();
    return (selected != null) ? selected.get() : Commands.none();
  }

  public double rescaleInputs(double input) {
    return MathUtil.applyDeadband(input, JOYSTICK_DEADBAND);
  }

  private final double[] scaledTranslation = new double[2];

  /** Deadband + squared-magnitude rescale using raw doubles. Zero allocations. */
  public double[] rescaleTranslation(double x, double y) {
    double mag = Math.hypot(x, y);
    if (mag < JOYSTICK_DEADBAND) {
      scaledTranslation[0] = 0;
      scaledTranslation[1] = 0;
      return scaledTranslation;
    }
    // Deadband: remap [deadband, 1] → [0, 1], clamp, then square for fine control
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
