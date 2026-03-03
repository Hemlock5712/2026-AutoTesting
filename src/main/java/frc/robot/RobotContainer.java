package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autonomous.AutoCommands;
import frc.robot.autonomous.AutoRoutines;
import frc.robot.commands.AxisLockDrive;
import frc.robot.commands.OrbitDrive;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeSIM;
import frc.robot.subsystems.shooter.ShooterLookup;
import frc.robot.utils.FieldInfo;

/**
 * RobotContainer - Sets up all the robot's parts and controls.
 *
 * <p>This class handles:
 *
 * <ul>
 *   <li>Creating all subsystems (drive, arm, flywheel, vision, etc.)
 *   <li>Setting up controller buttons
 *   <li>Building autonomous routines
 *   <li>Setting default actions for each subsystem
 * </ul>
 *
 * <p>The robot can run in two modes:
 *
 * <ul>
 *   <li><b>Real hardware:</b> Uses actual motors and sensors
 *   <li><b>Simulation:</b> Uses simulated physics for testing without a real robot
 * </ul>
 *
 * The code automatically picks the right version.
 */
@Logged
public class RobotContainer {
  private double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate =
      RotationsPerSecond.of(1)
          .in(RadiansPerSecond); // 1 of a rotation per second max angular velocity

  private final CommandXboxController joystick = new CommandXboxController(0);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  public final AutoCommands autoCommands = new AutoCommands(drivetrain);

  /* Create subsystems (uses simulated versions when running in simulation) */
  private final Superstructure superstructure = new Superstructure(drivetrain::getState);

  private final Intake intake = RobotBase.isSimulation() ? new IntakeSIM() : new Intake();

  // Vision camera for tracking robot position
  public final Limelight limelight = new Limelight("limelight-br", drivetrain);

  // Vision camera for tracking robot position
  public final Limelight limelight1 = new Limelight("limelight-bl", drivetrain);

  /* Autonomous mode selector */
  private final SendableChooser<Command> autoChooser;

  private final AutoRoutines autoRoutines;

  public RobotContainer() {
    ShooterLookup.initialize();

    // Set up autonomous routines
    autoChooser = new SendableChooser<>();
    autoRoutines = new AutoRoutines(autoCommands, superstructure);

    // Add autonomous mode options to dashboard
    autoChooser.addOption("Mobility Auto", autoRoutines.sequentialScoringAuto());
    // AutoHumanPlayerSIMONLY
    autoChooser.addOption("AutoHumanPlayerSIMONLY", autoRoutines.AutoHumanPlayerSIMONLY());

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
              // Not the cleanest but claculate scaled joystick values
              Vector<N2> scaled = rescaleTranslation(joystick.getLeftY(), joystick.getLeftX());
              translationVel[0] = -scaled.get(0) * MaxSpeed;
              translationVel[1] = -scaled.get(1) * MaxSpeed;
              return translationVel[0];
            },
            () -> translationVel[1],
            () -> -rescaleInputs(joystick.getRightX()) * MaxAngularRate));

    joystick
        .start()
        .onTrue(drivetrain.runOnce(() -> drivetrain.resetPose(new Pose2d(0, 0, Rotation2d.kZero))));

    joystick
        .back()
        .onTrue(
            drivetrain.runOnce(() -> drivetrain.resetPose(new Pose2d(1.5, 1, Rotation2d.kZero))));

    // AxisLockDrive - Lock Y axis to reef center, driver controls X, rotation free
    joystick
        .leftBumper()
        .whileTrue(
            AxisLockDrive.lockY(
                drivetrain,
                () -> -rescaleInputs(joystick.getLeftX()) * MaxSpeed, // Driver controls X
                () -> -rescaleInputs(joystick.getRightX()) * MaxAngularRate,
                FieldInfo.flipY(FieldInfo.axisLockYLeft()),
                null)); // null = heading lock behavior (driver controls rotation)

    // // AxisLockDrive - Lock Y axis and rotation, driver controls X only
    joystick
        .rightBumper()
        .whileTrue(
            AxisLockDrive.lockY(
                drivetrain,
                () -> -rescaleInputs(joystick.getLeftX()) * MaxSpeed, // Driver controls X
                () -> -rescaleInputs(joystick.getRightX()) * MaxAngularRate,
                FieldInfo.flipY(FieldInfo.AXIS_LOCK_Y_RIGHT),
                FieldInfo.flip(FieldInfo.FACING_FORWARD))); // Lock rotation to 0°

    joystick
        .rightTrigger(0.5)
        .onTrue(superstructure.beginShoot())
        .onFalse(superstructure.stopShoot());

    joystick
        .leftTrigger(0.5)
        .onTrue(intake.intakeDown().andThen(intake.runIntake()))
        .onFalse(intake.stopWheel());

    joystick.a().onTrue(intake.runIntake());
    joystick.b().onFalse(intake.stopWheel());
  }

  public Command getAutonomousCommand() {
    /* Run the path selected from the auto chooser */
    return autoChooser.getSelected();
  }

  public double rescaleInputs(double input) {
    return MathUtil.applyDeadband(input, 0.05);
  }

  public Vector<N2> rescaleTranslation(double x, double y) {
    Vector<N2> scaledJoyStick = VecBuilder.fill(x, y);
    scaledJoyStick = MathUtil.applyDeadband(scaledJoyStick, 0.05);
    return MathUtil.copyDirectionPow(scaledJoyStick, 2);
  }

  public Superstructure getSuperstructure() {
    return superstructure;
  }
}
