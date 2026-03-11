package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autonomous.AutoCommands;
import frc.robot.autonomous.AutoRoutines;
import frc.robot.commands.AxisLockDrive;
import frc.robot.commands.OrbitDrive;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.BallPhysicsSimulation;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.intake.IntakeCoordinator;
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
  private static final double JOYSTICK_DEADBAND = 0.05;

  private double maxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double maxAngularRate =
      RotationsPerSecond.of(1)
          .in(RadiansPerSecond); // 1 of a rotation per second max angular velocity

  private final CommandXboxController joystick = new CommandXboxController(0);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  public final AutoCommands autoCommands = new AutoCommands(drivetrain);

  /* Create subsystems (uses simulated versions when running in simulation) */
  private final Superstructure superstructure = new Superstructure(drivetrain::getState);

  private final IntakeCoordinator intakeCoordinator = new IntakeCoordinator();

  // Vision camera for tracking robot position
  public final Limelight limelight = new Limelight("limelight-br", drivetrain);
  public final Limelight limelight1 = new Limelight("limelight-bl", drivetrain);
  public final Limelight limelight2 = new Limelight("limelight-fl", drivetrain);
  public final Limelight limelight3 = new Limelight("limelight-fr", drivetrain);
  public final Limelight limelight4 = new Limelight("limelight-mm", drivetrain);

  // Create ball physics simulation if in simulation mode
  public final BallPhysicsSimulation ballPhysicsSimulation =
      new BallPhysicsSimulation(drivetrain, superstructure);

  /* Autonomous mode selector */
  private final SendableChooser<Command> autoChooser;

  private final AutoRoutines autoRoutines;

  public RobotContainer() {

    // Set up autonomous routines
    autoChooser = new SendableChooser<>();
    autoRoutines = new AutoRoutines(autoCommands, superstructure, intakeCoordinator);

    // Add autonomous mode options to dashboard
    autoChooser.addOption("Mobility Auto", autoRoutines.sequentialScoringAuto());
    // AutoHumanPlayerSIMONLY
    autoChooser.addOption("Right Auto", autoRoutines.rightAuto());
    autoChooser.addOption("Pizza Auto", autoRoutines.pizzaAuto());

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
              translationVel[0] = -scaled.get(0) * maxSpeed;
              translationVel[1] = -scaled.get(1) * maxSpeed;
              return translationVel[0];
            },
            () -> translationVel[1],
            () -> -rescaleInputs(joystick.getRightX()) * maxAngularRate));

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
                () -> -rescaleInputs(joystick.getLeftY()) * maxSpeed, // Driver controls X
                () -> -rescaleInputs(joystick.getRightX()) * maxAngularRate,
                () -> FieldInfo.flipY(FieldInfo.axisLockYLeft()),
                () ->
                    AutoRoutines.snapToNearest180Degrees(
                        drivetrain.getRotation()))); // Lock to closest 180
    // rotation)

    joystick
        .rightBumper()
        .whileTrue(
            AxisLockDrive.lockY(
                drivetrain,
                () -> -rescaleInputs(joystick.getLeftY()) * maxSpeed, // Driver controls X
                () -> -rescaleInputs(joystick.getRightX()) * maxAngularRate,
                () -> FieldInfo.flipY(FieldInfo.AXIS_LOCK_Y_RIGHT),
                () ->
                    AutoRoutines.snapToNearest180Degrees(
                        drivetrain.getRotation()))); // Lock to closest 180

    joystick.rightTrigger().whileTrue(superstructure.shoot()).onFalse(superstructure.stopShoot());

    joystick
        .leftTrigger(0.5)
        .onTrue(
            Commands.either(
                intakeCoordinator.deployAndRun(),
                intakeCoordinator.stopAndRetract(),
                () -> intakeCoordinator.getTargetPositionRotations() != 0));

    joystick.povUp().onTrue(intakeCoordinator.intakeUp());

    joystick.y().whileTrue(superstructure.shoot()).onFalse(superstructure.stopShoot());

    joystick.a().onTrue(intakeCoordinator.runIntake());
    joystick.b().onFalse(intakeCoordinator.stopWheel());

    joystick
        .x()
        .onTrue(superstructure.spinSpinDexerBack())
        .onFalse(superstructure.spinSpinDexerStop());
  }

  public Command getAutonomousCommand() {
    /* Run the path selected from the auto chooser */
    return autoChooser.getSelected();
  }

  public double rescaleInputs(double input) {
    return MathUtil.applyDeadband(input, JOYSTICK_DEADBAND);
  }

  public Vector<N2> rescaleTranslation(double x, double y) {
    Vector<N2> scaledJoyStick = VecBuilder.fill(x, y);
    scaledJoyStick = MathUtil.applyDeadband(scaledJoyStick, JOYSTICK_DEADBAND);
    return MathUtil.copyDirectionPow(scaledJoyStick, 2);
  }

  public Superstructure getSuperstructure() {
    return superstructure;
  }

  /** Sets the rumble intensity on the driver controller (0.0 = off, 1.0 = full). */
  public void setRumble(double value) {
    joystick.getHID().setRumble(RumbleType.kBothRumble, value);
  }
}
