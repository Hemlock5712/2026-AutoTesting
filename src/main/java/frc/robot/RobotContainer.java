package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autonomous.AutoCommands;
import frc.robot.autonomous.AutoRoutines;
import frc.robot.commands.AxisLockDrive;
import frc.robot.commands.OrbitDrive;
import frc.robot.constants.FieldConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelSIM;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretSIM;

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
  public final Flywheel flywheel = RobotBase.isSimulation() ? new FlywheelSIM() : new Flywheel();
  public final Turret turret = RobotBase.isSimulation() ? new TurretSIM() : new Turret();
  private final Superstructure superstructure =
      new Superstructure(flywheel, turret, drivetrain::getState);

  // Vision camera for tracking robot position
  //   public final LimelightSubsystem limelight = new LimelightSubsystem("limelight", drivetrain);

  /* Autonomous mode selector */
  private final SendableChooser<Command> autoChooser;

  private final AutoRoutines autoRoutines;

  public RobotContainer() {

    // Set up autonomous routines
    autoChooser = new SendableChooser<>();
    autoRoutines = new AutoRoutines(autoCommands, superstructure);

    // Add autonomous mode options to dashboard
    autoChooser.addOption("Mobility Auto", autoRoutines.sequentialScoringAuto());

    SmartDashboard.putData("Auto Mode", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(
        new OrbitDrive(
            drivetrain,
            () -> -rescaleInputs(joystick.getLeftY()) * MaxSpeed,
            () -> -rescaleInputs(joystick.getLeftX()) * MaxSpeed,
            () -> -rescaleInputs(joystick.getRightX()) * MaxAngularRate));

    // Drive to point - press A to drive to target pose
    // joystick
    // .a()
    // .whileTrue(pathRequest.withTarget(new Pose2d(2, 0,
    // Rotation2d.kZero)).createCommand());

    // PathRequest with Waypoint enum - press B to drive to predefined positions
    // joystick
    //     .b()
    //     .whileTrue(new DriveToPoint(drivetrain, new Pose2d(5, 5, Rotation2d.fromDegrees(45))));
    // DriveTOPoint

    joystick
        .leftBumper()
        .whileTrue(autoRoutines.DrivePointInLeft(() -> drivetrain.getPose().getRotation()));
    joystick
        .rightBumper()
        .whileTrue(autoRoutines.DrivePointInRight(() -> drivetrain.getPose().getRotation()));

    joystick
        .start()
        .onTrue(drivetrain.runOnce(() -> drivetrain.resetPose(new Pose2d(0, 0, Rotation2d.kZero))));

    // AxisLockDrive - Lock Y axis to reef center, driver controls X, rotation free
    joystick
        .a()
        .whileTrue(
            AxisLockDrive.lockY(
                drivetrain,
                () -> -rescaleInputs(joystick.getLeftX()) * MaxSpeed, // Driver controls X
                () -> -rescaleInputs(joystick.getRightX()) * MaxAngularRate,
                FieldConstants.YAxisLockPosition.REEF_CENTER.getY(),
                null)); // null = heading lock behavior (driver controls rotation)

    // AxisLockDrive - Lock Y axis and rotation, driver controls X only
    joystick
        .b()
        .whileTrue(
            AxisLockDrive.lockY(
                drivetrain,
                () -> -rescaleInputs(joystick.getLeftX()) * MaxSpeed, // Driver controls X
                () -> -rescaleInputs(joystick.getRightX()) * MaxAngularRate,
                FieldConstants.YAxisLockPosition.REEF_CENTER.getY(),
                FieldConstants.RotationLockAngle.FACING_FORWARD.getAngle())); // Lock rotation to 0°
  }

  public Command getAutonomousCommand() {
    /* Run the path selected from the auto chooser */
    return autoChooser.getSelected();
  }

  public double rescaleInputs(double input) {
    return MathUtil.applyDeadband(input, 0.05);
  }
}
