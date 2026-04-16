package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
import frc.robot.commands.TurretDrive;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.BallPhysicsSimulation;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.FeedMode;
import frc.robot.subsystems.intake.IntakeCoordinator;
import frc.robot.utils.FieldInfo;
import frc.robot.utils.path.PathData;
import java.util.Optional;

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
public class RobotContainer {
  private static final double JOYSTICK_DEADBAND = 0.05;

  private double maxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double maxAngularRate =
      RotationsPerSecond.of(1)
          .in(RadiansPerSecond); // 1 of a rotation per second max angular velocity

  private double maxShootSpeed = 1.5;
  private double maxFeedSpeed = 3.0;
  private double maxShootAngularRate = maxAngularRate * 0.75;

  private final CommandXboxController joystick = new CommandXboxController(0);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  public final AutoCommands autoCommands = new AutoCommands(drivetrain);

  /* Create subsystems (uses simulated versions when running in simulation) */
  private final Superstructure superstructure = new Superstructure(drivetrain::getState);

  private final IntakeCoordinator intakeCoordinator = new IntakeCoordinator();

  // Vision camera for tracking robot position
  public final Limelight limelightBR = new Limelight("limelight-br", drivetrain, 1.0);
  public final Limelight limelightBL = new Limelight("limelight-bl", drivetrain, 1.0);
  public final Limelight limelightFL = new Limelight("limelight-fl", drivetrain);
  public final Limelight limelightFR = new Limelight("limelight-fr", drivetrain);
  // public final Limelight limelightMM = new Limelight("limelight-mm",
  // drivetrain);

  // Create ball physics simulation if in simulation mode
  public final BallPhysicsSimulation ballPhysicsSimulation =
      new BallPhysicsSimulation(drivetrain, superstructure);

  /* Autonomous mode selector — builds only the selected path during disabled */
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  private final AutoRoutines autoRoutines;
  private Command cachedAutoCommand = Commands.none();
  private PathData lastBuiltPath;
  private Optional<Alliance> lastBuiltAlliance = Optional.empty();

  public RobotContainer() {
    autoRoutines = new AutoRoutines(autoCommands, superstructure, intakeCoordinator);

    // Register available paths (lightweight — no SplinePath/VelocityProfile computation)
    // addPathAutoOption(Paths.START_LEFT_TO_RIGHT_TRENCH_TO_DEPOT);
    // addPathAutoOption(Paths.START_LEFT_TO_RIGHT_BUMP);
    // addPathAutoOption(Paths.START_LEFT_TO_RIGHT_BUMP_LONG);
    // addPathAutoOption(Paths.OP);
    // addPathAutoOption(Paths.OP_RIGHT);
    // addPathAutoOption(Paths.SHARK);
    // addPathAutoOption(Paths.MADTOWN);
    autoChooser.addOption("Left Side 2 Passes", autoRoutines.leftSide2Passes());
    autoChooser.addOption("Left Side Auto", autoRoutines.leftAutoFeed(8.1));

    SmartDashboard.putData("Auto Mode", autoChooser);

    configureBindings();
  }

  private static boolean bumpIsInAllianceZone = true;

  private void configureBindings() {
    // Cached translation velocities - computed once per cycle in velocityX supplier
    double[] translationVel = {0, 0};

    drivetrain.setDefaultCommand(
        new OrbitDrive(
            drivetrain,
            () -> {
              // Not the cleanest but calculate scaled joystick values
              Vector<N2> scaled = rescaleTranslation(joystick.getLeftY(), joystick.getLeftX());
              translationVel[0] = -scaled.get(0) * maxSpeed;
              translationVel[1] = -scaled.get(1) * maxSpeed;
              return translationVel[0];
            },
            () -> translationVel[1],
            () -> -rescaleInputs(joystick.getRightX()) * maxAngularRate));

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

    joystick
        .x()
        .whileTrue(
            AxisLockDrive.lockY(
                drivetrain,
                () ->
                    bumpIsInAllianceZone
                        ? (Math.abs(rescaleInputs(joystick.getLeftY())) > 0.2 ? maxSpeed / 3 : 0)
                        : (Math.abs(rescaleInputs(joystick.getLeftY())) > 0.2
                            ? -maxSpeed / 3
                            : 0), // Driver controls X
                () -> -rescaleInputs(joystick.getRightX()) * maxAngularRate,
                () -> FieldInfo.flipY(FieldInfo.axisLockYTrenchLeft()),
                () -> Rotation2d.fromDegrees(bumpIsInAllianceZone ? 135 : 45)
                /* AutoRoutines.snapToNearest180Degrees(drivetrain.getRotation()) */ ))
        .onTrue(
            Commands.runOnce(
                () ->
                    bumpIsInAllianceZone =
                        FieldInfo.flipX(drivetrain.getPose().getX())
                            < FieldInfo.ALLIANCE_ZONE_X)); // Lock to
    // closest
    // 180

    joystick
        .b()
        .whileTrue(
            AxisLockDrive.lockY(
                drivetrain,
                () ->
                    bumpIsInAllianceZone
                        ? (Math.abs(rescaleInputs(joystick.getLeftY())) > 0.2 ? maxSpeed / 3 : 0)
                        : (Math.abs(rescaleInputs(joystick.getLeftY())) > 0.2
                            ? -maxSpeed / 3
                            : 0), // Driver controls X
                () -> -rescaleInputs(joystick.getRightX()) * maxAngularRate,
                () -> FieldInfo.flipY(FieldInfo.AXIS_LOCK_Y_TRENCH_RIGHT),
                () -> Rotation2d.fromDegrees(bumpIsInAllianceZone ? -135 : -45)
                /* AutoRoutines.snapToNearest180Degrees(drivetrain.getRotation()) */ ))
        .onTrue(
            Commands.runOnce(
                () ->
                    bumpIsInAllianceZone =
                        FieldInfo.flipX(drivetrain.getPose().getX())
                            < FieldInfo.ALLIANCE_ZONE_X)); // Lock to
    // closest
    // 180

    // Right trigger toggles auto-shoot mode on/off.
    // Debounce prevents analog trigger noise from causing multiple toggles.
    joystick
        .rightTrigger()
        .debounce(0.1)
        .toggleOnTrue(
            Commands.parallel(
                superstructure.autoShootMode(),
                new TurretDrive(
                    drivetrain,
                    () -> {
                      double speed = superstructure.isHubShot() ? maxShootSpeed : maxFeedSpeed;
                      Vector<N2> scaled =
                          rescaleTranslation(joystick.getLeftY(), joystick.getLeftX());
                      translationVel[0] = -scaled.get(0) * speed;
                      translationVel[1] = -scaled.get(1) * speed;
                      return translationVel[0];
                    },
                    () -> translationVel[1],
                    () -> -rescaleInputs(joystick.getRightX()) * maxShootAngularRate)));

    // joystick
    // .leftTrigger(0.5)
    // .onTrue(
    // Commands.either(
    // intakeCoordinator.deployAndRun(),
    // intakeCoordinator.upAndRun(),
    // () -> intakeCoordinator.getTargetPositionRotations() != 0));

    joystick.leftTrigger(0.5).onTrue(intakeCoordinator.deployAndRun());

    // joystick.y().onTrue(superstructure.shootManual()).onFalse(superstructure.stopShoot());

    joystick.y().onTrue(superstructure.tuningShoot()).onFalse(superstructure.stopShoot());

    joystick
        .povLeft()
        .onTrue(Commands.runOnce(() -> superstructure.setTeleopFeedMode(FeedMode.FORCE_LEFT)));
    joystick
        .povRight()
        .onTrue(Commands.runOnce(() -> superstructure.setTeleopFeedMode(FeedMode.FORCE_RIGHT)));
    joystick
        .povDown()
        .onTrue(Commands.runOnce(() -> superstructure.setTeleopFeedMode(FeedMode.AUTO)));
    joystick.povUp().onTrue(intakeCoordinator.straightUp());

    joystick.a().onTrue(intakeCoordinator.reverseIntake()).onFalse(intakeCoordinator.stopWheel());
  }

  public Command getAutonomousCommand() {
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

  public Command fmsInitCommand() {
    return Commands.parallel(intakeCoordinator.deployAndRun(), superstructure.stopShoot());
  }

  public Command stopCommand() {
    return Commands.parallel(intakeCoordinator.stopBoth(), superstructure.stopShoot());
  }

  /** Sets the rumble intensity on the driver controller (0.0 = off, 1.0 = full). */
  public void setRumble(double value) {
    joystick.getHID().setRumble(RumbleType.kBothRumble, value);
  }

  // private void addPathAutoOption(PathData path) {
  //   autoChooser.addOption(Paths.nameOf(path), path);
  // }

  /**
   * Polls for changes in auto selection or alliance during disabled. Rebuilds the cached auto
   * command only when something changes. Call from {@code disabledPeriodic()}.
   */
  public void updateAutoSelection() {
    // PathData selected = autoChooser.getSelected();
    Optional<Alliance> alliance = DriverStation.getAlliance();

    if (alliance.equals(lastBuiltAlliance)) {
      return;
    }

    // lastBuiltPath = selected;
    lastBuiltAlliance = alliance;

    // Update tag filters on rear cameras when alliance changes
    updateRearCameraTagFilters(alliance);

    // if (selected == null) {
    //   cachedAutoCommand = Commands.none();
    //   return;
    // }

    // cachedAutoCommand = autoRoutines.autoBuilder(selected);
  }

  /**
   * Sets fiducial ID filters on rear-facing cameras to only trust tags on our alliance side. Clears
   * filters (accept all) when alliance is unknown.
   */
  private void updateRearCameraTagFilters(Optional<Alliance> alliance) {
    int[] validIDs = FieldInfo.getAllianceTags(alliance);
    limelightBL.setTagFilter(validIDs);
    limelightBR.setTagFilter(validIDs);
  }
}
