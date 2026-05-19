package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AxisLockDrive;
import frc.robot.commands.PathPlannerAutos;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utils.geometry.ExtPose;
import java.util.function.Supplier;

/** Owns the dashboard auto chooser and the registered routines. */
public final class AutoSelector {

  /**
   * Demo start/goal for the on-the-fly pathfinding auto. Defined in blue-origin coordinates;
   * {@link ExtPose#get()} returns the alliance-correct variant. Replace with your scoring
   * positions.
   *
   * <p>{@link #DEMO_GOAL} is also reused by the driver-A "drive here, avoiding obstacles" binding
   * in {@code RobotContainer} so the two demos point at the same spot.
   */
  private static final ExtPose DEMO_START = new ExtPose(2.5, 1.5, Rotation2d.kZero);

  public static final ExtPose DEMO_GOAL = new ExtPose(14.5, 6.5, Rotation2d.fromDegrees(90));

  private final SendableChooser<Supplier<Command>> chooser = new SendableChooser<>();
  private final Drive drive;

  /** Verification target for the AxisLock auto: forward 3m + lateral 2m + 90° turn. */
  private static final Pose2d AXIS_LOCK_TEST_TARGET =
      new Pose2d(5.0, 6.0, Rotation2d.fromDegrees(90));

  public AutoSelector(Drive drive) {
    this.drive = drive;
    chooser.setDefaultOption("NewPath (PathPlanner)", this::newPathAutoPP);
    chooser.addOption(
        "Straight + U (back-to-back)",
        () ->
            Commands.sequence(
                PathPlannerAutos.runAuto(drive, "Straight"),
                PathPlannerAutos.runAuto(drive, "New Auto")));
    chooser.addOption(
        "AxisLock test (drive-to-pose)",
        () ->
            new AxisLockDrive(
                    drive,
                    () -> 0.0,
                    () -> 0.0,
                    () -> 0.0,
                    AXIS_LOCK_TEST_TARGET::getX,
                    AXIS_LOCK_TEST_TARGET::getY,
                    AXIS_LOCK_TEST_TARGET::getRotation));
    chooser.addOption("Straight (3m)", () -> PathPlannerAutos.runAuto(drive, "Straight"));
    chooser.addOption("None", Commands::none);
    chooser.addOption(
        "Pathfind Demo (reset + pathfind to goal)",
        () ->
            Commands.sequence(
                Commands.runOnce(() -> drive.resetPose(DEMO_START.get()), drive),
                PathPlannerAutos.pathfindToPose(drive, DEMO_GOAL.get())));
    SmartDashboard.putData("Auto Mode", chooser);
  }

  /** Returns the currently-selected routine (or {@link Commands#none()} if nothing is set). */
  public Command getSelected() {
    Supplier<Command> selected = chooser.getSelected();
    return (selected != null) ? selected.get() : Commands.none();
  }

  /**
   * Run {@code New Auto.auto} from {@code src/main/deploy/pathplanner/autos/} via PathPlannerLib's
   * distance-based follower. Handles pose reset, path sequencing, and named commands. Inspect in
   * AdvantageScope under the {@code PathPlanner/} namespace.
   */
  private Command newPathAutoPP() {
    return PathPlannerAutos.runAuto(drive, "New Auto");
  }
}
