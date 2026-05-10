package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utils.FieldInfo;
import frc.robot.utils.path.GeneratedPath;
import frc.robot.utils.path.Obstacle;
import frc.robot.utils.path.ObstacleField;
import frc.robot.utils.path.ObstacleVisualizer;
import frc.robot.utils.path.PathGenerator;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 * Demo auto routine for the runtime path generator.
 *
 * <p>Drops three obstacles on the field, plans a path from one side to the other, and drives it.
 * Obstacles are logged to AdvantageScope so you can see the path avoiding them.
 */
public final class PathPlanningDemo {

  // Path planner cell size.
  private static final double CELL_SIZE = 0.1;

  /** Standard FRC bumper thickness (3.25 inches). */
  private static final double BUMPER_THICKNESS_M = 0.0826;

  /** Half the robot's length and width, including bumpers. */
  private static final double ROBOT_HALF_X =
      Math.abs(TunerConstants.FrontLeft.LocationX) + BUMPER_THICKNESS_M;

  private static final double ROBOT_HALF_Y =
      Math.abs(TunerConstants.FrontLeft.LocationY) + BUMPER_THICKNESS_M;

  /** Just for the visualizer. */
  private static final double VISUAL_RADIUS = Drive.DRIVE_BASE_RADIUS + BUMPER_THICKNESS_M;

  /** Thin walls around the field perimeter so the planner doesn't try to route off the field. */
  private static final double WALL_THICKNESS = 0.05;

  // Default demo scenario: one big circle + one rectangle + one small circle.
  private static final Pose2d DEFAULT_START = new Pose2d(1.0, 4.0, Rotation2d.kZero);
  private static final Pose2d DEFAULT_GOAL = new Pose2d(16.0, 4.0, Rotation2d.fromDegrees(90));
  private static final List<Obstacle> DEFAULT_OBSTACLES =
      List.of(
          new Obstacle.Circle(8.27, 4.03, 1.0), // big pillar in the middle
          new Obstacle.Rectangle(3.0, 1.0, 4.0, 5.0), // left-side wall
          new Obstacle.Circle(13.0, 2.5, 0.6)); // small post

  private PathPlanningDemo() {}

  /** Default demo scenario (one circle + one rectangle + one small circle). */
  public static Command create(Drive drive) {
    return create(drive, DEFAULT_START, DEFAULT_GOAL, DEFAULT_OBSTACLES);
  }

  /**
   * Builds the demo command. When run, it:
   *
   * <ol>
   *   <li>Logs obstacles to AdvantageScope.
   *   <li>Resets the robot to the start pose.
   *   <li>Plans a path that goes around the obstacles.
   *   <li>Drives the path.
   * </ol>
   */
  public static Command create(Drive drive, Pose2d start, Pose2d goal, List<Obstacle> obstacles) {
    ObstacleField field = new ObstacleField();
    obstacles.forEach(field::addStatic);
    fieldWalls().forEach(field::addStatic);

    Translation2d[] startGoalMarkers =
        new Translation2d[] {start.getTranslation(), goal.getTranslation()};

    Command logScene =
        Commands.runOnce(
            () -> {
              ObstacleVisualizer.log("PathDemo/Obstacles", field, VISUAL_RADIUS);
              Logger.recordOutput("PathDemo/StartGoal", startGoalMarkers);
            });

    Command resetPose = Commands.runOnce(() -> drive.resetPose(start), drive);

    PathGenerator.Request req = new PathGenerator.Request(start, goal, 0.0, 0.0);
    Optional<GeneratedPath> result =
        PathGenerator.generateOrientedWithFallback(
            field,
            0.0,
            0.0,
            FieldInfo.lengthMeters(),
            FieldInfo.widthMeters(),
            CELL_SIZE,
            CELL_SIZE,
            req,
            ROBOT_HALF_X,
            ROBOT_HALF_Y,
            PathGenerator.Config.defaults());

    if (result.isEmpty()) {
      return Commands.sequence(
          logScene,
          Commands.print("[PathPlanningDemo] Path generation FAILED — check obstacle layout"));
    }

    GeneratedPath path = result.get();
    Command logPathLength =
        Commands.runOnce(
            () -> Logger.recordOutput("PathDemo/PlannedPathLength", path.getTotalLength()));

    FollowPath follow = new FollowPath(drive, path).withCompletionTolerance(0.1);

    return Commands.sequence(
        logScene,
        logPathLength,
        resetPose,
        follow,
        Commands.print("[PathPlanningDemo] Reached goal."));
  }

  /** Field-perimeter walls so the planner can't route the robot off the field. */
  private static List<Obstacle> fieldWalls() {
    return List.of(
        new Obstacle.Rectangle(
            -WALL_THICKNESS, -WALL_THICKNESS, FieldInfo.lengthMeters() + WALL_THICKNESS, 0.0),
        new Obstacle.Rectangle(
            -WALL_THICKNESS,
            FieldInfo.widthMeters(),
            FieldInfo.lengthMeters() + WALL_THICKNESS,
            FieldInfo.widthMeters() + WALL_THICKNESS),
        new Obstacle.Rectangle(-WALL_THICKNESS, 0.0, 0.0, FieldInfo.widthMeters()),
        new Obstacle.Rectangle(
            FieldInfo.lengthMeters(),
            0.0,
            FieldInfo.lengthMeters() + WALL_THICKNESS,
            FieldInfo.widthMeters()));
  }
}
