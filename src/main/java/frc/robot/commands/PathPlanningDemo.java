package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Field2026Obstacles;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utils.FieldInfo;
import frc.robot.utils.path.GeneratedPath;
import frc.robot.utils.path.ObstacleField;
import frc.robot.utils.path.ObstacleVisualizer;
import frc.robot.utils.path.PathGenerator;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 * Demo auto routine for the runtime path generator. Plans a diagonal across the 2026 field, going
 * around the Hubs and floor bumps, and drives it. Reuses the same {@link ObstacleField} the teleop
 * avoidance clamp consumes so what the planner avoids and what the clamp avoids stay in sync.
 */
public final class PathPlanningDemo {

  private static final double CELL_SIZE = 0.1;
  private static final double VISUAL_RADIUS = Drive.DRIVE_BASE_RADIUS + Drive.BUMPER_THICKNESS_M;

  // Diagonal across the 2026 field: bottom-left clear of Blue structures, top-right clear of Red.
  private static final Pose2d DEFAULT_START = new Pose2d(2.5, 1.5, Rotation2d.kZero);
  private static final Pose2d DEFAULT_GOAL = new Pose2d(14.5, 6.5, Rotation2d.fromDegrees(90));

  private PathPlanningDemo() {}

  /** Default demo using the live {@link Field2026Obstacles}. */
  public static Command create(Drive drive) {
    return create(drive, DEFAULT_START, DEFAULT_GOAL, Field2026Obstacles.build());
  }

  /**
   * Plans a path from {@code start} to {@code goal} around the given obstacles, then drives it. The
   * obstacle field is expected to include perimeter walls already (Field2026Obstacles does).
   */
  public static Command create(Drive drive, Pose2d start, Pose2d goal, ObstacleField field) {
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
            Drive.ROBOT_HALF_X,
            Drive.ROBOT_HALF_Y,
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
}
