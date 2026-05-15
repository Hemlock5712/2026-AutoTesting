package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utils.FieldInfo;
import frc.robot.utils.path.GeneratedPath;
import frc.robot.utils.path.ObstacleField;
import frc.robot.utils.path.PathGenerator;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Plans a path from the robot's <em>current</em> pose to a supplied goal, around the given
 * obstacles, and drives it with {@link FollowPath}. Suitable for driver buttons (re-plans on each
 * trigger) and for auto routines (call {@link Drive#resetPose} first to fix the start).
 *
 * <pre>{@code
 * driver.a().onTrue(DriveToWithAvoidance.create(drive, () -> SCORING_POSE, obstacleField));
 * }</pre>
 */
public final class DriveToWithAvoidance {

  /** Planner grid resolution (m). 0.1 m is a good default for FRC field-sized graphs. */
  private static final double CELL_SIZE = 0.1;

  /**
   * How close to the goal counts as arrived (m). Wider than {@link FollowPath}'s default (5 cm)
   * because the planner spline's last segment curves and fighting that last centimeter wastes time.
   */
  private static final double COMPLETION_TOLERANCE_M = 0.1;

  /** Surfaced on the AdvantageScope alerts panel + DS when the planner can't find a route. */
  private static final Alert PLAN_FAILED_ALERT =
      new Alert("DriveToWithAvoidance: no clear route to goal — check obstacles or starting pose.",
          AlertType.kWarning);

  private DriveToWithAvoidance() {}

  /** Drive from current pose to {@code goalSupplier.get()}, planning around {@code field}. */
  public static Command create(Drive drive, Supplier<Pose2d> goalSupplier, ObstacleField field) {
    return Commands.defer(() -> planAndFollow(drive, goalSupplier.get(), field), Set.of(drive));
  }

  private static Command planAndFollow(Drive drive, Pose2d goal, ObstacleField field) {
    Pose2d start = drive.getPose();
    Optional<GeneratedPath> path =
        PathGenerator.generateOrientedWithFallback(
            field,
            0.0,
            0.0,
            FieldInfo.lengthMeters(),
            FieldInfo.widthMeters(),
            CELL_SIZE,
            CELL_SIZE,
            new PathGenerator.Request(start, goal, 0.0, 0.0),
            Drive.ROBOT_HALF_X,
            Drive.ROBOT_HALF_Y,
            PathGenerator.Config.defaults());
    if (path.isPresent()) {
      Logger.recordOutput("DriveToWithAvoidance/PlanFailed", false);
      PLAN_FAILED_ALERT.set(false);
      return new FollowPath(drive, path.get()).withCompletionTolerance(COMPLETION_TOLERANCE_M);
    }
    return Commands.runOnce(
        () -> {
          Logger.recordOutput("DriveToWithAvoidance/PlanFailed", true);
          PLAN_FAILED_ALERT.set(true);
          DriverStation.reportWarning(
              "DriveToWithAvoidance: no clear route from " + start + " to " + goal, false);
        });
  }
}
