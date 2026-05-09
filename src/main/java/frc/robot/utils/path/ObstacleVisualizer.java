package frc.robot.utils.path;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Logs obstacles for viewing in AdvantageScope.
 *
 * <p>Each obstacle gets its own log key so AdvantageScope draws clean separate outlines instead of
 * connecting them all together.
 *
 * <p>How to view (2D Field tab):
 *
 * <ul>
 *   <li>Drag {@code <basePath>/Static/<i>} or {@code <basePath>/Dynamic/<i>} onto a trajectory
 *       channel to see each obstacle's outline.
 *   <li>{@code <basePath>/Inflated/...} shows the inflated outline the planner uses.
 *   <li>{@code <basePath>/AllOutlineDots} is one big dot cloud of every obstacle.
 * </ul>
 */
public final class ObstacleVisualizer {

  private static final double POINT_SPACING = 0.05;
  private static final int CIRCLE_MIN_SAMPLES = 24;

  private ObstacleVisualizer() {}

  /**
   * Logs every obstacle in the field. Safe to call every loop.
   *
   * @param basePath Log key prefix (e.g. {@code "Planner/Obstacles"})
   * @param field Obstacles to visualize
   * @param robotRadius If positive, also logs inflated outlines under {@code Inflated/...}
   */
  public static void log(String basePath, ObstacleField field, double robotRadius) {
    List<Obstacle> stat = field.staticObstacles();
    List<Obstacle> dyn = field.dynamicObstacles();

    for (int i = 0; i < stat.size(); i++) {
      Logger.recordOutput(basePath + "/Static/" + i, outline(stat.get(i), 0.0));
      if (robotRadius > 0) {
        Logger.recordOutput(basePath + "/Inflated/Static/" + i, outline(stat.get(i), robotRadius));
      }
    }
    for (int i = 0; i < dyn.size(); i++) {
      Logger.recordOutput(basePath + "/Dynamic/" + i, outline(dyn.get(i), 0.0));
      if (robotRadius > 0) {
        Logger.recordOutput(basePath + "/Inflated/Dynamic/" + i, outline(dyn.get(i), robotRadius));
      }
    }

    List<Translation2d> dots = new ArrayList<>();
    for (Obstacle o : stat) appendDots(dots, o, 0.0);
    for (Obstacle o : dyn) appendDots(dots, o, 0.0);
    Logger.recordOutput(basePath + "/AllOutlineDots", dots.toArray(new Translation2d[0]));
  }

  /** Logs a flat list of obstacles under {@code basePath/<i>}. */
  public static void logFlat(String basePath, List<Obstacle> obstacles, double robotRadius) {
    for (int i = 0; i < obstacles.size(); i++) {
      Logger.recordOutput(basePath + "/" + i, outline(obstacles.get(i), 0.0));
      if (robotRadius > 0) {
        Logger.recordOutput(basePath + "/Inflated/" + i, outline(obstacles.get(i), robotRadius));
      }
    }
  }

  /** Returns points around the outline of the obstacle, optionally puffed outward. */
  public static Pose2d[] outline(Obstacle o, double inflate) {
    List<Pose2d> pts = new ArrayList<>();
    if (o instanceof Obstacle.Circle c) {
      double r = Math.max(0, c.radius() + inflate);
      int samples =
          Math.max(CIRCLE_MIN_SAMPLES, (int) Math.ceil(2.0 * Math.PI * r / POINT_SPACING));
      for (int i = 0; i <= samples; i++) {
        double t = 2.0 * Math.PI * i / samples;
        pts.add(new Pose2d(c.cx() + r * Math.cos(t), c.cy() + r * Math.sin(t), Rotation2d.kZero));
      }
    } else if (o instanceof Obstacle.Rectangle r) {
      // We draw inflated rectangles with sharp corners. The actual planner uses rounded corners,
      // but the difference is small and only matters for visualization.
      double minX = r.minX() - inflate;
      double minY = r.minY() - inflate;
      double maxX = r.maxX() + inflate;
      double maxY = r.maxY() + inflate;
      addEdge(pts, minX, minY, maxX, minY);
      addEdge(pts, maxX, minY, maxX, maxY);
      addEdge(pts, maxX, maxY, minX, maxY);
      addEdge(pts, minX, maxY, minX, minY);
      pts.add(new Pose2d(minX, minY, Rotation2d.kZero)); // close the loop
    }
    return pts.toArray(new Pose2d[0]);
  }

  private static void appendDots(List<Translation2d> out, Obstacle o, double inflate) {
    for (Pose2d p : outline(o, inflate)) out.add(p.getTranslation());
  }

  private static void addEdge(List<Pose2d> out, double x0, double y0, double x1, double y1) {
    double dx = x1 - x0;
    double dy = y1 - y0;
    double len = Math.hypot(dx, dy);
    int n = Math.max(2, (int) Math.ceil(len / POINT_SPACING));
    for (int i = 0; i < n; i++) {
      double t = (double) i / n;
      out.add(new Pose2d(x0 + t * dx, y0 + t * dy, Rotation2d.kZero));
    }
  }
}
