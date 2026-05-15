package frc.robot.utils.path;

import edu.wpi.first.math.geometry.Ellipse2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Logs obstacles as native {@link Rectangle2d} and {@link Ellipse2d} arrays so AdvantageScope can
 * render them directly on its 2D-Field tab without any helper geometry.
 *
 * <p>Drag {@code <basePath>/Rectangles} or {@code <basePath>/Ellipses} onto the 2D-Field display in
 * AdvantageScope and it will draw outlined shapes — no trajectory tricks needed.
 */
public final class ObstacleVisualizer {

  private ObstacleVisualizer() {}

  /**
   * Logs every obstacle in the field. Safe to call every loop.
   *
   * @param basePath Log key prefix (e.g. {@code "World/Obstacles"})
   * @param field Obstacles to visualize
   * @param robotRadius If positive, also logs each obstacle inflated by this much so the planner's
   *     effective clearance can be seen on the field.
   */
  public static void log(String basePath, ObstacleField field, double robotRadius) {
    List<Rectangle2d> rects = new ArrayList<>();
    List<Ellipse2d> ellipses = new ArrayList<>();
    collect(field.staticObstacles(), rects, ellipses);
    collect(field.dynamicObstacles(), rects, ellipses);
    Logger.recordOutput(basePath + "/Rectangles", rects.toArray(new Rectangle2d[0]));
    Logger.recordOutput(basePath + "/Ellipses", ellipses.toArray(new Ellipse2d[0]));

    if (robotRadius > 0.0) {
      List<Rectangle2d> inflatedRects = new ArrayList<>(rects.size());
      List<Ellipse2d> inflatedEllipses = new ArrayList<>(ellipses.size());
      for (Rectangle2d r : rects) {
        Pose2d c = r.getCenter();
        inflatedRects.add(
            new Rectangle2d(c, r.getXWidth() + 2 * robotRadius, r.getYWidth() + 2 * robotRadius));
      }
      for (Ellipse2d e : ellipses) {
        inflatedEllipses.add(
            new Ellipse2d(
                e.getCenter(), e.getXSemiAxis() + robotRadius, e.getYSemiAxis() + robotRadius));
      }
      Logger.recordOutput(
          basePath + "/Inflated/Rectangles", inflatedRects.toArray(new Rectangle2d[0]));
      Logger.recordOutput(
          basePath + "/Inflated/Ellipses", inflatedEllipses.toArray(new Ellipse2d[0]));
    }
  }

  private static void collect(
      List<Obstacle> obstacles, List<Rectangle2d> rects, List<Ellipse2d> ellipses) {
    for (Obstacle o : obstacles) {
      if (o instanceof Obstacle.Rectangle r) {
        rects.add(r.shape());
      } else if (o instanceof Obstacle.Circle c) {
        ellipses.add(c.shape());
      }
    }
  }
}
