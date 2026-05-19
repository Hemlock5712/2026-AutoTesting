package frc.robot.utils.path;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Logs obstacle AABBs as native {@link Rectangle2d} arrays so AdvantageScope can render them
 * directly on its 2D-Field tab.
 *
 * <p>Drag {@code <basePath>/Rectangles} onto the 2D-Field display in AdvantageScope and it will
 * draw outlined shapes — no trajectory tricks needed.
 */
public final class ObstacleVisualizer {

  private ObstacleVisualizer() {}

  /**
   * Logs every obstacle. Safe to call every loop.
   *
   * @param basePath log key prefix (e.g. {@code "World/Obstacles"})
   * @param obstacles AABB list (min/max corner pairs)
   * @param inflation if positive, also logs each obstacle inflated by this much so the planner's
   *     effective clearance can be seen on the field
   */
  public static void log(
      String basePath, List<Pair<Translation2d, Translation2d>> obstacles, double inflation) {
    Rectangle2d[] rects = new Rectangle2d[obstacles.size()];
    for (int i = 0; i < obstacles.size(); i++) {
      rects[i] = toRectangle(obstacles.get(i), 0.0);
    }
    Logger.recordOutput(basePath + "/Rectangles", rects);

    if (inflation > 0.0) {
      Rectangle2d[] inflated = new Rectangle2d[obstacles.size()];
      for (int i = 0; i < obstacles.size(); i++) {
        inflated[i] = toRectangle(obstacles.get(i), inflation);
      }
      Logger.recordOutput(basePath + "/Inflated/Rectangles", inflated);
    }
  }

  private static Rectangle2d toRectangle(Pair<Translation2d, Translation2d> box, double inflation) {
    Translation2d min = box.getFirst();
    Translation2d max = box.getSecond();
    double cx = (min.getX() + max.getX()) / 2.0;
    double cy = (min.getY() + max.getY()) / 2.0;
    double widthX = (max.getX() - min.getX()) + 2 * inflation;
    double widthY = (max.getY() - min.getY()) + 2 * inflation;
    return new Rectangle2d(new Pose2d(cx, cy, Rotation2d.kZero), widthX, widthY);
  }
}
