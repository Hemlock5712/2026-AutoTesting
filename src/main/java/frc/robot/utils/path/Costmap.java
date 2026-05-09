package frc.robot.utils.path;

/**
 * Grid of distances to the nearest obstacle. Theta* uses this to plan paths.
 *
 * <p>Each cell knows how far it is from the closest obstacle. A cell is "free" if it's at least one
 * robot radius away. All coordinates are field-relative meters.
 */
public final class Costmap {

  // Stored as one flat array for speed (distance[gx * rows + gy]).
  private final double[] distance;
  private final double cellSize;
  private final double originX;
  private final double originY;
  private final int cols;
  private final int rows;
  private final double robotRadius;

  private final double plannerMargin;

  private Costmap(
      double[] distance,
      double cellSize,
      double originX,
      double originY,
      int cols,
      int rows,
      double robotRadius,
      double plannerMargin) {
    this.distance = distance;
    this.cellSize = cellSize;
    this.originX = originX;
    this.originY = originY;
    this.cols = cols;
    this.rows = rows;
    this.robotRadius = robotRadius;
    this.plannerMargin = plannerMargin;
  }

  /**
   * Builds a costmap with a default safety margin of one cell. The margin gives the path smoother a
   * little extra room so it doesn't curve too close to obstacles.
   */
  public static Costmap build(
      ObstacleField field,
      double minX,
      double minY,
      double maxX,
      double maxY,
      double cellSize,
      double robotRadius) {
    return build(field, minX, minY, maxX, maxY, cellSize, robotRadius, cellSize);
  }

  /**
   * Builds a costmap from the obstacle field. A cell is blocked during planning if it's within
   * {@code robotRadius + plannerMargin} of any obstacle. The margin is extra cushion so the
   * smoothed path won't clip obstacles, even if the planner uses cell centers.
   *
   * @param field Obstacles to use
   * @param minX Field bounds (m)
   * @param minY Field bounds (m)
   * @param maxX Field bounds (m)
   * @param maxY Field bounds (m)
   * @param cellSize Grid resolution (m)
   * @param robotRadius Robot's bumper-to-center radius (m)
   * @param plannerMargin Extra safety cushion for the planner only (m)
   */
  public static Costmap build(
      ObstacleField field,
      double minX,
      double minY,
      double maxX,
      double maxY,
      double cellSize,
      double robotRadius,
      double plannerMargin) {
    if (cellSize <= 0) throw new IllegalArgumentException("cellSize must be positive");
    if (maxX <= minX || maxY <= minY) {
      throw new IllegalArgumentException("Costmap bounds must be ordered");
    }
    if (plannerMargin < 0) {
      throw new IllegalArgumentException("plannerMargin must be non-negative");
    }
    int cols = (int) Math.ceil((maxX - minX) / cellSize);
    int rows = (int) Math.ceil((maxY - minY) / cellSize);
    double[] dist = new double[cols * rows];
    double half = cellSize / 2.0;

    for (int gx = 0; gx < cols; gx++) {
      double x = minX + gx * cellSize + half;
      for (int gy = 0; gy < rows; gy++) {
        double y = minY + gy * cellSize + half;
        dist[gx * rows + gy] = field.signedDistance(x, y);
      }
    }
    return new Costmap(dist, cellSize, minX, minY, cols, rows, robotRadius, plannerMargin);
  }

  /** True if the cell is on the grid and far enough from obstacles for the robot to pass. */
  public boolean isFree(int gx, int gy) {
    if (gx < 0 || gx >= cols || gy < 0 || gy >= rows) return false;
    return distance[gx * rows + gy] >= robotRadius + plannerMargin;
  }

  /** Distance from this cell to the nearest obstacle, in meters. */
  public double clearance(int gx, int gy) {
    if (gx < 0 || gx >= cols || gy < 0 || gy >= rows) return Double.NEGATIVE_INFINITY;
    return distance[gx * rows + gy];
  }

  /** True if a straight line between two cells doesn't cross any blocked cells. */
  public boolean lineOfSight(int gx0, int gy0, int gx1, int gy1) {
    // Walks every cell the line touches. If the line cuts through an exact corner where two
    // obstacles meet, we treat that as blocked too - otherwise the robot would clip the corner.
    if (!isFree(gx0, gy0)) return false;
    if (gx0 == gx1 && gy0 == gy1) return true;

    // Work in cell-units, where each cell is 1x1.
    double x = gx0 + 0.5;
    double y = gy0 + 0.5;
    double dx = (gx1 + 0.5) - x;
    double dy = (gy1 + 0.5) - y;
    int stepX = dx > 0 ? 1 : (dx < 0 ? -1 : 0);
    int stepY = dy > 0 ? 1 : (dy < 0 ? -1 : 0);

    double tMaxX =
        (stepX != 0) ? (Math.floor(x) + (stepX > 0 ? 1 : 0) - x) / dx : Double.POSITIVE_INFINITY;
    double tMaxY =
        (stepY != 0) ? (Math.floor(y) + (stepY > 0 ? 1 : 0) - y) / dy : Double.POSITIVE_INFINITY;
    double tDeltaX = (stepX != 0) ? Math.abs(1.0 / dx) : Double.POSITIVE_INFINITY;
    double tDeltaY = (stepY != 0) ? Math.abs(1.0 / dy) : Double.POSITIVE_INFINITY;

    int cx = gx0;
    int cy = gy0;
    while (cx != gx1 || cy != gy1) {
      if (Math.abs(tMaxX - tMaxY) < 1e-12) {
        // Line passes exactly through a corner - both diagonal neighbors must be clear.
        if (!isFree(cx + stepX, cy) || !isFree(cx, cy + stepY)) return false;
        cx += stepX;
        cy += stepY;
        tMaxX += tDeltaX;
        tMaxY += tDeltaY;
      } else if (tMaxX < tMaxY) {
        cx += stepX;
        tMaxX += tDeltaX;
      } else {
        cy += stepY;
        tMaxY += tDeltaY;
      }
      if (!isFree(cx, cy)) return false;
    }
    return true;
  }

  /** Field-relative x of this cell's center. */
  public double cellCenterX(int gx) {
    return originX + (gx + 0.5) * cellSize;
  }

  /** Field-relative y of this cell's center. */
  public double cellCenterY(int gy) {
    return originY + (gy + 0.5) * cellSize;
  }

  /** Converts a field x-coordinate to a grid column. */
  public int xToCell(double x) {
    int gx = (int) Math.floor((x - originX) / cellSize);
    if (gx < 0) return 0;
    if (gx >= cols) return cols - 1;
    return gx;
  }

  /** Converts a field y-coordinate to a grid row. */
  public int yToCell(double y) {
    int gy = (int) Math.floor((y - originY) / cellSize);
    if (gy < 0) return 0;
    if (gy >= rows) return rows - 1;
    return gy;
  }

  public int cols() {
    return cols;
  }

  public int rows() {
    return rows;
  }

  public double cellSize() {
    return cellSize;
  }

  public double plannerMargin() {
    return plannerMargin;
  }

  public double robotRadius() {
    return robotRadius;
  }
}
