package frc.robot.utils.path;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.PriorityQueue;

/**
 * Finds a path through a {@link Costmap}, allowing any angle (not just 45-degree turns).
 *
 * <p>This is like A*, but smarter: instead of always stepping cell-by-cell, it shortcuts straight
 * lines through open space whenever possible. Output is a list of waypoints. Feed those into a
 * spline fitter to make a smooth path.
 */
public final class ThetaStar {

  /** 8-connected neighbor offsets. */
  private static final int[] DX = {-1, 0, 1, -1, 1, -1, 0, 1};

  private static final int[] DY = {-1, -1, -1, 0, 0, 1, 1, 1};

  /** Result of a search. */
  public record Result(boolean success, List<Translation2d> waypoints) {
    public static Result failure() {
      return new Result(false, List.of());
    }
  }

  private ThetaStar() {}

  /**
   * Searches for a path from {@code start} to {@code goal}.
   *
   * <p>The first and last waypoints are the exact start and goal. Anything in between is a corner
   * cell where the path turns. Returns a failure result if no path exists.
   */
  public static Result search(Costmap map, Translation2d start, Translation2d goal) {
    int startGx = map.xToCell(start.getX());
    int startGy = map.yToCell(start.getY());
    int goalGx = map.xToCell(goal.getX());
    int goalGy = map.yToCell(goal.getY());

    if (!map.isFree(startGx, startGy) || !map.isFree(goalGx, goalGy)) {
      return Result.failure();
    }

    int cols = map.cols();
    int rows = map.rows();
    int n = cols * rows;
    int startIdx = idx(startGx, startGy, cols);
    int goalIdx = idx(goalGx, goalGy, cols);

    if (startIdx == goalIdx) {
      return new Result(true, List.of(start, goal));
    }

    double[] g = new double[n];
    int[] parent = new int[n];
    boolean[] closed = new boolean[n];
    Arrays.fill(g, Double.POSITIVE_INFINITY);
    Arrays.fill(parent, -1);

    g[startIdx] = 0.0;
    parent[startIdx] = startIdx; // self-parent so the first step behaves like normal A*

    PriorityQueue<Node> open = new PriorityQueue<>();
    open.add(new Node(startIdx, heuristic(startGx, startGy, goalGx, goalGy, map.cellSize())));

    double cellSize = map.cellSize();

    while (!open.isEmpty()) {
      Node cur = open.poll();
      int s = cur.idx;
      if (closed[s]) continue; // already processed
      closed[s] = true;
      if (s == goalIdx) {
        return reconstruct(parent, startIdx, goalIdx, map, cols, start, goal);
      }

      int sx = s % cols;
      int sy = s / cols;
      int sParent = parent[s];
      int pgx = sParent % cols;
      int pgy = sParent / cols;

      for (int d = 0; d < 8; d++) {
        int nx = sx + DX[d];
        int ny = sy + DY[d];
        if (!map.isFree(nx, ny)) continue;
        // Don't cut diagonal corners between obstacles - would clip the robot through them.
        if (DX[d] != 0
            && DY[d] != 0
            && (!map.isFree(sx + DX[d], sy) || !map.isFree(sx, sy + DY[d]))) {
          continue;
        }
        int sp = idx(nx, ny, cols);
        if (closed[sp]) continue;

        // Theta* trick: if our grandparent has a clear line to this neighbor, skip the parent.
        double candidateG;
        int candidateParent;
        if (map.lineOfSight(pgx, pgy, nx, ny)) {
          candidateG = g[sParent] + euclidean(pgx, pgy, nx, ny, cellSize);
          candidateParent = sParent;
        } else {
          candidateG = g[s] + euclidean(sx, sy, nx, ny, cellSize);
          candidateParent = s;
        }

        if (candidateG < g[sp]) {
          g[sp] = candidateG;
          parent[sp] = candidateParent;
          double f = candidateG + heuristic(nx, ny, goalGx, goalGy, cellSize);
          open.add(new Node(sp, f));
        }
      }
    }

    return Result.failure();
  }

  private static int idx(int gx, int gy, int cols) {
    return gy * cols + gx;
  }

  private static double heuristic(int gx, int gy, int gxGoal, int gyGoal, double cellSize) {
    return Math.hypot(gx - gxGoal, gy - gyGoal) * cellSize;
  }

  private static double euclidean(int gx0, int gy0, int gx1, int gy1, double cellSize) {
    return Math.hypot(gx0 - gx1, gy0 - gy1) * cellSize;
  }

  private static Result reconstruct(
      int[] parent,
      int startIdx,
      int goalIdx,
      Costmap map,
      int cols,
      Translation2d start,
      Translation2d goal) {
    List<Translation2d> reversed = new ArrayList<>();
    int idx = goalIdx;
    int safety = parent.length + 1;
    while (idx != startIdx && safety-- > 0) {
      int gx = idx % cols;
      int gy = idx / cols;
      reversed.add(new Translation2d(map.cellCenterX(gx), map.cellCenterY(gy)));
      int p = parent[idx];
      if (p < 0 || p == idx) break;
      idx = p;
    }

    Collections.reverse(reversed);

    // Replace the first/last grid-cell waypoints with the exact start and goal positions.
    List<Translation2d> waypoints = new ArrayList<>(reversed.size() + 1);
    waypoints.add(start);
    for (int i = 0; i < reversed.size() - 1; i++) {
      waypoints.add(reversed.get(i));
    }
    waypoints.add(goal);
    return new Result(true, waypoints);
  }

  private record Node(int idx, double f) implements Comparable<Node> {
    @Override
    public int compareTo(Node o) {
      return Double.compare(f, o.f);
    }
  }
}
