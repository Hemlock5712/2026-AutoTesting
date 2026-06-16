package frc.robot.subsystems.shooter;

/**
 * Offline generator for the HighTide-style shooter lookup.
 *
 * <p>It sweeps {@link ShotSolver} over a grid of target distances and robot radial velocities and
 * produces:
 *
 * <ul>
 *   <li>{@code hood(distance)} — a quadratic-fit robust hood schedule (chosen at zero velocity);
 *   <li>{@code flywheel(distance, radialVel)} — a 2D table, the part radial motion changes;
 *   <li>{@code tof(distance, radialVel)} — a 2D table of physically real time-of-flight (replaces
 *       the {@code tofMult} fudge), used by the turret for tangential lead.
 * </ul>
 *
 * <p>This realizes the split the runtime always wanted: the hood ranges off the geometric distance
 * while the flywheel is radial-velocity compensated. Flywheel/ToF use a 2D table rather than a
 * global polynomial because the surface is non-quadratic (a single quadratic underfits by ~10 RPS,
 * mostly at the close-range-while-receding corner); a bilinear table is exact at the nodes.
 *
 * <p>Run {@link #main} to print the values to bake into {@link ShooterMap}; {@link ShotMapTest}
 * regenerates and fails if they drift. Pure (no HAL/WPILib), so it runs in a unit test directly.
 */
public final class ShotMapGenerator {
  private ShotMapGenerator() {}

  // Calibrated envelope. Keep ShooterMap's axes/clamp bounds in sync with these.
  public static final double MIN_DIST_M = 1.0;
  public static final double MAX_DIST_M = 6.5;
  public static final double DIST_STEP_M = 0.5;
  public static final double MIN_RADIAL_VEL = -2.0;
  public static final double MAX_RADIAL_VEL = 4.0;
  public static final double RADIAL_VEL_STEP = 1.0;

  public static final int N_DIST = (int) Math.round((MAX_DIST_M - MIN_DIST_M) / DIST_STEP_M) + 1;
  public static final int N_VEL =
      (int) Math.round((MAX_RADIAL_VEL - MIN_RADIAL_VEL) / RADIAL_VEL_STEP) + 1;

  /** Generated lookup data. Grids are {@code [distanceIndex][radialVelIndex]}, edge-filled. */
  public record Generated(
      double[] hoodCoeffs, // quadratic in distance: [1, d, d^2]
      double[][] flywheelGrid,
      double[][] tofGrid,
      double maxHoodResidualDeg,
      int gridPointsFeasible,
      int gridPointsTotal) {}

  public static Generated generate() {
    // --- Hood schedule: most robust hood at each distance (zero velocity), quadratic-fit. ---
    double[] hoodSamples = new double[N_DIST];
    double[][] hoodFeatures = new double[N_DIST][];
    for (int i = 0; i < N_DIST; i++) {
      double d = distanceAt(i);
      ShotSolver.Shot shot = ShotSolver.solve(d, 0.0);
      if (!shot.valid()) {
        throw new IllegalStateException("No robust shot found at distance " + d + " m");
      }
      hoodSamples[i] = shot.hoodDeg();
      hoodFeatures[i] = poly1(d);
    }
    double[] hoodCoeffs = LeastSquares.fit(hoodFeatures, hoodSamples);
    double maxHoodRes = maxResidual(hoodFeatures, hoodSamples, hoodCoeffs);

    // --- Flywheel & ToF tables, hood fixed by the fitted schedule. ---
    double[][] flywheelGrid = new double[N_DIST][N_VEL];
    double[][] tofGrid = new double[N_DIST][N_VEL];
    int feasible = 0;
    for (int i = 0; i < N_DIST; i++) {
      double d = distanceAt(i);
      double hood = eval(hoodCoeffs, poly1(d));
      for (int j = 0; j < N_VEL; j++) {
        double vr = radialVelAt(j);
        // Cheap path: just the on-target flywheel and its flight time (no make-window scan).
        double fw = ShotSolver.onTargetFlywheel(hood, d, vr);
        if (Double.isNaN(fw)) {
          flywheelGrid[i][j] = Double.NaN;
          tofGrid[i][j] = Double.NaN;
          continue;
        }
        ShotPhysics.ShotResult r = ShotPhysics.simulate(fw, hood, d, vr);
        flywheelGrid[i][j] = fw;
        tofGrid[i][j] = r.timeOfFlight();
        feasible++;
      }
    }
    edgeFillRows(flywheelGrid);
    edgeFillRows(tofGrid);

    return new Generated(hoodCoeffs, flywheelGrid, tofGrid, maxHoodRes, feasible, N_DIST * N_VEL);
  }

  public static double distanceAt(int i) {
    return MIN_DIST_M + i * DIST_STEP_M;
  }

  public static double radialVelAt(int j) {
    return MIN_RADIAL_VEL + j * RADIAL_VEL_STEP;
  }

  /**
   * Replace NaN (infeasible) cells by carrying the nearest feasible value outward along each
   * distance row. Infeasible corners (e.g. closing fast at close range) are outside the makeable
   * region; the runtime feasibility gate rejects them, but a filled table keeps interpolation
   * valid.
   */
  private static void edgeFillRows(double[][] grid) {
    for (double[] row : grid) {
      int firstFeasible = -1;
      int lastFeasible = -1;
      for (int j = 0; j < row.length; j++) {
        if (!Double.isNaN(row[j])) {
          if (firstFeasible < 0) {
            firstFeasible = j;
          }
          lastFeasible = j;
        }
      }
      if (firstFeasible < 0) {
        throw new IllegalStateException("Distance row has no feasible shot at any radial velocity");
      }
      for (int j = 0; j < firstFeasible; j++) {
        row[j] = row[firstFeasible];
      }
      for (int j = lastFeasible + 1; j < row.length; j++) {
        row[j] = row[lastFeasible];
      }
    }
  }

  /** Feature vector for a quadratic in one variable. */
  public static double[] poly1(double d) {
    return new double[] {1.0, d, d * d};
  }

  /** Evaluate a polynomial given its coefficients and the matching feature vector. */
  public static double eval(double[] coeffs, double[] features) {
    double sum = 0.0;
    for (int i = 0; i < coeffs.length; i++) {
      sum += coeffs[i] * features[i];
    }
    return sum;
  }

  private static double maxResidual(double[][] features, double[] y, double[] coeffs) {
    double max = 0.0;
    for (int i = 0; i < y.length; i++) {
      max = Math.max(max, Math.abs(eval(coeffs, features[i]) - y[i]));
    }
    return max;
  }

  public static void main(String[] args) {
    long start = System.nanoTime();
    Generated gen = generate();
    double seconds = (System.nanoTime() - start) / 1e9;

    System.out.println("// === Generated by ShotMapGenerator (bake into ShooterMap) ===");
    System.out.println("// generation time: " + String.format("%.2f", seconds) + " s");
    System.out.println(
        "// hood max residual: "
            + String.format("%.3f", gen.maxHoodResidualDeg())
            + " deg; feasible grid points: "
            + gen.gridPointsFeasible()
            + "/"
            + gen.gridPointsTotal());
    System.out.println(formatArray("HOOD_COEFFS", gen.hoodCoeffs()));
    System.out.println(formatGrid("FLYWHEEL_GRID", gen.flywheelGrid()));
    System.out.println(formatGrid("TOF_GRID", gen.tofGrid()));

    System.out.println("\n// distance | hood | flywheel(vr=0) | tof(vr=0)");
    int vZero = (int) Math.round((0.0 - MIN_RADIAL_VEL) / RADIAL_VEL_STEP);
    for (int i = 0; i < N_DIST; i++) {
      System.out.printf(
          "//  %.1f m   | %.1f  | %.1f | %.3f%n",
          distanceAt(i),
          eval(gen.hoodCoeffs(), poly1(distanceAt(i))),
          gen.flywheelGrid()[i][vZero],
          gen.tofGrid()[i][vZero]);
    }
  }

  private static String formatArray(String name, double[] a) {
    StringBuilder sb = new StringBuilder("  static final double[] " + name + " = {");
    for (int i = 0; i < a.length; i++) {
      sb.append(String.format("%.8g", a[i]));
      if (i < a.length - 1) {
        sb.append(", ");
      }
    }
    return sb.append("};").toString();
  }

  private static String formatGrid(String name, double[][] g) {
    StringBuilder sb = new StringBuilder("  static final double[][] " + name + " = {\n");
    for (double[] row : g) {
      sb.append("    {");
      for (int j = 0; j < row.length; j++) {
        sb.append(String.format("%.5g", row[j]));
        if (j < row.length - 1) {
          sb.append(", ");
        }
      }
      sb.append("},\n");
    }
    return sb.append("  };").toString();
  }

  /** Minimal ordinary-least-squares solver via normal equations + Gaussian elimination. */
  static final class LeastSquares {
    private LeastSquares() {}

    /** Fit coefficients c minimizing ||X c - y||^2. {@code X} rows are feature vectors. */
    static double[] fit(double[][] x, double[] y) {
      int k = x[0].length;
      double[][] a = new double[k][k];
      double[] b = new double[k];
      for (int row = 0; row < x.length; row++) {
        for (int p = 0; p < k; p++) {
          b[p] += x[row][p] * y[row];
          for (int q = 0; q < k; q++) {
            a[p][q] += x[row][p] * x[row][q];
          }
        }
      }
      return solve(a, b);
    }

    private static double[] solve(double[][] a, double[] b) {
      int k = b.length;
      for (int col = 0; col < k; col++) {
        int pivot = col;
        for (int r = col + 1; r < k; r++) {
          if (Math.abs(a[r][col]) > Math.abs(a[pivot][col])) {
            pivot = r;
          }
        }
        double[] tmpRow = a[col];
        a[col] = a[pivot];
        a[pivot] = tmpRow;
        double tmpB = b[col];
        b[col] = b[pivot];
        b[pivot] = tmpB;

        double diag = a[col][col];
        for (int r = 0; r < k; r++) {
          if (r == col) {
            continue;
          }
          double factor = a[r][col] / diag;
          for (int c = col; c < k; c++) {
            a[r][c] -= factor * a[col][c];
          }
          b[r] -= factor * b[col];
        }
      }
      double[] result = new double[k];
      for (int i = 0; i < k; i++) {
        result[i] = b[i] / a[i][i];
      }
      return result;
    }
  }
}
