package frc.robot.subsystems.shooter;

/**
 * Runtime shooter lookup for the HighTide-style targeting system. Allocation-free, no HAL/WPILib
 * dependencies, safe to call on the 50 Hz loop.
 *
 * <ul>
 *   <li>{@link #hoodDeg(double)} — hood angle from geometric distance (quadratic).
 *   <li>{@link #flywheelRps(double, double)} — flywheel speed, radial-velocity compensated (2D
 *       bilinear table).
 *   <li>{@link #tofSeconds(double, double)} — physically real time-of-flight for the turret's
 *       tangential lead (2D bilinear table). Replaces the hand-fudged {@code tofMult}.
 * </ul>
 *
 * <p>Inputs are clamped to the calibrated envelope to prevent extrapolation blow-up. The values
 * below are baked by {@link ShotMapGenerator}; {@link ShotMapTest} regenerates and fails on drift,
 * so re-run the generator and update these whenever the {@link ShotPhysics} constants change.
 */
public final class ShooterMap {
  private ShooterMap() {}

  // ====================== BAKED BY ShotMapGenerator ======================
  static final double[] HOOD_COEFFS = {-6.3365385, 7.3663836, -0.52297702};

  static final double[][] FLYWHEEL_GRID = {
    {57.727, 38.470, 29.457, 29.457, 29.457, 29.457, 29.457},
    {52.652, 39.301, 31.549, 29.027, 29.027, 29.027, 29.027},
    {50.806, 40.316, 33.518, 30.134, 30.134, 30.134, 30.134},
    {50.283, 41.485, 35.333, 31.641, 30.134, 30.134, 30.134},
    {50.498, 42.746, 37.055, 33.272, 31.211, 30.565, 30.565},
    {51.114, 44.100, 38.747, 34.933, 32.533, 31.365, 31.365},
    {52.036, 45.515, 40.408, 36.563, 33.979, 32.410, 31.703},
    {53.205, 47.022, 42.039, 38.194, 35.425, 33.610, 32.564},
    {54.528, 48.591, 43.700, 39.824, 36.932, 34.871, 33.549},
    {56.005, 50.221, 45.361, 41.454, 38.409, 36.194, 34.594},
    {57.635, 51.913, 47.053, 43.054, 39.916, 37.517, 35.733},
    {59.419, 53.667, 48.776, 44.684, 41.393, 38.840, 36.871},
  };

  static final double[][] TOF_GRID = {
    {1.9480, 1.2011, 0.69472, 0.69472, 0.69472, 0.69472, 0.69472},
    {1.7395, 1.2131, 0.82037, 0.56412, 0.56412, 0.56412, 0.56412},
    {1.6426, 1.2301, 0.90713, 0.67145, 0.67145, 0.67145, 0.67145},
    {1.5933, 1.2514, 0.97345, 0.75741, 0.60071, 0.60071, 0.60071},
    {1.5714, 1.2753, 1.0290, 0.83060, 0.67641, 0.56690, 0.56690},
    {1.5651, 1.3025, 1.0799, 0.89531, 0.74603, 0.63060, 0.63060},
    {1.5715, 1.3327, 1.1278, 0.95311, 0.81086, 0.69290, 0.60048},
    {1.5890, 1.3677, 1.1743, 1.0086, 0.86902, 0.75371, 0.66085},
    {1.6148, 1.4070, 1.2232, 1.0636, 0.92788, 0.81158, 0.71706},
    {1.6493, 1.4514, 1.2739, 1.1195, 0.98443, 0.87013, 0.77139},
    {1.6931, 1.5013, 1.3285, 1.1758, 1.0434, 0.92789, 0.82917},
    {1.7463, 1.5572, 1.3874, 1.2360, 1.1029, 0.98656, 0.88616},
  };
  // =======================================================================

  // Axes mirror ShotMapGenerator's grid.
  private static final double D0 = ShotMapGenerator.MIN_DIST_M;
  private static final double D_STEP = ShotMapGenerator.DIST_STEP_M;
  private static final double V0 = ShotMapGenerator.MIN_RADIAL_VEL;
  private static final double V_STEP = ShotMapGenerator.RADIAL_VEL_STEP;

  public static final double MIN_DIST_M = ShotMapGenerator.MIN_DIST_M;
  public static final double MAX_DIST_M = ShotMapGenerator.MAX_DIST_M;
  public static final double MIN_RADIAL_VEL = ShotMapGenerator.MIN_RADIAL_VEL;
  public static final double MAX_RADIAL_VEL = ShotMapGenerator.MAX_RADIAL_VEL;

  /** Hood angle (deg) for a geometric target distance. Distance is clamped to the envelope. */
  public static double hoodDeg(double distanceM) {
    double d = clamp(distanceM, MIN_DIST_M, MAX_DIST_M);
    return HOOD_COEFFS[0] + HOOD_COEFFS[1] * d + HOOD_COEFFS[2] * d * d;
  }

  /** Flywheel speed (RPS), radial-velocity compensated. Inputs clamped to the envelope. */
  public static double flywheelRps(double distanceM, double radialVelMps) {
    return bilinear(FLYWHEEL_GRID, distanceM, radialVelMps);
  }

  /** Physical time-of-flight (s) for the shot. Inputs clamped to the envelope. */
  public static double tofSeconds(double distanceM, double radialVelMps) {
    return bilinear(TOF_GRID, distanceM, radialVelMps);
  }

  /** Whether the requested point is inside the calibrated envelope (before clamping). */
  public static boolean inEnvelope(double distanceM, double radialVelMps) {
    return distanceM >= MIN_DIST_M
        && distanceM <= MAX_DIST_M
        && radialVelMps >= MIN_RADIAL_VEL
        && radialVelMps <= MAX_RADIAL_VEL;
  }

  private static double bilinear(double[][] grid, double distanceM, double radialVelMps) {
    double d = clamp(distanceM, MIN_DIST_M, MAX_DIST_M);
    double v = clamp(radialVelMps, MIN_RADIAL_VEL, MAX_RADIAL_VEL);

    double fi = (d - D0) / D_STEP;
    double fj = (v - V0) / V_STEP;
    int i = clampIndex((int) Math.floor(fi), grid.length);
    int j = clampIndex((int) Math.floor(fj), grid[0].length);
    double td = fi - i;
    double tv = fj - j;

    double v00 = grid[i][j];
    double v01 = grid[i][j + 1];
    double v10 = grid[i + 1][j];
    double v11 = grid[i + 1][j + 1];
    double top = v00 + (v01 - v00) * tv;
    double bottom = v10 + (v11 - v10) * tv;
    return top + (bottom - top) * td;
  }

  /** Clamp a floor index into {@code [0, length - 2]} so the +1 neighbor is always valid. */
  private static int clampIndex(int idx, int length) {
    if (idx < 0) {
      return 0;
    }
    if (idx > length - 2) {
      return length - 2;
    }
    return idx;
  }

  private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }
}
