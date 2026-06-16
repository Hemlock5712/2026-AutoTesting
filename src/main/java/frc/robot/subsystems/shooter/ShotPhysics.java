package frc.robot.subsystems.shooter;

/**
 * Physics forward model for the shooter.
 *
 * <p>Given a flywheel speed, hood angle, and the robot's radial velocity, this integrates the
 * ball's flight (gravity + aerodynamic drag + Magnus/backspin lift) and reports whether the ball
 * drops into the hub at a target distance, and with how much margin.
 *
 * <p>This is the trusted "what shot scores" layer of the HighTide-style targeting system. It is a
 * pure function of the small set of physical constants in the CALIBRATION SURFACE block below. Once
 * those constants are pinned, {@link ShotSolver} inverts this model to build the full lookup table,
 * so a ball change only requires re-pinning {@link #SLIP_EFFICIENCY} rather than re-tuning every
 * distance by hand.
 *
 * <p>Intentionally self-contained (only depends on {@link Math}) so it can run in a plain unit test
 * with no HAL / NetworkTables / WPILib subsystem initialization.
 */
public final class ShotPhysics {
  private ShotPhysics() {}

  // ============================================================
  // ==                  CALIBRATION SURFACE                   ==
  // ==  Tune these so the solver reproduces the known-good     ==
  // ==  stationary table (see ShotModelTest). After that, a    ==
  // ==  single field point re-pins SLIP_EFFICIENCY when balls  ==
  // ==  change. These are mutable (not final) so a future      ==
  // ==  calibration routine can set them at runtime.           ==
  // ============================================================

  /** Fraction of flywheel surface speed transferred to the ball. The compression-sensitive one. */
  public static double SLIP_EFFICIENCY = 0.62;

  /** Backspin model: ball angular rate (rad/s) = SPIN_FACTOR * surfaceSpeed / ballRadius. */
  public static double SPIN_FACTOR = 0.50;

  /** Effective Magnus lift coefficient per unit spin ratio S = (omega * r / v). Empirically fit. */
  public static double LIFT_COEFF_PER_SPIN = 0.30;

  /** Cap on the lift coefficient at high spin ratios (keeps the model sane at low speed). */
  public static double MAX_LIFT_COEFF = 0.35;

  /** Drag coefficient of the ball (smooth foam sphere ~0.4-0.5). Mirrors BallPhysicsSimulation. */
  public static double DRAG_COEFFICIENT = 0.50;

  /** Ball mass (kg). Mirrors BallPhysicsSimulation; verify against the game manual ball spec. */
  public static double BALL_MASS_KG = 0.2268;

  /** Ball diameter (m). Mirrors BallPhysicsSimulation; verify against the game manual ball spec. */
  public static double BALL_DIAMETER_M = 0.15;

  // ============================================================
  // ==                    FIXED GEOMETRY                       ==
  // ============================================================

  /** Flywheel wheel radius (m). Matches BallPhysicsSimulation (2 in). */
  public static final double WHEEL_RADIUS_M = 0.0508;

  /** Launch elevation (deg) = HOOD_ZERO_ELEVATION_DEG - hoodAngleDeg. Matches sim convention. */
  public static final double HOOD_ZERO_ELEVATION_DEG = 75.0;

  /** Ball exit height above the floor (m). Mirrors Superstructure.TURRET_HOLE_CENTER z. */
  public static final double LAUNCH_HEIGHT_M = 0.3556;

  /** Hub rim height (m). Mirrors FieldInfo.HUB_HEIGHT. */
  public static final double GOAL_HEIGHT_M = 1.828;

  /** Horizontal radius of the hub opening the ball must drop into (m). */
  public static double GOAL_OPENING_RADIUS_M = 0.34;

  // ============================================================
  // ==                     ENVIRONMENT                         ==
  // ============================================================

  private static final double GRAVITY = 9.81;
  private static final double AIR_DENSITY = 1.225;

  // ============================================================
  // ==                     INTEGRATION                         ==
  // ============================================================

  private static final double DT = 0.001;
  private static final double MAX_TIME = 3.0;

  /**
   * Outcome of a simulated shot. {@code rangeAtRim} is NaN when the ball never drops to the rim.
   */
  public record ShotResult(
      boolean scored, double rangeAtRim, double timeOfFlight, double margin, double entryAngleDeg) {
    public static final ShotResult MISS = new ShotResult(false, Double.NaN, Double.NaN, -1.0, 0.0);
  }

  private static double ballRadius() {
    return BALL_DIAMETER_M / 2.0;
  }

  /** Flywheel surface speed (m/s) for a given mechanism RPS. */
  public static double surfaceSpeed(double flywheelRps) {
    return flywheelRps * 2.0 * Math.PI * WHEEL_RADIUS_M;
  }

  /** Ball exit speed (m/s) after slip. */
  public static double exitSpeed(double flywheelRps) {
    return SLIP_EFFICIENCY * surfaceSpeed(flywheelRps);
  }

  /** Ball backspin (rad/s). */
  public static double backspinRadPerSec(double flywheelRps) {
    return SPIN_FACTOR * surfaceSpeed(flywheelRps) / ballRadius();
  }

  /**
   * Simulate a shot and report whether the ball drops into the hub at the given target distance.
   *
   * @param flywheelRps flywheel (mechanism) speed
   * @param hoodDeg hood angle (mechanism units; launch elevation = HOOD_ZERO_ELEVATION_DEG - hood)
   * @param targetDistanceM horizontal distance from the launch point to the hub center
   * @param radialVelMps robot velocity component toward the hub (closing is positive)
   */
  public static ShotResult simulate(
      double flywheelRps, double hoodDeg, double targetDistanceM, double radialVelMps) {
    final double elevationRad = Math.toRadians(HOOD_ZERO_ELEVATION_DEG - hoodDeg);
    final double v0 = exitSpeed(flywheelRps);
    final double omega = backspinRadPerSec(flywheelRps);

    double x = 0.0;
    double z = LAUNCH_HEIGHT_M;
    double vx = v0 * Math.cos(elevationRad) + radialVelMps;
    double vz = v0 * Math.sin(elevationRad);

    double t = 0.0;
    boolean pastApex = false;

    while (t < MAX_TIME) {
      double prevX = x;
      double prevZ = z;
      double prevVz = vz;

      // RK4 on the full state [x, z, vx, vz].
      double[] a1 = accel(vx, vz, omega);
      double k1x = vx, k1z = vz, k1vx = a1[0], k1vz = a1[1];

      double vx2 = vx + 0.5 * DT * k1vx, vz2 = vz + 0.5 * DT * k1vz;
      double[] a2 = accel(vx2, vz2, omega);
      double k2x = vx2, k2z = vz2, k2vx = a2[0], k2vz = a2[1];

      double vx3 = vx + 0.5 * DT * k2vx, vz3 = vz + 0.5 * DT * k2vz;
      double[] a3 = accel(vx3, vz3, omega);
      double k3x = vx3, k3z = vz3, k3vx = a3[0], k3vz = a3[1];

      double vx4 = vx + DT * k3vx, vz4 = vz + DT * k3vz;
      double[] a4 = accel(vx4, vz4, omega);
      double k4x = vx4, k4z = vz4, k4vx = a4[0], k4vz = a4[1];

      x += DT * (k1x + 2 * k2x + 2 * k3x + k4x) / 6.0;
      z += DT * (k1z + 2 * k2z + 2 * k3z + k4z) / 6.0;
      vx += DT * (k1vx + 2 * k2vx + 2 * k3vx + k4vx) / 6.0;
      vz += DT * (k1vz + 2 * k2vz + 2 * k3vz + k4vz) / 6.0;
      t += DT;

      if (prevVz > 0 && vz <= 0) {
        pastApex = true;
      }

      // Score on the descending crossing of the rim height (the ball must drop into the opening).
      if (pastApex && prevZ >= GOAL_HEIGHT_M && z < GOAL_HEIGHT_M) {
        double frac = (prevZ - GOAL_HEIGHT_M) / (prevZ - z);
        double xCross = prevX + frac * (x - prevX);
        double tCross = (t - DT) + frac * DT;
        double margin = GOAL_OPENING_RADIUS_M - Math.abs(xCross - targetDistanceM);
        double entryAngleDeg = Math.toDegrees(Math.atan2(-vz, Math.abs(vx)));
        return new ShotResult(margin >= 0.0, xCross, tCross, margin, entryAngleDeg);
      }

      if (z < 0.0) {
        break;
      }
    }
    return ShotResult.MISS;
  }

  /** Acceleration (m/s^2) from gravity + drag + Magnus at velocity (vx, vz) with backspin omega. */
  private static double[] accel(double vx, double vz, double omega) {
    double speed = Math.hypot(vx, vz);
    if (speed < 1e-6) {
      return new double[] {0.0, -GRAVITY};
    }
    double r = ballRadius();
    double area = Math.PI * r * r;

    // Drag: a = -(0.5 rho Cd A / m) * |v| * v   (opposes velocity)
    double dragAccelPerComp = 0.5 * AIR_DENSITY * DRAG_COEFFICIENT * area * speed / BALL_MASS_KG;
    double ax = -dragAccelPerComp * vx;
    double az = -dragAccelPerComp * vz - GRAVITY;

    // Magnus: lift perpendicular to velocity. For backspin, rotate the velocity +90 deg:
    // perp = (-vz, vx) / speed, which has an upward component for forward motion.
    double spinRatio = omega * r / speed;
    double cl = Math.min(LIFT_COEFF_PER_SPIN * spinRatio, MAX_LIFT_COEFF);
    double liftAccel = 0.5 * AIR_DENSITY * area * cl * speed * speed / BALL_MASS_KG;
    ax += liftAccel * (-vz / speed);
    az += liftAccel * (vx / speed);

    return new double[] {ax, az};
  }
}
