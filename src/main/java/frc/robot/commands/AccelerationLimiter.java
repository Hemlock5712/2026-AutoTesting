package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.Motor;

/**
 * Limits how hard the swerve drive can accelerate, based on real motor data.
 *
 * <p>Two limits are applied:
 *
 * <ul>
 *   <li>Motor limit: Motors can't push as hard at high speed (only affects speeding up)
 *   <li>Friction limit: Wheels can't accelerate harder than friction allows ({@code mu * g})
 * </ul>
 *
 * <p>Stops the wheels from slipping while still letting the robot move as fast as possible.
 */
public final class AccelerationLimiter {

  public static final double GRAVITY = 9.81; // m/s^2

  // Distance from robot center to a wheel.
  public static final double DRIVE_BASE_RADIUS =
      Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY);

  // Top speed of the robot.
  public static final double MAX_VELOCITY = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

  // Friction coefficient: ~1.0 for good tread on carpet. Max accel = mu * g.
  public static final double MU_FRICTION = 1.1;
  public static final double MAX_FRICTION_ACCEL = MU_FRICTION * GRAVITY;

  // Drivetrain parameters used to model motor torque.
  static final Motor MOTOR = Motor.KRAKEN_X60_FOC;
  static final double GEAR_RATIO = TunerConstants.FrontLeft.DriveMotorGearRatio;
  static final double WHEEL_RADIUS = TunerConstants.FrontLeft.WheelRadius;
  public static final double ROBOT_MASS = 60; // kg, including bumpers and battery
  static final int NUM_DRIVE_MOTORS = 4;

  // ---- Per-module weight transfer ----
  // Half the wheelbase (front-back) and trackwidth (left-right). Assumes a symmetric chassis.
  public static final double HALF_WHEELBASE = Math.abs(TunerConstants.FrontLeft.LocationX);
  public static final double HALF_TRACKWIDTH = Math.abs(TunerConstants.FrontLeft.LocationY);

  // Center of gravity offset from the chassis center. +X = forward, +Y = left, +Z = up.
  //
  // To measure: balance the robot on a pipe to find cx and cy. For cz, lift one side until it's
  // about to tip and measure the tip angle - cz = HALF_TRACKWIDTH / tan(angle). Tilt slowly!
  //
  // Defaults assume the CoG is centered at 20cm height. Tune after measuring.
  public static final double COG_X = 0.0;
  public static final double COG_Y = 0.0;
  public static final double COG_Z = 0.20;

  // Module positions in the robot frame, ordered FL, FR, BL, BR.
  public static final double[] MODULE_RX = {
    TunerConstants.FrontLeft.LocationX,
    TunerConstants.FrontRight.LocationX,
    TunerConstants.BackLeft.LocationX,
    TunerConstants.BackRight.LocationX,
  };
  public static final double[] MODULE_RY = {
    TunerConstants.FrontLeft.LocationY,
    TunerConstants.FrontRight.LocationY,
    TunerConstants.BackLeft.LocationY,
    TunerConstants.BackRight.LocationY,
  };

  // How much weight is on each wheel when the robot is sitting still. Computed once at startup.
  public static final double[] MODULE_N_STATIC = computeStaticNormals();

  private static double[] computeStaticNormals() {
    double base = ROBOT_MASS * GRAVITY / 4.0;
    double a2 = HALF_WHEELBASE * HALF_WHEELBASE;
    double b2 = HALF_TRACKWIDTH * HALF_TRACKWIDTH;
    double[] n = new double[MODULE_RX.length];
    for (int i = 0; i < n.length; i++) {
      double fx = 1.0 + MODULE_RX[i] * COG_X / a2;
      double fy = 1.0 + MODULE_RY[i] * COG_Y / b2;
      n[i] = base * fx * fy;
    }
    return n;
  }

  // Current limit per motor (more conservative than the hardware limit).
  static final double STATOR_CURRENT_LIMIT = 150.0;

  // Minimum time step to avoid divide-by-zero.
  private static final double MIN_DT = 1e-9;

  // Scratch space for the limited acceleration result. Thread-local since this can be called
  // from both the 50 Hz main loop and the 250 Hz fast loop.
  private static final ThreadLocal<double[]> ACCEL_RESULT =
      ThreadLocal.withInitial(() -> new double[3]);
  // Per-module scratch for normal forces. Same threading rationale as ACCEL_RESULT.
  private static final ThreadLocal<double[]> NORMALS_SCRATCH =
      ThreadLocal.withInitial(() -> new double[MODULE_RX.length]);

  // Last computed acceleration. volatile since multiple threads read/write it.
  // Stored in whatever frame the caller passed in (currently field-frame for all callers).
  private static volatile double lastAccelVx = 0;
  private static volatile double lastAccelVy = 0;
  private static volatile double lastAccelOmega = 0;

  // Last per-module friction utilization (a_i / limit_i). Logged by Drive.
  private static volatile double[] lastModuleFrictionRatios = new double[MODULE_RX.length];

  /** Snapshot of the most recent per-module friction utilization. */
  public static double[] getLastModuleFrictionRatios() {
    return lastModuleFrictionRatios.clone();
  }

  /** Test hook: clears the last-accel state used for weight-transfer estimation. */
  static void resetLastAccelForTest() {
    lastAccelVx = 0;
    lastAccelVy = 0;
    lastAccelOmega = 0;
    lastModuleFrictionRatios = new double[MODULE_RX.length];
  }

  private AccelerationLimiter() {}

  /**
   * Returns the last computed acceleration. The fields are in m/s^2 and rad/s^2 (NOT velocity, even
   * though the type is ChassisSpeeds).
   */
  public static ChassisSpeeds getLastAcceleration() {
    return new ChassisSpeeds(lastAccelVx, lastAccelVy, lastAccelOmega);
  }

  /** X acceleration from the most recent integration step (m/s^2). */
  public static double getLastAccelVx() {
    return lastAccelVx;
  }

  /** Y acceleration from the most recent integration step (m/s^2). */
  public static double getLastAccelVy() {
    return lastAccelVy;
  }

  /** Rotational acceleration from the most recent integration step (rad/s^2). */
  public static double getLastAccelOmega() {
    return lastAccelOmega;
  }

  /**
   * Overrides the most recent acceleration record. Use this when post-processing (e.g. an
   * obstacle-avoidance velocity clamp) modifies the integrator's output and the recorded
   * acceleration must reflect what the chassis is actually doing, not the unclamped delta.
   * Otherwise the next call's weight-transfer estimate drifts whenever the clamp is active.
   */
  public static void setLastAcceleration(double accelVx, double accelVy, double accelOmega) {
    lastAccelVx = accelVx;
    lastAccelVy = accelVy;
    lastAccelOmega = accelOmega;
  }

  /**
   * Applies motor and friction limits to a desired acceleration. Output is written into result as
   * [accelX, accelY, accelOmega].
   *
   * <p>{@code headingRadians} rotates field-frame accel into robot frame for the per-module
   * friction check. Pass {@code 0} when the caller is already operating in robot frame.
   */
  private static void applyLimits(
      double accelX,
      double accelY,
      double accelOmega,
      double velX,
      double velY,
      double velOmega,
      double maxAccel,
      double headingRadians,
      double[] result) {

    // First, limit how hard the motors can push (only matters when speeding up).
    applyMotorLimit(accelX, accelY, accelOmega, velX, velY, velOmega, result);

    // Then limit by friction (matters for both speeding up and braking).
    applyPerModuleFrictionLimit(result[0], result[1], result[2], maxAccel, headingRadians, result);
  }

  /**
   * Limits acceleration based on how much torque the motors can produce at the current speed.
   * Motors push less hard the faster they spin, so we can't accelerate as fast at high speeds.
   * Braking is unaffected (motors don't need to push hard to slow down).
   */
  private static void applyMotorLimit(
      double accelX,
      double accelY,
      double accelOmega,
      double velX,
      double velY,
      double velOmega,
      double[] result) {

    // Are we braking? (Acceleration opposing current velocity.)
    boolean linearBraking = isDecelerating(accelX, accelY, velX, velY);
    boolean angularBraking = accelOmega * velOmega < 0;

    // If we're only braking, no motor limit applies.
    if (linearBraking && angularBraking) {
      result[0] = accelX;
      result[1] = accelY;
      result[2] = accelOmega;
      return;
    }

    // Total accel from speeding-up parts only (braking doesn't count toward motor limit).
    double linearAccelMag = Math.hypot(accelX, accelY);
    double linearContrib = linearBraking ? 0 : linearAccelMag;
    double angularContrib = angularBraking ? 0 : Math.abs(accelOmega) * DRIVE_BASE_RADIUS;
    double combinedAccel = Math.hypot(linearContrib, angularContrib);

    // Worst-case wheel speed (for looking up motor torque at that speed).
    double linearVelMag = Math.hypot(velX, velY);
    double moduleSpeed = linearVelMag + Math.abs(velOmega) * DRIVE_BASE_RADIUS;
    double maxMotorAccel =
        MOTOR.getMaxAcceleration(
            moduleSpeed,
            GEAR_RATIO,
            WHEEL_RADIUS,
            ROBOT_MASS,
            NUM_DRIVE_MOTORS,
            STATOR_CURRENT_LIMIT);

    // Already within limits - pass through unchanged.
    if (combinedAccel <= maxMotorAccel) {
      result[0] = accelX;
      result[1] = accelY;
      result[2] = accelOmega;
      return;
    }

    // Scale down anything that's speeding up so the total fits under the motor limit.
    double scale = maxMotorAccel / combinedAccel;
    double linearScale = linearBraking ? 1.0 : scale;
    double angularScale = angularBraking ? 1.0 : scale;
    result[0] = accelX * linearScale;
    result[1] = accelY * linearScale;
    result[2] = accelOmega * angularScale;
  }

  /** True if we're braking (acceleration opposes velocity). */
  private static boolean isDecelerating(double accelX, double accelY, double velX, double velY) {
    return accelX * velX + accelY * velY < 0;
  }

  /**
   * Limits chassis acceleration so the wheels don't slip. Uses the friction-circle envelope {@code
   * sqrt(a_lin^2 + (alpha*r)^2) <= mu*g} for the chassis-total scaling, and computes per-module
   * over-ratios with weight transfer as a diagnostic (logged to {@code
   * Drive/Diagnostics/FrictionRatios}).
   */
  private static void applyPerModuleFrictionLimit(
      double accelX,
      double accelY,
      double accelOmega,
      double maxAccel,
      double headingRadians,
      double[] result) {
    double effectiveMu = Math.min(MAX_FRICTION_ACCEL, maxAccel) / GRAVITY;
    double effectiveLimit = effectiveMu * GRAVITY;

    double cosH = Math.cos(headingRadians);
    double sinH = Math.sin(headingRadians);
    double axR = accelX * cosH + accelY * sinH;
    double ayR = -accelX * sinH + accelY * cosH;
    double prevAxR = lastAccelVx * cosH + lastAccelVy * sinH;
    double prevAyR = -lastAccelVx * sinH + lastAccelVy * cosH;

    double[] normals = NORMALS_SCRATCH.get();
    normalForcesWithTransfer(prevAxR, prevAyR, normals);

    // Per-module ratios are diagnostic only — they show which corner is closest to its individual
    // friction limit (mu*4*N_i/m), with weight transfer accounted for.
    double[] ratios = new double[MODULE_RX.length];
    for (int i = 0; i < MODULE_RX.length; i++) {
      double aix = axR - accelOmega * MODULE_RY[i];
      double aiy = ayR + accelOmega * MODULE_RX[i];
      double mag = Math.hypot(aix, aiy);
      double limit = effectiveMu * 4.0 * normals[i] / ROBOT_MASS;
      ratios[i] = limit > 1e-9 ? mag / limit : Double.POSITIVE_INFINITY;
    }
    lastModuleFrictionRatios = ratios;

    // Actual scaling: chassis-level friction circle. Linear and angular contributions sum as a
    // 2-vector under the friction limit. This is the same envelope used pre-per-module change.
    double linearMag = Math.hypot(axR, ayR);
    double angularContribution = Math.abs(accelOmega) * DRIVE_BASE_RADIUS;
    double combinedAccel = Math.hypot(linearMag, angularContribution);

    if (combinedAccel > effectiveLimit) {
      double scale = effectiveLimit / combinedAccel;
      result[0] = accelX * scale;
      result[1] = accelY * scale;
      result[2] = accelOmega * scale;
    } else {
      result[0] = accelX;
      result[1] = accelY;
      result[2] = accelOmega;
    }
  }

  /**
   * Per-module acceleration caps (m/s²), one per wheel, for use as the per-call Acceleration
   * argument to Phoenix 6 MotionMagicVelocityVoltage. Each cap is the friction-circle limit that
   * the corresponding wheel can produce given its current normal force, including weight transfer
   * estimated from {@link #getLastAcceleration()}.
   *
   * <p>The cap is in chassis-frame units (m/s²) because that's what the wheel's tangential
   * acceleration is: {@code mu * N_i * 4 / m}. The factor of 4 is the dimensional bridge from "one
   * wheel's normal force out of four" to "chassis accel that wheel can deliver if it were the only
   * one pushing." With all four wheels pushing the chassis can do roughly the sum, but each one's
   * *individual* slip threshold is what we want as the per-module rate cap.
   *
   * <p>Pass the robot-frame chassis velocity so {@code lastAccel} can be rotated correctly if it
   * was logged in a different frame. Currently {@code lastAccel} is field-frame for all callers, so
   * {@code headingRadians} should be the current heading.
   *
   * @param headingRadians robot heading, used to rotate field-frame {@code lastAccel} into the
   *     robot frame for the weight-transfer estimate.
   * @param out length-4 array (FL, FR, BL, BR) written in place with per-module caps in m/s².
   */
  public static void perModuleAccelCaps(double headingRadians, double[] out) {
    double cosH = Math.cos(headingRadians);
    double sinH = Math.sin(headingRadians);
    double prevAxR = lastAccelVx * cosH + lastAccelVy * sinH;
    double prevAyR = -lastAccelVx * sinH + lastAccelVy * cosH;

    double[] normals = NORMALS_SCRATCH.get();
    normalForcesWithTransfer(prevAxR, prevAyR, normals);

    double[] ratios = new double[MODULE_RX.length];
    for (int i = 0; i < MODULE_RX.length; i++) {
      double limit = MU_FRICTION * 4.0 * normals[i] / ROBOT_MASS;
      out[i] = limit;

      double aix = prevAxR - lastAccelOmega * MODULE_RY[i];
      double aiy = prevAyR + lastAccelOmega * MODULE_RX[i];
      double mag = Math.hypot(aix, aiy);
      ratios[i] = limit > 1e-9 ? mag / limit : Double.POSITIVE_INFINITY;
    }
    lastModuleFrictionRatios = ratios;
  }

  /** Allocating convenience wrapper around {@link #perModuleAccelCaps(double, double[])}. */
  public static double[] perModuleAccelCaps(double headingRadians) {
    double[] out = new double[MODULE_RX.length];
    perModuleAccelCaps(headingRadians, out);
    return out;
  }

  /** Per-module normal force = static distribution + dynamic load shift from accel. */
  static void normalForcesWithTransfer(double axRobot, double ayRobot, double[] out) {
    double cz = COG_Z;
    // Per-axle delta is m*a*cz/(2*half), split across the two wheels on that axle (/2 again).
    double dxBase = ROBOT_MASS * axRobot * cz / (4.0 * HALF_WHEELBASE);
    double dyBase = ROBOT_MASS * ayRobot * cz / (4.0 * HALF_TRACKWIDTH);
    for (int i = 0; i < MODULE_RX.length; i++) {
      double signX = Math.signum(MODULE_RX[i]);
      double signY = Math.signum(MODULE_RY[i]);
      double n = MODULE_N_STATIC[i] - dxBase * signX - dyBase * signY;
      out[i] = Math.max(n, 0.0);
    }
  }

  /**
   * Limits jerk - how fast the acceleration itself can change. Keeps the robot from "jerking"
   * forward suddenly. Linear and angular jerk are limited separately.
   */
  private static void applyJerkLimit(
      double accelX,
      double accelY,
      double accelOmega,
      double prevAccelX,
      double prevAccelY,
      double prevAccelOmega,
      double dt,
      double maxLinearJerk,
      double maxOmegaJerk,
      double[] result) {

    // Linear jerk treated as a 2D vector.
    double jerkX = (accelX - prevAccelX) / dt;
    double jerkY = (accelY - prevAccelY) / dt;
    double jerkMag = Math.hypot(jerkX, jerkY);

    if (jerkMag > maxLinearJerk) {
      double scale = maxLinearJerk / jerkMag;
      result[0] = prevAccelX + jerkX * scale * dt;
      result[1] = prevAccelY + jerkY * scale * dt;
    } else {
      result[0] = accelX;
      result[1] = accelY;
    }

    // Angular jerk handled separately.
    double jerkOmega = (accelOmega - prevAccelOmega) / dt;
    if (Math.abs(jerkOmega) > maxOmegaJerk) {
      result[2] = prevAccelOmega + Math.copySign(maxOmegaJerk, jerkOmega) * dt;
    } else {
      result[2] = accelOmega;
    }
  }

  /**
   * Scales speeds down so no individual swerve wheel goes faster than its top speed. Wheel speed is
   * the sum of how fast the robot is translating + how fast it's spinning at the wheel radius.
   */
  public static ChassisSpeeds normalizeSpeeds(ChassisSpeeds speeds) {
    double translationSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    double maxModuleSpeed =
        translationSpeed + Math.abs(speeds.omegaRadiansPerSecond) * DRIVE_BASE_RADIUS;

    if (maxModuleSpeed > MAX_VELOCITY) {
      return speeds.times(MAX_VELOCITY / maxModuleSpeed);
    }
    return speeds;
  }

  /**
   * Same as {@link #normalizeSpeeds} but updates the input directly instead of returning a new
   * object. Use on the 250 Hz hot path to avoid allocations.
   */
  public static void normalizeSpeedsInPlace(ChassisSpeeds speeds) {
    double translationSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    double maxModuleSpeed =
        translationSpeed + Math.abs(speeds.omegaRadiansPerSecond) * DRIVE_BASE_RADIUS;

    if (maxModuleSpeed > MAX_VELOCITY) {
      double scale = MAX_VELOCITY / maxModuleSpeed;
      speeds.vxMetersPerSecond *= scale;
      speeds.vyMetersPerSecond *= scale;
      speeds.omegaRadiansPerSecond *= scale;
    }
  }

  // ==================== Zero-allocation API ====================

  /**
   * Integrates one timestep with physics limits. Reads the current velocity from {@code
   * currentAndOutput} and writes the limited next velocity back to it.
   *
   * @param currentAndOutput Current velocity in/out
   * @param desiredVx Target x velocity
   * @param desiredVy Target y velocity
   * @param desiredOmega Target spin rate
   * @param dt Timestep (seconds)
   */
  public static void integrateVelocityInPlace(
      ChassisSpeeds currentAndOutput,
      double desiredVx,
      double desiredVy,
      double desiredOmega,
      double dt) {
    integrateVelocityCore(
        currentAndOutput,
        currentAndOutput.vxMetersPerSecond,
        currentAndOutput.vyMetersPerSecond,
        currentAndOutput.omegaRadiansPerSecond,
        desiredVx,
        desiredVy,
        desiredOmega,
        dt,
        MAX_FRICTION_ACCEL,
        Double.MAX_VALUE,
        Double.MAX_VALUE,
        0.0);
  }

  /**
   * Same, but with the robot heading (radians) supplied so the per-module friction check can rotate
   * field-frame accel into robot frame. Pass the same frame the velocities are in.
   */
  public static void integrateVelocityInPlace(
      ChassisSpeeds currentAndOutput,
      double desiredVx,
      double desiredVy,
      double desiredOmega,
      double dt,
      double headingRadians) {
    integrateVelocityCore(
        currentAndOutput,
        currentAndOutput.vxMetersPerSecond,
        currentAndOutput.vyMetersPerSecond,
        currentAndOutput.omegaRadiansPerSecond,
        desiredVx,
        desiredVy,
        desiredOmega,
        dt,
        MAX_FRICTION_ACCEL,
        Double.MAX_VALUE,
        Double.MAX_VALUE,
        headingRadians);
  }

  /** Same, but with a custom max acceleration. */
  public static void integrateVelocityInPlace(
      ChassisSpeeds currentAndOutput,
      double desiredVx,
      double desiredVy,
      double desiredOmega,
      double dt,
      double maxAccel,
      double headingRadians) {
    integrateVelocityCore(
        currentAndOutput,
        currentAndOutput.vxMetersPerSecond,
        currentAndOutput.vyMetersPerSecond,
        currentAndOutput.omegaRadiansPerSecond,
        desiredVx,
        desiredVy,
        desiredOmega,
        dt,
        maxAccel,
        Double.MAX_VALUE,
        Double.MAX_VALUE,
        headingRadians);
  }

  /** Same, but with custom acceleration and jerk limits. */
  public static void integrateVelocityInPlace(
      ChassisSpeeds currentAndOutput,
      double desiredVx,
      double desiredVy,
      double desiredOmega,
      double dt,
      double maxAccel,
      double maxLinearJerk,
      double maxOmegaJerk,
      double headingRadians) {
    integrateVelocityCore(
        currentAndOutput,
        currentAndOutput.vxMetersPerSecond,
        currentAndOutput.vyMetersPerSecond,
        currentAndOutput.omegaRadiansPerSecond,
        desiredVx,
        desiredVy,
        desiredOmega,
        dt,
        maxAccel,
        maxLinearJerk,
        maxOmegaJerk,
        headingRadians);
  }

  /**
   * Same as integrateVelocityInPlace but with the current velocity passed separately from the
   * output. Useful when the current velocity isn't what's stored in the output object.
   */
  public static void integrateVelocity(
      ChassisSpeeds output,
      double curVx,
      double curVy,
      double curOmega,
      double desiredVx,
      double desiredVy,
      double desiredOmega,
      double dt) {
    integrateVelocityCore(
        output,
        curVx,
        curVy,
        curOmega,
        desiredVx,
        desiredVy,
        desiredOmega,
        dt,
        MAX_FRICTION_ACCEL,
        Double.MAX_VALUE,
        Double.MAX_VALUE,
        0.0);
  }

  // ==================== Core implementation ====================

  /**
   * The actual logic. Normalizes the desired speeds, computes the desired acceleration, applies
   * motor/friction/jerk limits, then integrates to get the next velocity.
   */
  private static void integrateVelocityCore(
      ChassisSpeeds output,
      double curVx,
      double curVy,
      double curOmega,
      double desVx,
      double desVy,
      double desOmega,
      double dt,
      double maxAccel,
      double maxLinearJerk,
      double maxOmegaJerk,
      double headingRadians) {

    // Cap target speeds so no wheel exceeds its max.
    double transSpeed = Math.hypot(desVx, desVy);
    double maxModSpeed = transSpeed + Math.abs(desOmega) * DRIVE_BASE_RADIUS;
    if (maxModSpeed > MAX_VELOCITY) {
      double scale = MAX_VELOCITY / maxModSpeed;
      desVx *= scale;
      desVy *= scale;
      desOmega *= scale;
    }

    // Guard against zero/negative timestep (can happen on the first loop).
    if (dt < MIN_DT) {
      dt = MIN_DT;
    }

    // Acceleration we want to achieve: (target - current) / dt.
    double accelX = (desVx - curVx) / dt;
    double accelY = (desVy - curVy) / dt;
    double accelOmega = (desOmega - curOmega) / dt;

    // Apply motor and friction limits.
    double[] scratch = ACCEL_RESULT.get();
    applyLimits(
        accelX, accelY, accelOmega, curVx, curVy, curOmega, maxAccel, headingRadians, scratch);

    // Apply jerk limit (compares with last frame's acceleration).
    applyJerkLimit(
        scratch[0],
        scratch[1],
        scratch[2],
        lastAccelVx,
        lastAccelVy,
        lastAccelOmega,
        dt,
        maxLinearJerk,
        maxOmegaJerk,
        scratch);

    // Save for anything that needs the latest acceleration (logging, prediction, etc.).
    lastAccelVx = scratch[0];
    lastAccelVy = scratch[1];
    lastAccelOmega = scratch[2];

    // Step forward: new velocity = current + acceleration * dt.
    output.vxMetersPerSecond = curVx + scratch[0] * dt;
    output.vyMetersPerSecond = curVy + scratch[1] * dt;
    output.omegaRadiansPerSecond = curOmega + scratch[2] * dt;
    normalizeSpeedsInPlace(output);
  }
}
