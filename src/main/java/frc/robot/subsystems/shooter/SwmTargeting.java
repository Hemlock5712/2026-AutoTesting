package frc.robot.subsystems.shooter;

/**
 * Shoot-while-moving + tilt-compensated targeting math for hub shots, as pure primitive functions
 * (no HAL/WPILib), so it is fully unit-testable.
 *
 * <p>Given the turret's field position and velocity, the target, and the robot's orientation
 * (yaw/pitch/roll), it returns the turret angle, hood angle, flywheel speed, and time-of-flight:
 *
 * <ul>
 *   <li>Radial velocity (toward/away from the target) is compensated by the flywheel via {@link
 *       ShooterMap#flywheelRps}, not by moving the target.
 *   <li>Tangential velocity (across the line of sight) is compensated by leading the turret aim.
 *   <li>Robot pitch/roll are compensated by transforming the desired field-frame launch direction
 *       into the robot frame, so the turret/hood produce the correct field trajectory while tilted
 *       (e.g. on the bump or under defense).
 * </ul>
 *
 * <p>The WPILib-specific kinematics (latency compensation, omega x r turret velocity, acceleration
 * prediction) live in {@code Superstructure}, which feeds the resulting primitives in here.
 */
public final class SwmTargeting {
  private SwmTargeting() {}

  private static final double TWO_PI = 2.0 * Math.PI;

  /** Result of a targeting solve. Angles in the units the mechanisms consume. */
  public record Aim(
      double turretAngleRot,
      double hoodDeg,
      double flywheelRps,
      double tofSeconds,
      double distanceM,
      double radialVelMps,
      boolean feasible) {}

  /**
   * Solve a hub shot.
   *
   * @param turretX,turretY turret position on the field (m)
   * @param turretVx,turretVy turret velocity on the field (m/s), already latency/omega compensated
   * @param targetX,targetY target position on the field (m)
   * @param robotYawRad robot heading (rad)
   * @param pitchRad,rollRad robot tilt from the IMU (rad); pass 0 to disable tilt compensation
   * @param slipEfficiency live exit-speed-per-surface-speed fraction. Only scales the flywheel
   *     command, so it can be tuned at runtime (NetworkTables) with no map regeneration — the hood
   *     schedule and time-of-flight are slip-independent. Pass {@link ShotPhysics#SLIP_EFFICIENCY}
   *     (the value the map was baked with) for the nominal shot.
   */
  public static Aim solve(
      double turretX,
      double turretY,
      double turretVx,
      double turretVy,
      double targetX,
      double targetY,
      double robotYawRad,
      double pitchRad,
      double rollRad,
      double slipEfficiency) {
    double dx = targetX - turretX;
    double dy = targetY - turretY;
    double dist = Math.hypot(dx, dy);
    if (dist < 1e-6) {
      return new Aim(0, ShooterMap.hoodDeg(0), ShooterMap.flywheelRps(0, 0), 0, 0, 0, false);
    }
    double azimuthToTarget = Math.atan2(dy, dx);

    // Desired ball velocity (field) = the stationary scoring shot at this distance. Giving the ball
    // this exact velocity reproduces the stationary trajectory (and its robust entry angle) no
    // matter how the robot is moving. This target EXIT SPEED is slip-independent (a function of the
    // baked map), so it uses the reference slip the map was generated with.
    double hoodLevelDeg = ShooterMap.hoodDeg(dist);
    double v0 = ShotPhysics.exitSpeed(ShooterMap.flywheelRps(dist, 0.0));
    double elevation = Math.toRadians(ShotPhysics.HOOD_ZERO_ELEVATION_DEG - hoodLevelDeg);
    double cosE = Math.cos(elevation);
    double desiredX = v0 * cosE * Math.cos(azimuthToTarget);
    double desiredY = v0 * cosE * Math.sin(azimuthToTarget);
    double desiredZ = v0 * Math.sin(elevation);

    // The shooter must supply (desired ball velocity - inherited robot velocity). This single vector
    // subtraction handles radial and tangential motion together, exactly, with no lead
    // approximation (robot velocity is horizontal, so it does not change the vertical component).
    double sx = desiredX - turretVx;
    double sy = desiredY - turretVy;
    double sz = desiredZ;
    double shooterSpeed = Math.sqrt(sx * sx + sy * sy + sz * sz);
    // Convert required exit speed to flywheel RPS with the LIVE slip (the runtime tuning knob).
    double flywheelRps =
        shooterSpeed / (slipEfficiency * 2.0 * Math.PI * ShotPhysics.WHEEL_RADIUS_M);

    // Tilt compensation: rotate the shooter's field-frame direction into the robot frame.
    double inv = 1.0 / shooterSpeed;
    double[] robotDir = fieldToRobot(sx * inv, sy * inv, sz * inv, robotYawRad, pitchRad, rollRad);
    double azimuthRobot = Math.atan2(robotDir[1], robotDir[0]);
    double elevationRobot = Math.atan2(robotDir[2], Math.hypot(robotDir[0], robotDir[1]));

    double hoodDeg = ShotPhysics.HOOD_ZERO_ELEVATION_DEG - Math.toDegrees(elevationRobot);
    double turretAngleRot = inputModulus(azimuthRobot / TWO_PI, -0.25, 0.75);

    // The ball flies the stationary trajectory, so its flight time is the stationary one.
    double tof = ShooterMap.tofSeconds(dist, 0.0);
    double vRadial =
        turretVx * Math.cos(azimuthToTarget) + turretVy * Math.sin(azimuthToTarget);
    boolean feasible =
        ShooterMap.inEnvelope(dist, 0.0)
            && flywheelRps >= ShotSolver.FW_MIN_RPS
            && flywheelRps <= ShotSolver.FW_MAX_RPS
            && hoodDeg >= ShotSolver.HOOD_MIN_DEG
            && hoodDeg <= ShotSolver.HOOD_MAX_DEG;

    return new Aim(turretAngleRot, hoodDeg, flywheelRps, tof, dist, vRadial, feasible);
  }

  /** Convenience overload using the reference (baked) slip — for tests and the nominal shot. */
  public static Aim solve(
      double turretX,
      double turretY,
      double turretVx,
      double turretVy,
      double targetX,
      double targetY,
      double robotYawRad,
      double pitchRad,
      double rollRad) {
    return solve(turretX, turretY, turretVx, turretVy, targetX, targetY, robotYawRad, pitchRad,
        rollRad, ShotPhysics.SLIP_EFFICIENCY);
  }

  /**
   * Transform a unit direction from the field frame into the robot frame: {@code d_robot = R^T
   * d_field} where {@code R = Rz(yaw) Ry(pitch) Rx(roll)}.
   */
  static double[] fieldToRobot(
      double x, double y, double z, double yawRad, double pitchRad, double rollRad) {
    // Rz(-yaw)
    double cz = Math.cos(yawRad);
    double sz = Math.sin(yawRad);
    double x1 = x * cz + y * sz;
    double y1 = -x * sz + y * cz;
    double z1 = z;
    // Ry(-pitch)
    double cp = Math.cos(pitchRad);
    double sp = Math.sin(pitchRad);
    double x2 = x1 * cp - z1 * sp;
    double z2 = x1 * sp + z1 * cp;
    // Rx(-roll)
    double cr = Math.cos(rollRad);
    double sr = Math.sin(rollRad);
    double y3 = y1 * cr + z2 * sr;
    double z3 = -y1 * sr + z2 * cr;
    return new double[] {x2, y3, z3};
  }

  /** Inverse of {@link #fieldToRobot}: {@code d_field = R d_robot}. Used for tests. */
  static double[] robotToField(
      double x, double y, double z, double yawRad, double pitchRad, double rollRad) {
    // Rx(roll)
    double cr = Math.cos(rollRad);
    double sr = Math.sin(rollRad);
    double y1 = y * cr - z * sr;
    double z1 = y * sr + z * cr;
    // Ry(pitch)
    double cp = Math.cos(pitchRad);
    double sp = Math.sin(pitchRad);
    double x2 = x * cp + z1 * sp;
    double z2 = -x * sp + z1 * cp;
    // Rz(yaw)
    double cz = Math.cos(yawRad);
    double sz = Math.sin(yawRad);
    double x3 = x2 * cz - y1 * sz;
    double y3 = x2 * sz + y1 * cz;
    return new double[] {x3, y3, z2};
  }

  static double inputModulus(double value, double min, double max) {
    double mod = max - min;
    double r = (value - min) % mod;
    if (r < 0) {
      r += mod;
    }
    return r + min;
  }
}
