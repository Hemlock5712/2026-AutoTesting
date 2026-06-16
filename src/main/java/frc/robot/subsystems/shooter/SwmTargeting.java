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

  private static final double AIR_DENSITY = 1.225; // kg/m^3
  private static final double MIN_EFFECTIVE_RADIAL_SPEED = 0.5; // ball must outrun the robot
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
      double rollRad) {
    double dx = targetX - turretX;
    double dy = targetY - turretY;
    double dist = Math.hypot(dx, dy);
    if (dist < 1e-6) {
      return new Aim(0, ShooterMap.hoodDeg(0), ShooterMap.flywheelRps(0, 0), 0, 0, 0, false);
    }

    // Radial (toward target) / tangential (across) split of the turret's velocity.
    double aimX = dx / dist;
    double aimY = dy / dist;
    double vRadial = turretVx * aimX + turretVy * aimY;
    double vTanX = turretVx - vRadial * aimX;
    double vTanY = turretVy - vRadial * aimY;

    // Flight time for this geometric distance and radial velocity.
    double tof = ShooterMap.tofSeconds(dist, vRadial);

    // Tangential lead with a drag correction: the inherited tangential velocity is the whole
    // tangential airspeed, so it decays over the flight (radial drag is already in the model).
    double effRadialSpeed = dist / tof + vRadial;
    double vTanMag = Math.hypot(vTanX, vTanY);
    double vRef = Math.sqrt(effRadialSpeed * effRadialSpeed + vTanMag * vTanMag);
    double beta = kDrag() * vRef / ShotPhysics.BALL_MASS_KG;
    double tofEff = (beta > 1e-8) ? (1.0 - Math.exp(-beta * tof)) / beta : tof;

    // Aim point in the field, led opposite the tangential drift.
    double leadX = targetX - vTanX * tofEff;
    double leadY = targetY - vTanY * tofEff;
    double azimuthField = Math.atan2(leadY - turretY, leadX - turretX);

    // Desired field-frame launch elevation from the hood schedule.
    double hoodLevelDeg = ShooterMap.hoodDeg(dist);
    double elevationField = Math.toRadians(ShotPhysics.HOOD_ZERO_ELEVATION_DEG - hoodLevelDeg);

    // Tilt compensation: rotate the desired field direction into the robot frame.
    double cosE = Math.cos(elevationField);
    double[] robotDir =
        fieldToRobot(
            cosE * Math.cos(azimuthField),
            cosE * Math.sin(azimuthField),
            Math.sin(elevationField),
            robotYawRad,
            pitchRad,
            rollRad);
    double azimuthRobot = Math.atan2(robotDir[1], robotDir[0]);
    double elevationRobot = Math.atan2(robotDir[2], Math.hypot(robotDir[0], robotDir[1]));

    double hoodDeg = ShotPhysics.HOOD_ZERO_ELEVATION_DEG - Math.toDegrees(elevationRobot);
    double turretAngleRot = inputModulus(azimuthRobot / TWO_PI, -0.25, 0.75);
    double flywheelRps = ShooterMap.flywheelRps(dist, vRadial);

    boolean feasible =
        ShooterMap.inEnvelope(dist, vRadial) && effRadialSpeed > MIN_EFFECTIVE_RADIAL_SPEED;

    return new Aim(turretAngleRot, hoodDeg, flywheelRps, tof, dist, vRadial, feasible);
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

  private static double kDrag() {
    double rBall = ShotPhysics.BALL_DIAMETER_M / 2.0;
    double area = Math.PI * rBall * rBall;
    return 0.5 * AIR_DENSITY * ShotPhysics.DRAG_COEFFICIENT * area;
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
