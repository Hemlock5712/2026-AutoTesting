package frc.robot.swm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Computes firing solutions for shooting while the robot is moving.
 *
 * <p>Runtime cost: ~1 μs. Two interpolation lookups + arithmetic. No iteration.
 *
 * <p>Architecture: 1. Decompose robot velocity into radial/tangential components 2. Look up
 * stationary shot: (speed, elevation) from empirical table 3. Look up moving correction:
 * (speedOffset, azimuthOffset) from precomputed table 4. Combine and output
 *
 * <p>The stationary table comes from REAL calibration on your physical robot. The correction table
 * is generated OFFLINE by a Python script (RK4 + Newton-Raphson). Neither simulation nor iteration
 * happens on the roboRIO.
 */
public class ShootWhileMovingSolver {

  private final StationaryShotTable stationaryTable;
  private final MovingCorrectionTable correctionTable;

  public ShootWhileMovingSolver(
      StationaryShotTable stationaryTable, MovingCorrectionTable correctionTable) {
    this.stationaryTable = stationaryTable;
    this.correctionTable = correctionTable;
  }

  /**
   * Compute the firing solution. Call every loop iteration.
   *
   * @param robotPose Current field-relative pose (from pose estimator)
   * @param fieldVelocity Field-relative velocity (m/s). MUST be field-relative, not robot-relative.
   * @param targetPosition Target location on field
   * @return Firing solution with speed, elevation, and turret azimuth
   */
  public ShotSolution solve(
      Pose2d robotPose, Translation2d fieldVelocity, Translation2d targetPosition) {

    // --- Geometry ---
    Translation2d toTarget = targetPosition.minus(robotPose.getTranslation());
    double distance = toTarget.getNorm();
    double angleToTarget = Math.atan2(toTarget.getY(), toTarget.getX());

    // Decompose velocity into radial (toward target) and tangential (perpendicular)
    double rux = toTarget.getX() / distance; // radial unit vector
    double ruy = toTarget.getY() / distance;
    double tux = -ruy; // tangential unit vector (90° CCW)
    double tuy = rux;

    double vRadial = fieldVelocity.getX() * rux + fieldVelocity.getY() * ruy;
    double vTangential = fieldVelocity.getX() * tux + fieldVelocity.getY() * tuy;

    // --- Stationary shot (from real calibration) ---
    double baseSpeed = stationaryTable.getSpeed(distance);
    double elevation = stationaryTable.getElevation(distance);

    // --- Moving correction (from precomputed table) ---
    // Note: Table uses convention where negative vRad = toward target,
    // but our vRadial is positive when toward target, so negate it.
    double[] correction = correctionTable.lookup(distance, -vRadial, vTangential);
    double speedOffset = correction[0];
    double azimuthOffset = correction[1];

    // --- Combine ---
    double finalSpeed = baseSpeed + speedOffset;
    double turretAzimuth = angleToTarget + azimuthOffset;

    return new ShotSolution(
        finalSpeed, elevation, turretAzimuth, distance, vRadial, vTangential, azimuthOffset);
  }

  /**
   * Convenience: accepts ChassisSpeeds instead of Translation2d velocity. Converts robot-relative
   * speeds to field-relative using the robot heading.
   */
  public ShotSolution solve(
      Pose2d robotPose, ChassisSpeeds robotRelativeSpeeds, Translation2d targetPosition) {
    // Convert robot-relative → field-relative
    ChassisSpeeds fieldSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, robotPose.getRotation());
    Translation2d fieldVelocity =
        new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
    return solve(robotPose, fieldVelocity, targetPosition);
  }

  /** Immutable result of the solver. */
  public static class ShotSolution {
    /** Flywheel speed in m/s (base + moving correction) */
    public final double speedMps;

    /** Hood elevation in radians (from stationary table, not modified) */
    public final double elevationRad;

    /** Field-relative turret azimuth in radians */
    public final double turretAzimuthRad;

    /** Distance to target (for telemetry) */
    public final double distanceM;

    /** Radial robot velocity component (for telemetry) */
    public final double vRadialMps;

    /** Tangential robot velocity component (for telemetry) */
    public final double vTangentialMps;

    /** Azimuth offset from direct aim (for validity checking) */
    public final double azimuthOffsetRad;

    public ShotSolution(
        double speedMps,
        double elevationRad,
        double turretAzimuthRad,
        double distanceM,
        double vRadialMps,
        double vTangentialMps,
        double azimuthOffsetRad) {
      this.speedMps = speedMps;
      this.elevationRad = elevationRad;
      this.turretAzimuthRad = turretAzimuthRad;
      this.distanceM = distanceM;
      this.vRadialMps = vRadialMps;
      this.vTangentialMps = vTangentialMps;
      this.azimuthOffsetRad = azimuthOffsetRad;
    }

    /**
     * Check if the solution is likely valid. Returns false if azimuth offset exceeds ~60° (physics
     * limits). Consider waiting to shoot if this returns false.
     */
    public boolean isValid() {
      return Math.abs(azimuthOffsetRad) < 1.1; // ~63 degrees
    }

    /**
     * Check if the shot is ready based on robot velocity. More conservative than isValid() - checks
     * if within comfortable operating range.
     */
    public boolean isReadyToShoot() {
      // Allow up to ~45° azimuth offset for comfortable shots
      return Math.abs(azimuthOffsetRad) < 0.8;
    }

    /** Flywheel RPM given wheel radius. */
    public double getFlywheelRPM(double wheelRadiusM) {
      return (speedMps / (2.0 * Math.PI * wheelRadiusM)) * 60.0;
    }

    /** Flywheel RPS given wheel radius. */
    public double getFlywheelRPS(double wheelRadiusM) {
      return speedMps / (2.0 * Math.PI * wheelRadiusM);
    }

    public double getElevationDeg() {
      return Math.toDegrees(elevationRad);
    }

    public double getTurretAzimuthDeg() {
      return Math.toDegrees(turretAzimuthRad);
    }
  }
}
