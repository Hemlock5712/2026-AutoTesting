package frc.robot.swm;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * Empirical calibration table for stationary shots.
 *
 * <p>Maps distance (meters) -> (flywheel speed, hood angle).
 *
 * <p>This is populated by REAL TESTING on the physical robot: 1. Place robot at measured distance
 * from target 2. Tune flywheel speed and hood angle until shots consistently score 3. Record the
 * values with addPoint()
 *
 * <p>This table IS your ground truth. It captures everything the simulation can't: ball
 * compression, spin variation, launcher geometry, air currents.
 *
 * <p>Recommend calibrating every 0.5m across your shooting range.
 */
public class StationaryShotTable {

  private final InterpolatingDoubleTreeMap speedTable = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap angleTable = new InterpolatingDoubleTreeMap();

  private double minDistance = Double.MAX_VALUE;
  private double maxDistance = Double.MIN_VALUE;

  /**
   * Add a calibration point from real-world testing.
   *
   * @param distanceM Distance to target in meters (measure with tape)
   * @param speedMps Ball exit speed in m/s that consistently scores
   * @param elevationRad Hood angle in radians from horizontal
   */
  public void addPoint(double distanceM, double speedMps, double elevationRad) {
    speedTable.put(distanceM, speedMps);
    angleTable.put(distanceM, elevationRad);
    minDistance = Math.min(minDistance, distanceM);
    maxDistance = Math.max(maxDistance, distanceM);
  }

  /** Convenience: angle in degrees, speed in RPM. */
  public void addPointRPM(
      double distanceM, double flywheelRPM, double wheelRadiusM, double elevationDeg) {
    double speedMps = flywheelRPM * 2.0 * Math.PI * wheelRadiusM / 60.0;
    addPoint(distanceM, speedMps, Math.toRadians(elevationDeg));
  }

  /** Convenience: angle in degrees, speed in RPS. */
  public void addPointRPS(
      double distanceM, double flywheelRPS, double wheelRadiusM, double elevationDeg) {
    double speedMps = flywheelRPS * 2.0 * Math.PI * wheelRadiusM;
    addPoint(distanceM, speedMps, Math.toRadians(elevationDeg));
  }

  /** Get elevation in degrees. */
  public double getElevationDeg(double distanceM) {
    return Math.toDegrees(angleTable.get(clamp(distanceM)));
  }

  public double getSpeed(double distanceM) {
    return speedTable.get(clamp(distanceM));
  }

  public double getElevation(double distanceM) {
    return angleTable.get(clamp(distanceM));
  }

  public double getMinDistance() {
    return minDistance;
  }

  public double getMaxDistance() {
    return maxDistance;
  }

  private double clamp(double d) {
    return Math.max(minDistance, Math.min(maxDistance, d));
  }
}
