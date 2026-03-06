package frc.robot.swm;

/**
 * Pre-computed 3D correction table for shooting while moving.
 *
 * <p>Maps (distance, vRadial, vTangential) → (speedOffset, azimuthOffset)
 *
 * <p>Generated OFFLINE by the Python script using RK4 simulation + Newton-Raphson. At runtime, this
 * is just a trilinear interpolation — microseconds, no iteration.
 *
 * <p>The table exploits tangential symmetry: only positive vTangential values are stored. Negative
 * vTangential flips the azimuth offset sign.
 *
 * <p>Typical size: 10 × 17 × 9 × 2 × 8 bytes = ~24 KB.
 */
public class MovingCorrectionTable {

  private final double[] distPoints; // sorted, e.g. [2.5, 3.0, 3.5, ..., 7.0]
  private final double[] vRadPoints; // sorted, e.g. [-4, -3.5, ..., 0, ..., 3.5, 4]
  private final double[] vTanPoints; // sorted, non-negative, e.g. [0, 0.5, ..., 4.0]
  private final double[][][] speedOffsets; // [dist][vRad][vTan]
  private final double[][][] azOffsets; // [dist][vRad][vTan]

  /**
   * @param distPoints Sorted distance grid points (meters)
   * @param vRadPoints Sorted radial velocity grid points (m/s, negative = toward target)
   * @param vTanPoints Sorted tangential velocity grid points (m/s, non-negative only)
   * @param speedOffsets 3D array of speed corrections (m/s) indexed [dist][vRad][vTan]
   * @param azOffsets 3D array of azimuth corrections (radians) indexed [dist][vRad][vTan]
   */
  public MovingCorrectionTable(
      double[] distPoints,
      double[] vRadPoints,
      double[] vTanPoints,
      double[][][] speedOffsets,
      double[][][] azOffsets) {
    this.distPoints = distPoints;
    this.vRadPoints = vRadPoints;
    this.vTanPoints = vTanPoints;
    this.speedOffsets = speedOffsets;
    this.azOffsets = azOffsets;
  }

  /**
   * Look up the correction for a given scenario.
   *
   * @param distance Distance to target (meters)
   * @param vRadial Robot radial velocity (m/s, negative = toward target)
   * @param vTangential Robot tangential velocity (m/s, either sign)
   * @return double[2]: {speedOffset (m/s), azimuthOffset (radians)}
   */
  public double[] lookup(double distance, double vRadial, double vTangential) {
    // Clamp to table bounds
    double d = clamp(distance, distPoints[0], distPoints[distPoints.length - 1]);
    double vr = clamp(vRadial, vRadPoints[0], vRadPoints[vRadPoints.length - 1]);
    double avt = Math.abs(vTangential);
    avt = clamp(avt, vTanPoints[0], vTanPoints[vTanPoints.length - 1]);
    double vtSign = vTangential >= 0 ? 1.0 : -1.0;

    // Find bracket indices and fractional positions
    int di = bracket(distPoints, d);
    double df = frac(distPoints, di, d);

    int vri = bracket(vRadPoints, vr);
    double vrf = frac(vRadPoints, vri, vr);

    int vti = bracket(vTanPoints, avt);
    double vtf = frac(vTanPoints, vti, avt);

    // Trilinear interpolation
    double speedOff = trilinear(speedOffsets, di, df, vri, vrf, vti, vtf);
    double azOff = trilinear(azOffsets, di, df, vri, vrf, vti, vtf) * vtSign;

    return new double[] {speedOff, azOff};
  }

  /** Find index i such that arr[i] <= val <= arr[i+1]. */
  private static int bracket(double[] arr, double val) {
    for (int i = 0; i < arr.length - 1; i++) {
      if (arr[i] <= val && val <= arr[i + 1]) return i;
    }
    return arr.length - 2;
  }

  /** Fractional position within bracket. */
  private static double frac(double[] arr, int i, double val) {
    if (i + 1 >= arr.length || arr[i + 1] == arr[i]) return 0.0;
    return (val - arr[i]) / (arr[i + 1] - arr[i]);
  }

  /** Trilinear interpolation in a 3D array. */
  private double trilinear(
      double[][][] data, int di, double df, int vri, double vrf, int vti, double vtf) {
    int di1 = Math.min(di + 1, distPoints.length - 1);
    int vri1 = Math.min(vri + 1, vRadPoints.length - 1);
    int vti1 = Math.min(vti + 1, vTanPoints.length - 1);

    // Interpolate along vTan
    double c00 = data[di][vri][vti] + vtf * (data[di][vri][vti1] - data[di][vri][vti]);
    double c01 = data[di][vri1][vti] + vtf * (data[di][vri1][vti1] - data[di][vri1][vti]);
    double c10 = data[di1][vri][vti] + vtf * (data[di1][vri][vti1] - data[di1][vri][vti]);
    double c11 = data[di1][vri1][vti] + vtf * (data[di1][vri1][vti1] - data[di1][vri1][vti]);

    // Interpolate along vRad
    double c0 = c00 + vrf * (c01 - c00);
    double c1 = c10 + vrf * (c11 - c10);

    // Interpolate along distance
    return c0 + df * (c1 - c0);
  }

  private static double clamp(double val, double min, double max) {
    return Math.max(min, Math.min(max, val));
  }
}
