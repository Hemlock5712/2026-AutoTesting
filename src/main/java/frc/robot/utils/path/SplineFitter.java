package frc.robot.utils.path;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;

/**
 * Smooth curve that passes through a list of waypoints, sampled evenly along its length.
 *
 * <p>We use centripetal Catmull-Rom for the tangents. This means the curve passes exactly through
 * every waypoint and stays close to the straight-line path between them - it won't bow inward at
 * corners. That matters because Theta* paths hug obstacles, so any bowing would clip them.
 *
 * <p>The curve is smooth (continuous slope) but the curvature can change suddenly at waypoints.
 * That's fine - the velocity profiler just slows down at those spots.
 */
public final class SplineFitter {

  /** How many points we sample per segment before evenly spacing along the arc length. */
  private static final int SAMPLES_PER_SEGMENT = 50;

  /** Catmull-Rom alpha. 0.5 = centripetal, no overshoot at corners. */
  private static final double ALPHA = 0.5;

  /** Result of fitting: x/y/curvature arrays, plus the gap between samples. */
  public record Samples(double[] x, double[] y, double[] curvature, double sampleSpacing) {
    public int length() {
      return x.length;
    }
  }

  private SplineFitter() {}

  /**
   * @param waypoints At least 2 waypoints; first is the curve start, last is the curve end
   * @param sampleSpacing Target arc-length spacing between consecutive output samples (m)
   */
  public static Samples fit(List<Translation2d> waypoints, double sampleSpacing) {
    if (waypoints.size() < 2) {
      throw new IllegalArgumentException("Need at least 2 waypoints, got " + waypoints.size());
    }
    if (sampleSpacing <= 0) {
      throw new IllegalArgumentException("sampleSpacing must be positive, got " + sampleSpacing);
    }
    if (waypoints.size() == 2) {
      return straightLine(waypoints.get(0), waypoints.get(1), sampleSpacing);
    }
    return splineFit(waypoints, sampleSpacing);
  }

  private static Samples straightLine(Translation2d a, Translation2d b, double ds) {
    double dx = b.getX() - a.getX();
    double dy = b.getY() - a.getY();
    double len = Math.hypot(dx, dy);
    int n = Math.max(2, (int) Math.ceil(len / ds) + 1);
    double[] x = new double[n];
    double[] y = new double[n];
    double[] kappa = new double[n];
    for (int i = 0; i < n; i++) {
      double t = (double) i / (n - 1);
      x[i] = a.getX() + t * dx;
      y[i] = a.getY() + t * dy;
    }
    return new Samples(x, y, kappa, len / Math.max(1, n - 1));
  }

  private static Samples splineFit(List<Translation2d> waypoints, double ds) {
    int k = waypoints.size();
    double[] wx = new double[k];
    double[] wy = new double[k];
    for (int i = 0; i < k; i++) {
      wx[i] = waypoints.get(i).getX();
      wy[i] = waypoints.get(i).getY();
    }

    // Compute the curve's direction at each waypoint.
    double[] tx = new double[k];
    double[] ty = new double[k];
    for (int i = 0; i < k; i++) {
      tangentAt(wx, wy, i, tx, ty);
    }

    // Densely sample every segment of the curve.
    int numSegments = k - 1;
    int denseN = numSegments * SAMPLES_PER_SEGMENT + 1;
    double[] denseX = new double[denseN];
    double[] denseY = new double[denseN];
    double[] denseK = new double[denseN];

    int idx = 0;
    for (int seg = 0; seg < numSegments; seg++) {
      int last = (seg == numSegments - 1) ? SAMPLES_PER_SEGMENT + 1 : SAMPLES_PER_SEGMENT;
      // Build a Hermite curve segment from waypoint[seg] to waypoint[seg+1].
      double p0x = wx[seg];
      double p0y = wy[seg];
      double p1x = wx[seg + 1];
      double p1y = wy[seg + 1];
      double t0x = tx[seg];
      double t0y = ty[seg];
      double t1x = tx[seg + 1];
      double t1y = ty[seg + 1];
      for (int j = 0; j < last; j++) {
        double u = (double) j / SAMPLES_PER_SEGMENT;
        evalHermite(p0x, p0y, p1x, p1y, t0x, t0y, t1x, t1y, u, idx, denseX, denseY, denseK);
        idx++;
      }
    }

    // Walk along the curve summing distances, then re-sample evenly.
    double[] cumLen = new double[denseN];
    cumLen[0] = 0.0;
    for (int i = 1; i < denseN; i++) {
      double dxi = denseX[i] - denseX[i - 1];
      double dyi = denseY[i] - denseY[i - 1];
      cumLen[i] = cumLen[i - 1] + Math.hypot(dxi, dyi);
    }
    double totalLen = cumLen[denseN - 1];
    if (totalLen < 1e-9) {
      Translation2d start = waypoints.get(0);
      Translation2d end = waypoints.get(k - 1);
      return new Samples(
          new double[] {start.getX(), end.getX()},
          new double[] {start.getY(), end.getY()},
          new double[] {0.0, 0.0},
          0.0);
    }

    int outN = Math.max(2, (int) Math.ceil(totalLen / ds) + 1);
    double[] outX = new double[outN];
    double[] outY = new double[outN];
    double[] outK = new double[outN];
    outX[0] = denseX[0];
    outY[0] = denseY[0];
    outK[0] = denseK[0];
    outX[outN - 1] = denseX[denseN - 1];
    outY[outN - 1] = denseY[denseN - 1];
    outK[outN - 1] = denseK[denseN - 1];

    int densePtr = 0;
    for (int i = 1; i < outN - 1; i++) {
      double targetS = totalLen * i / (outN - 1);
      while (densePtr < denseN - 2 && cumLen[densePtr + 1] < targetS) densePtr++;
      double segLen = cumLen[densePtr + 1] - cumLen[densePtr];
      double frac = segLen > 1e-12 ? (targetS - cumLen[densePtr]) / segLen : 0.0;
      outX[i] = lerp(denseX[densePtr], denseX[densePtr + 1], frac);
      outY[i] = lerp(denseY[densePtr], denseY[densePtr + 1], frac);
      outK[i] = lerp(denseK[densePtr], denseK[densePtr + 1], frac);
    }

    return new Samples(outX, outY, outK, totalLen / (outN - 1));
  }

  /** Computes the curve direction at waypoint i (centripetal Catmull-Rom). */
  private static void tangentAt(double[] wx, double[] wy, int i, double[] tx, double[] ty) {
    int k = wx.length;
    if (i == 0) {
      // First waypoint: scale the tangent so curvature stays smooth into the first segment.
      double dx = wx[1] - wx[0];
      double dy = wy[1] - wy[0];
      double lenSeg0 = Math.hypot(dx, dy);
      if (k >= 3 && lenSeg0 > 1e-12) {
        double lenSeg1 = Math.hypot(wx[2] - wx[1], wy[2] - wy[1]);
        double targetMag = 0.5 * (lenSeg0 + lenSeg1);
        tx[0] = dx / lenSeg0 * targetMag;
        ty[0] = dy / lenSeg0 * targetMag;
      } else {
        tx[0] = dx;
        ty[0] = dy;
      }
      return;
    }
    if (i == k - 1) {
      double dx = wx[k - 1] - wx[k - 2];
      double dy = wy[k - 1] - wy[k - 2];
      double lenLast = Math.hypot(dx, dy);
      if (k >= 3 && lenLast > 1e-12) {
        double lenPrev = Math.hypot(wx[k - 2] - wx[k - 3], wy[k - 2] - wy[k - 3]);
        double targetMag = 0.5 * (lenPrev + lenLast);
        tx[k - 1] = dx / lenLast * targetMag;
        ty[k - 1] = dy / lenLast * targetMag;
      } else {
        tx[k - 1] = dx;
        ty[k - 1] = dy;
      }
      return;
    }
    // Average the directions in/out of this waypoint, weighted toward shorter segments.
    double dxPrev = wx[i] - wx[i - 1];
    double dyPrev = wy[i] - wy[i - 1];
    double dxNext = wx[i + 1] - wx[i];
    double dyNext = wy[i + 1] - wy[i];
    double lenPrev = Math.hypot(dxPrev, dyPrev);
    double lenNext = Math.hypot(dxNext, dyNext);
    double tPrev = Math.pow(Math.max(lenPrev, 1e-9), ALPHA);
    double tNext = Math.pow(Math.max(lenNext, 1e-9), ALPHA);
    double avgTx = (dxPrev / tPrev * tNext + dxNext / tNext * tPrev) / (tPrev + tNext);
    double avgTy = (dyPrev / tPrev * tNext + dyNext / tNext * tPrev) / (tPrev + tNext);
    // Scale by average segment length so the curve doesn't squeeze or stretch.
    double avgLen = 0.5 * (lenPrev + lenNext);
    double mag = Math.hypot(avgTx, avgTy);
    if (mag < 1e-12) {
      tx[i] = 0.0;
      ty[i] = 0.0;
    } else {
      tx[i] = avgTx / mag * avgLen;
      ty[i] = avgTy / mag * avgLen;
    }
  }

  /** Evaluates one Hermite curve segment at u in [0, 1] and writes x/y/curvature out. */
  private static void evalHermite(
      double p0x,
      double p0y,
      double p1x,
      double p1y,
      double t0x,
      double t0y,
      double t1x,
      double t1y,
      double u,
      int outIdx,
      double[] outX,
      double[] outY,
      double[] outKappa) {
    double u2 = u * u;
    double u3 = u2 * u;

    double h00 = 2.0 * u3 - 3.0 * u2 + 1.0;
    double h10 = u3 - 2.0 * u2 + u;
    double h01 = -2.0 * u3 + 3.0 * u2;
    double h11 = u3 - u2;

    double dh00 = 6.0 * u2 - 6.0 * u;
    double dh10 = 3.0 * u2 - 4.0 * u + 1.0;
    double dh01 = -6.0 * u2 + 6.0 * u;
    double dh11 = 3.0 * u2 - 2.0 * u;

    double ddh00 = 12.0 * u - 6.0;
    double ddh10 = 6.0 * u - 4.0;
    double ddh01 = -12.0 * u + 6.0;
    double ddh11 = 6.0 * u - 2.0;

    outX[outIdx] = h00 * p0x + h10 * t0x + h01 * p1x + h11 * t1x;
    outY[outIdx] = h00 * p0y + h10 * t0y + h01 * p1y + h11 * t1y;

    double dx = dh00 * p0x + dh10 * t0x + dh01 * p1x + dh11 * t1x;
    double dy = dh00 * p0y + dh10 * t0y + dh01 * p1y + dh11 * t1y;
    double ddx = ddh00 * p0x + ddh10 * t0x + ddh01 * p1x + ddh11 * t1x;
    double ddy = ddh00 * p0y + ddh10 * t0y + ddh01 * p1y + ddh11 * t1y;

    double speed2 = dx * dx + dy * dy;
    if (speed2 < 1e-18) {
      outKappa[outIdx] = 0.0;
    } else {
      double speed = Math.sqrt(speed2);
      outKappa[outIdx] = (dx * ddy - dy * ddx) / (speed2 * speed);
    }
  }

  private static double lerp(double a, double b, double t) {
    return a + t * (b - a);
  }
}
