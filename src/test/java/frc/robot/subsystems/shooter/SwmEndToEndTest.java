package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

/**
 * Closed-loop end-to-end check: for a sweep of robot states (moving and tilted), run the real
 * {@link SwmTargeting} solve, then fire the resulting (turret angle, hood, flywheel) commands
 * through an INDEPENDENT 3D ball simulation (gravity + drag + Magnus) with the robot's inherited
 * velocity and tilt, and confirm the ball drops into the hub opening.
 *
 * <p>This is the test that catches frame / sign / lead / integration bugs the per-function tests
 * cannot — it exercises the whole targeting chain against physics. (It mirrors the offline Python
 * harness in tools/shot_model.)
 */
class SwmEndToEndTest {

  private static final double G = 9.81;
  private static final double RHO = 1.225;
  private static final double OPENING = ShotPhysics.GOAL_OPENING_RADIUS_M;
  private static final double RIM = ShotPhysics.GOAL_HEIGHT_M;
  private static final double H0 = ShotPhysics.LAUNCH_HEIGHT_M;

  private static double[] accel3d(double vx, double vy, double vz, double om,
      double wx, double wy, double wz) {
    double sp = Math.sqrt(vx * vx + vy * vy + vz * vz);
    if (sp < 1e-6) {
      return new double[] {0, 0, -G};
    }
    double r = ShotPhysics.BALL_DIAMETER_M / 2.0;
    double area = Math.PI * r * r;
    double drag = 0.5 * RHO * ShotPhysics.DRAG_COEFFICIENT * area * sp / ShotPhysics.BALL_MASS_KG;
    double ax = -drag * vx;
    double ay = -drag * vy;
    double az = -drag * vz - G;
    double cl = Math.min(ShotPhysics.LIFT_COEFF_PER_SPIN * om * r / sp, ShotPhysics.MAX_LIFT_COEFF);
    double k = 0.5 * RHO * area * cl * sp / ShotPhysics.BALL_MASS_KG;
    return new double[] {ax + k * (wy * vz - wz * vy), ay + k * (wz * vx - wx * vz),
        az + k * (wx * vy - wy * vx)};
  }

  /** Fire the commanded shot; return {x, y} where it crosses the rim height descending, or null. */
  private static double[] fire(double fw, double hoodDeg, double turretRot, double tvx, double tvy,
      double yaw, double pitch, double roll) {
    double v0 = ShotPhysics.exitSpeed(fw);
    double az = turretRot * 2 * Math.PI;
    double el = Math.toRadians(ShotPhysics.HOOD_ZERO_ELEVATION_DEG - hoodDeg);
    double[] dr = {Math.cos(el) * Math.cos(az), Math.cos(el) * Math.sin(az), Math.sin(el)};
    double[] df = SwmTargeting.robotToField(dr[0], dr[1], dr[2], yaw, pitch, roll);
    double vx = v0 * df[0] + tvx;
    double vy = v0 * df[1] + tvy;
    double vz = v0 * df[2];
    double x = 0, y = 0, z = H0, dt = 0.001, t = 0;
    boolean apex = false;
    double azf = Math.atan2(df[1], df[0]);
    double wx = Math.sin(azf), wy = -Math.cos(azf), wz = 0;
    double om = ShotPhysics.backspinRadPerSec(fw);
    while (t < 3.0) {
      double pz = z, pvz = vz, px = x, py = y;
      double[] a1 = accel3d(vx, vy, vz, om, wx, wy, wz);
      double[] a2 = accel3d(vx + .5 * dt * a1[0], vy + .5 * dt * a1[1], vz + .5 * dt * a1[2], om, wx, wy, wz);
      double[] a3 = accel3d(vx + .5 * dt * a2[0], vy + .5 * dt * a2[1], vz + .5 * dt * a2[2], om, wx, wy, wz);
      double[] a4 = accel3d(vx + dt * a3[0], vy + dt * a3[1], vz + dt * a3[2], om, wx, wy, wz);
      x += dt * (vx + 2 * (vx + .5 * dt * a1[0]) + 2 * (vx + .5 * dt * a2[0]) + (vx + dt * a3[0])) / 6;
      y += dt * (vy + 2 * (vy + .5 * dt * a1[1]) + 2 * (vy + .5 * dt * a2[1]) + (vy + dt * a3[1])) / 6;
      z += dt * (vz + 2 * (vz + .5 * dt * a1[2]) + 2 * (vz + .5 * dt * a2[2]) + (vz + dt * a3[2])) / 6;
      vx += dt * (a1[0] + 2 * a2[0] + 2 * a3[0] + a4[0]) / 6;
      vy += dt * (a1[1] + 2 * a2[1] + 2 * a3[1] + a4[1]) / 6;
      vz += dt * (a1[2] + 2 * a2[2] + 2 * a3[2] + a4[2]) / 6;
      t += dt;
      if (pvz > 0 && vz <= 0) {
        apex = true;
      }
      if (apex && pz >= RIM && z < RIM) {
        double f = (pz - RIM) / (pz - z);
        return new double[] {px + f * (x - px), py + f * (y - py)};
      }
      if (z < 0) {
        break;
      }
    }
    return null;
  }

  private final List<String> failures = new ArrayList<>();

  private void check(String label, double tvx, double tvy, double yaw, double pitch, double roll,
      double d) {
    SwmTargeting.Aim aim = SwmTargeting.solve(0, 0, tvx, tvy, d, 0, yaw, pitch, roll);
    if (!aim.feasible()) {
      return; // out of envelope / unreachable; the runtime gate rejects these
    }
    double[] land = fire(aim.flywheelRps(), aim.hoodDeg(), aim.turretAngleRot(), tvx, tvy,
        yaw, pitch, roll);
    double miss = (land == null) ? Double.POSITIVE_INFINITY : Math.hypot(land[0] - d, land[1]);
    if (miss > OPENING) {
      failures.add(String.format("%s d=%.1f: miss=%.2fm", label, d, miss));
    }
  }

  @Test
  void everyFeasibleShotLandsInTheHub() {
    double[] dists = {1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5, 5.5, 6};
    for (double d : dists) {
      check("stationary", 0, 0, 0, 0, 0, d);
      for (double yaw : new double[] {0, 0.7, -1.2}) {
        check("yaw", 0, 0, yaw, 0, 0, d);
      }
      for (double vr : new double[] {-2, -1, 1, 2, 3}) {
        check("radial", vr, 0, 0.4, 0, 0, d);
      }
      for (double vt : new double[] {-3, -2, -1, 1, 2, 3}) {
        check("tangential", 0, vt, 0.4, 0, 0, d);
      }
      for (double[] v : new double[][] {{1.5, 1.5}, {-1.5, 2}, {2, -2}}) {
        check("diagonal", v[0], v[1], 0.4, 0, 0, d);
      }
      for (double[] pr : new double[][] {{0.15, 0}, {-0.15, 0}, {0, 0.15}, {0.12, -0.12}}) {
        check("tilt", 0, 0, 0.4, pr[0], pr[1], d);
      }
      for (double[] v : new double[][] {{2, 1.5, 0.12, 0}, {-1, -2, 0, 0.12}}) {
        check("moving+tilt", v[0], v[1], 0.4, v[2], v[3], d);
      }
    }
    assertTrue(failures.isEmpty(), "shots that missed the hub opening: " + failures);
  }
}
