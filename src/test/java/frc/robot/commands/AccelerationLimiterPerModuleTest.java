package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Java mirror of {@code research/slip_skid/test_physics.py}. Exercises the per-module friction
 * physics ported into {@link AccelerationLimiter}: weight transfer, per-module accel limit, and the
 * comparison against the old scalar mu*g check.
 *
 * <p>Notes on the port:
 *
 * <ul>
 *   <li>Static normal forces use the {@code MODULE_N_STATIC} computed at startup (CoG defaults to
 *       centered, so each is mg/4).
 *   <li>Heading is passed in as the field-to-robot rotation. heading=0 means inputs are robot
 *       frame.
 * </ul>
 */
class AccelerationLimiterPerModuleTest {

  private static final double TOL = 1e-6;
  private static final double MASS = AccelerationLimiter.ROBOT_MASS;
  private static final double G = AccelerationLimiter.GRAVITY;
  private static final double MG_4 = MASS * G / 4.0;

  // Static state in AccelerationLimiter (lastAccel*) leaks between tests; reset before each.
  @BeforeEach
  void reset() {
    AccelerationLimiter.resetLastAccelForTest();
  }

  // ---------- Static load distribution ----------

  @Test
  void staticCenteredCogEqualLoads() {
    // Default CoG is centered, so each wheel carries mg/4.
    for (double n : AccelerationLimiter.MODULE_N_STATIC) {
      assertEquals(MG_4, n, TOL, "centered CoG should give mg/4 per wheel");
    }
    double total = 0;
    for (double n : AccelerationLimiter.MODULE_N_STATIC) total += n;
    assertEquals(MASS * G, total, TOL, "vertical equilibrium");
  }

  // ---------- Dynamic weight transfer ----------

  @Test
  void weightTransferZeroAccelEqualsStatic() {
    double[] n = new double[4];
    AccelerationLimiter.normalForcesWithTransfer(0.0, 0.0, n);
    for (double ni : n) assertEquals(MG_4, ni, TOL, "zero accel = static");
  }

  @Test
  void weightTransferForwardAccelShiftsToRear() {
    double[] n = new double[4];
    AccelerationLimiter.normalForcesWithTransfer(5.0, 0.0, n); // +ax robot frame
    double nFL = n[0];
    double nFR = n[1];
    double nBL = n[2];
    double nBR = n[3];
    assertTrue(nFL < MG_4, "front loses load, got " + nFL);
    assertTrue(nBL > MG_4, "rear gains load, got " + nBL);
    assertEquals(nFL, nFR, TOL, "left/right symmetric");
    assertEquals(nBL, nBR, TOL, "left/right symmetric");
    assertEquals(MASS * G, nFL + nFR + nBL + nBR, TOL, "total = mg");
  }

  @Test
  void weightTransferLeftAccelShiftsToRight() {
    double[] n = new double[4];
    AccelerationLimiter.normalForcesWithTransfer(0.0, 5.0, n); // +ay = leftward
    double nFL = n[0];
    double nFR = n[1];
    double nBL = n[2];
    double nBR = n[3];
    assertTrue(nFR > MG_4, "right gains load");
    assertTrue(nFL < MG_4, "left loses load");
    assertEquals(nFL, nBL, TOL, "front/back symmetric on left");
    assertEquals(nFR, nBR, TOL, "front/back symmetric on right");
    assertEquals(MASS * G, nFL + nFR + nBL + nBR, TOL, "total = mg");
  }

  @Test
  void weightTransferDiagonalAccel() {
    // Forward+left -> shifts to back-right.
    double[] n = new double[4];
    AccelerationLimiter.normalForcesWithTransfer(5.0, 5.0, n);
    double max = -1, min = Double.POSITIVE_INFINITY;
    int maxI = -1, minI = -1;
    for (int i = 0; i < 4; i++) {
      if (n[i] > max) {
        max = n[i];
        maxI = i;
      }
      if (n[i] < min) {
        min = n[i];
        minI = i;
      }
    }
    assertEquals(3, maxI, "BR (idx 3) should have most weight under +ax +ay");
    assertEquals(0, minI, "FL (idx 0) should have least weight");
  }

  @Test
  void weightTransferClampsAtZero() {
    // Huge accel should not produce negative normal forces.
    double[] n = new double[4];
    AccelerationLimiter.normalForcesWithTransfer(100.0, 0.0, n);
    for (double ni : n) {
      assertTrue(ni >= 0, "N must clamp >= 0, got " + ni);
    }
  }

  // ---------- Friction limit through integrateVelocityCore ----------

  @Test
  void noScalingWhenUnderLimit() {
    // 3 m/s^2 << mu*g = 10.79
    ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);
    double dt = 0.02;
    AccelerationLimiter.integrateVelocityInPlace(speeds, 3.0 * dt, 0.0, 0.0, dt, 0.0);
    assertEquals(3.0 * dt, speeds.vxMetersPerSecond, 1e-9, "no scaling expected");
    double[] ratios = AccelerationLimiter.getLastModuleFrictionRatios();
    for (double r : ratios) {
      assertTrue(r < 1.0, "ratio should be under 1, got " + r);
    }
  }

  @Test
  void scalesWhenOverLimit() {
    // Request 20 m/s^2 (well over mu*g). After scaling, magnitude should be at the limit.
    ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);
    double dt = 0.02;
    // Use a custom maxAccel = MAX_FRICTION_ACCEL to keep the motor limit out of the way.
    AccelerationLimiter.integrateVelocityInPlace(
        speeds, 20.0 * dt, 0.0, 0.0, dt, AccelerationLimiter.MAX_FRICTION_ACCEL, 0.0);
    double appliedAccel = speeds.vxMetersPerSecond / dt;
    // Motor limit may clip below mu*g at low speed; assert applied is at-or-below friction limit
    // and that we actually scaled down from the request.
    assertTrue(
        appliedAccel <= AccelerationLimiter.MAX_FRICTION_ACCEL + 1e-6,
        "applied " + appliedAccel + " should not exceed friction limit");
    assertTrue(appliedAccel < 20.0 - 1e-3, "request was scaled down");
  }

  @Test
  void perModuleCatchesAlignedTranslationPlusRotation() {
    // Per-module check should fire on aligned translation + rotation. With ax=8 and alpha=12:
    // FR module sees a = (8 + 12*track/2, 12*wheelbase/2) ≈ (11.31, 3.31) -> mag ≈ 11.79 > mu*g.
    ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);
    double dt = 0.02;
    double ax = 8.0;
    double alpha = 12.0;
    AccelerationLimiter.integrateVelocityInPlace(
        speeds, ax * dt, 0.0, alpha * dt, dt, AccelerationLimiter.MAX_FRICTION_ACCEL, 0.0);
    double[] ratios = AccelerationLimiter.getLastModuleFrictionRatios();
    double maxRatio = 0;
    for (double r : ratios) maxRatio = Math.max(maxRatio, r);
    // ratios are pre-scale, so the worst should be > 1.0 to indicate scaling fired.
    assertTrue(maxRatio > 1.0, "expected pre-scale max > 1, ratios were " + str(ratios));
  }

  @Test
  void weightTransferMakesFrontMoreRestrictiveDuringForwardAccel() {
    // Prime lastAccel with a strong forward accel so the next call sees front-loaded transfer.
    ChassisSpeeds prime = new ChassisSpeeds(0, 0, 0);
    double dt = 0.02;
    // Push hard forward to leave a high lastAccelVx.
    AccelerationLimiter.integrateVelocityInPlace(
        prime, 8.0 * dt, 0.0, 0.0, dt, AccelerationLimiter.MAX_FRICTION_ACCEL, 0.0);

    // Now request a higher accel - per-module check uses the lastAccel for weight transfer.
    ChassisSpeeds speeds = new ChassisSpeeds(prime.vxMetersPerSecond, 0, 0);
    AccelerationLimiter.integrateVelocityInPlace(
        speeds,
        prime.vxMetersPerSecond + 10.0 * dt,
        0.0,
        0.0,
        dt,
        AccelerationLimiter.MAX_FRICTION_ACCEL,
        0.0);
    double[] ratios = AccelerationLimiter.getLastModuleFrictionRatios();
    double frontRatio = Math.max(ratios[0], ratios[1]);
    double rearRatio = Math.max(ratios[2], ratios[3]);
    assertTrue(
        frontRatio > rearRatio,
        "front should be more constrained during forward accel: front="
            + frontRatio
            + " rear="
            + rearRatio);
  }

  // ---------- Frame rotation ----------

  @Test
  void headingRotationDoesNotChangePureTranslationLimit() {
    // For pure translation with centered CoG and zero alpha, rotating the input by any heading
    // should give the same scaled magnitude (heading just rotates which module is "front").
    double[] headings = {0.0, Math.PI / 6, Math.PI / 4, Math.PI / 2, Math.PI};
    double dt = 0.02;
    double targetAccel = 20.0; // over limit
    Double prevApplied = null;
    for (double h : headings) {
      // Reset per-iteration so weight-transfer term doesn't carry over between headings.
      AccelerationLimiter.resetLastAccelForTest();
      ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);
      AccelerationLimiter.integrateVelocityInPlace(
          speeds, targetAccel * dt, 0.0, 0.0, dt, AccelerationLimiter.MAX_FRICTION_ACCEL, h);
      double applied = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) / dt;
      if (prevApplied != null) {
        assertEquals(
            prevApplied,
            applied,
            1e-3,
            "applied accel magnitude should be heading-invariant for pure translation");
      }
      prevApplied = applied;
    }
  }

  private static String str(double[] a) {
    StringBuilder sb = new StringBuilder("[");
    for (int i = 0; i < a.length; i++) {
      if (i > 0) sb.append(", ");
      sb.append(String.format("%.4f", a[i]));
    }
    sb.append("]");
    return sb.toString();
  }
}
