package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.subsystems.shooter.ShotPhysics.ShotResult;
import org.junit.jupiter.api.Test;

/** Forward-model invariants for {@link ShotPhysics}. */
class ShotPhysicsTest {

  private static final double RIM = ShotPhysics.GOAL_HEIGHT_M;

  @Test
  void exitSpeedScalesWithSlipAndRps() {
    double surface = ShotPhysics.surfaceSpeed(40.0);
    assertEquals(40.0 * 2.0 * Math.PI * ShotPhysics.WHEEL_RADIUS_M, surface, 1e-9);
    assertEquals(ShotPhysics.SLIP_EFFICIENCY * surface, ShotPhysics.exitSpeed(40.0), 1e-9);
    assertTrue(ShotPhysics.surfaceSpeed(50.0) > ShotPhysics.surfaceSpeed(30.0));
  }

  @Test
  void higherFlywheelTravelsFarther() {
    double near = ShotPhysics.simulate(35.0, 14.0, 4.0, 0.0).rangeAtRim();
    double far = ShotPhysics.simulate(45.0, 14.0, 4.0, 0.0).rangeAtRim();
    assertTrue(far > near, "more speed should reach farther: " + near + " -> " + far);
  }

  @Test
  void closingVelocityExtendsRange() {
    double atRest = ShotPhysics.simulate(40.0, 14.0, 4.0, 0.0).rangeAtRim();
    double closing = ShotPhysics.simulate(40.0, 14.0, 4.0, 2.0).rangeAtRim();
    assertTrue(closing > atRest, "closing inherits horizontal velocity: " + atRest + " -> " + closing);
  }

  @Test
  void steeperHoodGivesSteeperEntry() {
    // Lower hood number = higher launch elevation = steeper descent into the hub.
    double steepEntry = ShotPhysics.simulate(55.0, 2.0, 3.0, 0.0).entryAngleDeg();
    double flatEntry = ShotPhysics.simulate(55.0, 24.0, 3.0, 0.0).entryAngleDeg();
    assertTrue(steepEntry > flatEntry, "steep=" + steepEntry + " flat=" + flatEntry);
  }

  @Test
  void tooSlowNeverReachesRim() {
    ShotResult r = ShotPhysics.simulate(13.0, 0.0, 4.0, 0.0);
    assertTrue(Double.isNaN(r.rangeAtRim()), "weak shot should not reach the rim");
    assertFalse(r.scored());
  }

  @Test
  void scoresWhenLandingWithinOpening() {
    // Put the on-target flywheel through and confirm it is flagged as scoring.
    double fw = ShotSolver.onTargetFlywheel(14.0, 4.0, 0.0);
    ShotResult r = ShotPhysics.simulate(fw, 14.0, 4.0, 0.0);
    assertTrue(r.scored());
    assertTrue(Math.abs(r.rangeAtRim() - 4.0) <= ShotPhysics.GOAL_OPENING_RADIUS_M);
    assertTrue(r.entryAngleDeg() > 0, "ball must be descending at the rim");
  }

  @Test
  void rk4MatchesClosedFormInVacuum() {
    // With no drag and no spin (=> no Magnus), the integrator must match analytic projectile motion.
    double drag = ShotPhysics.DRAG_COEFFICIENT;
    double spin = ShotPhysics.SPIN_FACTOR;
    try {
      ShotPhysics.DRAG_COEFFICIENT = 0.0;
      ShotPhysics.SPIN_FACTOR = 0.0;

      double fw = 50.0;
      double hoodDeg = 10.0;
      double v0 = ShotPhysics.exitSpeed(fw);
      double elevation = Math.toRadians(ShotPhysics.HOOD_ZERO_ELEVATION_DEG - hoodDeg);
      double vx = v0 * Math.cos(elevation);
      double vz = v0 * Math.sin(elevation);
      double h0 = ShotPhysics.LAUNCH_HEIGHT_M;
      double g = 9.81;

      // Descending root of h0 + vz t - g/2 t^2 = RIM.
      double disc = vz * vz - 2.0 * g * (RIM - h0);
      assertTrue(disc > 0, "test shot must clear the rim");
      double tExpected = (vz + Math.sqrt(disc)) / g;
      double xExpected = vx * tExpected;

      ShotResult r = ShotPhysics.simulate(fw, hoodDeg, 4.0, 0.0);
      assertEquals(xExpected, r.rangeAtRim(), 0.02, "vacuum range vs analytic");
      assertEquals(tExpected, r.timeOfFlight(), 0.02, "vacuum tof vs analytic");
    } finally {
      ShotPhysics.DRAG_COEFFICIENT = drag;
      ShotPhysics.SPIN_FACTOR = spin;
    }
  }

  @Test
  void dragShortensRangeVersusVacuum() {
    double drag = ShotPhysics.DRAG_COEFFICIENT;
    try {
      double withDrag = ShotPhysics.simulate(50.0, 10.0, 4.0, 0.0).rangeAtRim();
      ShotPhysics.DRAG_COEFFICIENT = 0.0;
      double noDrag = ShotPhysics.simulate(50.0, 10.0, 4.0, 0.0).rangeAtRim();
      assertTrue(noDrag > withDrag, "drag should shorten range: drag=" + withDrag + " vacuum=" + noDrag);
    } finally {
      ShotPhysics.DRAG_COEFFICIENT = drag;
    }
  }
}
