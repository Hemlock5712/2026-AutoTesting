package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/** Generator + runtime map tests for {@link ShotMapGenerator} and {@link ShooterMap}. */
class ShotMapTest {

  /**
   * Regenerates the map from the physics constants and checks it matches the values baked into
   * {@link ShooterMap}. If this fails, the constants changed: re-run {@link ShotMapGenerator#main}
   * and paste the new values. Tolerances allow for the printed values' rounding.
   */
  @Test
  void bakedValuesMatchFreshGeneration() {
    ShotMapGenerator.Generated gen = ShotMapGenerator.generate();

    assertEquals(ShooterMap.HOOD_COEFFS.length, gen.hoodCoeffs().length);
    for (int i = 0; i < gen.hoodCoeffs().length; i++) {
      assertEquals(gen.hoodCoeffs()[i], ShooterMap.HOOD_COEFFS[i], 1e-3, "hood coeff " + i);
    }
    for (int i = 0; i < ShotMapGenerator.N_DIST; i++) {
      for (int j = 0; j < ShotMapGenerator.N_VEL; j++) {
        assertEquals(
            gen.flywheelGrid()[i][j],
            ShooterMap.FLYWHEEL_GRID[i][j],
            0.01,
            "flywheel[" + i + "][" + j + "]");
        assertEquals(
            gen.tofGrid()[i][j], ShooterMap.TOF_GRID[i][j], 0.005, "tof[" + i + "][" + j + "]");
      }
    }
  }

  @Test
  void interpolationIsExactAtNodes() {
    for (int i = 0; i < ShotMapGenerator.N_DIST; i++) {
      for (int j = 0; j < ShotMapGenerator.N_VEL; j++) {
        double d = ShotMapGenerator.distanceAt(i);
        double vr = ShotMapGenerator.radialVelAt(j);
        assertEquals(ShooterMap.FLYWHEEL_GRID[i][j], ShooterMap.flywheelRps(d, vr), 1e-9);
        assertEquals(ShooterMap.TOF_GRID[i][j], ShooterMap.tofSeconds(d, vr), 1e-9);
      }
    }
  }

  @Test
  void flywheelDecreasesAsClosingIncreases() {
    for (int i = 0; i < ShotMapGenerator.N_DIST; i++) {
      double d = ShotMapGenerator.distanceAt(i);
      for (double vr = ShooterMap.MIN_RADIAL_VEL; vr < ShooterMap.MAX_RADIAL_VEL; vr += 0.5) {
        assertTrue(
            ShooterMap.flywheelRps(d, vr) >= ShooterMap.flywheelRps(d, vr + 0.5) - 1e-6,
            "flywheel should not increase as we close, d=" + d + " vr=" + vr);
      }
    }
  }

  @Test
  void flywheelIncreasesWithDistanceAtRest() {
    for (double d = ShooterMap.MIN_DIST_M; d < ShooterMap.MAX_DIST_M - 1e-9; d += 0.5) {
      assertTrue(
          ShooterMap.flywheelRps(d + 0.5, 0.0) > ShooterMap.flywheelRps(d, 0.0),
          "farther should need more speed at d=" + d);
    }
  }

  @Test
  void interpolatedValueLiesBetweenNeighbors() {
    double lo = ShooterMap.flywheelRps(1.0, 0.0);
    double hi = ShooterMap.flywheelRps(1.5, 0.0);
    double mid = ShooterMap.flywheelRps(1.25, 0.0);
    assertTrue(Math.min(lo, hi) < mid && mid < Math.max(lo, hi), "midpoint should interpolate");
  }

  @Test
  void inputsAreClampedToEnvelope() {
    assertEquals(ShooterMap.flywheelRps(1.0, 0.0), ShooterMap.flywheelRps(0.2, 0.0), 1e-9);
    assertEquals(ShooterMap.flywheelRps(6.5, 3.0), ShooterMap.flywheelRps(99.0, 3.0), 1e-9);
    assertEquals(ShooterMap.flywheelRps(4.0, -2.0), ShooterMap.flywheelRps(4.0, -9.0), 1e-9);
    assertEquals(ShooterMap.flywheelRps(4.0, 4.0), ShooterMap.flywheelRps(4.0, 9.0), 1e-9);
  }

  @Test
  void timeOfFlightIsPositiveAndBounded() {
    for (double d = ShooterMap.MIN_DIST_M; d <= ShooterMap.MAX_DIST_M; d += 0.25) {
      for (double vr = ShooterMap.MIN_RADIAL_VEL; vr <= ShooterMap.MAX_RADIAL_VEL; vr += 0.5) {
        double tof = ShooterMap.tofSeconds(d, vr);
        assertTrue(tof > 0.4 && tof < 2.1, "tof out of range at d=" + d + " vr=" + vr + ": " + tof);
      }
    }
  }

  @Test
  void hoodScheduleIsSaneAndIncreasing() {
    for (double d = ShooterMap.MIN_DIST_M; d < ShooterMap.MAX_DIST_M - 1e-9; d += 0.5) {
      double hood = ShooterMap.hoodDeg(d);
      assertTrue(
          hood >= 0.0 && hood <= 32.0, "hood out of mechanism range at d=" + d + ": " + hood);
      assertTrue(
          ShooterMap.hoodDeg(d + 0.5) > hood, "hood should increase with distance at d=" + d);
    }
    // Clamped outside the envelope.
    assertEquals(ShooterMap.hoodDeg(1.0), ShooterMap.hoodDeg(0.1), 1e-9);
    assertEquals(ShooterMap.hoodDeg(6.5), ShooterMap.hoodDeg(20.0), 1e-9);
  }

  @Test
  void envelopeFlagMatchesBounds() {
    assertTrue(ShooterMap.inEnvelope(4.0, 0.0));
    assertFalse(ShooterMap.inEnvelope(0.5, 0.0));
    assertFalse(ShooterMap.inEnvelope(4.0, 9.0));
  }
}
