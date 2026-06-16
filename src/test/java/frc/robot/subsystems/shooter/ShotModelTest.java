package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/**
 * Phase 1 calibration checkpoint for the HighTide-style targeting model.
 *
 * <p>{@link #printModelVsTableAtTableHood()} drives the physics solver at the SAME hood angles your
 * known-good stationary table uses, and prints the model's flywheel speed and physical
 * time-of-flight next to the table's. Run {@code ./gradlew test --tests ShotModelTest -i} and read
 * it: tune the constants in {@link ShotPhysics} (start with SLIP_EFFICIENCY, then the Magnus terms)
 * until the model flywheel column tracks the table flywheel column. The model's {@code tof} column
 * is the physically real time-of-flight that should replace the hand-fudged {@code tofMult} table.
 */
class ShotModelTest {

  @Test
  void printModelVsTableAtTableHood() {
    System.out.println();
    System.out.println("=== Physics model at the table's hood angles vs. competition table (vr=0) ===");
    System.out.printf(
        "%-5s | %9s %9s | %9s %9s | %9s%n",
        "dist", "fw(model)", "fw(table)", "tof(model)", "tof(table)", "hood");
    for (double d = 1.0; d <= 6.5 + 1e-9; d += 0.5) {
      double tableHood = ShooterLookup.getHoodMap().get(d);
      double tableFw = ShooterLookup.getFlywheelMap().get(d);
      double tableTof = ShooterLookup.getToFMap().get(d);
      ShotSolver.Shot s = ShotSolver.solveAtHood(d, 0.0, tableHood);
      if (s.valid()) {
        System.out.printf(
            "%-5.1f | %9.1f %9.1f | %9.3f %9.3f | %9.1f%n",
            d, s.flywheelRps(), tableFw, s.timeOfFlight(), tableTof, tableHood);
      } else {
        System.out.printf(
            "%-5.1f | %9s %9.1f | %9s %9.3f | %9.1f%n",
            d, "NONE", tableFw, "-", tableTof, tableHood);
      }
    }
    System.out.println();
  }

  @Test
  void midRangeHasScoringShot() {
    ShotSolver.Shot s = ShotSolver.solve(3.0, 0.0);
    assertTrue(s.valid(), "expected a scoring shot at 3 m");
    assertTrue(
        s.timeOfFlight() > 0.4 && s.timeOfFlight() < 2.6,
        "time of flight should be physically plausible, was " + s.timeOfFlight());
    assertTrue(
        s.hoodDeg() >= ShotSolver.HOOD_MIN_DEG && s.hoodDeg() <= ShotSolver.HOOD_MAX_DEG,
        "hood within mechanism range, was " + s.hoodDeg());
  }

  @Test
  void closingVelocityNeedsLessSpeedAtFixedHood() {
    // Driving toward the hub adds to the ball's horizontal velocity, so a given hood angle needs
    // less flywheel speed to land on target. Isolates the radial-velocity physics from the
    // robustness shot-selection (which can change the chosen hood).
    double atRest = ShotSolver.onTargetFlywheel(10.0, 4.0, 0.0);
    double closing = ShotSolver.onTargetFlywheel(10.0, 4.0, 2.0);
    assertTrue(!Double.isNaN(atRest) && !Double.isNaN(closing), "both shots should be solvable");
    assertTrue(
        closing < atRest,
        "closing on the hub should need less flywheel speed: closing="
            + closing
            + " atRest="
            + atRest);
  }

  @Test
  void radialCompensationLowersFlightTime() {
    // At a fixed hood, closing on the hub should not increase the time of flight.
    ShotSolver.Shot atRest = ShotSolver.solveAtHood(4.0, 0.0, 14.0);
    ShotSolver.Shot closing = ShotSolver.solveAtHood(4.0, 2.0, 14.0);
    assertTrue(atRest.valid() && closing.valid(), "both shots should be solvable");
    assertTrue(
        closing.timeOfFlight() <= atRest.timeOfFlight() + 1e-6,
        "closing should not increase tof: closing="
            + closing.timeOfFlight()
            + " atRest="
            + atRest.timeOfFlight());
  }
}
