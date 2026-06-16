package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.subsystems.shooter.ShotPhysics.ShotResult;
import org.junit.jupiter.api.Test;

/** Correctness of the inverse solver {@link ShotSolver}. */
class ShotSolverTest {

  @Test
  void onTargetFlywheelActuallyLandsOnTarget() {
    for (double d = 1.5; d <= 6.5; d += 0.5) {
      for (double hood = 4.0; hood <= 20.0; hood += 4.0) {
        double fw = ShotSolver.onTargetFlywheel(hood, d, 0.0);
        if (Double.isNaN(fw)) {
          continue;
        }
        ShotResult r = ShotPhysics.simulate(fw, hood, d, 0.0);
        assertTrue(
            Math.abs(r.rangeAtRim() - d) < 0.05,
            "d=" + d + " hood=" + hood + " landed at " + r.rangeAtRim());
      }
    }
  }

  @Test
  void unreachableTargetReturnsNaN() {
    assertTrue(Double.isNaN(ShotSolver.onTargetFlywheel(0.0, 50.0, 0.0)), "50 m is unreachable");
  }

  @Test
  void closingNeedsLessSpeedThanReceding() {
    double receding = ShotSolver.onTargetFlywheel(12.0, 4.0, -2.0);
    double atRest = ShotSolver.onTargetFlywheel(12.0, 4.0, 0.0);
    double closing = ShotSolver.onTargetFlywheel(12.0, 4.0, 2.0);
    assertTrue(receding > atRest && atRest > closing, receding + " > " + atRest + " > " + closing);
  }

  @Test
  void autoSolveProducesConsistentSteepEntry() {
    // The make-window optimum should be a consistent, robust drop angle across distances.
    for (double d = 2.0; d <= 6.0; d += 1.0) {
      ShotSolver.Shot s = ShotSolver.solve(d, 0.0);
      assertTrue(s.valid(), "no shot at d=" + d);
      assertTrue(
          s.entryAngleDeg() >= 40.0 && s.entryAngleDeg() <= 65.0,
          "entry angle out of robust band at d=" + d + ": " + s.entryAngleDeg());
      assertTrue(s.speedWindowRps() > 0 && s.hoodWindowDeg() > 0, "windows must be positive");
    }
  }

  @Test
  void autoSolveHoodIncreasesWithDistance() {
    ShotSolver.Shot near = ShotSolver.solve(2.0, 0.0);
    ShotSolver.Shot far = ShotSolver.solve(6.0, 0.0);
    assertTrue(near.valid() && far.valid());
    assertTrue(far.hoodDeg() > near.hoodDeg(), "near=" + near.hoodDeg() + " far=" + far.hoodDeg());
  }

  @Test
  void solveAtHoodInvalidWhenUnreachable() {
    assertFalse(ShotSolver.solveAtHood(50.0, 0.0, 10.0).valid());
  }

  @Test
  void robustnessIsMakeWindowArea() {
    ShotSolver.Shot s = ShotSolver.solveAtHood(4.0, 0.0, 14.0);
    assertTrue(s.valid());
    assertTrue(
        Math.abs(s.robustness() - s.speedWindowRps() * s.hoodWindowDeg()) < 1e-9,
        "robustness should equal speed-window x hood-window");
  }
}
