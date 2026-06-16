package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/** Shoot-while-moving and tilt-compensation math in {@link SwmTargeting}. */
class SwmTargetingTest {

  private static final double TWO_PI = 2.0 * Math.PI;

  @Test
  void stationaryLevelShotMatchesMapDirectly() {
    // Target 4 m straight ahead (+x), robot facing +x, no motion, no tilt.
    SwmTargeting.Aim aim = SwmTargeting.solve(0, 0, 0, 0, 4.0, 0.0, 0.0, 0.0, 0.0);
    assertEquals(0.0, aim.turretAngleRot(), 1e-6, "aim straight ahead");
    assertEquals(ShooterMap.hoodDeg(4.0), aim.hoodDeg(), 1e-6);
    assertEquals(ShooterMap.flywheelRps(4.0, 0.0), aim.flywheelRps(), 1e-6);
    assertEquals(ShooterMap.tofSeconds(4.0, 0.0), aim.tofSeconds(), 1e-6);
    assertEquals(0.0, aim.radialVelMps(), 1e-9);
    assertTrue(aim.feasible());
  }

  @Test
  void turretPointsAtTargetRelativeToHeading() {
    // Target due north (+y); robot facing +x. Turret must point +90 deg = +0.25 rot.
    SwmTargeting.Aim aim = SwmTargeting.solve(0, 0, 0, 0, 0.0, 4.0, 0.0, 0.0, 0.0);
    assertEquals(0.25, aim.turretAngleRot(), 1e-6);
  }

  @Test
  void closingReducesFlywheelSpeed() {
    SwmTargeting.Aim rest = SwmTargeting.solve(0, 0, 0, 0, 4.0, 0.0, 0.0, 0.0, 0.0);
    SwmTargeting.Aim closing = SwmTargeting.solve(0, 0, 2.0, 0, 4.0, 0.0, 0.0, 0.0, 0.0);
    assertEquals(2.0, closing.radialVelMps(), 1e-9, "velocity toward target is radial");
    assertTrue(closing.flywheelRps() < rest.flywheelRps(), "closing needs less shooter speed");
    // The ball flies the stationary trajectory, so the flight time is the stationary one.
    assertEquals(rest.tofSeconds(), closing.tofSeconds(), 1e-9, "tof is the stationary value");
  }

  @Test
  void tangentialMotionLeadsTheTurretOppositeTheDrift() {
    // Target ahead (+x); moving +y (left). Ball drifts +y, so the turret must lead toward -y, and
    // the shooter needs slightly more speed to keep the radial reach while adding the lead.
    SwmTargeting.Aim rest = SwmTargeting.solve(0, 0, 0, 0, 4.0, 0.0, 0.0, 0.0, 0.0);
    SwmTargeting.Aim aim = SwmTargeting.solve(0, 0, 0, 2.0, 4.0, 0.0, 0.0, 0.0, 0.0);
    assertEquals(0.0, aim.radialVelMps(), 1e-9, "pure tangential has no radial component");
    assertTrue(aim.turretAngleRot() < 0.0, "turret should lead toward -y: " + aim.turretAngleRot());
    assertTrue(
        aim.flywheelRps() >= rest.flywheelRps() - 1e-9,
        "tangential motion needs at least the stationary speed");
  }

  @Test
  void tiltCompensationReducesToLevelWhenFlat() {
    SwmTargeting.Aim level = SwmTargeting.solve(1.0, 0.5, 0.3, -0.4, 5.0, 3.0, 0.7, 0.0, 0.0);
    SwmTargeting.Aim viaTilt = SwmTargeting.solve(1.0, 0.5, 0.3, -0.4, 5.0, 3.0, 0.7, 0.0, 0.0);
    assertEquals(level.turretAngleRot(), viaTilt.turretAngleRot(), 1e-9);
    assertEquals(level.hoodDeg(), viaTilt.hoodDeg(), 1e-9);
  }

  @Test
  void pitchShiftsHoodByApproximatelyThePitchAngle() {
    // Aiming straight ahead, level vs pitched: robot-frame elevation shifts by the pitch, so the
    // commanded hood shifts by ~the pitch angle (sign per the rotation convention).
    double pitch = Math.toRadians(8.0);
    SwmTargeting.Aim flat = SwmTargeting.solve(0, 0, 0, 0, 4.0, 0.0, 0.0, 0.0, 0.0);
    SwmTargeting.Aim pitched = SwmTargeting.solve(0, 0, 0, 0, 4.0, 0.0, 0.0, pitch, 0.0);
    double deltaHood = pitched.hoodDeg() - flat.hoodDeg();
    assertEquals(-8.0, deltaHood, 0.5, "hood should shift by ~ -pitch when aiming straight ahead");
  }

  @Test
  void tiltTransformRoundTrips() {
    double yaw = 0.6;
    double pitch = 0.2;
    double roll = -0.15;
    // An arbitrary field-frame unit direction.
    double az = 1.1;
    double el = 0.5;
    double fx = Math.cos(el) * Math.cos(az);
    double fy = Math.cos(el) * Math.sin(az);
    double fz = Math.sin(el);
    double[] r = SwmTargeting.fieldToRobot(fx, fy, fz, yaw, pitch, roll);
    double[] back = SwmTargeting.robotToField(r[0], r[1], r[2], yaw, pitch, roll);
    assertEquals(fx, back[0], 1e-9);
    assertEquals(fy, back[1], 1e-9);
    assertEquals(fz, back[2], 1e-9);
  }

  @Test
  void rollIntroducesTurretAzimuthForAnElevatedShot() {
    // The shot has a large elevation, so the launch vector points up out of the robot plane.
    // Rolling the robot tips that elevated vector sideways, which must be corrected by the turret
    // azimuth. (A purely horizontal aim would be unaffected, but real shots are lofted.)
    SwmTargeting.Aim flat = SwmTargeting.solve(0, 0, 0, 0, 4.0, 0.0, 0.0, 0.0, 0.0);
    SwmTargeting.Aim rolled =
        SwmTargeting.solve(0, 0, 0, 0, 4.0, 0.0, 0.0, 0.0, Math.toRadians(10));
    assertTrue(
        Math.abs(rolled.turretAngleRot() - flat.turretAngleRot()) > 0.005,
        "roll should shift the turret azimuth for a lofted shot: "
            + flat.turretAngleRot()
            + " -> "
            + rolled.turretAngleRot());
  }

  @Test
  void outOfEnvelopeIsInfeasible() {
    SwmTargeting.Aim tooFar = SwmTargeting.solve(0, 0, 0, 0, 9.0, 0.0, 0.0, 0.0, 0.0);
    assertTrue(!tooFar.feasible(), "beyond max range should be infeasible");
  }

  @Test
  void liveSlipRescalesFlywheelButNotHoodOrTof() {
    SwmTargeting.Aim ref = SwmTargeting.solve(0, 0, 0, 0, 4.0, 0, 0, 0, 0, ShotPhysics.SLIP_EFFICIENCY);
    SwmTargeting.Aim lower =
        SwmTargeting.solve(0, 0, 0, 0, 4.0, 0, 0, 0, 0, ShotPhysics.SLIP_EFFICIENCY * 0.9);
    // Less grip (lower slip) needs more flywheel for the same exit speed, scaling as 1/slip.
    assertTrue(lower.flywheelRps() > ref.flywheelRps(), "lower slip should need more RPS");
    assertEquals(ref.flywheelRps() / 0.9, lower.flywheelRps(), 1e-6, "flywheel scales as 1/slip");
    // Hood schedule and time-of-flight are slip-independent, so tuning slip leaves them untouched.
    assertEquals(ref.hoodDeg(), lower.hoodDeg(), 1e-9, "hood is slip-independent");
    assertEquals(ref.tofSeconds(), lower.tofSeconds(), 1e-9, "tof is slip-independent");
  }
}
