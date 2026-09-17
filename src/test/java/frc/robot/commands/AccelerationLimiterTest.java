package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.config.ParadeConfig;
import org.junit.jupiter.api.Test;

class AccelerationLimiterTest {
  private static final double EPSILON = 1e-9;

  @Test
  void paradeProfileCapsLinearAndAngularSpeed() {
    ChassisSpeeds speeds = new ChassisSpeeds(10.0, 0.0, 10.0);

    AccelerationLimiter.normalizeSpeedsInPlace(speeds);

    assertTrue(Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) <= 1.0);
    assertTrue(
        Math.abs(speeds.omegaRadiansPerSecond)
            <= ParadeConfig.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND);
    double maxModuleSpeed =
        Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
            + Math.abs(speeds.omegaRadiansPerSecond) * AccelerationLimiter.DRIVE_BASE_RADIUS;
    assertTrue(maxModuleSpeed <= ParadeConfig.MAX_LINEAR_SPEED_METERS_PER_SECOND + EPSILON);
  }

  @Test
  void paradeProfileCapsAccelerationAndBraking() {
    ChassisSpeeds speeds = new ChassisSpeeds();

    AccelerationLimiter.integrateVelocityInPlace(speeds, 1.0, 0.0, 0.0, 0.02);
    assertEquals(0.02, speeds.vxMetersPerSecond, EPSILON);

    speeds.vxMetersPerSecond = 1.0;
    AccelerationLimiter.integrateVelocityInPlace(speeds, 0.0, 0.0, 0.0, 0.02);
    assertEquals(0.98, speeds.vxMetersPerSecond, EPSILON);
  }
}
