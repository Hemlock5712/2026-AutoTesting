package frc.robot.subsystems.turret;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

/**
 * Unit tests for DualEncoderCRT - the Chinese Remainder Theorem based dual-encoder position
 * calculator for the turret.
 *
 * <p>Tests the pure calculation logic with swapped encoders: - Encoder 1 (CAN 26) is on 22-tooth
 * gear: e1 = (mech * 110/22) mod 1 - Encoder 2 (CAN 27) is on 21-tooth gear: e2 = (mech * 110/21)
 * mod 1 Output is shifted to turret range [-0.25, 0.75).
 */
class DualEncoderCRTTest {

  // CRT search involves multiple multiplications; allow for floating point accumulation
  private static final double TOLERANCE = 1e-4;

  @Nested
  class BasicCRTCalculation {

    @Test
    void zeroPosition_returnsZero() {
      // At mechanism position 0: e1=0, e2=0
      double result = DualEncoderCRT.calculateMechanismPositionFromEncoders(0.0, 0.0);
      assertEquals(0.0, result, TOLERANCE);
    }

    @Test
    void quarterRotation_returnsCorrectPosition() {
      // At mechanism position 0.25:
      // e1 (22-tooth) = 0.25*110/22 mod 1 = 0.25
      // e2 (21-tooth) = 0.25*110/21 mod 1 = 0.3095
      double result = DualEncoderCRT.calculateMechanismPositionFromEncoders(0.25, 0.3095);
      assertEquals(0.25, result, TOLERANCE);
    }

    @Test
    void halfRotation_returnsCorrectPosition() {
      // At mechanism position 0.5:
      // e1 (22-tooth) = 0.5*5 mod 1 = 0.5
      // e2 (21-tooth) = 0.5*110/21 mod 1 = 0.619
      double result = DualEncoderCRT.calculateMechanismPositionFromEncoders(0.5, 0.619);
      assertEquals(0.5, result, TOLERANCE);
    }

    @Test
    void slightlyPastHalfRotation_returnsCorrectPosition() {
      // At mechanism position 0.546:
      // e1 (22-tooth) = 0.73
      // e2 (21-tooth) = 0.86
      double result = DualEncoderCRT.calculateMechanismPositionFromEncoders(0.73, 0.86);
      assertEquals(0.546, result, TOLERANCE);
    }

    @Test
    void threeQuarterRotation_shiftsToNegativeQuarter() {
      // At mechanism position 0.75:
      // e1 (22-tooth) = 0.75
      // e2 (21-tooth) = 0.9286
      // -> shifts to -0.25
      double result = DualEncoderCRT.calculateMechanismPositionFromEncoders(0.75, 0.9286);
      assertEquals(-0.25, result, TOLERANCE);
    }

    @Test
    void oneEighthRotation_returnsCorrectPosition() {
      // At mechanism position 0.125:
      // e1 (22-tooth) = 0.625
      // e2 (21-tooth) = 0.6548
      double result = DualEncoderCRT.calculateMechanismPositionFromEncoders(0.625, 0.6548);
      assertEquals(0.125, result, TOLERANCE);
    }
  }

  @Nested
  class RangeShift {

    @Test
    void boundaryAt075_shiftsToNegativeQuarter() {
      // At -90 deg (reverse limit):
      // e1 (22-tooth) = 0.75
      // e2 (21-tooth) = 0.9286
      // -> mech=0.75 -> shift to -0.25
      double result = DualEncoderCRT.calculateMechanismPositionFromEncoders(0.75, 0.9286);
      assertEquals(-0.25, result, TOLERANCE);
    }

    @Test
    void justAbove075_shiftsToNegative() {
      // Mechanism 0.8:
      // e1 (22-tooth) = 0.8*5 mod 1 = 0
      // e2 (21-tooth) = 0.19
      // -> shift to -0.2
      double result = DualEncoderCRT.calculateMechanismPositionFromEncoders(0.0, 0.1905);
      assertEquals(-0.2, result, TOLERANCE);
    }

    @Test
    void justBelow075_staysPositive() {
      // Mechanism 0.74:
      // e1 (22-tooth) = 0.7
      // e2 (21-tooth) = 0.876
      double result = DualEncoderCRT.calculateMechanismPositionFromEncoders(0.7, 0.8762);
      assertEquals(0.74, result, TOLERANCE);
    }

    @Test
    void nearOne_shiftsToNearNegativeQuarter() {
      // Mechanism 0.99 (= -0.01):
      // e1 (22-tooth) = 0.95
      // e2 (21-tooth) = 0.186
      double result = DualEncoderCRT.calculateMechanismPositionFromEncoders(0.95, 0.1857);
      assertEquals(-0.01, result, 0.02);
    }
  }

  @Nested
  class EncoderWrap {

    @Test
    void rawValuesOutsideZeroOne_wrapCorrectly() {
      // Mechanism -0.2 (= 0.8):
      // e1 (22-tooth) = 0
      // e2 (21-tooth) = 0.19
      // Use raw 2.0, 1.19 to wrap to these
      double result = DualEncoderCRT.calculateMechanismPositionFromEncoders(2.0, 1.1905);
      assertEquals(-0.2, result, TOLERANCE);
    }

    @Test
    void negativeRawValues_wrapCorrectly() {
      // Mechanism 0.1:
      // e1 (22-tooth) = 0.5
      // e2 (21-tooth) = 0.524
      // Use raw -0.5, -0.476
      double result = DualEncoderCRT.calculateMechanismPositionFromEncoders(-0.5, -0.476);
      assertEquals(0.1, result, TOLERANCE);
    }

    @Test
    void largePositiveValues_wrapCorrectly() {
      // Same as zero: e1=100 -> 0, e2=200 -> 0
      double result = DualEncoderCRT.calculateMechanismPositionFromEncoders(100.0, 200.0);
      assertEquals(0.0, result, TOLERANCE);
    }
  }

  @Nested
  class ConsistencyCheck {

    @Test
    void inconsistentReadings_returnNaN() {
      // e1=0, e2=0.5 - no mechanism position produces both
      double result = DualEncoderCRT.calculateMechanismPositionFromEncoders(0.0, 0.5);
      assertTrue(Double.isNaN(result));
    }

    @Test
    void slightlyInconsistentReadings_returnNaN() {
      // e1=0.1, e2=0.9 - no mechanism position produces both
      double result = DualEncoderCRT.calculateMechanismPositionFromEncoders(0.1, 0.9);
      assertTrue(Double.isNaN(result));
    }

    @Test
    void consistentReadingsWithinTolerance_returnValue() {
      // At mechanism 0.25:
      // e1 (22-tooth) = 0.25
      // e2 (21-tooth) = 0.3095
      double e1 = 0.25;
      double e2 = 0.3095;
      double result = DualEncoderCRT.calculateMechanismPositionFromEncoders(e1, e2);
      assertEquals(0.25, result, TOLERANCE);
    }
  }

  @Nested
  class ForwardLimit {

    @Test
    void atBoundary075_mapsToNegativeQuarter() {
      // e1 (22-tooth) = 0.75, e2 (21-tooth) = 0.9286
      double result = DualEncoderCRT.calculateMechanismPositionFromEncoders(0.75, 0.9286);
      assertEquals(-0.25, result, TOLERANCE);
    }
  }

  @Nested
  class ReverseLimit {

    @Test
    void atReverseLimit_returnsNegativeQuarter() {
      // e1 (22-tooth) = 0.75, e2 (21-tooth) = 0.9286
      double result = DualEncoderCRT.calculateMechanismPositionFromEncoders(0.75, 0.9286);
      assertEquals(-0.25, result, TOLERANCE);
    }
  }
}
