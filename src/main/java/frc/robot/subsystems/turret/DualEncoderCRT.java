package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

/**
 * Calculates absolute turret position using Chinese Remainder Theorem from two encoders with gear
 * ratios 21:1 and 22:1 from mechanism.
 *
 * <p>With consecutive coprime ratios (21 and 22), the difference between encoder readings directly
 * gives the mechanism position since (22-21) = 1. This provides unique position identification
 * within 1 full mechanism rotation.
 */
public class DualEncoderCRT {

  // ==================== Constants ====================

  // CAN IDs (all on canivore bus)
  public static final int MOTOR_ID = 25;
  public static final int ENCODER_1_ID = 26; // 21:1 from mechanism
  public static final int ENCODER_2_ID = 27; // 22:1 from mechanism

  // Gear ratios
  public static final double MOTOR_TO_MECHANISM_RATIO = 110.0 / 25.0 * 7.0; // 30.8

  // Encoder ratios (encoder rotations per mechanism rotation)
  public static final double ENCODER_1_MECHANISM_RATIO = 21.0;
  public static final double ENCODER_2_MECHANISM_RATIO = 22.0;

  // Motor to encoder ratios (for FusedCANcoder config)
  // RotorToSensorRatio: motor rotations per encoder rotation
  public static final double MOTOR_TO_ENCODER_1_RATIO =
      MOTOR_TO_MECHANISM_RATIO / ENCODER_1_MECHANISM_RATIO; // 30.8/21 = 1.467
  public static final double MOTOR_TO_ENCODER_2_RATIO =
      MOTOR_TO_MECHANISM_RATIO / ENCODER_2_MECHANISM_RATIO; // 30.8/22 = 1.4

  // CRT: unique range is 1 mechanism rotation (since gcd(21,22)=1)
  public static final double UNIQUE_MECHANISM_RANGE = 1.0;

  // Encoder offsets (calibrate so diff=0 when turret faces forward)
  // Set these after running calibration procedure
  public static final double ENCODER_1_OFFSET = 0.0;
  public static final double ENCODER_2_OFFSET = 0.0;

  // CRT consistency tolerance (rotations)
  public static final double CRT_CONSISTENCY_TOLERANCE = 0.02;

  // Position limits (mechanism rotations)
  public static final double FORWARD_LIMIT = 0.5; // +180 degrees
  public static final double REVERSE_LIMIT = -0.5; // -180 degrees

  // ==================== Instance Fields ====================

  private final CANcoder encoder1; // 21:1 from mechanism
  private final CANcoder encoder2; // 22:1 from mechanism
  private final double offset1;
  private final double offset2;
  private final double mechRatio1; // 21
  private final double mechRatio2; // 22
  private final double motorToMechanism;

  private final Alert inconsistentReadingAlert;

  /**
   * Creates a new DualEncoderCRT calculator with default constants.
   *
   * @param encoder1 The first CANcoder (21:1 from mechanism)
   * @param encoder2 The second CANcoder (22:1 from mechanism)
   */
  public DualEncoderCRT(CANcoder encoder1, CANcoder encoder2) {
    this(
        encoder1,
        ENCODER_1_MECHANISM_RATIO,
        ENCODER_1_OFFSET,
        encoder2,
        ENCODER_2_MECHANISM_RATIO,
        ENCODER_2_OFFSET,
        MOTOR_TO_MECHANISM_RATIO);
  }

  /**
   * Creates a new DualEncoderCRT calculator with custom parameters.
   *
   * @param encoder1 The first CANcoder (21:1 from mechanism)
   * @param mechRatio1 Encoder 1 rotations per mechanism rotation (21)
   * @param offset1 Offset for encoder 1 (raw reading when turret is at 0)
   * @param encoder2 The second CANcoder (22:1 from mechanism)
   * @param mechRatio2 Encoder 2 rotations per mechanism rotation (22)
   * @param offset2 Offset for encoder 2 (raw reading when turret is at 0)
   * @param motorToMechanism Motor to mechanism gear ratio (30.8)
   */
  public DualEncoderCRT(
      CANcoder encoder1,
      double mechRatio1,
      double offset1,
      CANcoder encoder2,
      double mechRatio2,
      double offset2,
      double motorToMechanism) {
    this.encoder1 = encoder1;
    this.encoder2 = encoder2;
    this.mechRatio1 = mechRatio1;
    this.mechRatio2 = mechRatio2;
    this.offset1 = offset1;
    this.offset2 = offset2;
    this.motorToMechanism = motorToMechanism;

    this.inconsistentReadingAlert =
        new Alert(
            "CRT: Encoder readings inconsistent - possible slip or failure", AlertType.kError);
  }

  /**
   * Calculate the absolute mechanism position using CRT. With 21:1 and 22:1 ratios, diff = e2 - e1
   * directly gives mechanism position.
   *
   * @return mechanism position in rotations (centered around 0), or NaN if failed
   */
  public double calculateMechanismPosition() {
    // Get absolute encoder positions (0 to 1 rotations)
    double e1Raw = encoder1.getAbsolutePosition().getValue().in(Rotations);
    double e2Raw = encoder2.getAbsolutePosition().getValue().in(Rotations);

    // Apply offsets (wrap to 0-1 range)
    double e1 = ((e1Raw - offset1) % 1.0 + 1.0) % 1.0;
    double e2 = ((e2Raw - offset2) % 1.0 + 1.0) % 1.0;

    // CRT: difference directly gives mechanism position within [0, 1)
    // Because (22-21) = 1, the diff advances 1 per mechanism rotation
    double diff = ((e2 - e1) % 1.0 + 1.0) % 1.0;
    double mechanismPosition = diff;

    // Verify consistency
    if (!verifyConsistency(mechanismPosition, e1, e2)) {
      inconsistentReadingAlert.set(true);
      return Double.NaN;
    }

    inconsistentReadingAlert.set(false);

    // Center around 0 for turret (±0.5 rotations)
    if (mechanismPosition > 0.5) {
      mechanismPosition -= 1.0;
    }

    return mechanismPosition;
  }

  /**
   * Verify that calculated position is consistent with both encoder readings.
   *
   * @param mech The calculated mechanism position
   * @param e1 The offset-adjusted encoder 1 reading
   * @param e2 The offset-adjusted encoder 2 reading
   * @return true if readings are consistent, false if possible slip/failure
   */
  private boolean verifyConsistency(double mech, double e1, double e2) {
    // Expected encoder readings for this mechanism position
    double expectedE1 = (mech * mechRatio1) % 1.0;
    if (expectedE1 < 0) expectedE1 += 1.0;

    double expectedE2 = (mech * mechRatio2) % 1.0;
    if (expectedE2 < 0) expectedE2 += 1.0;

    double tolerance = CRT_CONSISTENCY_TOLERANCE;

    double error1 = Math.abs(wrapDiff(e1, expectedE1));
    double error2 = Math.abs(wrapDiff(e2, expectedE2));

    return error1 < tolerance && error2 < tolerance;
  }

  /**
   * Calculate wrapped difference between two angles in [0, 1) range.
   *
   * @param a First angle (0 to 1)
   * @param b Second angle (0 to 1)
   * @return Wrapped difference in range (-0.5, 0.5]
   */
  private double wrapDiff(double a, double b) {
    double diff = a - b;
    if (diff > 0.5) diff -= 1.0;
    if (diff < -0.5) diff += 1.0;
    return diff;
  }

  /**
   * Convert mechanism position to rotor position.
   *
   * @param mechanism Mechanism position in rotations
   * @return Rotor position in rotations
   */
  public double mechanismToRotor(double mechanism) {
    return mechanism * motorToMechanism;
  }

  /**
   * Get raw encoder readings for calibration.
   *
   * @return Array of [encoder1, encoder2] raw absolute positions
   */
  public double[] getRawEncoderReadings() {
    return new double[] {
      encoder1.getAbsolutePosition().getValue().in(Rotations),
      encoder2.getAbsolutePosition().getValue().in(Rotations)
    };
  }
}
