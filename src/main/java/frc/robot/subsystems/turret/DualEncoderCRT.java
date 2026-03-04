package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

/**
 * Calculates absolute turret position using Chinese Remainder Theorem from two encoders driven by a
 * common 110-tooth gear (1:1 with mechanism).
 *
 * <p>Encoder gears: 21 and 22 teeth mesh with the 110-tooth mechanism gear. Ratios are 110/21 and
 * 110/22 encoder rotations per mechanism rotation. 21 and 22 are coprime, providing unique position
 * identification within 1 full mechanism rotation.
 */
public class DualEncoderCRT {

  // ==================== Constants ====================

  // CAN IDs (all on canivore bus)
  public static final int MOTOR_ID = 25;
  public static final int ENCODER_1_ID = 26; // 22:1 from mechanism (22-tooth gear)
  public static final int ENCODER_2_ID = 27; // 21:1 from mechanism (21-tooth gear)

  // Gear ratios
  public static final double MOTOR_TO_MECHANISM_RATIO = 110.0 / 15.0 * 5.0; // 36.67

  // Mechanism gear: 110 teeth, 1:1 with turret. Encoder gears: 21 and 22 teeth.
  // Encoder 1 (CAN 26) is on 22-tooth gear, Encoder 2 (CAN 27) is on 21-tooth gear.
  // Encoder rotations per mechanism rotation = 110/22 and 110/21
  public static final double MECHANISM_GEAR_TEETH = 110.0;
  public static final double ENCODER_1_GEAR_TEETH = 22.0; // Encoder 1 is on 22-tooth gear
  public static final double ENCODER_2_GEAR_TEETH = 21.0; // Encoder 2 is on 21-tooth gear
  public static final double ENCODER_1_MECHANISM_RATIO =
      MECHANISM_GEAR_TEETH / ENCODER_1_GEAR_TEETH;
  public static final double ENCODER_2_MECHANISM_RATIO =
      MECHANISM_GEAR_TEETH / ENCODER_2_GEAR_TEETH;

  // CRT consistency tolerance (rotations)
  public static final double CRT_CONSISTENCY_TOLERANCE = 0.02;

  // Position limits (mechanism rotations)
  public static final double FORWARD_LIMIT = 0.75; // +270 degrees
  public static final double REVERSE_LIMIT = -0.25; // -90 degrees

  public static final double ROTOR_TO_ENCODER_RATIO = MOTOR_TO_MECHANISM_RATIO / 21.0;

  // ==================== Instance Fields ====================

  private final CANcoder encoder1; // 22-tooth gear (110/22 ratio)
  private final CANcoder encoder2; // 21-tooth gear (110/21 ratio)

  private final Alert inconsistentReadingAlert;

  /**
   * Creates a new DualEncoderCRT calculator.
   *
   * @param encoder1 The first CANcoder (22-tooth gear, CAN ID 26)
   * @param encoder2 The second CANcoder (21-tooth gear, CAN ID 27)
   */
  public DualEncoderCRT(CANcoder encoder1, CANcoder encoder2) {
    this.encoder1 = encoder1;
    this.encoder2 = encoder2;

    this.inconsistentReadingAlert =
        new Alert(
            "CRT: Encoder readings inconsistent - possible slip or failure", AlertType.kError);
  }

  /**
   * Calculate the absolute mechanism position using CRT with 110:21 and 110:22 gear ratios.
   *
   * @return mechanism position in rotations (centered around 0), or NaN if failed
   */
  public double calculateMechanismPosition() {
    double e1Raw = encoder1.getPosition().getValue().in(Rotations);
    double e2Raw = encoder2.getPosition().getValue().in(Rotations);
    double result = calculateMechanismPositionFromEncoders(e1Raw, e2Raw);
    if (Double.isNaN(result)) {
      inconsistentReadingAlert.set(true);
    } else {
      inconsistentReadingAlert.set(false);
    }
    return result;
  }

  /**
   * Pure calculation of mechanism position from encoder readings. Extracted for unit testing.
   *
   * @param e1Raw Raw encoder 1 position in rotations (any range, will be wrapped to [0, 1))
   * @param e2Raw Raw encoder 2 position in rotations (any range, will be wrapped to [0, 1))
   * @return mechanism position in rotations in range [-0.25, 0.75), or NaN if consistency check
   *     fails
   */
  public static double calculateMechanismPositionFromEncoders(double e1Raw, double e2Raw) {
    // Wrap to 0-1 range
    double e1 = ((e1Raw % 1.0) + 1.0) % 1.0;
    double e2 = ((e2Raw % 1.0) + 1.0) % 1.0;

    // CRT search: turret = (n + e1) * e1_teeth / t_teeth. Search n from 0 to e2_teeth-1.
    // The matching n gives turret where (turret * R2) mod 1 = e2.
    double mechanismPosition = Double.NaN;
    int searchLimit = (int) ENCODER_2_GEAR_TEETH;
    for (int n = 0; n < searchLimit; n++) {
      double candidate = (n + e1) * ENCODER_1_GEAR_TEETH / MECHANISM_GEAR_TEETH;
      double mech = ((candidate % 1.0) + 1.0) % 1.0; // fractional part in [0, 1)

      if (verifyConsistencyStatic(mech, e1, e2)) {
        mechanismPosition = mech;
        break;
      }
    }

    if (Double.isNaN(mechanismPosition)) {
      return Double.NaN;
    }

    // Shift to turret range [-0.25, 0.75): values in [0.75, 1) map to [-0.25, 0)
    if (mechanismPosition >= 0.75) {
      mechanismPosition -= 1.0;
    }

    return mechanismPosition;
  }

  private static boolean verifyConsistencyStatic(double mech, double e1, double e2) {
    double expectedE1 = ((mech * ENCODER_1_MECHANISM_RATIO) % 1.0 + 1.0) % 1.0;
    double expectedE2 = ((mech * ENCODER_2_MECHANISM_RATIO) % 1.0 + 1.0) % 1.0;

    double tolerance = CRT_CONSISTENCY_TOLERANCE;

    double error1 = Math.abs(wrapDiffStatic(e1, expectedE1));
    double error2 = Math.abs(wrapDiffStatic(e2, expectedE2));

    return error1 < tolerance && error2 < tolerance;
  }

  private static double wrapDiffStatic(double a, double b) {
    double diff = a - b;
    if (diff > 0.5) diff -= 1.0;
    if (diff < -0.5) diff += 1.0;
    return diff;
  }
}
