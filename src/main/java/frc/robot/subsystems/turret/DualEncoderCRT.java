package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.AutoLogOutput;

/**
 * Calculates absolute turret position using Chinese Remainder Theorem from two encoders driven by a
 * common 110-tooth gear (1:1 with mechanism).
 *
 * <p>Encoder gears: 21 and 22 teeth mesh with the 110-tooth mechanism gear. Ratios are 110/21 and
 * 110/22 encoder rotations per mechanism rotation. 21 and 22 are coprime, providing unique position
 * identification within 1 full mechanism rotation.
 *
 * <p>At startup, CRT resolves which of the 5 encoder-1 wraps the mechanism is in. The result is
 * used to seed encoder 1's continuous position via {@link #seedEncoderPosition()}, after which
 * FusedCANcoder handles high-bandwidth tracking for the remainder of the match.
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
      MECHANISM_GEAR_TEETH / ENCODER_1_GEAR_TEETH; // 5.0
  public static final double ENCODER_2_MECHANISM_RATIO =
      MECHANISM_GEAR_TEETH / ENCODER_2_GEAR_TEETH; // 110/21 ≈ 5.238

  // CRT sanity-check tolerance (rotations). This is only used to detect hardware failures
  // (slipped gear, dead encoder), not for candidate selection. Set to half the candidate
  // spacing in encoder-2 space: 1/42 ≈ 0.0238 rotations.
  public static final double CRT_CONSISTENCY_TOLERANCE = 1.0 / 42.0;

  // Position limits (mechanism rotations)
  public static final double FORWARD_LIMIT = 0.75; // +270 degrees
  public static final double REVERSE_LIMIT = -0.25; // -90 degrees

  // Motor rotor rotations per encoder 1 rotation
  public static final double ROTOR_TO_ENCODER_RATIO =
      MOTOR_TO_MECHANISM_RATIO / ENCODER_1_MECHANISM_RATIO; // 36.67 / 5.0 ≈ 7.333

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
   * <p>Blocks up to 100ms waiting for fresh encoder data using {@code waitForAll}.
   *
   * @return mechanism position in rotations (centered around 0), or NaN if failed
   */
  @AutoLogOutput
  public double calculateMechanismPosition() {
    StatusSignal<Angle> e1Signal = encoder1.getAbsolutePosition();
    StatusSignal<Angle> e2Signal = encoder2.getAbsolutePosition();

    StatusCode status = BaseStatusSignal.waitForAll(0.1, e1Signal, e2Signal);
    if (!status.isOK()) {
      inconsistentReadingAlert.set(true);
      return Double.NaN;
    }

    double e1Raw = e1Signal.getValue().in(Rotations);
    double e2Raw = e2Signal.getValue().in(Rotations);
    double result = calculateMechanismPositionFromEncoders(e1Raw, e2Raw);
    if (Double.isNaN(result)) {
      inconsistentReadingAlert.set(true);
    } else {
      inconsistentReadingAlert.set(false);
    }
    return result;
  }

  /**
   * Seeds encoder 1's continuous position so that FusedCANcoder reports the correct mechanism
   * position. Call this once at startup before enabling closed-loop control.
   *
   * <p>Uses {@code waitForAll} to ensure fresh, synchronized encoder data. CRT determines which of
   * the 5 wraps the encoder is on (integer n), and the continuous position is simply n + e1. The
   * TalonFX then divides by SensorToMechanismRatio (5.0) to get mechanism rotations.
   *
   * @return true if seeding succeeded, false if CRT failed or signals were unavailable
   */
  public boolean seedEncoderPosition() {
    StatusSignal<Angle> e1Signal = encoder1.getAbsolutePosition();
    StatusSignal<Angle> e2Signal = encoder2.getAbsolutePosition();

    // Block until both encoder signals are fresh (up to 100ms)
    StatusCode status = BaseStatusSignal.waitForAll(0.1, e1Signal, e2Signal);
    if (!status.isOK()) {
      return false;
    }

    // Read both values once from the cached signals
    double e1Raw = e1Signal.getValue().in(Rotations);
    double e2Raw = e2Signal.getValue().in(Rotations);

    double e1 = ((e1Raw % 1.0) + 1.0) % 1.0;
    double e2 = ((e2Raw % 1.0) + 1.0) % 1.0;

    int n = findBestWrap(e1, e2);
    if (n < 0) {
      return false;
    }

    // Continuous encoder position is simply n + absolute reading.
    // TalonFX divides by SensorToMechanismRatio (5.0) to get mechanism rotations.
    // For negative mechanism positions (>= FORWARD_LIMIT before wrapping),
    // shift down by one full encoder cycle so the motor reports the correct signed position.
    double continuousPosition = n + e1;
    if (continuousPosition / ENCODER_1_MECHANISM_RATIO >= FORWARD_LIMIT) {
      continuousPosition -= ENCODER_1_MECHANISM_RATIO;
    }
    encoder1.setPosition(continuousPosition);

    return true;
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
    double e1 = ((e1Raw % 1.0) + 1.0) % 1.0;
    double e2 = ((e2Raw % 1.0) + 1.0) % 1.0;

    int n = findBestWrap(e1, e2);
    if (n < 0) {
      return Double.NaN;
    }

    // Mechanism position = (n + e1) / ENCODER_1_MECHANISM_RATIO
    double mechPosition = (n + e1) / ENCODER_1_MECHANISM_RATIO;

    // Shift to turret range [-0.25, 0.75): values in [0.75, 1) map to [-0.25, 0)
    if (mechPosition >= 0.75) {
      mechPosition -= 1.0;
    }

    return mechPosition;
  }

  /**
   * Finds which of the 5 encoder-1 wraps best matches the encoder 2 reading using CRT.
   *
   * <p>Uses best-candidate selection: all candidates are evaluated and the one with the smallest
   * encoder 2 error is chosen. The tolerance is only used as a sanity check to detect hardware
   * failures (slipped gear, dead encoder), not for candidate selection.
   *
   * @param e1 Encoder 1 absolute position wrapped to [0, 1)
   * @param e2 Encoder 2 absolute position wrapped to [0, 1)
   * @return the winning wrap index n (0-4), or -1 if the sanity check fails
   */
  static int findBestWrap(double e1, double e2) {
    int searchLimit = (int) ENCODER_1_MECHANISM_RATIO;
    int bestN = -1;
    double bestError = Double.MAX_VALUE;

    for (int n = 0; n < searchLimit; n++) {
      double mech = (n + e1) / ENCODER_1_MECHANISM_RATIO;
      if (mech >= FORWARD_LIMIT) {
        mech -= 1.0;
      }
      double expectedE2 = ((mech * ENCODER_2_MECHANISM_RATIO) % 1.0 + 1.0) % 1.0;
      double error = Math.abs(wrapDiffStatic(e2, expectedE2));

      if (error < bestError) {
        bestError = error;
        bestN = n;
      }
    }

    // Sanity check: if the best candidate still has large error, something is wrong
    if (bestError > CRT_CONSISTENCY_TOLERANCE) {
      return -1;
    }

    return bestN;
  }

  private static double wrapDiffStatic(double a, double b) {
    double diff = a - b;
    if (diff > 0.5) diff -= 1.0;
    if (diff < -0.5) diff += 1.0;
    return diff;
  }
}
