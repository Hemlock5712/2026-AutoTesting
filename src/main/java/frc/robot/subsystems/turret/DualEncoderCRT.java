package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Robot;

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
  public static final double MOTOR_TO_MECHANISM_RATIO = 110.0 / 15.0 * 5.0; // 30.8

  // Encoder ratios (encoder rotations per mechanism rotation)
  public static final double ENCODER_1_MECHANISM_RATIO = 21.0;
  public static final double ENCODER_2_MECHANISM_RATIO = 22.0;

  // Position limits (mechanism rotations)
  public static final double FORWARD_LIMIT = 0.75; // +270 degrees
  public static final double REVERSE_LIMIT = -0.25; // -90 degrees

  // ==================== Instance Fields ====================

  private final CANcoder encoder1; // 21:1 from mechanism
  private final CANcoder encoder2; // 22:1 from mechanism

  /**
   * Creates a new DualEncoderCRT calculator.
   *
   * @param encoder1 The first CANcoder (21:1 from mechanism)
   * @param encoder2 The second CANcoder (22:1 from mechanism)
   */
  public DualEncoderCRT(CANcoder encoder1, CANcoder encoder2) {
    this.encoder1 = encoder1;
    this.encoder2 = encoder2;
  }

  /**
   * Calculate the absolute mechanism position using CRT. With 21:1 and 22:1 ratios, diff = e2 - e1
   * directly gives mechanism position.
   *
   * @return mechanism position in rotations (centered around 0)
   */
  public double calculateMechanismPosition() {
    // Get status signals for both encoders
    StatusSignal<Angle> e1Signal = encoder1.getAbsolutePosition();
    StatusSignal<Angle> e2Signal = encoder2.getAbsolutePosition();

    // Wait for both signals to be valid (up to 10ms timeout)
    BaseStatusSignal.waitForAll(10, e1Signal, e2Signal);

    // Get absolute encoder positions
    double e1Raw = e1Signal.getValue().in(Rotations);
    double e2Raw = e2Signal.getValue().in(Rotations);

    // Wrap to [0, 1) range using MathUtil for robustness
    double e1 = MathUtil.inputModulus(e1Raw, 0.0, 1.0);
    double e2 = MathUtil.inputModulus(e2Raw, 0.0, 1.0);

    Robot.telemetry().log("Testing/E1Raw", e1Raw);
    Robot.telemetry().log("Testing/E2Raw", e2Raw);
    Robot.telemetry().log("Testing/E1Wrapped", e1);
    Robot.telemetry().log("Testing/E2Wrapped", e2);

    // CRT: difference directly gives mechanism position within [0, 1)
    // Because (22-21) = 1, the diff advances 1 per mechanism rotation
    double mechanismPosition = MathUtil.inputModulus(e2 - e1, 0.0, 1.0);

    Robot.telemetry().log("Testing/CRT_MechPos", mechanismPosition);

    // Shift to turret range [-0.25, 0.75)
    if (mechanismPosition > 0.75) {
      mechanismPosition -= 1.0;
    }

    Robot.telemetry().log("Testing/CRT_MechPosFinal", mechanismPosition);

    return mechanismPosition;
  }
}
