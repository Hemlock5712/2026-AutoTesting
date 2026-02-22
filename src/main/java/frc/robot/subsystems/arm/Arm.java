// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.TalonFXUtil;

@Logged
public class Arm extends SubsystemBase {
  // Position setpoints using Angle objects for type safety
  private static final Angle VERTICAL_POSITION = Degrees.of(90);
  private static final Angle HORIZONTAL_POSITION = Degrees.of(180);
  private static final Angle SCORING_POSITION = Degrees.of(30);
  private static final Angle SCORING_HIGH_POSITION = Degrees.of(45);
  private static final Angle TOLERANCE = Degrees.of(1.0);

  // Connect to the "canivore" CAN bus (communication network for motors)
  private final CANBus canivore = new CANBus("canivore");

  // Main motor that moves the arm (device ID 31)
  protected final TalonFX leader = new TalonFX(31, canivore);
  // Sensor that tells us the arm's exact angle (device ID 32)
  protected final CANcoder encoder = new CANcoder(32, canivore);

  // Configuration settings for the arm motor
  protected TalonFXConfiguration config = new TalonFXConfiguration();

  // Controller for moving the arm to specific positions
  private final MotionMagicVoltage positionOut = new MotionMagicVoltage(0);

  // Alert for motor configuration failures
  Alert motorConfigAlert = new Alert("Arm Motor Configuration Failed", AlertType.kError);

  public Arm() {
    // Coast mode: Motor can be moved by hand when disabled (easier for testing)
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // Set motor direction: positive power = counterclockwise rotation
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.Slot0.GravityType =
        GravityTypeValue.Arm_Cosine; // Automatically fights gravity using math

    // Control values (TODO: CRITICAL - Tune these on the real robot!)
    config.Slot0.kG = 0.0; // Gravity compensation
    config.Slot0.kS = 0.0; // Static friction
    config.Slot0.kP = 0.0; // Proportional gain (speed of correction)
    config.Slot0.kD = 0.0; // Derivative gain (smoothness)

    // Motion limits (TODO: CRITICAL - Set non-zero values!)
    config.MotionMagic.MotionMagicCruiseVelocity = 0.0; // Max speed
    config.MotionMagic.MotionMagicAcceleration = 0.0; // How fast to speed up
    // Tell the motor to use the CANcoder sensor for position measurements
    config.Feedback.withRemoteCANcoder(encoder);

    // Apply configuration with retries
    boolean success = TalonFXUtil.applyConfigWithRetries(leader, config);
    motorConfigAlert.set(!success);
  }

  @Override
  public void periodic() {
    // No periodic updates needed - control is entirely feedforward/feedback
  }

  /**
   * Move the arm to a specific angle.
   *
   * @param position Where to move the arm
   */
  private void setPosition(Angle position) {
    // Tell the motor to move to this position (convert to rotations for motor)
    leader.setControl(positionOut.withPosition(position.in(Rotations)));
  }

  /**
   * Command to move the arm to vertical position (safe for transport).
   *
   * @return Command that moves arm vertical
   */
  public Command vertical() {
    return runOnce(() -> setPosition(VERTICAL_POSITION));
  }

  /**
   * Command to move the arm to horizontal position (for ground intake).
   *
   * @return Command that moves arm horizontal
   */
  public Command horizontal() {
    return runOnce(() -> setPosition(HORIZONTAL_POSITION));
  }

  /**
   * Command to move the arm to scoring position.
   *
   * @return Command that moves arm to scoring angle
   */
  public Command scoringPosition() {
    return runOnce(() -> setPosition(SCORING_POSITION));
  }

  /**
   * Command to move the arm to high scoring position (far shots).
   *
   * @return Command that moves arm to high scoring angle
   */
  public Command scoringHighPosition() {
    return runOnce(() -> setPosition(SCORING_HIGH_POSITION));
  }

  /**
   * Command to stop the arm motor.
   *
   * @return Command that stops the arm
   */
  public Command stopCommand() {
    return runOnce(() -> stop());
  }

  // Stop the arm motor (private to enforce Command-based control flow)
  private void stop() {
    leader.stopMotor();
  }

  /**
   * Check if the arm has reached its target position.
   *
   * @return true if close enough to target, false otherwise
   */
  public boolean isAtTarget() {
    return getPosition().isNear(getTargetPosition(), TOLERANCE);
  }

  /**
   * Get where the arm currently is.
   *
   * @return Current arm angle
   */
  public Angle getPosition() {
    return encoder.getPosition().getValue();
  }

  /**
   * Get where the arm is trying to move to.
   *
   * @return Target arm angle
   */
  public Angle getTargetPosition() {
    return positionOut.getPositionMeasure();
  }

  /**
   * Get the position tolerance for "at target" checks.
   *
   * @return Position tolerance
   */
  public Angle getTolerance() {
    return TOLERANCE;
  }
}
