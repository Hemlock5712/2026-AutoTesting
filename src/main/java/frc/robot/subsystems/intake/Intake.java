// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

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
import frc.robot.generated.TunerConstants;
import frc.robot.utils.TalonFXUtil;

@Logged
public class Intake extends SubsystemBase {
  // Position setpoints using Angle objects for type safety
  private static final Angle UP = Rotations.of(.27);
  private static final Angle DOWN = Degrees.of(0);

  // Main motor that moves the arm (device ID 31)
  protected final TalonFX arm = new TalonFX(22, TunerConstants.kCANBus);
  // Sensor that tells us the arm's exact angle (device ID 32)
  protected final CANcoder arm_encoder = new CANcoder(24, TunerConstants.kCANBus);

  // Main motor that moves the intake (device ID 31)
  protected final TalonFX wheel = new TalonFX(23, TunerConstants.kCANBus);

  // Configuration settings for the arm motor
  protected TalonFXConfiguration config = new TalonFXConfiguration();
  protected TalonFXConfiguration configWheel = new TalonFXConfiguration();

  // Controller for moving the arm to specific positions
  private final MotionMagicVoltage positionOut = new MotionMagicVoltage(0);

  // Gets the error between current and target position
  private Angle TOLERANCE = Degrees.of(3);

  // Alert for motor configuration failures
  Alert motorConfigAlert = new Alert("Arm Motor Configuration Failed", AlertType.kError);

  public Intake() {
    // Coast mode: Motor can be moved by hand when disabled (easier for testing)
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // Set motor direction: positive power = counterclockwise rotation
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.Slot0.GravityType =
        GravityTypeValue.Arm_Cosine; // Automatically fights gravity using math

    // Control values (TODO: CRITICAL - Tune these on the real robot!)
    config.Slot0.kG = 0.7998046875; // Gravity compensation
    config.Slot0.kS = 0.0; // Static friction
    config.Slot0.kP = 16; // Proportional gain (speed of correction)
    config.Slot0.kD = 1; // Derivative gain (smoothness)

    // Motion limits (TODO: CRITICAL - Set non-zero values!)
    config.MotionMagic.MotionMagicCruiseVelocity = 0.0; // Max speed
    config.MotionMagic.MotionMagicAcceleration = 0.0; // How fast to speed up
    // Tell the motor to use the CANcoder sensor for position measurements
    config.Feedback.withRemoteCANcoder(arm_encoder);
    config.Feedback.RotorToSensorRatio = 25;

    // Coast mode: Motor can be moved by hand when disabled (easier for testing)
    configWheel.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Set motor direction: positive power = counterclockwise rotation
    configWheel.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    configWheel.Slot0.GravityType =
        GravityTypeValue.Arm_Cosine; // Automatically fights gravity using math

    // Control values (TODO: CRITICAL - Tune these on the real robot!)
    configWheel.Slot0.kG = 0.0; // Gravity compensation
    configWheel.Slot0.kS = 0.0; // Static friction
    configWheel.Slot0.kP = 0.0; // Proportional gain (speed of correction)
    configWheel.Slot0.kD = 0.0; // Derivative gain (smoothness)

    // Motion limits (TODO: CRITICAL - Set non-zero values!)
    configWheel.MotionMagic.MotionMagicCruiseVelocity = 0.0; // Max speed
    configWheel.MotionMagic.MotionMagicAcceleration = 0.0; // How fast to speed up

    // 2.33

    configWheel.Feedback.SensorToMechanismRatio = 2.33;

    // Apply configuration with retries
    boolean success = TalonFXUtil.applyConfigWithRetries(arm, config);
    motorConfigAlert.set(!success);

    boolean successWheel = TalonFXUtil.applyConfigWithRetries(wheel, configWheel);
    motorConfigAlert.set(!successWheel);
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
    arm.setControl(positionOut.withPosition(position.in(Rotations)));
  }

  /**
   * Command to move the arm to scoring position.
   *
   * @return Command that moves arm to scoring angle
   */
  public Command intakeStowed() {
    return runOnce(() -> setPosition(UP));
  }

  /**
   * Command to move the arm to high scoring position (far shots).
   *
   * @return Command that moves arm to high scoring angle
   */
  public Command intakeDown() {
    return runOnce(() -> setPosition(DOWN)).until(() -> isAtTarget()).andThen(stopArm());
  }

  /**
   * Command to stop the arm motor.
   *
   * @return Command that stops the wheels
   */
  public Command stopWheel() {
    return runOnce(() -> wheel_stop());
  }

  // Stop the wheel motor (private to enforce Command-based control flow)
  private void wheel_stop() {
    wheel.stopMotor();
  }

  /**
   * Command to stop the arm motor.
   *
   * @return Command that stops the wheels
   */
  public Command stopArm() {
    return runOnce(() -> arm_stop());
  }

  // Stop the wheel motor (private to enforce Command-based control flow)
  private void arm_stop() {
    arm.stopMotor();
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
    return arm_encoder.getPosition().getValue();
  }

  public double getVelocity() {
    return wheel.getVelocity().getValueAsDouble();
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

  public Command runIntake() {
    return run(() -> wheel.setVoltage(4));
  }
}
