// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.TalonFXUtil;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

@Logged
public class Shooter extends SubsystemBase {
  // Shooting speeds (typed AngularVelocity for type-safe unit handling)
  private static final AngularVelocity TOLERANCE = RotationsPerSecond.of(1);
  private static final Angle HOOD_TOLERANCE = Degree.of(1);

  // Main motor that spins the flywheel
  protected final TalonFX flywheel = new TalonFX(28, CANBus.roboRIO());
  protected final TalonFX follower = new TalonFX(52, CANBus.roboRIO());

  // Hood motor and encoder for position control
  protected final TalonFX hood = new TalonFX(29, CANBus.roboRIO());
  protected final CANcoder hoodEncoder = new CANcoder(30, CANBus.roboRIO());

  // Controller for spinning the flywheel at a target speed
  private final VelocityTorqueCurrentFOC velocityOut = new VelocityTorqueCurrentFOC(0);

  private final MotionMagicVoltage rotationOut = new MotionMagicVoltage(0);

  // Configuration settings for the flywheel motor
  protected TalonFXConfiguration config = new TalonFXConfiguration();

  // Configuration settings for the flywheel motor
  protected TalonFXConfiguration confighood = new TalonFXConfiguration();

  // Alert for motor configuration failures
  Alert motorConfigAlert = new Alert("Shooter Motor Configuration Failed", AlertType.kError);

  public Shooter() {
    // Coast mode: Flywheel can spin freely by hand when disabled
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // Set motor direction: positive power = counterclockwise spin
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Control values
    config.Slot0.kS = 5.0; // Static friction
    config.Slot0.kV = 0.16; // Velocity feedforward
    config.Slot0.kP = 10; // Proportional gain
    config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

    config.Feedback.SensorToMechanismRatio = 2.0;

    // Speed limits (CTRE uses rotations per second for velocity, RPS² for acceleration)
    config.MotionMagic.MotionMagicCruiseVelocity = 0.0; // RPS
    config.MotionMagic.MotionMagicAcceleration = 0.0; // RPS²

    // Apply configuration with retries
    boolean success = TalonFXUtil.applyConfigWithRetries(flywheel, config);
    motorConfigAlert.set(!success);

    // Coast mode: Flywheel can spin freely by hand when disabled
    confighood.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // Set motor direction: positive power = counterclockwise spin
    confighood.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Control values
    confighood.Slot0.kS = 0.33; // Static friction
    confighood.Slot0.kV = 0.0; // Velocity feedforward
    confighood.Slot0.kP = 200; // Proportional gain
    confighood.Slot0.kD = 3; // Proportional gain
    confighood.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

    confighood.Feedback.SensorToMechanismRatio = 2.0;

    // Speed limits (CTRE uses rotations per second for velocity, RPS² for acceleration)
    confighood.MotionMagic.MotionMagicCruiseVelocity = 0.5; // RPS
    confighood.MotionMagic.MotionMagicAcceleration = 1.0; // RPS²

    confighood.Feedback.FeedbackRemoteSensorID = hoodEncoder.getDeviceID();
    confighood.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    confighood.Feedback.SensorToMechanismRatio = 3;
    confighood.Feedback.RotorToSensorRatio = 75.38;

    // Soft limits to prevent exceeding -90 to +270 degree physical range
    confighood.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    confighood.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.0555;
    confighood.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    confighood.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

    // Apply configuration with retries
    boolean successhood = TalonFXUtil.applyConfigWithRetries(hood, confighood);
    motorConfigAlert.set(!successhood);

    flywheel.getTorqueCurrent().setUpdateFrequency(500);
    follower.setControl(new Follower(flywheel.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  @Override
  public void periodic() {
    // No periodic updates needed - control is entirely feedforward/feedback
  }

  /**
   * Spin the flywheel at a specific speed. Private to enforce Command-based control flow.
   *
   * @param velocity How fast to spin (rotations per second)
   */
  public void setVelocity(double velocity) {
    flywheel.setControl(velocityOut.withVelocity(velocity));
  }

  /**
   * Sets the hood to a specific position. Private to enforce Command-based control flow.
   *
   * @param angle What position to go to
   */
  public void setPosition(Angle angle) {
    hood.setControl(rotationOut.withPosition(angle));
  }

  /**
   * @param velocity
   * @return
   */
  public Command runVelocity(double velocity) {
    return runOnce(() -> setVelocity(velocity));
  }

  /**
   * @param velocity
   * @return
   */
  public Command runVelocity(DoubleSupplier velocity) {
    return runOnce(() -> setVelocity(velocity.getAsDouble()));
  }

  /**
   * @param angle
   * @return
   */
  public Command runPosition(Angle angle) {
    return runOnce(() -> setPosition(angle));
  }

  /**
   * @param angle
   * @return
   */
  public Command runPosition(Supplier<Angle> angle) {
    return runOnce(() -> setPosition(angle.get()));
  }

  public Command runShooterTestMode(DoubleSupplier velocity, Supplier<Angle> angle) {
    return run(
        () -> {
          setVelocity(velocity.getAsDouble());
          setPosition(angle.get());
        });
  }

  /**
   * Command to stop the motors.
   *
   * @return Command that stops the motors
   */
  public Command stopCommand() {
    return runOnce(() -> stopMotors());
  }

  /**
   * Check if the flywheel has reached its target speed.
   *
   * @return true if close enough to target speed, false otherwise
   */
  public boolean flywheelIsAtTarget() {
    return getVelocity().isNear(getTargetVelocity(), TOLERANCE);
  }

  /**
   * Check if the hood has reached its target position.
   *
   * @return true if close enough to target position, false otherwise
   */
  public boolean hoodIsAtTarget() {
    return getPosition().isNear(getTargetPosition(), HOOD_TOLERANCE);
  }

  /**
   * Get how fast the flywheel is currently spinning.
   *
   * @return Current flywheel speed
   */
  public AngularVelocity getVelocity() {
    return flywheel.getVelocity().getValue();
  }

  /**
   * Get what position the hood is at.
   *
   * @return Current hood position
   */
  public Angle getPosition() {
    return hood.getPosition().getValue();
  }

  /**
   * Get what speed the flywheel is trying to reach.
   *
   * @return Target flywheel speed
   */
  public AngularVelocity getTargetVelocity() {
    return velocityOut.getVelocityMeasure();
  }

  /**
   * Get what position the hood is trying to reach.
   *
   * @return Target hood position
   */
  public Angle getTargetPosition() {
    return rotationOut.getPositionMeasure();
  }

  /**
   * Get the speed tolerance for "at target" checks.
   *
   * @return Speed tolerance
   */
  public AngularVelocity getTolerance() {
    return TOLERANCE;
  }

  /**
   * Get the position tolerance for "at target" checks.
   *
   * @return Position tolerance
   */
  public Angle getHoodTolerance() {
    return HOOD_TOLERANCE;
  }

  // Stop the shooter motors (private to enforce Command-based control flow)
  private void stopMotors() {
    flywheel.stopMotor();
    hood.stopMotor();
  }

  /**
   * Set flywheel velocity and hood position from direct values (for SWM solver).
   *
   * @param flywheelRPS Flywheel speed in rotations per second
   * @param elevationDeg Hood elevation angle in degrees
   */
  public void setFromSolution(double flywheelRPS, double elevationDeg) {
    setVelocity(flywheelRPS);
    setPosition(Degrees.of(elevationDeg));
  }

  /**
   * Command that sets shooter from SWM solution values.
   *
   * @param rpsSupplier Supplier for flywheel RPS
   * @param elevationSupplier Supplier for elevation in degrees
   * @return Command that continuously sets flywheel and hood from solution
   */
  public Command runFromSolution(DoubleSupplier rpsSupplier, DoubleSupplier elevationSupplier) {
    return run(() -> setFromSolution(rpsSupplier.getAsDouble(), elevationSupplier.getAsDouble()));
  }

  /**
   * Command that continuously tracks hood position from a dynamic elevation supplier.
   *
   * @param elevationDegSupplier Supplier for elevation in degrees
   * @return Command that continuously sets hood position
   */
  public Command runHoodDynamic(DoubleSupplier elevationDegSupplier) {
    return run(() -> setPosition(Degrees.of(elevationDegSupplier.getAsDouble())));
  }
}
