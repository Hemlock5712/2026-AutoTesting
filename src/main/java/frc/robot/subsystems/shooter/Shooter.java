// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.utils.TalonFXUtil;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

@Logged(strategy = Strategy.OPT_IN)
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

  private final Debouncer atTargetDebouncer = new Debouncer(0.1, DebounceType.kFalling);

  // Configuration settings for the flywheel motor
  protected TalonFXConfiguration config = new TalonFXConfiguration();

  // Configuration settings for the hood motor
  protected TalonFXConfiguration hoodConfig = new TalonFXConfiguration();

  // Alert for motor configuration failures
  // Cached status signals for latency compensation
  private final StatusSignal<AngularVelocity> flywheelVelocitySignal;
  private final StatusSignal<Angle> hoodPositionSignal;
  private final StatusSignal<AngularVelocity> hoodVelocitySignal;

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

    config.Feedback.SensorToMechanismRatio = 1.66666666666667;

    // Speed limits (CTRE uses rotations per second for velocity, RPS² for acceleration)
    config.MotionMagic.MotionMagicCruiseVelocity = 0.0; // RPS
    config.MotionMagic.MotionMagicAcceleration = 0.0; // RPS²

    // Apply configuration with retries
    boolean success = TalonFXUtil.applyConfigWithRetries(flywheel, config);
    motorConfigAlert.set(!success);

    // Coast mode: Flywheel can spin freely by hand when disabled
    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // Set motor direction: positive power = counterclockwise spin
    hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Control values
    hoodConfig.Slot0.kG = 0.35; // Gravity compensation
    hoodConfig.Slot0.kS = 0.1; // Static friction
    hoodConfig.Slot0.kV = 0.0; // Velocity feedforward
    hoodConfig.Slot0.kP = 100; // Proportional gain
    hoodConfig.Slot0.kD = 0; // Proportional gain
    hoodConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    hoodConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    // Speed limits (CTRE uses rotations per second for velocity, RPS² for acceleration)
    hoodConfig.MotionMagic.MotionMagicCruiseVelocity = 1.0; // RPS
    hoodConfig.MotionMagic.MotionMagicAcceleration = 0.4; // RPS²

    hoodConfig.Feedback.FeedbackRemoteSensorID = hoodEncoder.getDeviceID();
    hoodConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    hoodConfig.Feedback.SensorToMechanismRatio = 3;
    hoodConfig.Feedback.RotorToSensorRatio = 75.38;

    // Soft limits to prevent exceeding -90 to +270 degree physical range
    hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.069444;
    hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

    // Apply configuration with retries
    boolean hoodConfigSuccess = TalonFXUtil.applyConfigWithRetries(hood, hoodConfig);
    motorConfigAlert.set(!hoodConfigSuccess);

    flywheel.getTorqueCurrent().setUpdateFrequency(500);
    follower.setControl(new Follower(flywheel.getDeviceID(), MotorAlignmentValue.Opposed));

    // Cache status signals and set update frequencies
    flywheelVelocitySignal = flywheel.getVelocity();
    hoodPositionSignal = hood.getPosition();
    hoodVelocitySignal = hood.getVelocity();

    // flywheelVelocitySignal.setUpdateFrequency(100);
    // hoodPositionSignal.setUpdateFrequency(100);
    // hoodVelocitySignal.setUpdateFrequency(100);

    // flywheel.optimizeBusUtilization();
    // hood.optimizeBusUtilization();
    // hoodEncoder.optimizeBusUtilization();
    // follower.optimizeBusUtilization();
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(flywheelVelocitySignal, hoodPositionSignal, hoodVelocitySignal);
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

  /** Tuning mode: continuously set flywheel velocity and hood angle from dashboard values. */
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
  @Logged
  public boolean flywheelIsAtTarget() {
    return getVelocity().isNear(getTargetVelocity(), TOLERANCE);
  }

  /**
   * Check if the hood has reached its target position.
   *
   * @return true if close enough to target position, false otherwise
   */
  @Logged
  public boolean hoodIsAtTarget() {
    return getPosition().isNear(getTargetPosition(), HOOD_TOLERANCE);
  }

  @Logged
  public boolean isAtTarget() {
    return flywheelIsAtTarget() && hoodIsAtTarget();
  }

  /** Distance-dependent check: would this flywheel/hood produce a scoring shot at this range? */
  public boolean isAtTarget(double distance) {
    double margin = 0.2; // ~50% of goal radius
    double minDist = Math.max(1.5, distance - margin);
    double maxDist = Math.min(5.5, distance + margin);

    double actualRPS = getVelocity().in(RotationsPerSecond);
    boolean flywheelOk =
        actualRPS >= ShooterLookup.getFlywheelMap().get(minDist)
            && actualRPS <= ShooterLookup.getFlywheelMap().get(maxDist);

    double actualHoodDeg = getPosition().in(Degrees);
    boolean hoodOk =
        actualHoodDeg >= ShooterLookup.getHoodMap().get(minDist)
            && actualHoodDeg <= ShooterLookup.getHoodMap().get(maxDist);
    boolean debouncedTrue = atTargetDebouncer.calculate(flywheelOk);
    Robot.telemetry().log("SWM/DebounceAtTarget", debouncedTrue);
    return debouncedTrue;
  }

  /**
   * Get how fast the flywheel is currently spinning.
   *
   * @return Current flywheel speed
   */
  @Logged
  public AngularVelocity getVelocity() {
    return flywheelVelocitySignal.getValue();
  }

  /**
   * Get what position the hood is at.
   *
   * @return Current hood position (latency-compensated)
   */
  @Logged
  public Angle getPosition() {
    return Rotations.of(
        BaseStatusSignal.getLatencyCompensatedValueAsDouble(
            hoodPositionSignal, hoodVelocitySignal));
  }

  /**
   * Get what speed the flywheel is trying to reach.
   *
   * @return Target flywheel speed
   */
  @Logged
  public AngularVelocity getTargetVelocity() {
    return velocityOut.getVelocityMeasure();
  }

  /**
   * Get what position the hood is trying to reach.
   *
   * @return Target hood position
   */
  @Logged
  public Angle getTargetPosition() {
    return rotationOut.getPositionMeasure();
  }

  /**
   * Get the speed tolerance for "at target" checks.
   *
   * @return Speed tolerance
   */
  @Logged
  public AngularVelocity getTolerance() {
    return TOLERANCE;
  }

  /**
   * Get the position tolerance for "at target" checks.
   *
   * @return Position tolerance
   */
  @Logged
  public Angle getHoodTolerance() {
    return HOOD_TOLERANCE;
  }

  // Stop the shooter motors (private to enforce Command-based control flow)
  private void stopMotors() {
    flywheel.stopMotor();
    hood.stopMotor();
  }

  /**
   * Set flywheel and hood using separate distances — flywheel uses effective (radial-compensated)
   * distance while hood uses geometric distance.
   */
  private void setForDistanceSWM(double flywheelDist, double hoodDist) {
    setVelocity(ShooterLookup.getFlywheelMap().get(flywheelDist));
    setPosition(Degrees.of(ShooterLookup.getHoodMap().get(hoodDist)));
  }

  public void setForDistance(double distanceMeters) {
    setVelocity(ShooterLookup.getFlywheelMap().get(distanceMeters));
    setPosition(Degrees.of(ShooterLookup.getHoodMap().get(distanceMeters)));
  }

  public void setForFeedDistance(double distanceMeters) {
    setVelocity(ShooterLookup.getFeedFlywheelMap().get(distanceMeters));
    setPosition(Degrees.of(ShooterLookup.getFeedHoodMap().get(distanceMeters)));
  }

  /** Check if flywheel is at target for a feed shot (wider tolerance). */
  public boolean isFeedAtTarget(double distance) {
    double margin = 0.5;
    double minDist = Math.max(0.0, distance - margin);
    double maxDist = Math.min(9.5, distance + margin);

    double actualRPS = getVelocity().in(RotationsPerSecond);
    boolean flywheelOk =
        actualRPS >= ShooterLookup.getFeedFlywheelMap().get(minDist)
            && actualRPS <= ShooterLookup.getFeedFlywheelMap().get(maxDist);

    return flywheelOk;
  }

  /** Command that continuously sets the hood position based on distance lookup. */
  public Command runHoodDynamic(DoubleSupplier distance) {
    return run(
        () -> setPosition(Degrees.of(ShooterLookup.getHoodMap().get(distance.getAsDouble()))));
  }

  /** Command that continuously sets flywheel and hood for a feed shot based on distance. */
  public Command runDynamicFeed(DoubleSupplier distance) {
    return run(() -> setForFeedDistance(distance.getAsDouble()));
  }

  /** Command that sets shooter for SWM with separate flywheel and hood distances. */
  public Command runDynamicSWM(DoubleSupplier flywheelDist, DoubleSupplier hoodDist) {
    return run(() -> setForDistanceSWM(flywheelDist.getAsDouble(), hoodDist.getAsDouble()));
  }
}
