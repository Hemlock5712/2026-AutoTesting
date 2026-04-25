// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.TalonFXUtil;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private static final double MAX_RADIAL_MISS_M = 0.25; // tune empirically
  private static final double FEED_RADIAL_MISS_M = 2.0;
  private static final double MIN_FLYWHEEL_TOLERANCE_RPS = 2.0;
  private static final double MAX_VERTICAL_MISS_M = 0.2; // tune empirically
  private static final double FEED_VERTICAL_MISS_M = 0.4;
  private static final double MIN_HOOD_TOLERANCE_DEG = 1.5;
  private static final double MAX_HOOD_OUTSIDE_ZONE_DEG = 0.0;

  private boolean inAllianceZone = false;

  // Cached — latency compensation involves computation, not just a field read
  private double cachedHoodPositionDeg;

  private CANBus turretCanBus = new CANBus("turret");

  // Main motor that spins the flywheel
  protected final TalonFX flywheel = new TalonFX(28, turretCanBus);
  protected final TalonFX follower = new TalonFX(52, turretCanBus);

  // Hood motor and encoder for position control
  protected final TalonFX hood = new TalonFX(29, turretCanBus);
  protected final CANcoder hoodEncoder = new CANcoder(30, turretCanBus);

  // Controller for spinning the flywheel at a target speed
  private final VelocityTorqueCurrentFOC velocityOut = new VelocityTorqueCurrentFOC(0);

  private final PositionTorqueCurrentFOC rotationOut =
      new PositionTorqueCurrentFOC(Rotations.of(0));

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

  private final Debouncer atHubSpeed = new Debouncer(0.25, DebounceType.kFalling);
  private final Debouncer atFeedSpeed = new Debouncer(0.5, DebounceType.kFalling);

  public Shooter() {
    // Coast mode: Flywheel can spin freely by hand when disabled
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // Set motor direction: positive power = counterclockwise spin
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Control values
    config.Slot0.kS = 10.0; // Static friction
    config.Slot0.kV = 0.03; // Velocity feedforward
    config.Slot0.kP = 6; // Proportional gain
    config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

    config.Feedback.SensorToMechanismRatio = 1.33333333333;

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
    hoodConfig.Slot0.kG = 5; // Gravity compensation
    hoodConfig.Slot0.kS = 3; // Static friction
    hoodConfig.Slot0.kV = 0.0; // Velocity feedforward
    hoodConfig.Slot0.kP = 6400; // Proportional gain
    hoodConfig.Slot0.kD = 175; // Proportional gain
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
    hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.22;
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

    flywheelVelocitySignal.setUpdateFrequency(100);
    hoodPositionSignal.setUpdateFrequency(100);
    hoodVelocitySignal.setUpdateFrequency(100);

    flywheel.optimizeBusUtilization();
    hood.optimizeBusUtilization();
    hoodEncoder.optimizeBusUtilization();
    follower.optimizeBusUtilization();
  }

  @Override
  public void periodic() {
    long _t = System.nanoTime();
    BaseStatusSignal.refreshAll(flywheelVelocitySignal, hoodPositionSignal, hoodVelocitySignal);
    cachedHoodPositionDeg =
        BaseStatusSignal.getLatencyCompensatedValueAsDouble(hoodPositionSignal, hoodVelocitySignal)
            * 360.0;
    Logger.recordOutput("Timing/ShooterMs", (System.nanoTime() - _t) / 1e6);
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
    double angleDeg = angle.in(Degrees);
    if (inAllianceZone && angleDeg >= MAX_HOOD_OUTSIDE_ZONE_DEG) {
      angleDeg = MAX_HOOD_OUTSIDE_ZONE_DEG;
    }
    hood.setControl(rotationOut.withPosition(angleDeg / 360.0));
  }

  /** Primitive-degrees overload to avoid Degrees.of() allocations in hot loops. */
  public void setPositionDeg(double angleDeg) {
    if (inAllianceZone && angleDeg >= MAX_HOOD_OUTSIDE_ZONE_DEG) {
      angleDeg = MAX_HOOD_OUTSIDE_ZONE_DEG;
    }
    hood.setControl(rotationOut.withPosition(angleDeg / 360.0));
  }

  public void setInAllianceZone(boolean inZone) {
    if (inZone != this.inAllianceZone) {
      this.inAllianceZone = inZone;
      setPosition(getTargetPosition());
    }
  }

  /** Tuning mode: continuously set flywheel velocity and hood angle from dashboard values. */
  public Command runShooterTestMode(DoubleSupplier velocity, Supplier<Angle> angle) {
    return run(
        () -> {
          setVelocity(velocity.getAsDouble());
          setPosition(angle.get());
        });
  }

  /** Primitive overload — velocity in RPS, hood angle in degrees. Zero allocations. */
  public Command runShooterTestModeDeg(DoubleSupplier velocity, DoubleSupplier angleDeg) {
    return run(
        () -> {
          setVelocity(velocity.getAsDouble());
          setPositionDeg(angleDeg.getAsDouble());
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
    return flywheelVelocitySignal.isNear(velocityOut.Velocity, 2.0);
  }

  /**
   * Check if the hood has reached its target position.
   *
   * @return true if close enough to target position, false otherwise
   */
  public boolean hoodIsAtTarget() {
    return Math.abs(cachedHoodPositionDeg - rotationOut.Position * 360.0) <= MIN_HOOD_TOLERANCE_DEG;
  }

  public boolean isAtTarget() {
    return flywheelIsAtTarget() && hoodIsAtTarget();
  }

  /** Distance-dependent check: would this flywheel/hood produce a scoring shot at this range? */
  public boolean isAtTarget(double distance) {
    double targetRPS = velocityOut.Velocity;
    double flywheelTolerance =
        Math.max(MIN_FLYWHEEL_TOLERANCE_RPS, targetRPS * MAX_RADIAL_MISS_M / distance);
    boolean flywheelOk =
        Math.abs(flywheelVelocitySignal.getValueAsDouble() - targetRPS) <= flywheelTolerance;
    double hoodToleranceDeg =
        Math.max(MIN_HOOD_TOLERANCE_DEG, Math.toDegrees(Math.atan(MAX_VERTICAL_MISS_M / distance)));
    boolean hoodOk =
        Math.abs(cachedHoodPositionDeg - rotationOut.Position * 360.0) <= hoodToleranceDeg;
    return atHubSpeed.calculate(flywheelOk && hoodOk);
  }

  /** Looser check for feed shots - wider tolerance than hub shots. */
  public boolean isAtFeedTarget(double distance) {
    double targetRPS = velocityOut.Velocity;
    double flywheelTolerance =
        Math.max(MIN_FLYWHEEL_TOLERANCE_RPS, targetRPS * FEED_RADIAL_MISS_M / distance);
    boolean flywheelOk =
        Math.abs(flywheelVelocitySignal.getValueAsDouble() - targetRPS) <= flywheelTolerance;
    double hoodToleranceDeg =
        Math.max(
            MIN_HOOD_TOLERANCE_DEG, Math.toDegrees(Math.atan(FEED_VERTICAL_MISS_M / distance)));
    boolean hoodOk =
        Math.abs(cachedHoodPositionDeg - rotationOut.Position * 360.0) <= hoodToleranceDeg;
    return atFeedSpeed.calculate(flywheelOk && hoodOk);
  }

  /**
   * Get how fast the flywheel is currently spinning.
   *
   * @return Current flywheel speed
   */
  public AngularVelocity getVelocity() {
    return flywheelVelocitySignal.getValue();
  }

  @AutoLogOutput
  public double getVelocityRPS() {
    return flywheelVelocitySignal.getValueAsDouble();
  }

  /**
   * Get what position the hood is at.
   *
   * @return Current hood position (latency-compensated)
   */
  @AutoLogOutput
  public double getPositionDeg() {
    return cachedHoodPositionDeg;
  }

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
  public AngularVelocity getTargetVelocity() {
    return velocityOut.getVelocityMeasure();
  }

  @AutoLogOutput
  public double getTargetVelocityRPS() {
    return velocityOut.Velocity;
  }

  /**
   * Get what position the hood is trying to reach.
   *
   * @return Target hood position
   */
  public Angle getTargetPosition() {
    return rotationOut.getPositionMeasure();
  }

  @AutoLogOutput
  public double getTargetPositionDeg() {
    return rotationOut.Position * 360.0;
  }

  public void stopMotors() {
    flywheel.stopMotor();
    hood.stopMotor();
  }

  /**
   * Set flywheel and hood using separate distances — flywheel uses effective (radial-compensated)
   * distance while hood uses geometric distance.
   */
  private void setForDistanceSWM(double flywheelDist, double hoodDist) {
    setVelocity(ShooterLookup.getFlywheelMap().get(flywheelDist));
    setPositionDeg(ShooterLookup.getHoodMap().get(hoodDist));
  }

  public void setForDistance(double distanceMeters) {
    setVelocity(ShooterLookup.getFlywheelMap().get(distanceMeters));
    setPositionDeg(ShooterLookup.getHoodMap().get(distanceMeters));
  }

  public void setForFeedDistance(double flywheelDist, double hoodDist) {
    setVelocity(ShooterLookup.getFeedFlywheelMap().get(flywheelDist));
    setPositionDeg(ShooterLookup.getFeedHoodMap().get(hoodDist));
  }

  /** Command that continuously sets the hood position based on distance lookup. */
  public Command runHoodDynamic(DoubleSupplier distance, BooleanSupplier isShooting) {
    return Commands.either(
        run(() -> setPositionDeg(ShooterLookup.getHoodMap().get(distance.getAsDouble()))),
        run(() -> setPositionDeg(0)),
        isShooting);
  }

  /** Command that continuously sets flywheel and hood for a feed shot based on distance. */
  public Command runDynamicFeed(DoubleSupplier flywheelDist, DoubleSupplier hoodDist) {
    return run(() -> setForFeedDistance(flywheelDist.getAsDouble(), hoodDist.getAsDouble()));
  }

  /** Command that sets shooter for SWM with separate flywheel and hood distances. */
  public Command runDynamicSWM(DoubleSupplier flywheelDist, DoubleSupplier hoodDist) {
    return run(() -> setForDistanceSWM(flywheelDist.getAsDouble(), hoodDist.getAsDouble()));
  }
}
