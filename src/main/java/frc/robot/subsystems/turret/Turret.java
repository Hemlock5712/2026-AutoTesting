package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.TalonFXUtil;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  // Motor
  protected final TalonFX leader = new TalonFX(DualEncoderCRT.MOTOR_ID, TunerConstants.kCANBus);

  private static final double MAX_LATERAL_MISS_M = 0.4; // 20cm — 50% of goal radius
  private static final double FEED_LATERAL_MISS_M = 0.8; // Wider tolerance for feed shots

  // Dual absolute encoders for CRT positioning
  protected final CANcoder encoder1 =
      new CANcoder(DualEncoderCRT.ENCODER_1_ID, TunerConstants.kCANBus);
  protected final CANcoder encoder2 =
      new CANcoder(DualEncoderCRT.ENCODER_2_ID, TunerConstants.kCANBus);

  // CRT calculator for absolute position determination
  private final DualEncoderCRT crt;

  private final MotionMagicTorqueCurrentFOC angleOut = new MotionMagicTorqueCurrentFOC(0);

  private final Debouncer atTurret = new Debouncer(0.25, DebounceType.kFalling);

  // Shooting gate: distance-dependent position tolerance (~half the effective scoring radius)

  protected TalonFXConfiguration config = new TalonFXConfiguration();

  // Cached status signals for latency compensation
  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;

  // Alerts
  Alert motorConfigAlert = new Alert("Turret Motor Configuration Failed", AlertType.kError);
  Alert crtInitAlert = new Alert("Turret CRT Position Initialization Failed", AlertType.kWarning);

  public Turret() {
    // Initialize CRT calculator using default constants
    crt = new DualEncoderCRT(encoder1, encoder2);

    // Seed encoder 1's continuous position before configuring FusedCANcoder,
    // so the motor sees the correct position from the start.
    initializePosition();
    configureMotor();

    // Cache status signals and set update frequencies for latency compensation
    positionSignal = leader.getPosition();
    velocitySignal = leader.getVelocity();

    positionSignal.setUpdateFrequency(250);
    velocitySignal.setUpdateFrequency(250);

    leader.optimizeBusUtilization();
    encoder1.optimizeBusUtilization();
    encoder2.optimizeBusUtilization();
  }

  /** Configure motor with FusedCANcoder feedback (encoder 1 fused with internal rotor). */
  private void configureMotor() {
    // Fuse encoder 1 (22-tooth gear) with the motor's internal rotor for high-bandwidth
    // absolute position tracking. CRT seeds encoder 1's continuous position at startup
    // via initializePosition(), then FusedCANcoder handles tracking from there.
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    config.Feedback.FeedbackRemoteSensorID = encoder1.getDeviceID();

    // SensorToMechanismRatio: encoder rotations per mechanism rotation
    config.Feedback.SensorToMechanismRatio = DualEncoderCRT.ENCODER_1_MECHANISM_RATIO;
    // RotorToSensorRatio: motor rotor rotations per encoder rotation
    config.Feedback.RotorToSensorRatio = DualEncoderCRT.ROTOR_TO_ENCODER_RATIO;

    // PID gains
    config.Slot0.kS = 0.42; // Static friction compensation
    config.Slot0.kP = 3000; // Proportional gain
    config.Slot0.kD = 64; // Derivative gain
    config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // MotionMagic settings - units are mechanism rotations
    config.MotionMagic.MotionMagicCruiseVelocity = 2.37; // RPS
    config.MotionMagic.MotionMagicAcceleration = 10; // RPS^2

    // Soft limits to prevent exceeding -90 to +270 degree physical range
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = DualEncoderCRT.FORWARD_LIMIT;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = DualEncoderCRT.REVERSE_LIMIT;

    boolean success = TalonFXUtil.applyConfigWithRetries(leader, config);
    motorConfigAlert.set(!success);
  }

  /**
   * Initialize the encoder position using CRT calculation from dual encoders. This seeds encoder
   * 1's continuous position so FusedCANcoder reports the correct mechanism position. Should be
   * called once at startup when the turret is stationary.
   */
  private void initializePosition() {
    boolean success = crt.seedEncoderPosition();
    crtInitAlert.set(!success);
  }

  // Cached primitive values updated in periodic()
  private double cachedPositionRot;

  @Override
  public void periodic() {
    long _t = System.nanoTime();
    BaseStatusSignal.refreshAll(positionSignal, velocitySignal);
    cachedPositionRot =
        BaseStatusSignal.getLatencyCompensatedValueAsDouble(positionSignal, velocitySignal);
    Logger.recordOutput("Timing/TurretMs", (System.nanoTime() - _t) / 1e6);
  }

  public void setAngle(double angle) {
    leader.setControl(angleOut.withPosition(angle));
  }

  @AutoLogOutput
  public double getAngleRot() {
    return cachedPositionRot;
  }

  public Angle getAngle() {
    return Rotations.of(cachedPositionRot);
  }

  @AutoLogOutput
  public double getTargetAngleRot() {
    return angleOut.Position;
  }

  public Angle getTargetAngle() {
    return angleOut.getPositionMeasure();
  }

  @AutoLogOutput
  public double getVelocityRPS() {
    return velocitySignal.getValueAsDouble();
  }

  private double getPositionError() {
    return Math.abs(cachedPositionRot - angleOut.Position);
  }

  /** Distance-dependent shoot gate: tighter position tolerance at longer range. */
  public boolean isAtTarget(double distanceToTarget) {
    boolean turretAtPosition =
        atTurret.calculate(
            getPositionError()
                <= Math.atan(MAX_LATERAL_MISS_M / distanceToTarget) / (2.0 * Math.PI));
    return turretAtPosition && isNotFlipping();
  }

  /** Looser check for feed shots - wider tolerance than hub shots. */
  public boolean isAtFeedTarget(double distanceToTarget) {
    return getPositionError()
        <= Math.atan(FEED_LATERAL_MISS_M / distanceToTarget) / (2.0 * Math.PI);
  }

  /** Loose check: turret is not way off target (e.g. mid-slew). */
  private static final double NOT_FLIPPING_TOLERANCE_ROT = 10.0 / 360.0;

  public boolean isNotFlipping() {
    return getPositionError() <= NOT_FLIPPING_TOLERANCE_ROT;
  }

  /** Command that continuously tracks the hub using a supplied angle. */
  public Command trackHubCommand(Supplier<Double> angleSupplier) {
    return run(() -> setAngle(angleSupplier.get()));
  }

  public Command stopCommand() {
    return runOnce(() -> stop());
  }

  private void stop() {
    leader.stopMotor();
  }
}
