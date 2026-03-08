package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.TalonFXUtil;
import java.util.function.Supplier;

@Logged
public class Turret extends SubsystemBase {
  // Motor
  // protected final TalonFX leader = new TalonFX(DualEncoderCRT.MOTOR_ID,
  // TunerConstants.kCANBus);
  protected final TalonFX leader = new TalonFX(DualEncoderCRT.MOTOR_ID, TunerConstants.kCANBus);

  // Dual absolute encoders for CRT positioning
  protected final CANcoder encoder1 = new CANcoder(DualEncoderCRT.ENCODER_1_ID, TunerConstants.kCANBus);
  protected final CANcoder encoder2 = new CANcoder(DualEncoderCRT.ENCODER_2_ID, TunerConstants.kCANBus);

  // CRT calculator for absolute position determination
  private final DualEncoderCRT crt;

  // Motion magic parameters for fast mode
  private final double FAST_MOTION_MAGIC_CRUISE_VELOCITY = 2;
  private final double FAST_MOTION_MAGIC_ACCELERATION = 8;

  // Motion magic parameters for smooth mode
  private final double SMOOTH_MOTION_MAGIC_CRUISE_VELOCITY = 1;
  private final double SMOOTH_MOTION_MAGIC_ACCELERATION = 3;

  private final DynamicMotionMagicVoltage angleOut = new DynamicMotionMagicVoltage(0,
      SMOOTH_MOTION_MAGIC_CRUISE_VELOCITY,
      SMOOTH_MOTION_MAGIC_ACCELERATION);

  private static final Angle TOLERANCE = Rotations.of(0.01); // ~3.6 degrees

  protected TalonFXConfiguration config = new TalonFXConfiguration();

  private boolean isSmoothMotionMagic = true;

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
  }

  /**
   * Configure motor with FusedCANcoder feedback (encoder 1 fused with internal
   * rotor).
   */
  private void configureMotor() {
    // Fuse encoder 1 (22-tooth gear) with the motor's internal rotor for
    // high-bandwidth
    // absolute position tracking. CRT seeds encoder 1's continuous position at
    // startup
    // via initializePosition(), then FusedCANcoder handles tracking from there.
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    config.Feedback.FeedbackRemoteSensorID = encoder1.getDeviceID();

    // SensorToMechanismRatio: encoder rotations per mechanism rotation
    config.Feedback.SensorToMechanismRatio = DualEncoderCRT.ENCODER_1_MECHANISM_RATIO;
    // RotorToSensorRatio: motor rotor rotations per encoder rotation
    config.Feedback.RotorToSensorRatio = DualEncoderCRT.ROTOR_TO_ENCODER_RATIO;

    // PID gains
    config.Slot0.kS = 0.349609375; // Static friction compensation
    config.Slot0.kP = 128; // Proportional gain
    config.Slot0.kD = 0; // Derivative gain
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // MotionMagic settings - units are mechanism rotations
    config.MotionMagic.MotionMagicCruiseVelocity = 2; // RPS
    config.MotionMagic.MotionMagicAcceleration = 8; // RPS^2

    // Soft limits to prevent exceeding -90 to +270 degree physical range
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = DualEncoderCRT.FORWARD_LIMIT;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = DualEncoderCRT.REVERSE_LIMIT;

    boolean success = TalonFXUtil.applyConfigWithRetries(leader, config);
    motorConfigAlert.set(!success);
  }

  /**
   * Initialize the encoder position using CRT calculation from dual encoders.
   * This seeds encoder
   * 1's continuous position so FusedCANcoder reports the correct mechanism
   * position. Should be
   * called once at startup when the turret is stationary.
   */
  private void initializePosition() {
    boolean success = crt.seedEncoderPosition();
    crtInitAlert.set(!success);
  }

  public void setAngle(double angle) {
    if (isSmoothMotionMagic) {
      angleOut.Velocity = SMOOTH_MOTION_MAGIC_CRUISE_VELOCITY;
      angleOut.Acceleration = SMOOTH_MOTION_MAGIC_ACCELERATION;
    } else {
      angleOut.Velocity = FAST_MOTION_MAGIC_CRUISE_VELOCITY;
      angleOut.Acceleration = FAST_MOTION_MAGIC_ACCELERATION;
    }

    leader.setControl(angleOut.withPosition(angle));
  }

  public Angle getAngle() {
    return Rotations.of(leader.getPosition().getValueAsDouble());
  }

  public Angle getTargetAngle() {
    return angleOut.getPositionMeasure();
  }

  public Angle getTolerance() {
    return TOLERANCE;
  }

  public boolean isAtTarget() {
    return getAngle().isNear(getTargetAngle(), TOLERANCE);
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

  public Command setSmoothMotionMagic() {
    return runOnce(() -> isSmoothMotionMagic = true);
  }

  public Command setFastMotionMagic() {
    return runOnce(() -> isSmoothMotionMagic = false);
  }

  public void setSmoothMotionMagic(boolean isSmoothMotionMagic) {
    this.isSmoothMotionMagic = isSmoothMotionMagic;
  }

  public boolean isSmoothMotionMagic() {
    return isSmoothMotionMagic;
  }
}
