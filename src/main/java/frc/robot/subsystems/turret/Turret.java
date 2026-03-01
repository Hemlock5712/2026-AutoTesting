package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.TalonFXUtil;
import java.util.function.Supplier;

@Logged
public class Turret extends SubsystemBase {
  // Motor
  protected final TalonFX leader = new TalonFX(DualEncoderCRT.MOTOR_ID, TunerConstants.kCANBus);

  // Dual absolute encoders for CRT positioning
  protected final CANcoder encoder1 =
      new CANcoder(DualEncoderCRT.ENCODER_1_ID, TunerConstants.kCANBus);
  protected final CANcoder encoder2 =
      new CANcoder(DualEncoderCRT.ENCODER_2_ID, TunerConstants.kCANBus);

  // CRT calculator for absolute position determination
  private final DualEncoderCRT crt;

  private final PositionVoltage angleOut = new PositionVoltage(0);

  private static final Angle TOLERANCE = Rotations.of(0.01); // ~3.6 degrees

  protected TalonFXConfiguration config = new TalonFXConfiguration();

  // Alerts
  Alert motorConfigAlert = new Alert("Turret Motor Configuration Failed", AlertType.kError);
  Alert crtInitAlert = new Alert("Turret CRT Position Initialization Failed", AlertType.kWarning);

  public Turret() {
    // Initialize CRT calculator using default constants
    crt = new DualEncoderCRT(encoder1, encoder2);

    configureMotor();
    initializePosition();
  }

  /** Configure motor with RotorSensor feedback (internal encoder only). */
  private void configureMotor() {
    // Use motor's internal rotor sensor for feedback (RotorSensor is default)
    // Position is seeded from CRT calculation at startup via initializePosition()

    // SensorToMechanismRatio: rotor rotations per mechanism rotation
    // Motor spins 30.8 times per mechanism rotation
    config.Feedback.SensorToMechanismRatio = DualEncoderCRT.MOTOR_TO_MECHANISM_RATIO;

    // PID gains
    config.Slot0.kS = 0.349609375; // Static friction compensation
    config.Slot0.kP = 1000; // Proportional gain
    config.Slot0.kD = 3; // Derivative gain
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // MotionMagic settings - units are mechanism rotations
    config.MotionMagic.MotionMagicCruiseVelocity = 2; // RPS
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
   * Initialize the motor position using CRT calculation from dual encoders. This should be called
   * once at startup when the turret is stationary.
   */
  private void initializePosition() {
    // Calculate absolute mechanism position using CRT
    double mechanismPosition = crt.calculateMechanismPosition();

    // Set the motor's internal position to match the calculated position
    // This does NOT affect the CANcoder - it only syncs the motor's position tracking
    StatusCode setPosition = leader.setPosition(mechanismPosition);

    Robot.telemetry().log("Testing/", mechanismPosition);

    crtInitAlert.set(setPosition.isError());
  }

  public void setAngle(double angle) {
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
}
