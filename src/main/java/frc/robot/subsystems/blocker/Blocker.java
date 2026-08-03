package frc.robot.subsystems.blocker;

import static org.wpilib.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.utils.RobotMechanism;
import frc.robot.utils.TalonFXUtil;
import org.littletonrobotics.junction.AutoLogOutput;
import org.wpilib.command3.Command;
import org.wpilib.driverstation.Alert;
import org.wpilib.units.measure.Angle;

/**
 * Raises and lowers the four-bar blocker through a pulley-driven shaft.
 *
 * <p>The integrated TalonFX encoder is zeroed at the physical down position before the first
 * enabled test. The travel and Motion Magic constraints are intentionally conservative; increase
 * them only after confirming that the mechanism moves in the expected direction without binding.
 */
public class Blocker extends RobotMechanism {
  // Hardware/calibration constants. Confirm these values before enabling the mechanism.
  private static final int MOTOR_ID = 53;
  private static final double DOWN_POSITION_ROTATIONS = 0.0;
  private static final double UP_POSITION_ROTATIONS = 6.58;
  private static final double CRUISE_VELOCITY_RPS = 3;
  private static final double ACCELERATION_RPS2 = 6;
  private static final double POSITION_TOLERANCE_ROTATIONS = 0.01;

  private final TalonFX motor = new TalonFX(MOTOR_ID, new CANBus());
  private final PositionVoltage positionRequest = new PositionVoltage(DOWN_POSITION_ROTATIONS);
  private final StatusSignal<Angle> positionSignal = motor.getPosition();
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final Alert configAlert =
      new Alert("Blocker motor configuration failed", Alert.Level.HIGH);

  private boolean isUp;

  public Blocker() {
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // These are intentionally gentle starting gains and constraints for first motion tests.
    config.Slot0.kP = 8.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kS = 0.2;
    config.Slot0.kV = 0.12;
    config.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VELOCITY_RPS;
    config.MotionMagic.MotionMagicAcceleration = ACCELERATION_RPS2;

    // Keep commanded motion inside the initially tested range.
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 6.58;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

    configAlert.set(!TalonFXUtil.applyConfigWithRetries(motor, config));
    positionSignal.setUpdateFrequency(50);
    motor.optimizeBusUtilization();

    motor.setPosition(0);
    addPeriodicCallback(this::periodic);
  }

  public void periodic() {
    BaseStatusSignal.refreshAll(positionSignal);
  }

  /** Moves the blocker to its calibrated down position. */
  public Command down() {
    return runOnce(() -> setPosition(DOWN_POSITION_ROTATIONS, false));
  }

  /** Moves the blocker to its calibrated up position. */
  public Command up() {
    return runOnce(() -> setPosition(UP_POSITION_ROTATIONS, true));
  }

  /** Toggles between up and down; the default state is down. */
  public Command toggle() {
    return runOnce(
        () -> setPosition(isUp ? DOWN_POSITION_ROTATIONS : UP_POSITION_ROTATIONS, !isUp));
  }

  private void setPosition(double positionRotations, boolean up) {
    motor.setControl(positionRequest.withPosition(positionRotations));
    isUp = up;
  }

  @AutoLogOutput
  public boolean isUp() {
    return isUp;
  }

  @AutoLogOutput
  public double getPositionRotations() {
    return positionSignal.getValueAsDouble();
  }

  @AutoLogOutput
  public double getTargetPositionRotations() {
    return positionRequest.Position;
  }

  @AutoLogOutput
  public boolean isAtTarget() {
    return Math.abs(getPositionRotations() - positionRequest.Position)
        <= POSITION_TOLERANCE_ROTATIONS;
  }

  /** Seeds the integrated encoder when the four-bar is manually placed at its down hard stop. */
  public Command zeroAtDownPosition() {
    return runOnce(() -> motor.setPosition(Rotations.of(DOWN_POSITION_ROTATIONS)));
  }
}
