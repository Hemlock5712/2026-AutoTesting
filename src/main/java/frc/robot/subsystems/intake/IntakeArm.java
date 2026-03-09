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
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.TalonFXUtil;

@Logged(strategy = Strategy.OPT_IN)
public class IntakeArm extends SubsystemBase {

  protected final TalonFX arm = new TalonFX(22, TunerConstants.kCANBus);
  protected final CANcoder arm_encoder = new CANcoder(24, TunerConstants.kCANBus);

  protected TalonFXConfiguration config = new TalonFXConfiguration();

  private final MotionMagicVoltage positionOut = new MotionMagicVoltage(0);

  private Angle TOLERANCE = Degrees.of(3);

  Alert motorConfigAlert = new Alert("Intake Arm Motor Configuration Failed", AlertType.kError);

  public IntakeArm() {
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    config.Slot0.kG = 0.44;
    config.Slot0.kS = 0.0;
    config.Slot0.kP = 16;
    config.Slot0.kD = 1;

    config.MotionMagic.MotionMagicCruiseVelocity = 4;
    config.MotionMagic.MotionMagicAcceleration = 8;
    config.Feedback.withRemoteCANcoder(arm_encoder);
    config.Feedback.RotorToSensorRatio = 25;

    boolean success = TalonFXUtil.applyConfigWithRetries(arm, config);
    motorConfigAlert.set(!success);
  }

  @Override
  public void periodic() {}

  private void setPosition(Angle position) {
    arm.setControl(positionOut.withPosition(position.in(Rotations)));
  }

  public Command intakeDown() {
    return runOnce(() -> setPosition(Degrees.of(0)));
  }

  public Command intakeUp() {
    return runOnce(() -> setPosition(Rotations.of(.27)));
  }

  @Logged
  public boolean isAtBumpHight() {
    return getPosition().in(Rotations) >= .1;
  }

  public Command killArm() {
    return runOnce(() -> arm.stopMotor());
  }

  public Command stopArm() {
    return runOnce(() -> arm.stopMotor());
  }

  @Logged
  public boolean isAtTarget() {
    return getPosition().isNear(getTargetPosition(), TOLERANCE);
  }

  @Logged
  public Angle getPosition() {
    return arm_encoder.getPosition().getValue();
  }

  @Logged
  public Angle getTargetPosition() {
    return positionOut.getPositionMeasure();
  }

  @Logged
  public Angle getTolerance() {
    return TOLERANCE;
  }
}
