package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.TalonFXUtil;
import org.littletonrobotics.junction.AutoLogOutput;

public class IntakeArm extends SubsystemBase {

  protected final TalonFX arm = new TalonFX(22, TunerConstants.kCANBus);
  protected final CANcoder armEncoder = new CANcoder(24, TunerConstants.kCANBus);

  protected TalonFXConfiguration config = new TalonFXConfiguration();

  private final DynamicMotionMagicTorqueCurrentFOC positionOut =
      new DynamicMotionMagicTorqueCurrentFOC(0, 4, 8);

  private static final Angle TOLERANCE = Degrees.of(3);

  Alert motorConfigAlert = new Alert("Intake Arm Motor Configuration Failed", AlertType.kError);

  public IntakeArm() {
    // Coast mode: Motor can be moved by hand when disabled (easier for testing)
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // Set motor direction: positive power = counterclockwise rotation
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.Slot0.GravityType =
        GravityTypeValue.Arm_Cosine; // Automatically fights gravity using math

    config.Slot0.kG = 15; // Gravity compensation
    config.Slot0.kS = 0.0; // Static friction
    config.Slot0.kP = 600; // Proportional gain (speed of correction)
    config.Slot0.kD = 40; // Derivative gain (smoothness)

    // Motion limits
    config.MotionMagic.MotionMagicCruiseVelocity = 4; // Max speed
    config.MotionMagic.MotionMagicAcceleration = 8; // How fast to speed up
    // Tell the motor to use the CANcoder sensor for position measurements
    config.Feedback.withFusedCANcoder(armEncoder);
    config.Feedback.RotorToSensorRatio = 25;

    boolean success = TalonFXUtil.applyConfigWithRetries(arm, config);
    motorConfigAlert.set(!success);

    arm.optimizeBusUtilization();
    armEncoder.optimizeBusUtilization();
  }

  @Override
  public void periodic() {}

  private void setPosition(Angle position) {
    arm.setControl(
        positionOut.withPosition(position.in(Rotations)).withVelocity(5).withAcceleration(8));
  }

  private void setPositionSlow(Angle position) {
    arm.setControl(
        positionOut.withPosition(position.in(Rotations)).withVelocity(0.15).withAcceleration(1));
  }

  public Command intakeDown() {
    return runOnce(() -> arm.setControl(positionOut.withPosition(0)));
  }

  public Command intakeDownAUTO() {
    return runOnce(() -> arm.setControl(positionOut.withPosition(0).withFeedForward(-40)));
  }

  public Command intakeUp() {
    return runOnce(() -> setPosition(Rotations.of(.17)));
  }

  public Command straightUp() {
    return runOnce(() -> setPosition(Rotations.of(0.28)));
  }

  public Command intakeUpSlow() {
    return runOnce(() -> setPositionSlow(Rotations.of(.17)));
  }

  @AutoLogOutput
  public boolean isAtBumpHeight() {
    return getPosition().in(Rotations) >= .1;
  }

  public Command stopArm() {
    return runOnce(() -> arm.stopMotor());
  }

  @AutoLogOutput
  public boolean isAtTarget() {
    return getPosition().isNear(Rotations.of(getTargetPosition()), TOLERANCE);
  }

  @AutoLogOutput
  public Angle getPosition() {
    return armEncoder.getPosition().getValue();
  }

  @AutoLogOutput
  public double getTargetPosition() {
    return positionOut.Position;
  }

  @AutoLogOutput
  public Angle getTolerance() {
    return TOLERANCE;
  }
}
