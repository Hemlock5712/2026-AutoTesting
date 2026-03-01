package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.FieldInfo;
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

  private final MotionMagicVoltage angleOut = new MotionMagicVoltage(0);

  private static final Angle TOLERANCE = Rotations.of(0.01); // ~3.6 degrees

  protected TalonFXConfiguration config = new TalonFXConfiguration();

  // Alerts
  Alert motorConfigAlert = new Alert("Turret Motor Configuration Failed", AlertType.kError);
  Alert crtInitAlert = new Alert("Turret CRT Position Initialization Failed", AlertType.kWarning);

  @NotLogged
  public static final Pose3d TURRET_HOLE_CENTER =
      new Pose3d(-0.127, 0.13018, 0.3556, Rotation3d.kZero);

  public static final Transform2d TURRET_TRANSFORM =
      new Transform2d(TURRET_HOLE_CENTER.getX(), TURRET_HOLE_CENTER.getY(), Rotation2d.kZero);

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
    config.Slot0.kS = 0; // Static friction compensation
    config.Slot0.kP = 0; // Proportional gain
    config.Slot0.kD = 0; // Derivative gain
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // MotionMagic settings - units are mechanism rotations
    config.MotionMagic.MotionMagicCruiseVelocity = 0; // RPS
    config.MotionMagic.MotionMagicAcceleration = 0; // RPS^2

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

  private void trackHub(SwerveDriveState currentState) {
    Pose2d robotPose = currentState.Pose;
    Pose2d turretPose = robotPose.transformBy(TURRET_TRANSFORM);
    Translation2d hubPosition = FieldInfo.flip(FieldInfo.HUB_POSITION);
    Translation2d toTarget = hubPosition.minus(turretPose.getTranslation());

    // Skip tracking if robot is too close to hub (avoids numerical instability)
    if (toTarget.getNorm() < 0.1) {
      return;
    }

    // Calculate the angle to the target in field coordinates
    Rotation2d angleToTargetField = toTarget.getAngle();

    // Calculate turret angle relative to robot forward (oppose robot rotation)
    double turretToTarget = angleToTargetField.minus(robotPose.getRotation()).getRotations();
    // Wrap angle to [-0.25, 0.75] rotations (-90 to +270 degrees) turret range
    turretToTarget = MathUtil.inputModulus(turretToTarget, -0.25, 0.75);

    setAngle(Rotations.of(turretToTarget));
  }

  public Command trackHubCommand(Supplier<SwerveDriveState> swerveState) {
    return run(() -> trackHub(swerveState.get()));
  }

  public void setAngle(Angle angle) {
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

  public Command stopCommand() {
    return runOnce(() -> stop());
  }

  private void stop() {
    leader.stopMotor();
  }
}
