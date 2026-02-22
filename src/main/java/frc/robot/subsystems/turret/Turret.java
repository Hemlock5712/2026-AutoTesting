package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldConstants;
import frc.robot.utils.TalonFXUtil;
import java.util.function.Supplier;

@Logged
public class Turret extends SubsystemBase {

  protected final TalonFX leader = new TalonFX(41, CANBus.roboRIO());

  private final MotionMagicExpoTorqueCurrentFOC angleOut = new MotionMagicExpoTorqueCurrentFOC(0);

  protected final double GEAR_RATIO = 110.0 / 25.0 * 7.0;
  private static final double ANGLE_TOLERANCE_ROTATIONS = 0.01; // ~3.6 degrees
  private final Angle tolerance = Rotations.of(ANGLE_TOLERANCE_ROTATIONS);

  Alert motorConfigAlert = new Alert("Turret Motor Configuration Failed", AlertType.kError);

  public Turret() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

    // PID gains
    config.Slot0.kS = 1.0; // Static friction compensation
    config.Slot0.kP = 20; // Proportional gain
    config.Slot0.kD = 0; // Derivative gain (damping to reduce overshoot)

    // MotionMagic settings - limits how fast the turret can move
    // Values are in sensor (motor) rotations per second
    // For ~1 rotation/sec at mechanism (57 deg/sec), motor needs: 1.0 * gearRatio =
    // 30.8 RPS
    config.MotionMagic.MotionMagicCruiseVelocity =
        30.0; // motor rotations/sec (~1 mechanism rot/sec)
    // Acceleration: how fast it can speed up (motor rotations per second squared)
    config.MotionMagic.MotionMagicAcceleration = 60.0; // motor rotations/sec²

    boolean success = TalonFXUtil.applyConfigWithRetries(leader, config);
    motorConfigAlert.set(!success);
  }

  private void trackHub(SwerveDriveState currentState) {
    Pose2d robotPose = currentState.Pose;

    // Calculate the angle to the target in field coordinates
    Rotation2d angleToTargetField =
        FieldConstants.HUB_POSITION.minus(robotPose.getTranslation()).getAngle();

    // Calculate turret angle relative to robot forward (oppose robot rotation)
    double turretToTarget = angleToTargetField.minus(robotPose.getRotation()).getRotations();
    // Wrap angle to [-0.5, 0.5] rotations (±180°) for shortest path
    turretToTarget = MathUtil.inputModulus(turretToTarget, -0.5, 0.5);

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
    return tolerance;
  }

  public boolean isAtTarget() {
    return getAngle().isNear(getTargetAngle(), tolerance);
  }

  public Command stopCommand() {
    return runOnce(() -> stop());
  }

  private void stop() {
    leader.stopMotor();
  }
}
